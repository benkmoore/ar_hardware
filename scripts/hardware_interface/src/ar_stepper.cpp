#include "SPI.h"
#include "Arduino.h"
#include "ar_stepper.h"


// ---------- STEPPER FUNCTIONS -----------

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper::Stepper(int stepsIn2pi, float phi_step, int steps_threshold, int max_phi_delta, int max_vel,
                int min_vel, int max_milliamps, int micro_step_size, StepperDecayMode decay_mode) {
  // set constants
  this->phi_step = phi_step;
  this->max_vel = max_vel;
  this->min_vel = min_vel;
  this->steps_threshold = steps_threshold;
  this->max_phi_delta = max_phi_delta;
  this->max_milliamps = max_milliamps;
  this->decay_mode = decay_mode;

  // init class variables
  this->direction = -1;
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->stepsIn2pi = stepsIn2pi; // total number of steps for this motor
  this->phi_flag = false;
  this->revolutions = 0;
  this->prev_encoder_data = 0;
}

void Stepper::setupDriver(int cs_pin) {
  // setup driver and stepper modes
  this->driver = Driver();
  this->driver.setCSPin(cs_pin);
  delay(1);                 // allow driver time to power up
  this->driver.resetSettings();
  this->setDecayMode(this->decay_mode);
  this->setMaxCurrent(this->max_milliamps);
  this->setStepMode(this->micro_step_size);
  this->enableDriver();
}

/*
 * Command number of steps (cw/ccw) for stepper
 */
void Stepper::commandStepper(int enc_pos, int phi_des) {
  //this->driver.clearFaults();

  int steps = this->calculateSteps(enc_pos, phi_des);
  this->controlSpeed(steps);

  if (steps > 0) { // counter-clockwise
    this->direction = 1;
    this->setDirection(1);
  } else if (steps < 0) { // clockwise
    this->direction = 0;
    this->setDirection(0);
  }
  this->step(steps);

}

/*
 * Calculate steps from encoder data pos to desired phi.
 * wraps the number of steps to [-100, 99] = [-pi, pi]
 * commands the shortest numbers of steps and direction
 * between the encoder data and phi desired. Checks if
 * difference between phi desired and phi currrent is less
 * than the max allowable number of steps and updates the
 * phi flag to stop or conutinue actuating the DC motors.
 */
int Stepper::calculateSteps(int encoder_data, int phi_des) {
  int steps = phi_des - encoder_data;
  if (abs(steps) > int(180.0/this->phi_step)) {
    if (steps > 0) {
      steps = steps - int(360.0/this->phi_step);
    } else if (steps < 0) {
      steps = steps + int(360.0/this->phi_step);
    }
  }

  if (abs(steps) > this->max_phi_delta) { // check phi delta
      this->phi_flag = true;
  } else {
      this->phi_flag = false;
  }

  this->checkRevolutions(encoder_data); // update revolution count

  return steps;
}

void Stepper::checkRevolutions(int encoder_data) {
    if (abs(encoder_data) > 90 and this->prev_encoder_data != 0) { // check revolutions about the -100, 99 position
        int sign_curr = encoder_data/abs(encoder_data);
        int sign_prev = this->prev_encoder_data/abs(this->prev_encoder_data);

        if (sign_curr != sign_prev) { // compare signs
            if (sign_curr == 1 and sign_prev == -1) {
                this->revolutions -= 1; // clockwise
            } else if (sign_curr == -1 and sign_prev == 1) {
                this->revolutions += 1; // counter-clockwise
            }
        }
    }
    this->prev_encoder_data = encoder_data;
}

/*
 * Control stepper speed to decelerate to stop. Adjusts
 * velocity based on number of steps remaining and never drops
 * below MIN_VEL.
 */
void Stepper::controlSpeed(int steps) {
  if ((abs(steps) > this->steps_threshold) or steps == 0) {
    this->setSpeedRPM(this->max_vel);
  } else {
    this->setSpeedRPM(max(this->min_vel,int(this->max_vel/this->steps_threshold)*abs(steps)));
  }

}

/*
 * Sets the speed in steps per sec
 */
void Stepper::setSpeedRPM(long whatSpeed) {
  this->step_delay = abs(1000L * 1000L /  whatSpeed);
}

/*
 * Sets direction of rotation
 */
void Stepper::setDirection(bool value) {
  if (value) { this->driver.ctrl |= (1 << 1); }
  else { this->driver.ctrl &= ~(1 << 1); }
  this->driver.writeReg(StepperRegAddr::CTRL, this->driver.ctrl);
}

/*
 * Get current direction of rotation
 */
bool Stepper::getDirection() {
  return this->driver.ctrl >> 1 & 1;
}

/*
 * Moves the motor if steps_to_move is greater than 0 and the time between steps
 * is greater than or equal to step_delay.
 */
void Stepper::step(int steps_to_move) {
  if (abs(steps_to_move) > 0) {
    long now = micros();

    if ((now - this->last_step_time) >= this->step_delay) {
        this->last_step_time = now;
  	    this->driver.writeReg(StepperRegAddr::CTRL, this->driver.ctrl | (1 << 2));
    }
  }
}

/*
 * Unwind stepper wire and update revolution counter
 */
void Stepper::unwind(int encoder_data) {
  this->checkRevolutions(encoder_data); // update during unwind

  if (this->revolutions > 0) {
    this->setDirection(0); // clockwise
    this->direction = 0;
    this->step(this->revolutions * this->stepsIn2pi);
  } else if (this->revolutions < 0) {
    this->setDirection(1); // counter-clockwise
    this->direction = 1;
    this->step(this->revolutions * this->stepsIn2pi);
  }

}

/*
 * Enables the driver (ENBL = 1).
 */
void Stepper::enableDriver() {
  this->driver.ctrl |= (1 << 0);
  this->driver.writeReg(StepperRegAddr::CTRL, this->driver.ctrl);
}

/*
 * Set stepper step size in micro steps.
 */
void Stepper::setStepMode(uint16_t mode) {
  uint8_t sm = 0b0010; // 1/4 micro-step by default.

  switch (mode) {
  case 1:   sm = 0b0000; break;
  case 2:   sm = 0b0001; break;
  case 4:   sm = 0b0010; break;
  case 8:   sm = 0b0011; break;
  case 16:  sm = 0b0100; break;
  case 32:  sm = 0b0101; break;
  case 64:  sm = 0b0110; break;
  case 128: sm = 0b0111; break;
  case 256: sm = 0b1000; break;
  }

  this->driver.ctrl = (this->driver.ctrl & 0b111110000111) | (sm << 3);
  this->driver.writeReg(StepperRegAddr::CTRL, this->driver.ctrl);
}

void Stepper::setMaxCurrent(uint16_t current) {
  if (current > 8000) { current = 8000; }

  uint8_t isgainBits = 0b11;
  uint16_t torqueBits = ((uint32_t)768  * current) / 6875;

  while (torqueBits > 0xFF) {
    isgainBits--;
    torqueBits >>= 1;
  }

  this->driver.ctrl = (this->driver.ctrl & 0b110011111111) | (isgainBits << 8);
  this->driver.writeReg(StepperRegAddr::CTRL, this->driver.ctrl);
  this->driver.torque = (this->driver.torque & 0b111100000000) | torqueBits;
  this->driver.writeReg(StepperRegAddr::TORQUE, this->driver.torque);
}

void Stepper::setDecayMode(StepperDecayMode mode) {
  this->driver.decay = (this->driver.decay & 0b00011111111) | (((uint8_t)mode & 0b111) << 8);
  this->driver.writeReg(StepperRegAddr::DECAY, this->driver.decay);
}

uint8_t Stepper::readStatus() {
  return this->driver.readReg((uint8_t)StepperRegAddr::STATUS);
}


// ---------- DRIVER FUNCTIONS -----------

Driver::Driver() {
  this->settings = SPISettings(500000, MSBFIRST, SPI_MODE0);
}


/*
 * Select CS pin for stepper, handles address/id of stepper.
 */
void Driver::setCSPin(uint8_t cs_pin) {
  this->cs_pin = cs_pin;
  digitalWrite(cs_pin, LOW);
  pinMode(cs_pin, OUTPUT);
}

uint16_t Driver::readReg(uint8_t address) {
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).

    selectChip();
    uint16_t dataOut = transferToSPI((0x8 | (address & 0b111)) << 12);
    deselectChip();
    return dataOut & 0xFFF;
}

/*
 * Reset driver setttings.
 */
void Driver::resetSettings() {
  this->ctrl   = 0xC90;
  this->torque = 0x1FF;
  this->off    = 0x030;
  this->blank  = 0x080;
  this->decay  = 0x110;
  this->stall  = 0x040;
  this->drive  = 0xA59;

  this->writeReg(StepperRegAddr::TORQUE, this->torque);
  this->writeReg(StepperRegAddr::OFF, this->off);
  this->writeReg(StepperRegAddr::BLANK, this->blank);
  this->writeReg(StepperRegAddr::DECAY, this->decay);
  this->writeReg(StepperRegAddr::STALL, this->stall);
  this->writeReg(StepperRegAddr::DRIVE, this->drive);
  this->writeReg(StepperRegAddr::CTRL, this->ctrl);

  // clear status of motor on driver
  this->writeReg(StepperRegAddr::STATUS, 0);
}

void Driver::clearFaults() {
  this->writeReg(StepperRegAddr::STATUS, ~0b00111111);
}

// Writes the specified value to a register.
void Driver::writeReg(uint8_t address, uint16_t value) {
  // Read/write bit and register address are the first 4 bits of the first
  // byte; data is in the remaining 4 bits of the first byte combined with
  // the second byte (12 bits total).
  this->selectChip();
  this->transferToSPI(((address & 0b111) << 12) | (value & 0xFFF));
  this->deselectChip();
}

// Writes the specified value to a register.
void Driver::writeReg(StepperRegAddr address, uint16_t value) {
  writeReg((uint8_t)address, value);
  delayMicroseconds(5); // required to allow time to write to registers
}

void Driver::selectChip() {
  digitalWrite(this->cs_pin, HIGH);
  SPI.beginTransaction(this->settings);
}

void Driver::deselectChip() {
 SPI.endTransaction();
 digitalWrite(this->cs_pin, LOW);
}

uint16_t Driver::transferToSPI(uint16_t value) {
  return SPI.transfer16(value);
}


// ---------- Encoder FUNCTIONS -----------

AMTEncoder::AMTEncoder(int Re, int De){
  this->response = 0;
  this->Re = Re;
  this->De = De;
  Serial2.begin(115200);
 // Serial.begin(115200);
}


int AMTEncoder::checkEncoder(int address) {

  byteOut = address;
  this->RS485Transmit();
  //int available = Serial2.availableForWrite();
  //Serial.print("start");
  //Serial.println(available);
  Serial2.write(byteOut);
   //available = Serial2.availableForWrite();
   //Serial.println(available);
   delayMicroseconds(1000);
  Serial2.flush();
  //available = Serial2.availableForWrite();
//Serial.println(available);
  this->RS485Receive();
  i = 0;
  // Look for data from encoder
  while (Serial2.available())
  {
    byteIn[i] = Serial2.read();
//Serial.println(byteIn[i],BIN);
    i ++;
  }
  byteIn[2] = byteIn[2] << 2; // remove checksum (two most significant bits)
  byteIn[2] = byteIn[2] >> 2; // remove checksum (two most significant bits)
  byteIn[1] = byteIn[1] >> 2; //remove two least significant bits as our encoders are 12 bit, not 14 bit (2 bytes read = 16 bits - 2 LSB's and 2 MSB'2 gives 12 bit encoder data)

  response = byteIn[2];
  response = (response << 6) + byteIn[1];
 // Serial.println(response);
  return response;
}

void AMTEncoder::RS485Transmit()
{
  digitalWrite(this->Re, LOW);
  digitalWrite(this->De, HIGH);
}

void AMTEncoder::RS485Receive()
{
  digitalWrite(this->Re, HIGH);
  digitalWrite(this->De, LOW);
}
