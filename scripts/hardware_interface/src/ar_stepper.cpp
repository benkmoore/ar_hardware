
#include "SPI.h"
#include "Arduino.h"
#include "ar_stepper.h"

/// Addresses of control and status registers.
enum class StepperRegAddr : uint8_t
{
  CTRL   = 0x00,
  TORQUE = 0x01,
  OFF    = 0x02,
  BLANK  = 0x03,
  DECAY  = 0x04,
  STALL  = 0x05,
  DRIVE  = 0x06,
  STATUS = 0x07,
};

// Possible arguments to set decay mode.
enum class StepperDecayMode : uint8_t
{
  Slow                = 0b000,
  SlowIncMixedDec     = 0b001,
  Fast                = 0b010,
  Mixed               = 0b011,
  SlowIncAutoMixedDec = 0b100,
  AutoMixed           = 0b101,
};

/// Possible arguments to set micro-stepping.
enum class StepperStepMode : uint16_t
{
  MicroStep256 = 256,
  MicroStep128 = 128,
  MicroStep64  =  64,
  MicroStep32  =  32,
  MicroStep16  =  16,
  MicroStep8   =   8,
  MicroStep4   =   4,
  MicroStep2   =   2,
  MicroStep1   =   1,
};

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper::Stepper(int stepsIn2pi, float phi_step, int steps_threshold, int max_vel,
                int min_vel, int max_milliamps, string micro_step_size, string decay_mode)
{
  // set constants
  this->phi_step = phi_step;
  this->max_vel = max_vel;
  this->min_vel = min_vel;
  this->steps_threshold = steps_threshold;
  this->max_milliamps = max_milliamps;

  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->stepsIn2pi = stepsIn2pi; // total number of steps for this motor

  // setup
  this->setChipSelectPin(cs_pin);
  delay(1);                 // allow driver time to power up
  this->resetSettings();
  this->clearStatus();
  this->setDecayMode(StepperDecayMode::decay_mode);
  this->setMaxCurrent(max_milliamps);
  this->setStepMode(StepperStepMode::micro_step_size);
  this->enableDriver();

}

/*
 * Control stepper speed to decelerate to stop. Adjusts
 * velocity based on number of steps remaining and never drops
 * below MIN_VEL.
 */
void Stepper::controlSpeed(int steps) {
  if (abs(steps) > this->steps_threshold) {
    this->setSpeedRPM(this->max_vel);
  } else {
    this->setSpeedRPM(max(this->min_vel,int(this->max_vel/this->steps_threshold)*abs(steps)));
  }
}

/*
 * Sets the speed in revs per minute
 */
void Stepper::setSpeedRPM(long whatSpeed)
{
  this->step_delay = 60L * 1000L * 1000L / this->stepsIn2pi / whatSpeed;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void Stepper::step(Driver driver)
{
  driver.writeReg(StepperRegAddr::CTRL, ctrl | (1 << 2));
}

/*
 * Command number of steps (cw/ccw) for stepper
 */
void Stepper::commandStepper(int enc_pos, int phi_des) {
  int steps = -1*(this->calculateSteps(enc_pos, phi_des)); // -1, fix stepper cw/ccw mappings
  this->setStepperSpeed(steps);
  this->step(steps);
}

/*
 * Calculate steps from encoder data pos to desired phi.
 * wraps the number of steps to [-100, 100] = [-pi, pi]
 * commands the shortest numbers of steps and direction
 * between the encoder data and phi desired.
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
  return steps;
}

/*
 * Enables the driver (ENBL = 1).
 */
void Stepper::enableDriver(Driver driver)
{
  ctrl |= (1 << 0);
  driver.writeReg(StepperRegAddr::CTRL, ctrl);
}

/*
 * Sets direction of rotation
 */
void Stepper::setDirection(bool value)
{
  if (value)
  {
    ctrl |= (1 << 1);
  }
  else
  {
    ctrl &= ~(1 << 1);
  }
  driver.writeReg(StepperRegAddr::CTRL, ctrl);
}

void Stepper::setStepMode(StepperStepMode mode, Driver driver)
{
  uint8_t sm = 0b0010; // Pick 1/4 micro-step by default.

  switch (mode)
  {
  case StepperStepMode::MicroStep1:   sm = 0b0000; break;
  case StepperStepMode::MicroStep2:   sm = 0b0001; break;
  case StepperStepMode::MicroStep4:   sm = 0b0010; break;
  case StepperStepMode::MicroStep8:   sm = 0b0011; break;
  case StepperStepMode::MicroStep16:  sm = 0b0100; break;
  case StepperStepMode::MicroStep32:  sm = 0b0101; break;
  case StepperStepMode::MicroStep64:  sm = 0b0110; break;
  case StepperStepMode::MicroStep128: sm = 0b0111; break;
  case StepperStepMode::MicroStep256: sm = 0b1000; break;
  }

  ctrl = (ctrl & 0b111110000111) | (sm << 3);
  driver.writeReg(StepperRegAddr::CTRL, ctrl);
}

void Stepper::setStepMode(uint16_t mode)
{
  setStepMode((StepperStepMode)mode);
}

void Stepper::setCurrentMilliamps36v4(uint16_t current)
  {
    if (current > 8000) { current = 8000; }

    uint8_t isgainBits = 0b11;
    uint16_t torqueBits = ((uint32_t)768  * current) / 6875;

    while (torqueBits > 0xFF)
    {
      isgainBits--;
      torqueBits >>= 1;
    }

    ctrl = (ctrl & 0b110011111111) | (isgainBits << 8);
    driver.writeReg(StepperRegAddr::CTRL, ctrl);
    torque = (torque & 0b111100000000) | torqueBits;
    driver.writeReg(StepperRegAddr::TORQUE, torque);
  }

// ---------- DRIVER FUNCTIONS -----------

/*
 * Select CS pin for stepper, handles address/id of stepper.
 */
void Driver::setCSPin(uint8_t cs_pin)
{
  digitalWrite(cs_pin, LOW);
  pinMode(cs_pin, OUTPUT);
}

/*
 * Reset driver setttings.
 */
void Driver::resetSettings()
{
  ctrl   = 0xC10;
  torque = 0x1FF;
  off    = 0x030;
  blank  = 0x080;
  decay  = 0x110;
  stall  = 0x040;
  drive  = 0xA59;
  this->writeReg(StepperRegAddr::CTRL, ctrl);
  this->writeReg(StepperRegAddr::TORQUE, torque);
  this->writeReg(StepperRegAddr::OFF, off);
  this->writeReg(StepperRegAddr::BLANK, blank);
  this->writeReg(StepperRegAddr::DECAY, decay);
  this->writeReg(StepperRegAddr::STALL, stall);
  this->writeReg(StepperRegAddr::DRIVE, drive);
}

// Writes the specified value to a register.
void Driver::writeReg(uint8_t address, uint16_t value)
{
  // Read/write bit and register address are the first 4 bits of the first
  // byte; data is in the remaining 4 bits of the first byte combined with
  // the second byte (12 bits total).

  selectChip();
  transfer(((address & 0b111) << 12) | (value & 0xFFF));

  // The CS line must go low after writing for the value to actually take
  // effect.
  deselectChip();
}

// Writes the specified value to a register.
void Driver::writeReg(StepperRegAddr address, uint16_t value)
{
  writeReg((uint8_t)address, value);
}

uint16_t Driver::transferToSPI(uint16_t value)
{
  return SPI.transfer16(value);
}
