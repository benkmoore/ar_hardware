
#include "Arduino.h"
#include "ar_stepper.h"

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, float phi_step, int steps_threshold, int max_vel, int min_vel)
{	
  // set constants
  this->phi_step = phi_step;
  this->max_vel = max_vel;
  this->min_vel = min_vel;
  this->steps_threshold = steps_threshold;

  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);

  // When there are only 2 pins, set the others to 0:
  this->motor_pin_3 = 0;
  this->motor_pin_4 = 0;
  this->motor_pin_5 = 0;

  // pin_count is used by the stepMotor() method:
  this->pin_count = 2;
}

/*
 * Sets the speed in revs per minute
 */
void Stepper::setSpeed(long whatSpeed)
{
  this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void Stepper::step(int steps_to_move)
{
  int steps_left = abs(steps_to_move);  // how many steps to take

  // determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0) { this->direction = 1; }
  if (steps_to_move < 0) { this->direction = 0; }


  // decrement the number of steps, moving one step each time:
  if (steps_left > 0)
  {
    unsigned long now = micros();
    // move only if the appropriate delay has passed:
    if (now - this->last_step_time >= this->step_delay)
    {
      // get the timeStamp of when you stepped:
      this->last_step_time = now;
      // increment or decrement the step number,
      // depending on direction:
      if (this->direction == 1)
      {
        this->step_number++;
        if (this->step_number == this->number_of_steps) {
          this->step_number = 0;
        }
      }
      else
      {
        if (this->step_number == 0) {
          this->step_number = this->number_of_steps;
        }
        this->step_number--;
      }
      // decrement the steps left:
      steps_left--;
      // step the motor to step number 0, 1, ..., {3 or 10}
      if (this->pin_count == 5)
        stepMotor(this->step_number % 10);
      else
        stepMotor(this->step_number % 4);
    }
  }
}

/*
 * Set stepper speed  to decelerate to stop. Adjusts 
 * velocity based on number of steps remaining and never drops
 * below MIN_VEL.
 */
void Stepper::setStepperSpeed(int steps) {
  if (abs(steps) > this->steps_threshold) {
    this->setSpeed(this->max_vel);
  } else {
    this->setSpeed(max(this->min_vel,int(this->max_vel/this->steps_threshold)*abs(steps)));
  }
}

/*
 * Command number of steps (cw/ccw) for each stepper
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
 * Moves the motor forward or backwards.
 */
void Stepper::stepMotor(int thisStep)
{
  if (this->pin_count == 2) {
    switch (thisStep) {
      case 0:  // 01
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
      break;
      case 1:  // 11
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, HIGH);
      break;
      case 2:  // 10
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
      break;
      case 3:  // 00
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
      break;
    }
  }
  if (this->pin_count == 4) {
    switch (thisStep) {
      case 0:  // 1010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 1:  // 0110
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 2:  //0101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
      case 3:  //1001
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
    }
  }

  if (this->pin_count == 5) {
    switch (thisStep) {
      case 0:  // 01101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 1:  // 01001
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 2:  // 01011
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 3:  // 01010
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 4:  // 11010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 5:  // 10010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 6:  // 10110
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 7:  // 10100
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 8:  // 10101
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 9:  // 00101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
    }
  }
}

/*
  version() returns the version of the library:
*/
int Stepper::version(void)
{
  return 5;
}
