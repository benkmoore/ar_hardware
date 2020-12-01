#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"

/*
 * ------------- FILE DEFINITION & SETUP ------------------
 */

// ROS Serial constants
#define NUM_PUBS 2
#define NUM_SUBS 1
#define BAUD_RATE 57600                               // bits/s
#define IN_BUFFER_SIZE 512                            // bytes
#define OUT_BUFFER_SIZE 512                           // bytes

// Stepper motor constants
#define MAX_MILLIAMPS 2800                            // mA
#define MICRO_STEP_SIZE 1                             // 1 step = 1/MICRO_STEP_SIZE
#define DECAY_MODE StepperDecayMode::AutoMixed        // PWM decay mode (recommended default)
#define MAX_STEPPER_VEL 200                            // step/s
#define MIN_STEPPER_VEL 40                            // step/s
#define STEPS_THRESHOLD 10                            // step
#define MAX_PHI_DELTA 10                             // steps from
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.295779513082320876798154814105

// Encoder constants
#define ENC_CPR 4096                                  // Counts Per Revolution

// Input number of DC motors, stepper motors in use
const int N_StepperMotors = 4;

// Step Motor pins: inner Y axis arm, outer Y axis arm, inner X axis arm, outer X axis arm
int StepperCSPins[N_StepperMotors] = {10, 36, 37, 38};


// Define steppers
Stepper stepper1(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);


/*
 * ------------- SETUP INTERFACE ------------------
 */

void setup() {


  // Setup stepper motors
  SPI.begin();
  stepper1.setupDriver(StepperCSPins[0]);
  stepper2.setupDriver(StepperCSPins[1]);
  stepper3.setupDriver(StepperCSPins[2]);
  stepper4.setupDriver(StepperCSPins[3]);


}

/*
 * ------------- MAIN ------------------
 */
void loop() {


  // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
  for (int i=0; i<=20000; i++) {
    stepper1.commandStepper(50, 1);
    stepper2.commandStepper(50, 1);
    stepper3.commandStepper(50, 1);
    stepper4.commandStepper(50, 1);
  }

//  for (int i=0; i<=20000; i++) {
//    stepper1.commandStepper(-50, 1);
//    stepper2.commandStepper(-50, 1);
//    stepper3.commandStepper(-50, 1);
//    stepper4.commandStepper(-50, 1);
//  }

}
