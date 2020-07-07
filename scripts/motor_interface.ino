#define USE_USBCON
#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <Stepper.h>

/*
 * ------------- FILE DEFINITION & SETUP ------------------
 */

// Max outputs, step to degrees
#define STEPPER_VEL 50
#define PHI_STEP 1.8
#define BAUD_RATE 57600
#define RAD_2_DEG 57.295779513082320876798154814105
 
// Input number of DC motors, stepper motors in use
const int N_DCMotors = 1;
const int N_StepperMotors = 4;

// Num of steppers steps for 360 degree rotation
const int oneRotationInSteps = 200;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

//DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{17, 16, 15}};

// Define steppers
Stepper stepper1(oneRotationInSteps, StepperPins[0][0], StepperPins[0][1]);
Stepper stepper2(oneRotationInSteps, StepperPins[1][0], StepperPins[1][1]);
Stepper stepper3(oneRotationInSteps, StepperPins[2][0], StepperPins[2][1]);
Stepper stepper4(oneRotationInSteps, StepperPins[3][0], StepperPins[3][1]);

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, 1, 1, 2048, 2048> motor_interface;

int stepper1_pos = 0;
int stepper2_pos = 0;
int stepper3_pos = 0;
int stepper4_pos = 0;

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
//  for(int i = 0; i < N_DCMotors; i++) {
//    if (msg.velocity_arr.data[i] > 0) {
//      Forward_DCMotor(msg.velocity_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
//    }
//    else if (msg.velocity_arr.data[i] < 0) {
//      Reverse_DCMotor(msg.velocity_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
//    }
//  }
//
  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  int numSteps1 = (int) ( (msg.phi_arr.data[0]*RAD_2_DEG)/PHI_STEP );
  int numSteps2 = (int) ( (msg.phi_arr.data[1]*RAD_2_DEG)/PHI_STEP );
  int numSteps3 = (int) ( (msg.phi_arr.data[2]*RAD_2_DEG)/PHI_STEP );
  int numSteps4 = (int) ( (msg.phi_arr.data[3]*RAD_2_DEG)/PHI_STEP );
  
//  if (stepper1_pos != numSteps1) {
//    stepper1.step(numSteps1);
//    stepper1_pos += stepper1_pos + numSteps1;
//  }

  stepStepper(stepper1, numSteps1, stepper1_pos);
  stepStepper(stepper2, numSteps2, stepper2_pos);
  stepStepper(stepper3, numSteps3, stepper3_pos);
  stepStepper(stepper4, numSteps4, stepper4_pos);
}

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds",controllerCmdCallback);

/*
 * ------------- SUPPORT FUNCTIONS ------------------
 */

void stepStepper(Stepper stepper, int numSteps, int &stepper_pos) {
  if (stepper_pos != numSteps) {
    stepper.step(numSteps);
    stepper_pos += stepper_pos + numSteps;
  }
}

// Note may need to add in speed ramp as to overcome motor bearing inertia
// eg : for (int i = 80; i < 250; i++) {
//     Forward(i);
//  }

// Define forward rotation - DC Motor
void Forward_DCMotor(int PWMspeed, byte in1 , byte in2 , byte en) {
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  analogWrite(en, PWMspeed); 
}

// Define reverse rotation - DC Motor
void Reverse_DCMotor(int PWMspeed, byte in1 , byte in2 , byte en) {
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW); 
  analogWrite(en, PWMspeed);  
}

/*
 * ------------- SETUP INTERFACE ------------------
 */

void setup() {
  //Serial.begin(BAUD_RATE);
  // Set pins to Output for DC motors
//  for(int i = 0; i < N_DCMotors; i++) {
//    pinMode(DCMotorPins[i][0], OUTPUT);   
//    pinMode(DCMotorPins[i][1], OUTPUT);
//    pinMode(DCMotorPins[i][2], OUTPUT); 
//  }

  // Set stepper velocity
  stepper1.setSpeed(STEPPER_VEL);
  stepper2.setSpeed(STEPPER_VEL);
  stepper3.setSpeed(STEPPER_VEL);
  stepper4.setSpeed(STEPPER_VEL);
  
  // Init node and Subscribe to /controller_cmds
  motor_interface.getHardware()->setBaud(BAUD_RATE);
  motor_interface.initNode();
  motor_interface.subscribe(controller_cmds_sub);
}

/*
 * ------------- MAIN ------------------
 */

void loop() {
  motor_interface.spinOnce();
}
