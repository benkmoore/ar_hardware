#define USE_USBCON
#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <std_msgs/Float64.h>
#include <Encoder.h>
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
#define MAX_STEPPER_VEL 80                            // step/s
#define MIN_STEPPER_VEL 25                            // step/s
#define STEPS_THRESHOLD 20                            // step
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.295779513082320876798154814105

// Encoder constants
#define ENC_CPR 4000                                  // Counts Per Revolution
#define DEADBAND 2                                    // steps

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

// DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{38, 37, 36}, {35, 34, 33}, {22, 21, 23}, {13, 14, 15}};

// Motor interface type for stppers
byte motorInterfaceType = 1;

// Define steppers
Stepper stepper1(int(360.0/PHI_STEP), StepperPins[0][0], StepperPins[0][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);
Stepper stepper2(int(360.0/PHI_STEP), StepperPins[1][0], StepperPins[1][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);
Stepper stepper3(int(360.0/PHI_STEP), StepperPins[2][0], StepperPins[2][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);
Stepper stepper4(int(360.0/PHI_STEP), StepperPins[3][0], StepperPins[3][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);

int phi_des1 = 0;
int phi_des2 = 0;
int phi_des3 = 0;
int phi_des4 = 0;

// Encoder pin outs
int encPinA_1 = 26;                                   // enc1 -> stepper1
int encPinB_1 = 25;
int encPinA_2 = 28;                                   // enc2 -> stepper2
int encPinB_2 = 27;
int encPinA_3 = 30;                                   // enc3 -> stepper3
int encPinB_3 = 29;
int encPinA_4 = 32;                                   // enc4 -> stepper4
int encPinB_4 = 31;

// Define encoders
Encoder enc1(encPinA_1, encPinB_1);
Encoder enc2(encPinA_2, encPinB_2);
Encoder enc3(encPinA_3, encPinB_3);
Encoder enc4(encPinA_4, encPinB_4);

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

std_msgs::Float64 fl_msg;
ros::Publisher chatter("chatter", &fl_msg);

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
  for(int i = 0; i < N_DCMotors; i++) {
    if (msg.omega_arr.data[i] > 0) {
      Forward_DCMotor(msg.omega_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
    else if (msg.omega_arr.data[i] < 0) {
      Reverse_DCMotor(msg.omega_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
    else {
      Forward_DCMotor(0, DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  phi_des1 = (int) ( (-msg.phi_arr.data[0]*RAD_2_DEG)/PHI_STEP );
  phi_des2 = (int) ( (-msg.phi_arr.data[1]*RAD_2_DEG)/PHI_STEP );
  phi_des3 = (int) ( (-msg.phi_arr.data[2]*RAD_2_DEG)/PHI_STEP );
  phi_des4 = (int) ( (-msg.phi_arr.data[3]*RAD_2_DEG)/PHI_STEP );
  
}

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds",controllerCmdCallback);


/*
 * ------------- SUPPORT FUNCTIONS ------------------
 */

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

// wrap encoder output to [-100, 100] steps = [-pi, pi] rads
int wrapToPi(float encoder_data) {
  // counts * (degs/count) * (step/deg) = steps
  int encoder_pos = int( (encoder_data)*(360.0/ENC_CPR)*(1.0/PHI_STEP) ) % int( 360.0/PHI_STEP );
  if (encoder_pos > int( 180.0/PHI_STEP )) { encoder_pos = encoder_pos - int( 360.0/PHI_STEP ); }
  else if (encoder_pos < int( -180.0/PHI_STEP )) { encoder_pos = encoder_pos + int( 360.0/PHI_STEP ); }

  return encoder_pos;
}

/*
 * ------------- SETUP INTERFACE ------------------
 */

void setup() {
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  hardware_interface.subscribe(controller_cmds_sub);
  hardware_interface.advertise(chatter);

  // Setup motors
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DCMotorPins[i][0], OUTPUT);
    pinMode(DCMotorPins[i][1], OUTPUT);
    pinMode(DCMotorPins[i][2], OUTPUT);
  }
  
}

/*
 * ------------- MAIN ------------------
 */
void loop() {
  hardware_interface.spinOnce();

  // Feedback encoder data & wrap to [-pi, pi] = [-100, 100] steps
  int enc1_pos = wrapToPi(enc1.read());
  int enc2_pos = wrapToPi(enc2.read());
  int enc3_pos = wrapToPi(enc3.read());
  int enc4_pos = wrapToPi(enc4.read());
  
  stepper1.commandStepper(enc1_pos, phi_des1);
  stepper2.commandStepper(enc2_pos, phi_des2);
  stepper3.commandStepper(enc3_pos, phi_des3);
  stepper4.commandStepper(enc4_pos, phi_des4);

}
