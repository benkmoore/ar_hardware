#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

/*
 * ------------- FILE DEFINITION & SETUP ------------------
 */

// Max outputs, step to degrees
#define MAX_SPEED 200
#define PHI_STEP 1.8
#define BAUD_RATE 9600
#define RAD_2_DEG 57.295779513082320876798154814105
 
// Input number of DC motors and stepper motors in use
const int N_DCMotors = 1;
const int N_StepperMotors = 1;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{0, 1}};

//DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{17, 16, 15}};

// Interface type for stepper motors
byte InterfaceType = 1;

// Define steppers
AccelStepper stepper1(InterfaceType, StepperPins[0][0], StepperPins[0][1]);
MultiStepper steppers;

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, 1, 1, 2048, 2048> motor_interface;

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
  for(int i = 0; i < N_DCMotors; i++) {
    if (msg.velocity_arr.data[i] > 0) {
      Forward_DCMotor(msg.velocity_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
    else if (msg.velocity_arr.data[i] < 0) {
      Reverse_DCMotor(msg.velocity_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  while(stepper1.currentPosition() != (int) ( (msg.phi_arr.data[0]*RAD_2_DEG)/PHI_STEP )) {
    stepper1.setSpeed(300);
    stepper1.runSpeed();
  }
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

/*
 * ------------- SETUP INTERFACE ------------------
 */

void setup() {
  Serial.begin(BAUD_RATE);
  // Set pins to Output for DC motors
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DCMotorPins[i][0], OUTPUT);   
    pinMode(DCMotorPins[i][1], OUTPUT);
    pinMode(DCMotorPins[i][2], OUTPUT); 
  }
  // Define stepper motors max speeds (otherwise won't rotate)
  stepper1.setMaxSpeed(MAX_SPEED);
  steppers.addStepper(stepper1);
  
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
  delay(5000);
}
