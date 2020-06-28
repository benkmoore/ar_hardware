
#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <AccelStepper.h>


/*
 * ------------- FILE DEFINTIONS ------------------
 */

// ROS node
ros::NodeHandle motor_interface;

// Max outputs, step to degrees
#define MAX_SPEED 200
#define PHI_STEP 1.8

// Interface type for stepper motors
byte InterfaceType = 1;

// Input number of DC motors and stepper motors in use
const int N_DCMotors = 1;
const int N_StepperMotors = 1;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{1, 0}};

//DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{17, 16, 15}};

// Define steppers
AccelStepper stepper1(InterfaceType, StepperPins[0][0], StepperPins[0][1]);

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
 * ------------- SETUP MOTORS ------------------
 */

void setup() {
  Serial.println("Setting up motors...");
  // Set pins to Output for DC motors
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DCMotorPins[i][0], OUTPUT);   
    pinMode(DCMotorPins[i][1], OUTPUT);
    pinMode(DCMotorPins[i][2], OUTPUT); 
  }
  
  // Define stepper motors max speeds (otherwise won't rotate)
  stepper1.setMaxSpeed(MAX_SPEED);
  
  // Init node and Subscribe to /controller_cmds
  motor_interface.initNode();
  ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds", &controllerCmdCallback);
  motor_interface.subscribe(controller_cmds_sub);
}


/*
 * ------------- RECEIVE ROS MSGS ------------------
 */

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &controller_cmd_msg) {
  Serial.println("Received cmd msg...");
  for(int i = 0; i < N_DCMotors; i++) {
    if (controller_cmd_msg.velocity_arr.data[i] > 0) {
      Forward_DCMotor(controller_cmd_msg.velocity_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
    else if (controller_cmd_msg.velocity_arr.data[i] < 0) {
      Reverse_DCMotor(controller_cmd_msg.velocity_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
  }

  stepper1.moveTo(controller_cmd_msg.phi_arr.data[0]);
  stepper1.run();
}


/*
 * ------------- MAIN ------------------
 */

void loop() {
  motor_interface.spinOnce();
}
