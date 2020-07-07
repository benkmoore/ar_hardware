#define USE_USBCON
#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <AccelStepper.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

/*
 * ------------- FILE DEFINITION & SETUP ------------------
 */

// Max outputs, step to degrees
#define MAX_STEPPER_VEL 2000
#define STEPPER_VEL 1200
#define STEPPER_ACCEL 10
#define PHI_STEP 1.8
#define BAUD_RATE 57600
#define RAD_2_DEG 57.295779513082320876798154814105
 
// Input number of DC motors and stepper motors in use
const int N_DCMotors = 1;
const int N_StepperMotors = 4;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

//DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{17, 16, 15}};

// Interface type for stepper motors
byte InterfaceType = 1;

// Define steppers
AccelStepper stepper1(InterfaceType, StepperPins[0][0], StepperPins[0][1]);
AccelStepper stepper2(InterfaceType, StepperPins[1][0], StepperPins[1][1]);
AccelStepper stepper3(InterfaceType, StepperPins[2][0], StepperPins[2][1]);
AccelStepper stepper4(InterfaceType, StepperPins[3][0], StepperPins[3][1]);

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, 1, 1, 2048, 2048> motor_interface;

std_msgs::Float64 float_msg;
ros::Publisher chatter("chatter", &float_msg);

char hello[13] = "hello world!";
float step_pos = 0.0;

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
  //
//  stepper1.setCurrentPosition(1);
//  while(stepper1.currentPosition() != 400) { //(int) ( (msg.phi_arr.data[0]*RAD_2_DEG)/PHI_STEP )
//    stepper1.runSpeed();
//    stepper1.setCurrentPosition(-1);
//    float_msg.data = stepper1.currentPosition();
//    chatter.publish( &float_msg );
//    //t += 1;
//  }

//  int t = 0;
//  while( t<100000 ){
//    stepper1.runSpeed();
////    float_msg.data = stepper1.currentPosition();
////    chatter.publish( &float_msg );
//    t += 1;
//  }
  stepper1.move(500);
  stepper1.runSpeedToPosition();
}

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds",controllerCmdCallback);

/*
 * ------------- SUPPORT FUNCTIONS ------------------
 */

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
  // Define stepper motors max speeds (otherwise won't rotate)
  stepper1.setMaxSpeed(MAX_STEPPER_VEL);
//  stepper2.setMaxSpeed(MAX_STEPPER_VEL);
//  stepper3.setMaxSpeed(MAX_STEPPER_VEL);
//  stepper4.setMaxSpeed(MAX_STEPPER_VEL);

  stepper1.setCurrentPosition(0);
//  stepper2.setCurrentPosition(0);
//  stepper3.setCurrentPosition(0);
//  stepper4.setCurrentPosition(0);

  stepper1.setSpeed(STEPPER_VEL); //Set speed before acceleration NB!!
  
  stepper1.setAcceleration(STEPPER_ACCEL); //Set this low to overcome inertia from stationary start
  
  
  // Init node and Subscribe to /controller_cmds
  motor_interface.getHardware()->setBaud(BAUD_RATE);
  motor_interface.initNode();
  motor_interface.subscribe(controller_cmds_sub);
  motor_interface.advertise(chatter);
}

/*
 * ------------- MAIN ------------------
 */

void loop() {
  float_msg.data = stepper1.currentPosition();
  chatter.publish( &float_msg );
  motor_interface.spinOnce();
  delay(0.1);
}
