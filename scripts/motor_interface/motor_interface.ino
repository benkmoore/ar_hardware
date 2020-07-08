#define USE_USBCON
#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

/*
 * ------------- FILE DEFINITION & SETUP ------------------
 */

// Max outputs, step to degrees
#define STEPPER_VEL 40
#define MAX_STEPPER_VEL 100
#define STEPPER_ACCEL 30
#define PHI_STEP 1.8
#define BAUD_RATE 57600
#define RAD_2_DEG 57.295779513082320876798154814105
 
// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

// DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{15, 14, 13}, {17, 16, 18}, {20, 21, 19}, {22, 23, 12}};

// Motor interface type for stppers
byte motorInterfaceType = 1;

// Define steppers
AccelStepper stepper1(motorInterfaceType, StepperPins[0][0], StepperPins[0][1]);
AccelStepper stepper2(motorInterfaceType, StepperPins[1][0], StepperPins[1][1]);
AccelStepper stepper3(motorInterfaceType, StepperPins[2][0], StepperPins[2][1]);
AccelStepper stepper4(motorInterfaceType, StepperPins[3][0], StepperPins[3][1]);
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
    else {
      Forward_DCMotor(0, DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  int numSteps1 = (int) ( (msg.phi_arr.data[0]*RAD_2_DEG)/PHI_STEP );
  int numSteps2 = (int) ( (msg.phi_arr.data[1]*RAD_2_DEG)/PHI_STEP );
  int numSteps3 = (int) ( (msg.phi_arr.data[2]*RAD_2_DEG)/PHI_STEP );
  int numSteps4 = (int) ( (msg.phi_arr.data[3]*RAD_2_DEG)/PHI_STEP );

  long pos[4];
  pos[0] = numSteps1;
  pos[1] = numSteps2;
  pos[2] = numSteps3;
  pos[3] = numSteps4;
  steppers.moveTo(pos);
  steppers.run();
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
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DCMotorPins[i][0], OUTPUT);   
    pinMode(DCMotorPins[i][1], OUTPUT);
    pinMode(DCMotorPins[i][2], OUTPUT); 
  }

  // Set stepper velocity
  stepper1.setMaxSpeed(MAX_STEPPER_VEL);
  stepper2.setMaxSpeed(MAX_STEPPER_VEL);
  stepper3.setMaxSpeed(MAX_STEPPER_VEL);
  stepper4.setMaxSpeed(MAX_STEPPER_VEL);
  
  stepper1.setSpeed(STEPPER_VEL);
  stepper2.setSpeed(STEPPER_VEL);
  stepper3.setSpeed(STEPPER_VEL);
  stepper4.setSpeed(STEPPER_VEL);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);

  stepper1.setAcceleration(STEPPER_ACCEL);
  stepper2.setAcceleration(STEPPER_ACCEL);
  stepper3.setAcceleration(STEPPER_ACCEL);
  stepper4.setAcceleration(STEPPER_ACCEL);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  
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
