#define USE_USBCON
#include <ros.h>
#include <ar_commander/ControllerCmd.h>
#include <Encoder.h>
#include "src/ar_stepper.h"
#include <std_msgs/Float64.h>
//#include <ros/console.h>
#define RX        7 
#define TX        8

//#define RxTx
#define Re    3
#define De    4


/*
   ------------- FILE DEFINITION & SETUP ------------------
*/

// ROS Serial constants
#define NUM_PUBS 2
#define NUM_SUBS 1
#define BAUD_RATE 57600                               // bits/s
#define IN_BUFFER_SIZE 512                            // bytes
#define OUT_BUFFER_SIZE 512                           // bytes

// Stepper motor constants
#define MAX_STEPPER_VEL 80                            // step/s
#define MIN_STEPPER_VEL 35                            // step/s
#define STEPS_THRESHOLD 25                            // step
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.2957795

// Encoder constants
#define ENC_CPR 4096                                  // Counts Per Revolution

//##############encoder###############
long response = 0;
int byteOut;
uint8_t byteIn[3];
int i = 0;

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

// Step Motor pins: outer Y axis arm, inner Y axis arm, inner X axis arm, outer X axis arm
int StepperMotorPins[N_StepperMotors] = {37, 38, 10, 36};

// DC Motor pins 
//int DCMotorPins[N_DCMotors][3] = {{13, 14, 15}, {22, 21, 23}, {38, 37, 36}, {35, 34, 33}};

// Motor interface type for stppers
byte motorInterfaceType = 1;

// Define steppers
Stepper stepper1(int(360.0 / PHI_STEP), StepperPins[0][0], StepperPins[0][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);
Stepper stepper2(int(360.0 / PHI_STEP), StepperPins[1][0], StepperPins[1][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);
Stepper stepper3(int(360.0 / PHI_STEP), StepperPins[2][0], StepperPins[2][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);
Stepper stepper4(int(360.0 / PHI_STEP), StepperPins[3][0], StepperPins[3][1], PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL);

int phi_des1 = 0;
int phi_des2 = 0;
int phi_des3 = 0;
int phi_des4 = 0;


std_msgs::Float64 test;
ros::Publisher chatter_pub("chatter", &test);

/*
   ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
*/

// ROS node
ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;
//
// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
//  for(int i = 0; i < N_DCMotors; i++) {
//    if (msg.omega_arr.data[i] >= 0) {
//      Forward_DCMotor(msg.omega_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
//    }
//    else if (msg.omega_arr.data[i] < 0) {
//      //msg data for reverse comes as a negative value, this is changed to positive below
//      msg.omega_arr.data[i] *= -1;
//      Reverse_DCMotor(msg.omega_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
//    }
//  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  phi_des1 = (int) ( (-msg.phi_arr.data[0] * RAD_2_DEG) / PHI_STEP );
  phi_des2 = (int) ( (-msg.phi_arr.data[1] * RAD_2_DEG) / PHI_STEP );
  phi_des3 = (int) ( (-msg.phi_arr.data[2] * RAD_2_DEG) / PHI_STEP );
  phi_des4 = (int) ( (-msg.phi_arr.data[3] * RAD_2_DEG) / PHI_STEP );

}

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds", controllerCmdCallback);


/*
   ------------- SUPPORT FUNCTIONS ------------------
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

// wrap encoder output to [-100, 99] steps = [-pi, pi] rads
int wrapToPi(float encoder_data) {
  // counts * (degs/count) * (step/deg) = steps
  int encoder_pos = int( (encoder_data) * (360.0 / ENC_CPR) * (1.0 / PHI_STEP) ) % int( 360.0 / PHI_STEP );
  if (encoder_pos >= int( 180.0 / PHI_STEP )) {
    encoder_pos = encoder_pos - int( 360.0 / PHI_STEP );
  }
  else if (encoder_pos < int( -180.0 / PHI_STEP )) {
    encoder_pos = encoder_pos + int( 360.0 / PHI_STEP );
  }

  return encoder_pos;
}


int checkEncoder(int address) {
  //when the encoder  sees its address being transmitted it returns its position
  byteOut = address;
  RS485Transmit();

  // Send byte to encoder
  Serial2.write(byteOut);           
  delay(1);
  Serial2.flush();
  RS485Receive();
  i = 0;
  // Look for data from encoder
  while (Serial2.available())       
  {
    byteIn[i] = Serial2.read();     
    i ++;
  }
  //byte 2 is the most significant (MS) byte, with the biggest two being the checkbits
  byteIn[2] = byteIn[2] << 2;
  byteIn[2] = byteIn[2] >> 2;
  //byte 1 the two least significant (LS) bits need to be removed as our encoder is only 12 bit
  byteIn[1] = byteIn[1] >> 2;

  response = byteIn[2];
  //encoder position data comes as a 12 bit value, we need the last 6 MSB's of the LS byte and the 6 LSB's of the MS byte
  response = (response << 6) + byteIn[1];

  return response;
}

void RS485Transmit()
{
  digitalWrite(Re, LOW);
  digitalWrite(De, HIGH);
}

void RS485Receive()
{
  digitalWrite(Re, HIGH);
  digitalWrite(De, LOW);
}

/*
   ------------- SETUP INTERFACE ------------------
*/

void setup() {
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  hardware_interface.advertise(chatter_pub);
  hardware_interface.subscribe(controller_cmds_sub);

  // Setup stepper motors 
  SPI.begin();
  stepper1.setupDriver(StepperMotorPins[0]);
  stepper2.setupDriver(StepperMotorPins[1]);
  stepper3.setupDriver(StepperMotorPins[2]);
  stepper4.setupDriver(StepperMotorPins[3]);
  
  // Setup DC motors
//  for(int i = 0; i < N_DCMotors; i++) {
//    pinMode(DCMotorPins[i][0], OUTPUT);
//    pinMode(DCMotorPins[i][1], OUTPUT);
//    pinMode(DCMotorPins[i][2], OUTPUT);
//  }

  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);

  RS485Receive();
  Serial2.begin(115200);        // set the data rate

}

/*
   ------------- MAIN ------------------
*/


void loop() {
  hardware_interface.spinOnce();
  int out_88 = wrapToPi(checkEncoder(88));
  int out_84 = wrapToPi(checkEncoder(84));
  int out_80 = wrapToPi(checkEncoder(80));
  int out_76 = wrapToPi(checkEncoder(76));
  test.data = out_76;
  chatter_pub.publish(&test);
  // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
  stepper1.commandStepper(out_76, phi_des1);
  stepper2.commandStepper(out_80, phi_des2);
  stepper3.commandStepper(out_84, phi_des3);
  stepper4.commandStepper(out_88, phi_des4);
}
