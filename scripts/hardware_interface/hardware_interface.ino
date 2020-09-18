#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"

#define RX        0  //For Arduino Mega
#define TX        1

//#define RxTx 3
#define Re    3
#define De    5

#define Transmit    HIGH
#define Receive     LOW

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
#define MAX_MILLIAMPS 3920                            // mA
#define MICRO_STEP_SIZE 1                             // 1 step = 1/MICRO_STEP_SIZE
#define DECAY_MODE StepperDecayMode::AutoMixed        // PWM decay mode (recommended default)
#define MAX_STEPPER_VEL 80                            // step/s
#define MIN_STEPPER_VEL 35                            // step/s
#define STEPS_THRESHOLD 25                            // step
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.295779513082320876798154814105

// Encoder constants
#define ENC_CPR 4000                                  // Counts Per Revolution

//##############encoder###############
long response = 0;
int byteOut;
uint8_t byteIn[3];
int i = 0;

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

// Step Motor pins: inner Y axis arm, outer Y axis arm, inner X axis arm, outer X axis arm
int StepperMotorPins[N_StepperMotors] = {37, 36, 10, 38};

// DC Motor pins [ [in1, in2, en], ... ] inner Y axis arm, outer Y axis arm, inner X axis arm, outer X axis arm
int DCMotorPins[N_DCMotors][3] = {{13, 14, 15}, {22, 21, 23}, {38, 37, 36}, {35, 34, 33}};

// Define steppers
Stepper stepper1(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);

int phi_des1 = 0;
int phi_des2 = 0;
int phi_des3 = 0;
int phi_des4 = 0;


/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
  for(int i = 0; i < N_DCMotors; i++) {
    if (msg.omega_arr.data[i] >= 0) {
      Forward_DCMotor(msg.omega_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
    }
    else if (msg.omega_arr.data[i] < 0) {
      //msg data for reverse comes as a negative value, this is changed to positive below
      msg.omega_arr.data[i] *= -1;
      Reverse_DCMotor(msg.omega_arr.data[i], DCMotorPins[i][0], DCMotorPins[i][1], DCMotorPins[i][2]);
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

// wrap encoder output to [-100, 99] steps = [-pi, pi] rads
int wrapToPi(float encoder_data) {
  // counts * (degs/count) * (step/deg) = steps
  int encoder_pos = int( (encoder_data)*(360.0/ENC_CPR)*(1.0/PHI_STEP) ) % int( 360.0/PHI_STEP );
  if (encoder_pos >= int( 180.0/PHI_STEP )) { encoder_pos = encoder_pos - int( 360.0/PHI_STEP ); }
  else if (encoder_pos < int( -180.0/PHI_STEP )) { encoder_pos = encoder_pos + int( 360.0/PHI_STEP ); }

  return encoder_pos;
}

int checkEncoder(int address) {

  byteOut = address;
  RS485Transmit();

  Serial1.write(byteOut);      // Send byte to encoder
  delay(10);
  Serial1.flush();
  RS485Receive();
  delay(25);
  i = 0;
  while (Serial1.available())       //Look for data from encoder
  {
    //Serial.println("Received");
    byteIn[i] = Serial1.read();     // Read received byte

    delay(10);
    i ++;
  }
  byteIn[2] = byteIn[2] << 2;
  byteIn[1] = byteIn[1] >> 2;
  byteIn[2] = byteIn[2] >> 2;
  response = byteIn[2];
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
 * ------------- SETUP INTERFACE ------------------
 */

void setup() {
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  hardware_interface.subscribe(controller_cmds_sub);

  // Setup stepper motors
  SPI.begin();
  stepper1.setupDriver(StepperMotorPins[0]);
  stepper2.setupDriver(StepperMotorPins[1]);
  stepper3.setupDriver(StepperMotorPins[2]);
  stepper4.setupDriver(StepperMotorPins[3]);

  // Setup DC motors
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DCMotorPins[i][0], OUTPUT);
    pinMode(DCMotorPins[i][1], OUTPUT);
    pinMode(DCMotorPins[i][2], OUTPUT);
  }

  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);
  Serial1.begin(115200);        // set the data rate

}

/*
 * ------------- MAIN ------------------
 */
void loop() {
  hardware_interface.spinOnce();

  // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
  stepper1.commandStepper(wrapToPi(checkEncoder(76)), phi_des1);
  stepper2.commandStepper(wrapToPi(checkEncoder(80)), phi_des2);
  stepper3.commandStepper(wrapToPi(checkEncoder(84)), phi_des3);
  stepper4.commandStepper(wrapToPi(checkEncoder(88)), phi_des4);

}
