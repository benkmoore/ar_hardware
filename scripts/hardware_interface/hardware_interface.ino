#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"
#include "Wire.h"
#include "Adafruit_MCP4728.h"
#include "RF24.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int8.h"

// ROS Serial constants
#define NUM_PUBS 2
#define NUM_SUBS 2
#define BAUD_RATE 57600                               // bits/s
#define IN_BUFFER_SIZE 512                            // bytes
#define OUT_BUFFER_SIZE 512                           // bytes
#define MAX_CALLBACK_TIME 1000                        // ms

// DC motor velocity map
#define MAX_PWM 3200                                  // 12 bit value (0 -> 4095) converted to analog voltage (0v -> 2.048v)
#define MIN_PWM 2115                                  // 12 bit value converted to analog voltage
#define MAX_VEL 2.02                                   // m/s
#define MIN_VEL 0.25                                  // m/s

// Stepper motor constants
#define MAX_MILLIAMPS 2800                            // mA
#define MICRO_STEP_SIZE 1                             // 1 step = 1/MICRO_STEP_SIZE
#define DECAY_MODE StepperDecayMode::AutoMixed        // PWM decay mode (recommended default)
#define MAX_STEPPER_VEL 200                           // step/s
#define MIN_STEPPER_VEL 40                            // step/s
#define STEPS_THRESHOLD 10                            // steps from target to start stepper decceleration
#define MAX_PHI_DELTA 10                              // steps
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.2957795

// Encoder constants
#define ENC_CPR 4096                                  // Counts Per Revolution
#define ENCODERWAIT 50
#define Re    3                                       // serial data read/write enable pins
#define De    4

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4, N_StepperMotors = 4;

// Define rf radio
RF24 rf_Coms (5, 6);
struct package
{
  int kill = 0;
  float throttle = 0.0;
  float phi = 0.0;
};
byte addresses[][6] = {"3"};
typedef struct package Package;
Package rf_data;

// Step Motor pins: outer Y axis arm, inner Y axis arm, inner X axis arm, outer X axis arm
int StepperMotorPins[N_StepperMotors] = {10, 36, 37, 38};
bool phi_flag = false;
int unwindFlag = 0;
// DC Motor pins
int DC_reverse[N_DCMotors] = {20, 21, 22, 23};
float VEL_SCALING = 0.94;

// Define steppers
Stepper stepper1(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);

// Define encoders and PWM to analog board
AMTEncoder encoder(Re, De);
Adafruit_MCP4728 mcp;

// Variables for controller callback
int phi_des1 = 25, phi_des2 = 25, phi_des3 = 25, phi_des4 = 25;
int pwmVal[N_DCMotors] = {0,0,0,0};
int callbackTime;

float encoder76 = encoder.checkEncoder(76);
float encoder80 = encoder.checkEncoder(80);
float encoder84 = encoder.checkEncoder(84);
float encoder88 = encoder.checkEncoder(88);
float encTime = millis();
std_msgs::Float64 test;
ros::Publisher chatter_pub("chatter", &test);

/*
   -------------------------- Controller commands to motor actuation --------------------------
*/

ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
// test.data = 1.0;

  for (int i = 0; i < N_DCMotors; i++) {
    float omega = msg.omega_arr.data[i];
	  msg.omega_arr.data[i] = constrain(omega, 0, MAX_VEL);
    if (msg.omega_arr.data[i] >= MIN_VEL && rf_data.kill == 0) {
      pwmVal[i] =  map(VEL_SCALING*msg.omega_arr.data[i], MIN_VEL, MAX_VEL, MIN_PWM, MAX_PWM);
    } else {
      pwmVal[i] = 0;
    }
  }

  phi_flag = (stepper1.phi_flag or stepper2.phi_flag or stepper3.phi_flag or stepper4.phi_flag);
  if (phi_flag && pwmVal[0] != 0) {
      mcp.fastWrite(0,0,0,0);
  }
  else {
      mcp.fastWrite(pwmVal[0], pwmVal[1], pwmVal[2], pwmVal[3]);
  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  phi_des1 = (int) ((msg.phi_arr.data[0] * RAD_2_DEG) / PHI_STEP);
  phi_des2 = (int) ((msg.phi_arr.data[1] * RAD_2_DEG) / PHI_STEP);
  phi_des3 = (int) ((msg.phi_arr.data[2] * RAD_2_DEG) / PHI_STEP);
  phi_des4 = (int) ((msg.phi_arr.data[3] * RAD_2_DEG) / PHI_STEP);

  callbackTime = millis();
  if (millis() - encTime > ENCODERWAIT){
    encoder76 = encoder.checkEncoder(76);
    encoder80 = encoder.checkEncoder(80);
    encoder84 = encoder.checkEncoder(84);
    encoder88 = encoder.checkEncoder(88);
    encTime = millis();
  }
  chatter_pub.publish(&test);
}

void modeCallback(std_msgs::Int8 &msg) {
  if (msg.data == 1){
    stepper1.unwind();
    stepper2.unwind();
    stepper3.unwind();
    stepper4.unwind();
    if (stepper1.totalSteps != 0 or stepper2.totalSteps != 0 or stepper3.totalSteps != 0 stepper4.totalSteps != 0){
      unwindFlag = 1;
    }
    else
    {
      unwindFlag = 0;
    }
    
  } 
}


ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds", controllerCmdCallback);
ros::Subscriber<std_msgs::Int8> mode_sub("state_machine/mode", modeCallback);

/*
   -------------------------- Support function --------------------------
*/

// wrap encoder output to [-100, 99] steps = [-pi, pi] rads
int wrapToSteps(float encoder_data) {
  int encoder_pos = round((encoder_data) * (360.0 / ENC_CPR) * (1.0 / PHI_STEP)) ;
  if (encoder_pos >= 100) {
    encoder_pos = encoder_pos - int( 360.0 / PHI_STEP );
  }
  return encoder_pos;
}

/*
   -------------------------- Setup Interface --------------------------
*/

void setup() {
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  hardware_interface.subscribe(controller_cmds_sub);
  hardware_interface.subscribe(mode_sub);

  hardware_interface.advertise(chatter_pub);

  // Setup analog board to use 2.048v as vref
  mcp.begin();
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
  mcp.saveToEEPROM();

  // Setup stepper motors
  SPI.begin();
  stepper1.setupDriver(StepperMotorPins[0]);
  stepper2.setupDriver(StepperMotorPins[1]);
  stepper3.setupDriver(StepperMotorPins[2]);
  stepper4.setupDriver(StepperMotorPins[3]);

  // Setup encoder serial pins
  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);

  // Setup rf communication for kill switch
  rf_Coms.begin();
  rf_Coms.setChannel(115);
  rf_Coms.setPALevel(RF24_PA_MAX);
  rf_Coms.setDataRate( RF24_250KBPS ) ;
  rf_Coms.openReadingPipe(1, addresses[0]);
  rf_Coms.startListening();

  encoder76 = encoder.checkEncoder(76);
  encoder80 = encoder.checkEncoder(80);
  encoder84 = encoder.checkEncoder(84);
  encoder88 = encoder.checkEncoder(88);
}

/*
   -------------------------- Main Loop --------------------------
*/
void loop() {
  hardware_interface.spinOnce();
  chatter_pub.publish(&test);
  if (rf_Coms.available()) {
    rf_Coms.read( &rf_data, sizeof(rf_data) );
//rf_data.kill = 0;	
  }
test.data = rf_data.kill;
  int enc76_wrap = wrapToSteps(encoder76);
  int enc80_wrap = wrapToSteps(encoder80);
  int enc84_wrap = wrapToSteps(encoder84);
  int enc88_wrap = wrapToSteps(encoder88);

  //test.data = stepper4.readStatus();

  if ((rf_data.kill == 0) and (millis()-callbackTime < MAX_CALLBACK_TIME) and unwindFlag == 0) {
    // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
    stepper1.commandStepper(enc76_wrap, phi_des1);
    stepper2.commandStepper(enc80_wrap, phi_des2);
    stepper3.commandStepper(enc84_wrap, phi_des3);
    stepper4.commandStepper(enc88_wrap, phi_des4);
  } else if (unwindFlag == 0){
    // shutdown robot if kill switch is on or no cmds recieved within last time window
    mcp.fastWrite(0,0,0,0);
    stepper1.commandStepper(enc76_wrap, 25);
    stepper2.commandStepper(enc80_wrap, 25);
    stepper3.commandStepper(enc84_wrap, 25);
    stepper4.commandStepper(enc88_wrap, 25);
    encoder76 = encoder.checkEncoder(76);
    encoder80 = encoder.checkEncoder(80);
    encoder84 = encoder.checkEncoder(84);
    encoder88 = encoder.checkEncoder(88);
  }
}
