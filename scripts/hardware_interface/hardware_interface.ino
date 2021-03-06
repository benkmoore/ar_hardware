#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"
#include "Wire.h"
#include "Adafruit_MCP4728.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int8.h"
#include "stdlib.h"

// ROS Serial constants
#define NUM_PUBS 3
#define NUM_SUBS 3
#define BAUD_RATE 57600                               // bits/s
#define IN_BUFFER_SIZE 512                            // bytes
#define OUT_BUFFER_SIZE 512                           // bytes
#define MAX_CALLBACK_TIME 1000                        // ms

// Stepper motor constants
#define MAX_MILLIAMPS 2800                            // mA
#define MICRO_STEP_SIZE 1                             // 1 step = 1/MICRO_STEP_SIZE
#define DECAY_MODE StepperDecayMode::AutoMixed        // PWM decay mode (recommended default)
#define MAX_STEPPER_VEL 200                           // step/s
#define MIN_STEPPER_VEL 60                            // step/s
#define STEPS_THRESHOLD 10                            // steps from target to start stepper decceleration
#define MAX_PHI_DELTA 10                              // steps
#define PHI_STEP 1.8                                  // deg/step
#define UNWIND_ON 2                                   // revolutions
#define UNWIND_OFF 0                                  // revolutions
#define RAD_2_DEG 57.2957795

// Encoder constants
#define ENC_CPR 4096                                  // Counts Per Revolution
#define ENCODERWAIT 50                                // wait time in ms to check encoder position
#define Re    3                                       // serial data read/write enable pins
#define De    4

#define DCREVIVE 2000                                 // time in ms
#define KILLTIME 1000                                 // time in ms

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4, N_StepperMotors = 4;

int kill = 0;
// Step Motor pins: outer Y axis arm, inner Y axis arm, inner X axis arm, outer X axis arm
int StepperMotorPins[N_StepperMotors] = {10, 36, 37, 38};
bool phi_flag = false;
int unwindFlag = 0;

// DC Motor pins
int DC_reverse[N_DCMotors] = {20, 21, 22, 23};

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
int killTime;

float encoder76 = encoder.checkEncoder(76);
float encoder80 = encoder.checkEncoder(80);
float encoder84 = encoder.checkEncoder(84);
float encoder88 = encoder.checkEncoder(88);
float encTime = millis();
float dcTime = millis();
std_msgs::Float64 test;

//ros::Publisher chatter_pub("chatter", &test);

// 0 column = vel scale on robot, 1-4 column = vel scale on wheels
int ns_int = 3; // robot1 = 0, ... robot4 = 3 

float VEL_SCALES[4][5] = { {180,0,0,0,0},                // robot1
                           {0,190,200,-40,130},                  // robot2
                           {0,220,210,15,-80},                 // robot3
                           {0,0,180,180,0} }; // robot4

float VEL_ANALOG[4][4] = { {0.3,1.09,2250,2700},                // robot1
                           {0.25,0.98,2200,2700},                  // robot2
                           {0.3,1.08,2290,2700},                 // robot3
                           {0.3,1.05,2150,2600} }; // robot4


/*
   -------------------------- Controller commands to motor actuation --------------------------
*/

float min_vel  = VEL_ANALOG[ns_int][0];
float max_vel  = VEL_ANALOG[ns_int][1];
int min_pwm  = VEL_ANALOG[ns_int][2];
int max_pwm  = VEL_ANALOG[ns_int][3]; // 12 bit value (0 -> 4095) converted to analog voltage (0v -> 2.048v)


ros::NodeHandle_<ArduinoHardware, NUM_SUBS, NUM_PUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

float wheel_scales[4] = {VEL_SCALES[ns_int][1], VEL_SCALES[ns_int][2], VEL_SCALES[ns_int][3], VEL_SCALES[ns_int][4]};
float vel_scale = VEL_SCALES[ns_int][0];

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
// test.data = 1.0;

  for (int i = 0; i < N_DCMotors; i++) {
    float omega = msg.omega_arr.data[i];
	  msg.omega_arr.data[i] = constrain(omega, 0, max_vel);
    if (msg.omega_arr.data[i] >= min_vel && kill == 0) {
      pwmVal[i] =  map(msg.omega_arr.data[i], min_vel, max_vel, min_pwm, max_pwm);
    } else {
      pwmVal[i] = 0;
    }
  }

  phi_flag = (stepper1.phi_flag or stepper2.phi_flag or stepper3.phi_flag or stepper4.phi_flag);
  if (phi_flag && pwmVal[0] != 0) {
      mcp.fastWrite(0,0,0,0);
  }
  else {
      mcp.fastWrite(pwmVal[0]+wheel_scales[0], pwmVal[1]+wheel_scales[1], pwmVal[2]+wheel_scales[2], pwmVal[3]+wheel_scales[3]);
  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  phi_des1 = (int) ((msg.phi_arr.data[0] * RAD_2_DEG) / PHI_STEP);
  phi_des2 = (int) ((msg.phi_arr.data[1] * RAD_2_DEG) / PHI_STEP);
  phi_des3 = (int) ((msg.phi_arr.data[2] * RAD_2_DEG) / PHI_STEP);
  phi_des4 = (int) ((msg.phi_arr.data[3] * RAD_2_DEG) / PHI_STEP);

  callbackTime = millis();
  //chatter_pub.publish(&test);
}

void modeCallback(const std_msgs::Int8 &msg) {

  if (msg.data == 1) { // if in idle mode and wheels are wrapped turn unwind on
    if ((abs(stepper1.revolutions) >= UNWIND_ON or abs(stepper2.revolutions) >= UNWIND_ON or
        abs(stepper3.revolutions) >= UNWIND_ON or abs(stepper4.revolutions) >= UNWIND_ON)) {
      unwindFlag = 1;
    }
    if (abs(stepper1.revolutions) == UNWIND_OFF and abs(stepper2.revolutions) == UNWIND_OFF and
        abs(stepper3.revolutions) == UNWIND_OFF and abs(stepper4.revolutions) == UNWIND_OFF) {
      unwindFlag = 0;
    }
  } else { // turn off in trajectory mode (any other mode)
      unwindFlag = 0;
  }
  //test.data = stepper4.revolutions;
  //chatter_pub.publish(&test);
}

void killCallback(const std_msgs::Int8 &msg){
  kill = msg.data;
  killTime = millis();
}

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds", controllerCmdCallback);
ros::Subscriber<std_msgs::Int8> mode_sub("state_machine/mode", modeCallback);
ros::Subscriber<std_msgs::Int8> kill_sub("/kill_switch", killCallback);


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
  hardware_interface.subscribe(kill_sub);
  hardware_interface.subscribe(mode_sub);

  //hardware_interface.advertise(chatter_pub);

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
  //chatter_pub.publish(&test);
  //test.data = stepper4.readStatus();

  
  if(millis() - killTime > KILLTIME){ //if kill cmd not received within time window, unkill
    kill = 0;
  }

  if (millis() - encTime > ENCODERWAIT){
    encoder76 = encoder.checkEncoder(76);
    encoder80 = encoder.checkEncoder(80);
    encoder84 = encoder.checkEncoder(84);
    encoder88 = encoder.checkEncoder(88);
    encTime = millis();
  }
  if(millis() - dcTime > DCREVIVE){
    mcp.fastWrite(0,0,0,0);
    dcTime = millis();
  }

  int enc76_wrap = wrapToSteps(encoder76);
  int enc80_wrap = wrapToSteps(encoder80);
  int enc84_wrap = wrapToSteps(encoder84);
  int enc88_wrap = wrapToSteps(encoder88);


  if ((kill == 0) and (millis()-callbackTime < MAX_CALLBACK_TIME) and unwindFlag == 0) { // actuate robot if callback is within time window, kill switch is off and not unwinding
    // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
    stepper1.commandStepper(enc76_wrap, phi_des1);
    stepper2.commandStepper(enc80_wrap, phi_des2);
    stepper3.commandStepper(enc84_wrap, phi_des3);
    stepper4.commandStepper(enc88_wrap, phi_des4);
  } else if (unwindFlag == 1) { // if unwind flag on unwind steppers
    stepper1.unwind(enc76_wrap);
    stepper2.unwind(enc80_wrap);
    stepper3.unwind(enc84_wrap);
    stepper4.unwind(enc88_wrap);
  } else { // shutdown robot if kill switch is on or no cmds recieved within last time window
    mcp.fastWrite(0,0,0,0);
    stepper1.commandStepper(enc76_wrap, 25);
    stepper2.commandStepper(enc80_wrap, 25);
    stepper3.commandStepper(enc84_wrap, 25);
    stepper4.commandStepper(enc88_wrap, 25);
  }
}
