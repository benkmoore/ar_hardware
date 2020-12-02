#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"
#include "src/ar_bldc.h"
#include "Wire.h"
#include "Adafruit_MCP4728.h"
#include "RF24.h"
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

// BLDC motor settings and constants
#define DCREVIVE 2000                                 // time in ms
#define KILLTIME 1000                                 // time in ms
#define NUM_PULSES_2PI 20                             // number of outer magnets in hub motor = number of pulses per revolution per hall sensor
#define MAX_PWM 255                                   // max duty cycle
#define MIN_PWM 0                                     // min duty cycle
#define MAX_OMEGA 102.86                              // max v=7.2 m/s; r=0.07 m; w = v/r = 102.86 (rad/s)

// Sensor and motor counts
const int N_DCMotors = 4; // number of DC motors in use
const int NUM_HALLS = 1; // Number of hall sensors in use
const int N_StepperMotors = 4; // number of stepper motors in use

struct package
{
  int kill = 0;
  float throttle = 0.0;
  float phi = 0.0;
};
typedef struct package Package;
Package rf_data;

int kill = 0;
// Step Motor pins: outer Y axis arm, inner Y axis arm, inner X axis arm, outer X axis arm
int StepperMotorPins[N_StepperMotors] = {10, 36, 37, 38};
bool phi_flag = false;
int unwindFlag = 0;

// DC Motor pins
int DC_throttle[N_DCMotors] = {1, 2, 3, 4}; //TODO update these
int DC_reverse[N_DCMotors] = {20, 21, 22, 23};
float controller_gains[3] = {1, 0, 0}; // P, I, D

// Setup BLDCs
int hall_pins_1[NUM_HALLS] = {1};
int hall_pins_2[NUM_HALLS] = {2};
int hall_pins_3[NUM_HALLS] = {3};
int hall_pins_4[NUM_HALLS] = {4};
BLDC bldc_1(DC_throttle[0], hall_pins_1, controller_gains, NUM_PULSES_2PI, MAX_OMEGA, MAX_PWM, MIN_PWM);
BLDC bldc_2(DC_throttle[1], hall_pins_2, controller_gains, NUM_PULSES_2PI, MAX_OMEGA, MAX_PWM, MIN_PWM);
BLDC bldc_3(DC_throttle[2], hall_pins_3, controller_gains, NUM_PULSES_2PI, MAX_OMEGA, MAX_PWM, MIN_PWM);
BLDC bldc_4(DC_throttle[3], hall_pins_4, controller_gains, NUM_PULSES_2PI, MAX_OMEGA, MAX_PWM, MIN_PWM);

// Define steppers
Stepper stepper1(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);

// Define encoders and PWM to analog board
AMTEncoder encoder(Re, De);

// Variables for controller callback
int phi_des1 = 25, phi_des2 = 25, phi_des3 = 25, phi_des4 = 25;
float omega_cmd[N_DCMotors] = {0, 0, 0, 0};
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
                           {0,0,200,200,0} }; // robot4

float VEL_ANALOG[4][4] = { {0.5,1.09,2380,2680},                // robot1
                           {0.25,0.98,2200,2700},                  // robot2
                           {0.31,1.08,2200,2700},                 // robot3
                           {0.42,1.05,2200,2600} }; // robot4

float min_vel  = VEL_ANALOG[ns_int][0];
float max_vel  = VEL_ANALOG[ns_int][1];
int min_pwm  = VEL_ANALOG[ns_int][2];
int max_pwm  = VEL_ANALOG[ns_int][3]; // 12 bit value (0 -> 4095) converted to analog voltage (0v -> 2.048v)


/*
   -------------------------- Controller commands to motor actuation --------------------------
*/

ros::NodeHandle_<ArduinoHardware, NUM_SUBS, NUM_PUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

float wheel_scales[4] = {VEL_SCALES[ns_int][1], VEL_SCALES[ns_int][2], VEL_SCALES[ns_int][3], VEL_SCALES[ns_int][4]};
float vel_scale = VEL_SCALES[ns_int][0];

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
// test.data = 1.0;
  phi_flag = (stepper1.phi_flag or stepper2.phi_flag or stepper3.phi_flag or stepper4.phi_flag);

  for (int i = 0; i < N_DCMotors; i++) { // saturate cmds
    float omega = msg.omega_arr.data[i];
	omega_cmd[i] = constrain(omega, 0, max_vel);
    if (phi_flag || kill != 0) { // write all BLDCs to 0 if phi flag or kill switch is on
      omega_cmd[i] = 0;
    }
  }
  commandBLDCS(omega_cmd);

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

void commandBLDCS(float omega_cmd[N_DCMotors]) {
  bldc_1.commandBLDC(omega_cmd[0]);
  bldc_2.commandBLDC(omega_cmd[1]);
  bldc_3.commandBLDC(omega_cmd[2]);
  bldc_4.commandBLDC(omega_cmd[3]);
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

  // Setup hall sensor interrupts
  pinMode(hall_pins_1[0], INPUT);
  pinMode(hall_pins_2[0], INPUT);
  pinMode(hall_pins_3[0], INPUT);
  pinMode(hall_pins_4[0], INPUT);
  attachInterrupt(digitalPinToInterrupt(hall_pins_1[0]), updatePulseTime1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall_pins_2[0]), updatePulseTime2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall_pins_3[0]), updatePulseTime3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall_pins_4[0]), updatePulseTime4, CHANGE);
}

// TODO: move away from arduino intterupts - require static member functions forcing this duplication
void updatePulseTime1() {
    bldc_1.hall_arr[0].updatePulseTime();
}
void updatePulseTime2() {
    bldc_2.hall_arr[0].updatePulseTime();
}
void updatePulseTime3() {
    bldc_3.hall_arr[0].updatePulseTime();
}
void updatePulseTime4() {
    bldc_4.hall_arr[0].updatePulseTime();
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
    //mcp.fastWrite(0,0,0,0);
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
    float omega_cmd[N_DCMotors] = {0, 0, 0, 0};
    commandBLDCS(omega_cmd);
    stepper1.commandStepper(enc76_wrap, 25);
    stepper2.commandStepper(enc80_wrap, 25);
    stepper3.commandStepper(enc84_wrap, 25);
    stepper4.commandStepper(enc88_wrap, 25);
  }
}
