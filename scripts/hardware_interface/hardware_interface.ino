#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"
#include "src/ar_dc.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "Wire.h"
#include "VL53L1X.h"
#include "ar_commander/TOF.h"

#define RX        7
#define TX        8

//#define RxTx
#define Re    3
#define De    4

#define MAX_PWM 255                                   // pwm
#define MIN_PWM -255
#define MAX_OMEGA 255
#define MIN_OMEGA -255
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
#define MAX_MILLIAMPS 3920                            // mA
#define MICRO_STEP_SIZE 1                             // 1 step = 1/MICRO_STEP_SIZE
#define DECAY_MODE StepperDecayMode::AutoMixed        // PWM decay mode (recommended default)
#define MAX_STEPPER_VEL 80                            // step/s
#define MIN_STEPPER_VEL 35                            // step/s
#define STEPS_THRESHOLD 25                            // step
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.2957795

// TOF constants
#define MEASUREMENT_TIME_MS 50                        // ms
#define MEASUREMENT_TIME_US 50000                     // us
#define I2C_HZ 400000                                 // 400 kHz I2C
#define TIMEOUT 100

// Encoder constants
#define ENC_CPR 4096                                  // Counts Per Revolution

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

// Step Motor pins: outer Y axis arm, inner Y axis arm, inner X axis arm, outer X axis arm
int StepperMotorPins[N_StepperMotors] = {10, 36, 37, 38};

// DC Motor pins
int DC_reverse[N_DCMotors] = {20, 21, 22, 23};
// analog pins A0 to A3 correspond to pins 14, 15 and 18, 19 on the teensy
int DC_throttlePins[N_DCMotors] = {A0, A1, A4, A5};

int reverseFlags[N_DCMotors] = {0, 0, 0, 0};
int flip[N_DCMotors] = {0, 0, 0, 0};


// Define steppers
Stepper stepper1(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);

// TOF sensors
VL53L1X tof1;
VL53L1X tof2;
VL53L1X tof3;

// TOF pin outs
int tofOut1 = 39;
int tofOut2 = 40;
int tofOut3 = 41;

// TOF I2C addresses
uint8_t tofAddress1 = 0x33;
uint8_t tofAddress2 = 0x35;
uint8_t tofAddress3 = 0x37;

DC_Motors DC_motors(reverseFlags, DC_reverse, DC_throttlePins, N_DCMotors, flip);
AMTEncoder encoder(Re, De);

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


ar_commander::TOF tof_msg;
ros::Publisher tof_publisher("tof_data", &tof_msg);

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
  for (int i = 0; i < N_DCMotors; i++) {
    //int omega =  map(msg.omega_arr.data[i],MIN_OMEGA,MAX_OMEGA, MIN_PWM, MAX_PWM);
    DC_motors.PowerDC(DC_throttlePins[i], msg.omega_arr.data[i], i);
  }

  if (DC_motors.flipFlag == 1){
      DC_motors.flipDirection();
  }


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

// period = 0.001 //seconds
//rate = 100 //Hz

void pubCallback(const ar_commander::TOF &tof_msg){
  tof_publisher.publish(&tof_msg);

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
  hardware_interface.advertise(tof_publisher);
  //hardware_interface.advertise(chatter);

  // Setup TOF sensors
  pinMode(tofOut1, OUTPUT);
  pinMode(tofOut2, OUTPUT);
  pinMode(tofOut3, OUTPUT);
  digitalWrite(tofOut1, LOW);
  digitalWrite(tofOut2, LOW);
  digitalWrite(tofOut3, LOW);

  Wire.begin();
  Wire.setClock(I2C_HZ);

  digitalWrite(tofOut1, HIGH);
  tof1.init();
  tof1.setAddress(tofAddress1);

  digitalWrite(tofOut2, HIGH);
  tof2.init();
  tof2.setAddress(tofAddress2);

  digitalWrite(tofOut3, HIGH);
  tof3.init();
  tof3.setAddress(tofAddress3);

  tof1.setDistanceMode(VL53L1X::Long);
  tof1.setMeasurementTimingBudget(MEASUREMENT_TIME_US);
  tof1.startContinuous(MEASUREMENT_TIME_MS);
  tof1.setTimeout(TIMEOUT);

  tof2.setDistanceMode(VL53L1X::Long);
  tof2.setMeasurementTimingBudget(MEASUREMENT_TIME_US);
  tof2.startContinuous(MEASUREMENT_TIME_MS);
  tof2.setTimeout(TIMEOUT);

  tof3.setDistanceMode(VL53L1X::Long);
  tof3.setMeasurementTimingBudget(MEASUREMENT_TIME_US);
  tof3.startContinuous(MEASUREMENT_TIME_MS);
  tof3.setTimeout(TIMEOUT);


  // Setup stepper motors
  SPI.begin();
  stepper1.setupDriver(StepperMotorPins[0]);
  stepper2.setupDriver(StepperMotorPins[1]);
  stepper3.setupDriver(StepperMotorPins[2]);
  stepper4.setupDriver(StepperMotorPins[3]);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  // Setup DC reverse pins
  for (int i = 0; i < N_DCMotors; i++) {
    pinMode(DC_reverse[i], OUTPUT);
    digitalWrite(DC_reverse[i], HIGH);
  }
  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);      
}

/*
   ------------- MAIN ------------------
*/
void loop() {


  tof_msg.tof1 = tof1.readRangeContinuousMillimeters();
  tof_msg.tof2 = tof2.readRangeContinuousMillimeters();
  tof_msg.tof3 = tof3.readRangeContinuousMillimeters();
  // tof_publisher.publish(&tof_msg);


  
  hardware_interface.spinOnce();
  // int out_76 = wrapToPi(encoder.checkEncoder(76));
  // test.data = out_76;
  // chatter_pub.publish(&test);

  // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
  stepper1.commandStepper(wrapToPi(encoder.checkEncoder(76)), phi_des1);
  stepper2.commandStepper(wrapToPi(encoder.checkEncoder(80)), phi_des2);
  stepper3.commandStepper(wrapToPi(encoder.checkEncoder(84)), phi_des3);
  stepper4.commandStepper(wrapToPi(encoder.checkEncoder(88)), phi_des4);
}
