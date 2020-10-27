#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"
#include "Wire.h"
#include <Adafruit_MCP4728.h>
#include <string>
#include "RF24.h"

#include <std_msgs/Float64.h>
#define RX        7
#define TX        8

//#define RxTx
#define Re    3
#define De    4

#define MAX_PWM 1650                                   // pwm
#define MIN_PWM 1495
#define MAX_VEL 1.0                                     // m/s
#define MIN_VEL 0.15
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
#define MAX_STEPPER_VEL 200                            // step/s
#define MIN_STEPPER_VEL 40                            // step/s
#define STEPS_THRESHOLD 10                            // steps
#define MAX_PHI_DELTA 10                              // steps
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.2957795
// Encoder constants
#define ENC_CPR 4096                                  // Counts Per Revolution
#define DEADBAND 2
// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

RF24 myRadio (5, 6);
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

// DC Motor pins
int DC_reverse[N_DCMotors] = {20, 21, 22, 23};
// analog pins A0 to A3 correspond to pins 14, 15 and 28, 29 on the teensy
//int DC_throttlePins[N_DCMotors] = {A0, A1, 28, 29};

int DC_reverseFlags[N_DCMotors] = {0, 0, 0, 0};
int flip[N_DCMotors] = {0, 0, 0, 0};
int flipFlag = 0;
//std::string channel[4] = {"MCP4728_CHANNEL_A", "MCP4728_CHANNEL_B", "MCP4728_CHANNEL_C", "MCP4728_CHANNEL_D"};

// Define steppers
Stepper stepper1(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0 / PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_PHI_DELTA, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);

//DC_Motors DC_motors(DC_reverseFlags, DC_reverse, DC_throttlePins, N_DCMotors, flip);
AMTEncoder encoder(Re, De);
Adafruit_MCP4728 mcp;

int phi_des1 = 0;
int phi_des2 = 0;
int phi_des3 = 0;
int phi_des4 = 0;
int pwmVal[N_DCMotors] = {0,0,0,0};
int callbackTime;

std_msgs::Float64 test;
// ros::Publisher chatter_pub("chatter", &test);

/*
   ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
*/

// ROS node
ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
  for (int i = 0; i < N_DCMotors; i++) {
    float omega = msg.omega_arr.data[i];
	  msg.omega_arr.data[i] = constrain(omega, -1*MAX_VEL, MAX_VEL);
    if (msg.omega_arr.data[i] >= MIN_VEL && rf_data.kill == 0) {
      pwmVal[i] =  map(msg.omega_arr.data[i], MIN_VEL, MAX_VEL, MIN_PWM, MAX_PWM);
      if (DC_reverseFlags[i] != 0) {
        flip[i] = 1;
        flipFlag = 1;
      }
    }

    else if (msg.omega_arr.data[i] <= -1 * MIN_VEL && rf_data.kill == 0) {
       pwmVal[i] = map(msg.omega_arr.data[i], -1 * MAX_VEL, -1 * MIN_VEL, -1 * MAX_PWM, -1 * MIN_PWM);
      if (DC_reverseFlags[i] != 1) {
        flip[i] = 1;
        flipFlag = 1;
      }
    }
    else{
      pwmVal[i] = 0;
    }
  }

  if (flipFlag == 1) {
    for (int i = 0; i < N_DCMotors; i++) {
      	if (flip[i] == 1) {
        	digitalWrite(DC_reverse[i], LOW);
        	flip[i] = 0;
        	DC_reverseFlags[i] = !DC_reverseFlags[i];
	}
    }
    flipFlag = 0;
    delay(200);
    for (int i = 0; i < N_DCMotors; i++) {
      digitalWrite(DC_reverse[i], HIGH);
    }
  }
phi_flag = (stepper1.phi_flag or stepper2.phi_flag or stepper3.phi_flag or stepper4.phi_flag);
    if (phi_flag && pwmVal[0] != 0) {
          //mcp.fastWrite(max(abs(pwmVal[0]/3),MIN_PWM), max(abs(pwmVal[1]/3),MIN_PWM), max(abs(pwmVal[2]/3),MIN_PWM), max(abs(pwmVal[3]/3),MIN_PWM));
  mcp.fastWrite(0,0,0,0);

    }
else{
  mcp.fastWrite(abs(pwmVal[0]), abs(pwmVal[1]), abs(pwmVal[2]), abs(pwmVal[3]));
}

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  phi_des1 = (int) ( (msg.phi_arr.data[0] * RAD_2_DEG) / PHI_STEP );
  phi_des2 = (int) ( (msg.phi_arr.data[1] * RAD_2_DEG) / PHI_STEP );
  phi_des3 = (int) ( (msg.phi_arr.data[2] * RAD_2_DEG) / PHI_STEP );
   phi_des4 = (int) ( (msg.phi_arr.data[3] * RAD_2_DEG) / PHI_STEP );

   callbackTime = millis();
 }

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds", controllerCmdCallback);


/*
   ------------- SUPPORT FUNCTIONS ------------------
*/

// wrap encoder output to [-100, 99] steps = [-pi, pi] rads
int wrapToPi(float encoder_data) {
  int encoder_pos = int( (encoder_data) * (360.0 / ENC_CPR) * (1.0 / PHI_STEP) ) % int( 360.0 / PHI_STEP );
  if (encoder_pos >= int( 180.0 / PHI_STEP )) {
    encoder_pos = encoder_pos - int( 360.0 / PHI_STEP );
  }
  else if (encoder_pos < int( -180.0 / PHI_STEP )) {
    encoder_pos = encoder_pos + int( 360.0 / PHI_STEP );
  }

  return encoder_pos;
}


/*
   ------------- SETUP INTERFACE ------------------
*/

void setup() {
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  // hardware_interface.advertise(chatter_pub);
  hardware_interface.subscribe(controller_cmds_sub);
  mcp.begin();
  mcp.fastWrite(0, 0, 0, 0);
  mcp.saveToEEPROM();

  // Setup stepper motors
  SPI.begin();
  stepper1.setupDriver(StepperMotorPins[0]);
  stepper2.setupDriver(StepperMotorPins[1]);
  stepper3.setupDriver(StepperMotorPins[2]);
  stepper4.setupDriver(StepperMotorPins[3]);

  // Setup DC DC_reverse pins
    for (int i = 0; i < N_DCMotors; i++) {
      pinMode(DC_reverse[i], OUTPUT);
      digitalWrite(DC_reverse[i], HIGH);
    }
  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);
  //  Serial.begin(57600);

  //rf communication
  myRadio.begin();
  myRadio.setChannel(115);
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS ) ;
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}

/*
   ------------- MAIN ------------------
*/
void loop() {
  hardware_interface.spinOnce();
  // chatter_pub.publish(&test);

  if (myRadio.available()){
    while (myRadio.available())
    {
      myRadio.read( &rf_data, sizeof(rf_data) );
    }
  }

  // Feedback encoder data & wrap to [-pi, pi] = [-100, 99] steps
  if (rf_data.kill == 0){
     if ((millis()-callbackTime<1000)) {
        stepper1.commandStepper(wrapToPi(encoder.checkEncoder(76)), rf_data.phi);
        stepper2.commandStepper(wrapToPi(encoder.checkEncoder(80)), rf_data.phi);
        stepper3.commandStepper(wrapToPi(encoder.checkEncoder(84)), rf_data.phi);
        stepper4.commandStepper(wrapToPi(encoder.checkEncoder(88)), rf_data.phi);
    }
   
    // shutdown robot if no cmds recieved within last time window
    if ((millis()-callbackTime>1000)) {
        mcp.fastWrite(0, 0, 0, 0);
	stepper1.commandStepper(wrapToPi(encoder.checkEncoder(76)), 0);
    	stepper2.commandStepper(wrapToPi(encoder.checkEncoder(80)), 0);
    	stepper3.commandStepper(wrapToPi(encoder.checkEncoder(84)), 0);
    	stepper4.commandStepper(wrapToPi(encoder.checkEncoder(88)), 0);
    }
  }
  //if kill switch is on
  else{
    mcp.fastWrite(0,0,0,0);
    stepper1.commandStepper(wrapToPi(encoder.checkEncoder(76)), 0);
    stepper2.commandStepper(wrapToPi(encoder.checkEncoder(80)), 0);
    stepper3.commandStepper(wrapToPi(encoder.checkEncoder(84)), 0);
    stepper4.commandStepper(wrapToPi(encoder.checkEncoder(88)), 0);
  }
}
