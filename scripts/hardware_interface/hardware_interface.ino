#define USE_USBCON
#include <ros.h>
#include <ar_commander/TOF.h>
#include <ar_commander/ControllerCmd.h>
#include <std_msgs/Float64.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <VL53L1X.h>


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
#define STEPPER_VEL 40                                // step/s
#define MAX_STEPPER_VEL 100                           // step/s
#define STEPPER_ACCEL 30                              // step/s^2
#define PHI_STEP 1.8                                  // deg/step
#define RAD_2_DEG 57.295779513082320876798154814105

// TOF constants
#define MEASUREMENT_TIME_MS 50                        // ms
#define MEASUREMENT_TIME_US 50000                     // us
#define I2C_HZ 400000                                 // 400 kHz I2C
#define TIMEOUT 100                                   // ms

// Input number of DC motors, stepper motors in use
const int N_DCMotors = 4;
const int N_StepperMotors = 4;

// Step Motor pins [ [StepPin, DirPin], ... ]
int StepperPins[N_StepperMotors][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

// DC Motor pins [ [in1, in2, en], ... ]
int DCMotorPins[N_DCMotors][3] = {{17, 16, 18}, {15, 14, 13}, {22, 23, 12}, {20, 21, 19}};

// Motor interface type for stppers
byte motorInterfaceType = 1;

// Define steppers
AccelStepper stepper1(motorInterfaceType, StepperPins[0][0], StepperPins[0][1]);
AccelStepper stepper2(motorInterfaceType, StepperPins[1][0], StepperPins[1][1]);
AccelStepper stepper3(motorInterfaceType, StepperPins[2][0], StepperPins[2][1]);
AccelStepper stepper4(motorInterfaceType, StepperPins[3][0], StepperPins[3][1]);
MultiStepper steppers;

// TOF sensors
VL53L1X tof1;
VL53L1X tof2;
VL53L1X tof3;

// TOF pin outs
int tofOut1 = 35;
int tofOut2 = 34;
int tofOut3 = 33;

// TOF I2C addresses
uint8_t tofAddress1 = 0x33;
uint8_t tofAddress2 = 0x35;
uint8_t tofAddress3 = 0x37;

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;

ar_commander::TOF tof_msg;
ros::Publisher tof_publisher("tof_data", &tof_msg);

std_msgs::Float64 fl_msg;
ros::Publisher chatter("chatter", &fl_msg);

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
  fl_msg.data = msg.phi_arr.data[0];
  chatter.publish(&fl_msg);
  
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
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  hardware_interface.subscribe(controller_cmds_sub);
  hardware_interface.advertise(tof_publisher);
  hardware_interface.advertise(chatter);

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

  // Setup motors
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DCMotorPins[i][0], OUTPUT);   
    pinMode(DCMotorPins[i][1], OUTPUT);
    pinMode(DCMotorPins[i][2], OUTPUT); 
  }

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
}

/*
 * ------------- MAIN ------------------
 */

void loop() {
  tof_msg.tof1 = tof1.readRangeContinuousMillimeters();
  tof_msg.tof2 = tof2.readRangeContinuousMillimeters();
  tof_msg.tof3 = tof3.readRangeContinuousMillimeters();
  tof_publisher.publish(&tof_msg);
  hardware_interface.spinOnce();
  
}
