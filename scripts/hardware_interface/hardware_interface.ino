#define USE_USBCON
#include <ros.h>
#include <ar_commander/TOF.h>
#include <std_msgs/Float64.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <VL53L1X.h>


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
#define MEASUREMENT_TIME 50

// TOF sensors
VL53L1X tof1;
VL53L1X tof2;
VL53L1X tof3;

int tofOut1 = 35;
int tofOut2 = 34;
int tofOut3 = 33;

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, 1, 1, 2048, 2048> hardware_interface;

ar_commander::TOF tof_msg;
ros::Publisher tof_publisher("tof_data", &tof_msg);

/*
 * ------------- SUPPORT FUNCTIONS ------------------
 */


/*
 * ------------- SETUP INTERFACE ------------------
 */

void setup() {
  // Init node and Subscribe to /controller_cmds
  hardware_interface.getHardware()->setBaud(BAUD_RATE);
  hardware_interface.initNode();
  hardware_interface.advertise(tof_publisher);

  // Init TOF XSHUT pins
  pinMode(tofOut1, OUTPUT);
  pinMode(tofOut2, OUTPUT);
  pinMode(tofOut3, OUTPUT);
  digitalWrite(tofOut1, LOW);
  digitalWrite(tofOut2, LOW);
  digitalWrite(tofOut3, LOW);
  delay(500);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  delay(2000);

  digitalWrite(tofOut1, HIGH);
  delay(150);
  tof1.init();
  delay(100);
  tof1.setAddress((uint8_t)0x33);

  digitalWrite(tofOut2, HIGH);
  delay(150);
  tof2.init();
  delay(100);
  tof2.setAddress((uint8_t)0x35);

  digitalWrite(tofOut3, HIGH);
  delay(150);
  tof3.init();
  delay(100);
  tof3.setAddress((uint8_t)0x37);

  tof1.setDistanceMode(VL53L1X::Long);
  tof1.setMeasurementTimingBudget(MEASUREMENT_TIME*1000);
  tof1.startContinuous(MEASUREMENT_TIME); //ms
  tof1.setTimeout(100);

  tof2.setDistanceMode(VL53L1X::Long);
  tof2.setMeasurementTimingBudget(MEASUREMENT_TIME*1000);
  tof2.startContinuous(MEASUREMENT_TIME); //ms
  tof2.setTimeout(100);

  tof3.setDistanceMode(VL53L1X::Long);
  tof3.setMeasurementTimingBudget(MEASUREMENT_TIME*1000);
  tof3.startContinuous(MEASUREMENT_TIME); //ms
  tof3.setTimeout(100);
}

/*
 * ------------- MAIN ------------------
 */

void loop() {
  hardware_interface.spinOnce();

  int dist1 = tof1.readRangeContinuousMillimeters() + 0.0;
  if (tof1.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF1"); }
  int dist2 = tof2.readRangeContinuousMillimeters();
  if (tof2.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF2"); }
  int dist3 = tof3.readRangeContinuousMillimeters();
  if (tof3.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF3"); }

  tof_msg.tof1 = dist1;
  tof_msg.tof2 = dist2;
  tof_msg.tof3 = dist3;
  tof_publisher.publish(&tof_msg);
}
