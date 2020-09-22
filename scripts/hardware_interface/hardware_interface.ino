#define USE_USBCON
#include "ros.h"
#include "ar_commander/ControllerCmd.h"
#include "Encoder.h"
#include "SPI.h"
#include "src/ar_stepper.h"
#include <std_msgs/Float64.h>
#define RX        7 
#define TX        8

//#define RxTx
#define Re    3
#define De    4


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
static const int DC_reverse[N_DCMotors] = {18,19,20,21};
static const int DC_throttlePins[N_DCMotors] = {A0,A1,A2,A3};

// Define steppers
Stepper stepper1(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper2(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper3(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);
Stepper stepper4(int(360.0/PHI_STEP), PHI_STEP, STEPS_THRESHOLD, MAX_STEPPER_VEL, MIN_STEPPER_VEL, MAX_MILLIAMPS, MICRO_STEP_SIZE, DECAY_MODE);

DC_Motors DC_1();
DC_Motors DC_2();
DC_Motors DC_3();
DC_Motors DC_4();


int phi_des1 = 0;
int phi_des2 = 0;
int phi_des3 = 0;
int phi_des4 = 0;


std_msgs::Float64 test;
ros::Publisher chatter_pub("chatter", &test);

/*
 * ------------- RECEIVE ROS MSGS & CMD MOTORS ------------------
 */

// ROS node
ros::NodeHandle_<ArduinoHardware, NUM_PUBS, NUM_SUBS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE> hardware_interface;
//
// define ROS node name, rate, subscriber to /controller_cmds
void controllerCmdCallback(const ar_commander::ControllerCmd &msg) {
  for(int i = 0; i < N_DCMotors; i++) {    
      PowerDC(msg.omega_arr.data[i], DC_throttlePins[i]);
  }

  // rads to degrees to int steps: (rad*(deg/rad) / (deg/step) = step
  phi_des1 = (int) ( (-msg.phi_arr.data[0]*RAD_2_DEG)/PHI_STEP );
  phi_des2 = (int) ( (-msg.phi_arr.data[1]*RAD_2_DEG)/PHI_STEP );
  phi_des3 = (int) ( (-msg.phi_arr.data[2]*RAD_2_DEG)/PHI_STEP );
  phi_des4 = (int) ( (-msg.phi_arr.data[3]*RAD_2_DEG)/PHI_STEP );

}

ros::Subscriber<ar_commander::ControllerCmd> controller_cmds_sub("controller_cmds",controllerCmdCallback);


/*
   ------------- SUPPORT FUNCTIONS ------------------
*/
class DC_Motors{
  public:
    const int *reverse;
    const int *throttlePins;

    void PowerDC (int, int);
    void flipDirection (int);

    // Define forward rotation - DC Motor
}

DC_Motors::DC_Motors (){
  bool reverseFlags[N_DCMotors] = {0,0,0,0};
  this->reverseFlags =  reverseFlags;
  reverse = DC_reverse;
  throttlePins = DC_throttlePins;

  
  
}

void DC_Motors::PowerDC(int PWMspeed, int aPin) {
      int index = aPin-throttlePins[0]

      if(PWMspeed >= 0 ){             
        if(this->reverseFlags[index] == 0){
          analogWrite(aPin, PWMspeed);
        }
        else{
          flipDirection(index);
          analogWrite(aPin, PWMspeed);
        }
      }

      if(PWMspeed < 0 {
        if(this->reverseFlags[index] == 1){
          analogWrite(aPin, PWMspeed);
        }
        else{
          flipDirection(index);
          analogWrite(aPin, PWMspeed);
        }
      }

  
    }

    void DC_Motors::flipDirection(int reversePin){
      digitalWrite(reverse[reversePin],HIGH);
      delay(1)
      digitalWrite(reverse[reversePin],LOW);
      this->reverseFlags[reversePin] = !this->reverseFlags[reversePin]
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

  Serial2.write(byteOut);           // Send byte to encoder
  delay(1);
  Serial2.flush();
  RS485Receive();
  i = 0;
//ROS_DEBUG("in checkencoder");
  while (Serial2.available())       // Look for data from encoder
  {
    byteIn[i] = Serial2.read();     // Read received byte
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
  hardware_interface.advertise(chatter_pub);
  hardware_interface.subscribe(controller_cmds_sub);
  
  // Setup stepper motors 
  SPI.begin();
  stepper1.setupDriver(StepperMotorPins[0]);
  stepper2.setupDriver(StepperMotorPins[1]);
  stepper3.setupDriver(StepperMotorPins[2]);
  stepper4.setupDriver(StepperMotorPins[3]);
  

  // Setup DC reverse pins
  for(int i = 0; i < N_DCMotors; i++) {
    pinMode(DC_reverse[i], OUTPUT);
  }

  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);
  RS485Receive();
//  Serial.begin(9600);
  Serial2.begin(115200);        // set the data rate

}

/*
 * ------------- MAIN ------------------
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
