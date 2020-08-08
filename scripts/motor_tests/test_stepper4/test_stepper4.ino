#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Encoder.h>

#define dirPin1 1
#define stepPin1 0
#define dirPin2 3
#define stepPin2 2
#define dirPin3 5
#define stepPin3 4
#define dirPin4 7
#define stepPin4 6
#define motorInterfaceType 1

#define accel 0 //0.0025, 10000
#define vel 20
#define max_vel 100

#define PHI_STEP 1.8 
#define ENC_CPR 4000  

// Define some steppers and the pins the will use
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3(motorInterfaceType, stepPin3, dirPin3);
AccelStepper stepper4(motorInterfaceType, stepPin4, dirPin4);
MultiStepper steppers;

// Encoder pin outs
int encPinA_1 = 26;                                   // enc1 -> stepper1
int encPinB_1 = 25;
int encPinA_2 = 28;                                   // enc2 -> stepper2
int encPinB_2 = 27;
int encPinA_3 = 30;                                   // enc3 -> stepper3
int encPinB_3 = 29;
int encPinA_4 = 32;                                   // enc4 -> stepper4
int encPinB_4 = 31;

// Define encoders
Encoder enc1(encPinA_1, encPinB_1);
Encoder enc2(encPinA_2, encPinB_2);
Encoder enc3(encPinA_3, encPinB_3);
Encoder enc4(encPinA_4, encPinB_4);

void setup()
{  
   
   stepper1.setMaxSpeed(max_vel);
   stepper1.setSpeed(vel); 
   stepper1.setAcceleration(accel);
   stepper2.setMaxSpeed(max_vel);
   stepper2.setSpeed(vel);  
   stepper2.setAcceleration(accel);
   stepper3.setMaxSpeed(max_vel);
   stepper3.setSpeed(vel);
   stepper3.setAcceleration(accel);
   stepper4.setMaxSpeed(max_vel);
   stepper4.setSpeed(vel); 
   stepper4.setAcceleration(accel);  

   steppers.addStepper(stepper1);
   steppers.addStepper(stepper2);
   steppers.addStepper(stepper3);
   steppers.addStepper(stepper4);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
}

int i = 0;
void loop()
{  
   
//   stepper1.runSpeed();
//   stepper2.runSpeed();
//   stepper3.runSpeed();
//   stepper4.runSpeed();
   i = i + 1;
  if ( i % 100000 == 0 ) {
    // counts * (degs/count) * (step/deg) = steps
    int enc1_pos = int( (enc1.read())*(360.0/ENC_CPR)*(1.0/PHI_STEP) ) % int( 360.0/PHI_STEP );
    int enc2_pos = int( (enc2.read())*(360.0/ENC_CPR)*(1.0/PHI_STEP) ) % int( 360.0/PHI_STEP );
    int enc3_pos = int( (enc3.read())*(360.0/ENC_CPR)*(1.0/PHI_STEP) ) % int( 360.0/PHI_STEP );
    int enc4_pos = int( (enc4.read())*(360.0/ENC_CPR)*(1.0/PHI_STEP) ) % int( 360.0/PHI_STEP );
    stepper1.setCurrentPosition(-enc1_pos);
    stepper2.setCurrentPosition(-enc2_pos);
    stepper3.setCurrentPosition(-enc3_pos);
    stepper4.setCurrentPosition(-enc4_pos);
  }

  if (i % 60 == 0 ) { //Controller msg rate
    long pos[4];
    pos[0] = 100;
    pos[1] = 100;
    pos[2] = 100;
    pos[3] = 100; 
    steppers.moveTo(pos);
    steppers.run();
  }
}
