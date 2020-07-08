#include <AccelStepper.h>

#define dirPin1 1
#define stepPin1 0
#define dirPin2 3
#define stepPin2 2
#define dirPin3 5
#define stepPin3 4
#define dirPin4 7
#define stepPin4 6
#define motorInterfaceType 1

#define accel 1400
#define vel 1400
#define max_vel 2000

// Define some steppers and the pins the will use
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(motorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3(motorInterfaceType, stepPin3, dirPin3);
AccelStepper stepper4(motorInterfaceType, stepPin4, dirPin4);

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
}
void loop()
{  
   
   stepper1.runSpeed();
   stepper2.runSpeed();
   stepper3.runSpeed();
   stepper4.runSpeed();
}
