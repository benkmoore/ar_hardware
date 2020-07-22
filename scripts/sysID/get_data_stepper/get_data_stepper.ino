#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Encoder.h>

#define dirPin1 37
#define stepPin1 36
#define motorInterfaceType 1

#define accel 30
#define vel 40
#define max_vel 100

// Define stepper and pins
AccelStepper stepper1(motorInterfaceType, stepPin1, dirPin1);
MultiStepper steppers;

// Setup encoder
Encoder encoder(3, 4);
long pos[1];
long u = 250; //amplitude of input

float generateGBN(float prob_switch, float u) {
  long prob = random(1000000);
  float new_u = u;
  if (prob < prob_switch) { 
    new_u = -u; 
  }
  return new_u;
}

void setup()
{  
   
   Serial.begin(57600);
   pos[0] = u;
   stepper1.setMaxSpeed(max_vel);
   stepper1.setSpeed(vel); 
   stepper1.setAcceleration(accel); 
   steppers.addStepper(stepper1);   
}

int i = 0;

void loop()
{   
   pos[0] = generateGBN(1, pos[0]);
   steppers.moveTo(pos);
   steppers.run();
   stepper1.setCurrentPosition(0);
   i = i + 1;
   if ( i % 1000 == 0 ) {
     Serial.print(encoder.read()); // y
     Serial.print(";");
     Serial.print(pos[0]);         // u
     Serial.print(";");
     Serial.print(millis());       // t
     Serial.println();
   }
}
