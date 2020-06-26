#include <AccelStepper.h>

#define dirPin 1
#define stepPin 0
#define motorInterfaceType 1

// Define some steppers and the pins the will use
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

void setup()
{  
    stepper.setMaxSpeed(200.0);
}

void loop()
{
    // Change direction at the limits
    Serial.println("running...");
    stepper.setCurrentPosition(0);

    while(stepper.currentPosition() != 400) {
      stepper.setSpeed(300);
      stepper.runSpeed();
    }

    delay(1000);
}
