#include "Arduino.h"
#include "ar_dc.h"



DC_Motors::DC_Motors(int* reverseFlags, int* DC_reverse, int* DC_throttlePins){
  //reverseFlags = {0,0,0,0};
  this->reverseFlags =  reverseFlags;
  this->reverse = DC_reverse;
  this->throttlePins = DC_throttlePins;  
}

void DC_Motors::PowerDC(int PWMspeed, int aPin) {
    int index = aPin-this->throttlePins[0];
    if(PWMspeed >= 0 ){             
        if(this->reverseFlags[index] == 0){
            analogWrite(aPin, PWMspeed);
        }
        else{
analogWrite(aPin, 0);
            this->flipDirection(index);
	    
	    delay(1);
            analogWrite(aPin, PWMspeed);
        }
    }

    if(PWMspeed < 0) {
        if(this->reverseFlags[index] == 1){
            analogWrite(aPin, PWMspeed*-1);
        }
        else{
		analogWrite(aPin, 0);
            this->flipDirection(index);
	    
	    
            analogWrite(aPin, PWMspeed*-1);
        }
    }
}

void DC_Motors::flipDirection(int reversePin){
  digitalWrite(this->reverse[reversePin],HIGH);
  delay(1000);
  digitalWrite(this->reverse[reversePin],LOW);
  this->reverseFlags[reversePin] = !this->reverseFlags[reversePin];
}
