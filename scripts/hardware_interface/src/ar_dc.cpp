#include "Arduino.h"
#include "ar_dc.h"



DC_Motors::DC_Motors(int* reverseFlags, int* DC_reverse, int* DC_throttlePins, int N_DCMotors, int* flip){
  this->reverseFlags =  reverseFlags;
  this->reverse = DC_reverse;
  this->throttlePins = DC_throttlePins;  
  this->N_DCMotors = N_DCMotors;
  this->flip = flip;
}

void DC_Motors::PowerDC(int aPin, int PWMspeed, int index) {
    if(PWMspeed > 0 ){             
        if(this->reverseFlags[index] == 0){
            analogWrite(aPin, PWMspeed);
        }
        else{
            this->flip[index] = 1;
            this->flipFlag = 1;    	    
            analogWrite(aPin, PWMspeed);
        }
    }

    else if(PWMspeed < 0) {
        if(this->reverseFlags[index] == 1){
            analogWrite(aPin, PWMspeed*-1);
        }
        else{
            this->flip[index] = 1;	
            this->flipFlag = 1;    
            analogWrite(aPin, PWMspeed*-1);
        }
    }
    else if(PWMspeed == 0 ){             
            analogWrite(aPin, PWMspeed);
    }
}

void DC_Motors::flipDirection(){
  for(int i = 0; i < this->N_DCMotors; i++) {   
      if(this->flip[i] == 1){
         digitalWrite(this->reverse[i],LOW);
         this->flip[i] = 0;
         this->reverseFlags[i] = !this->reverseFlags[i];
      }
  }
  this->flipFlag = 0;    
  delay(500);
  for(int i = 0; i < this->N_DCMotors; i++) {   
      digitalWrite(this->reverse[i],HIGH);
  }  
}
