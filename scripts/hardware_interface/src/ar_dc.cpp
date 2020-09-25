#include "Arduino.h"
#include "ar_dc.h"



DC_Motors::DC_Motors(int* reverseFlags, int* DC_reverse, int* DC_throttlePins){
  //reverseFlags = {0,0,0,0};
  this->reverseFlags =  reverseFlags;
  this->reverse = DC_reverse;
  this->throttlePins = DC_throttlePins;  
  //this->flip[] = {0,0,0,0};
}

void DC_Motors::PowerDC(int aPin, int PWMspeed, int index) {
    if(PWMspeed > 0 ){             
        if(this->reverseFlags[index] == 0){
            analogWrite(aPin, PWMspeed);
        }
        else{
	    //analogWrite(aPin, 0);
            this->flip[index] = 1;	    
            analogWrite(aPin, PWMspeed);
        }
    }

    else if(PWMspeed < 0) {
        if(this->reverseFlags[index] == 1){
            analogWrite(aPin, PWMspeed*-1);
        }
        else{
	    //analogWrite(aPin, 0);
            this->flip[index] = 1;	    
            analogWrite(aPin, PWMspeed*-1);
        }
    }
    else if(PWMspeed == 0 ){             
            analogWrite(aPin, PWMspeed);
    }
}

void DC_Motors::flipDirection(){
  for(int i = 0; i < 4; i++) {   
      if(this->flip[i] == 1){
         digitalWrite(this->reverse[i],LOW);
         this->flip[i] = 0;
this->reverseFlags[i] = !this->reverseFlags[i];
      }

  }
  delay(500);
  for(int i = 0; i < 4; i++) {   
  digitalWrite(this->reverse[i],HIGH);
  
      }


  
}
