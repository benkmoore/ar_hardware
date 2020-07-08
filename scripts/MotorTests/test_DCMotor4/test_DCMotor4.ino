//Teensy PWM pins
#define In1pin1 15 
#define In2pin1 14 
#define En1pin1 13

#define In1pin2 17 
#define In2pin2 16 
#define En1pin2 18

#define In1pin3 20 
#define In2pin3 21 
#define En1pin3 19

#define In1pin4 22 
#define In2pin4 23 
#define En1pin4 12

void setup() {
  // Set pins to Output
  pinMode(In1pin1, OUTPUT);   
  pinMode(In2pin1, OUTPUT);
  pinMode(En1pin1, OUTPUT);  
  pinMode(In1pin2, OUTPUT);   
  pinMode(In2pin2, OUTPUT);
  pinMode(En1pin2, OUTPUT);
  pinMode(In1pin3, OUTPUT);   
  pinMode(In2pin3, OUTPUT);
  pinMode(En1pin3, OUTPUT);
  pinMode(In1pin4, OUTPUT);   
  pinMode(In2pin4, OUTPUT);
  pinMode(En1pin4, OUTPUT);
}

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

void loop() {
  // put your main code here, to run repeatedly:
  Forward_DCMotor(200, In1pin1, In2pin1, En1pin1);
  Forward_DCMotor(200, In1pin2, In2pin2, En1pin2);
}
