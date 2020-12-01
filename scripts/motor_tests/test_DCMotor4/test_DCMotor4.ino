//Teensy PWM pins
#define In1pin1 38 
#define In2pin1 37 
#define En1pin1 36

#define In1pin2 35 
#define In2pin2 34 
#define En1pin2 33

#define In1pin3 13 
#define In2pin3 14 
#define En1pin3 15

#define In1pin4 22 
#define In2pin4 21 
#define En1pin4 23 

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
  Forward_DCMotor(200, In1pin3, In2pin3, En1pin3);
  Forward_DCMotor(200, In1pin4, In2pin4, En1pin4);
}
