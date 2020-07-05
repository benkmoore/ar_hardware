//Teensy PWM pins
#define In1pin 15 
#define In2pin 14 
#define En1pin 13

void setup() {
  // Set pins to Output
  Serial.begin(9600);
  pinMode(In1pin, OUTPUT);   
  pinMode(In2pin, OUTPUT);
  pinMode(En1pin, OUTPUT); 
  // Set initial rotation direction
  digitalWrite(In1pin, LOW);
  digitalWrite(In2pin, HIGH); 
}

void Forward(int PWMspeed) {
  digitalWrite(In1pin, LOW); 
  digitalWrite(In2pin, HIGH); 
  analogWrite(En1pin, PWMspeed); 
  delay(1000);
}

void Reverse(int PWMspeed) {
  digitalWrite(In1pin, HIGH); 
  digitalWrite(In2pin, LOW);
  //analogWrite(En1pin, PWMspeed);  
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Forward ->");
  for (int i = 80; i < 250; i++) {
     Forward(i);
  }
  
}
