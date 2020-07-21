#include <Encoder.h>

// Setup encoder
Encoder encoder(3, 4);

#define In1pin 15 
#define In2pin 14 
#define En1pin 13

float u = 180.0; //amplitude of input

float generateGBN(float prob_switch, float u) {
  long prob = random(1000000);
  float new_u = u;
  if (prob < prob_switch) { 
    new_u = -u; 
  }
  return new_u;
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

void setup() {
  Serial.begin(57600);
  pinMode(In1pin, OUTPUT);   
  pinMode(In2pin, OUTPUT);
  pinMode(En1pin, OUTPUT);  
}

int i = 0;
 
void loop() {
  u = generateGBN(1, u);
  if (u > 0) {
    Forward_DCMotor(u, In1pin, In2pin, En1pin);
  } else {
    Reverse_DCMotor(u, In1pin, In2pin, En1pin);
  }
  i = i + 1;
  if ( i % 2000 == 0 ) {
    float y = encoder.read();
    float t = millis();
    Serial.print(y);
    Serial.print(";");
    Serial.print(u);
    Serial.print(";");
    Serial.print(t);
    Serial.println();
  }
}
