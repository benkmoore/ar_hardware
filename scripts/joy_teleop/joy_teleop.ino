#include <SPI.h>  
#include "RF24.h" 

const int X_pin = A2;
const int Y_pin = A3;
int x = 512;
int y = 512;

RF24 myRadio (5, 6);
byte addresses[][6] = {"3"};

struct package
{
  bool kill = 0;
  float throttle = 0.0;
  float phi = 0.0;
};

typedef struct package Package;
Package data;


void setup()
{
  Serial.begin(9600);
  delay(1000);
  myRadio.begin();  
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate(RF24_250KBPS); 
  myRadio.openWritingPipe(addresses[0]);
  pinMode(15,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);


  delay(1000);
}

void loop()
{
  x = analogRead(A2) - 512;
  y = (analogRead(A3) - 512)*-1;

  data.phi = atan2(y,x);
  // data.throttle = analogRead(20);
  data.kill = digitalRead(15);
  myRadio.write(&data, sizeof(data)); 

  
  Serial.print("\nPackage:");
  Serial.print("\n");
  Serial.println(data.kill);
  Serial.println(data.throttle);
  Serial.println(data.phi);
  Serial.println("X: "+ String( x));
  Serial.println("Y: "+ String( y));

  delay(100);

}