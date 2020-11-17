#include <SPI.h>  
#include "RF24.h" 

//42 is orange (left side when face down 2nd from top)
RF24 myRadio (5, 6); 
struct package
{
  int kill = 0;
  float throttle = 0.0;
  float phi = 0.0;
};

byte addresses[][6] = {"3"}; 



typedef struct package Package;
Package data;

void setup() 
{
  Serial.begin(9600);
  delay(1000);

  myRadio.begin(); 
  myRadio.setChannel(115); 
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate( RF24_250KBPS ) ; 
  myRadio.openReadingPipe(1, addresses[0]);
  myRadio.startListening();
}


void loop()  
{

  if ( myRadio.available()) 
  {
    
    myRadio.read( &data, sizeof(data) );
    Serial.print("\nPackage Received:");
    Serial.print("\n");
    Serial.println(data.kill);
    Serial.println(data.throttle);
    Serial.println(data.phi);
  }

}
