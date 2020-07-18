#include <Wire.h>
#include <VL53L1X.h>

VL53L1X tof1;
VL53L1X tof2;
VL53L1X tof3;

int MEASUREMENT_TIME = 50;

void setup()
{
  // Note Teensy 4.0 and Teensy 4.1 pin numbers are different
  int out1 = 35;
  int out2 = 34;
  int out3 = 33;
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  digitalWrite(out1, LOW);
  digitalWrite(out2, LOW);
  digitalWrite(out3, LOW);
  delay(500);
  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Wire.beginTransmission(0x29);
  delay(2000);

  digitalWrite(out1, HIGH);
  delay(150);
  tof1.init();
  delay(100);
  tof1.setAddress((uint8_t)0x33);
  Serial.println("setup 1");

  digitalWrite(out2, HIGH);
  delay(150);
  tof2.init();
  delay(100);
  tof2.setAddress((uint8_t)0x35);
  Serial.println("setup 2");

  digitalWrite(out3, HIGH);
  delay(150);
  tof3.init();
  delay(100);
  tof3.setAddress((uint8_t)0x37);
  Serial.println("setup 3");

  tof1.setDistanceMode(VL53L1X::Long);
  tof1.setMeasurementTimingBudget(MEASUREMENT_TIME*1000);
  tof1.startContinuous(MEASUREMENT_TIME); //ms
  tof1.setTimeout(100);

  tof2.setDistanceMode(VL53L1X::Long);
  tof2.setMeasurementTimingBudget(MEASUREMENT_TIME*1000);
  tof2.startContinuous(MEASUREMENT_TIME); //ms
  tof2.setTimeout(100);

  tof3.setDistanceMode(VL53L1X::Long);
  tof3.setMeasurementTimingBudget(MEASUREMENT_TIME*1000);
  tof3.startContinuous(MEASUREMENT_TIME); //ms
  tof3.setTimeout(100);


  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  
    } 
  } 
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

}

void loop()
{
  Serial.print("dist1: ");
  int dist1 = tof1.readRangeContinuousMillimeters();
  Serial.print(dist1);
  if (tof1.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF1"); }
  
  Serial.print(", dist2: ");
  int dist2 = tof2.readRangeContinuousMillimeters();
  Serial.print(dist2);
  if (tof2.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF2"); }
  
  Serial.print(", dist3: ");
  int dist3 = tof3.readRangeContinuousMillimeters();
  Serial.print(dist3);
  if (tof3.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF3"); }

  Serial.println();
}
