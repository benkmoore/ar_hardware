#include <Wire.h>
#include <VL53L0X.h>

VL53L0X tof1;
VL53L0X tof2;
VL53L0X tof3;

void setup()
{
  pinMode(15, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite(14, LOW);
  digitalWrite(13, LOW);
  delay(500);
  
  Serial.begin(9600);
  Wire.begin();

  digitalWrite(15, HIGH);
  delay(150);
  tof1.init();
  delay(100);
  tof1.setTimeout(500);
  tof1.setAddress((uint8_t)01);
  Serial.println("setup 1");

  digitalWrite(14, HIGH);
  delay(150);
  tof2.init();
  delay(100);
  tof2.setTimeout(500);
  tof2.setAddress((uint8_t)02);
  Serial.println("setup 2");

  digitalWrite(13, HIGH);
  delay(150);
  tof3.init();
  delay(100);
  tof3.setTimeout(500);
  tof3.setAddress((uint8_t)03);
  Serial.println("setup 3");

  tof1.startContinuous();
  tof2.startContinuous();
  tof3.startContinuous();
}

void loop()
{
  int dist1 = tof1.readRangeContinuousMillimeters();
  int dist2 = tof2.readRangeContinuousMillimeters();
  int dist3 = tof3.readRangeContinuousMillimeters();
  
  Serial.print("dist1: ");
  Serial.print(dist1);
  Serial.print(", dist2: ");
  Serial.print(dist2);
  Serial.print(", dist3: ");
  Serial.print(dist3);
  if (tof1.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF1"); }
  if (tof2.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF2"); }
  if (tof3.timeoutOccurred()) { Serial.print(" TIMEOUT - TOF3"); }
  Serial.println();
  delay(100);
}
