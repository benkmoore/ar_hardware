#include "src/ar_bldc.h"
#define MAX_OMEGA 102.86
#define MAX_PWM 255
#define MIN_PWM 0
#define NUM_PULSES_2PI 20

int HALL_0 = 25;
int HALL_1 = 26;
int HALL_2 = 27;
int OUT_PIN = 0;
int NUM_HALLS = 3;

int hall_pins_1[3] = {HALL_0, HALL_1, HALL_2};
float controller_gains[3] = {1, 0, 0};
float omega_des = 1; // rad/s

BLDC bldc_1(OUT_PIN, hall_pins_1, NUM_HALLS, controller_gains, NUM_PULSES_2PI, MAX_OMEGA, MAX_PWM, MIN_PWM);

void setup() {
  Serial.begin(9600);
  delay(100);
  pinMode(HALL_0, INPUT);
  pinMode(HALL_1, INPUT);
  pinMode(HALL_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_0), updatePulseTime0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_1), updatePulseTime1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_2), updatePulseTime2, CHANGE);
}

void loop() {
  float omega = bldc_1.getOmega();
  Serial.println(omega);
//  Serial.print(' ');
//  int cmd = bldc_1.calculateCommand(omega_des);
//  Serial.print(cmd);
//  Serial.print(' ');
//  float error = omega_des - omega;
//  Serial.print(error);
}

void updatePulseTime0() {
    //bldc_1.hall_arr[0].updatePulseTime();
//    Serial.println(analogRead(HALL_0));
}
void updatePulseTime1() {
   // bldc_1.hall_arr[1].updatePulseTime();
//   Serial.println(analogRead(HALL_1));
}
void updatePulseTime2() {
    //bldc_1.hall_arr[2].updatePulseTime();
    Serial.println(analogRead(HALL_2));
}
