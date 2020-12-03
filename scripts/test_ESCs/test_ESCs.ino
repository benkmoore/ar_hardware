#include "src/ar_bldc.h"
#define MAX_OMEGA 102.86
#define MAX_PWM 255
#define MIN_PWM 0
#define NUM_PULSES_2PI 20
#define ZERO_OMEGA_TIME 5

int HALL_0 = 25;
int HALL_1 = 26;
int HALL_2 = 27;
int OUT_PIN = 0;
int NUM_HALLS = 3;

int hall_pins_1[3] = {HALL_0, HALL_1, HALL_2};
float controller_gains[3] = {1, 0, 0};
float omega_des = 1; // rad/s

float omega2 = 0.0;
float omega1 = 0.0;
float omega = 0.0;
float om_avg = 0.0;

int pwm_out = 0.0;

BLDC bldc_1(OUT_PIN, hall_pins_1, NUM_HALLS, controller_gains, NUM_PULSES_2PI, MAX_OMEGA, MAX_PWM, MIN_PWM, ZERO_OMEGA_TIME);

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
int i = 0;
void loop() {
    bldc_1.hall_arr[0]->updateOmega();
    bldc_1.hall_arr[1]->updateOmega();
    bldc_1.hall_arr[2]->updateOmega();
    omega = bldc_1.hall_arr[0]->omega;
    omega1 = bldc_1.hall_arr[1]->omega;
    omega2 = bldc_1.hall_arr[2]->omega;

    om_avg = bldc_1.getOmega();

    pwm_out = bldc_1.calculateCommand(10);

    if (i % 1000 == 0) {
      Serial.print(omega);
      Serial.print(' ');
      Serial.print(omega1);
      Serial.print(' ');
      Serial.print(omega2);
      Serial.print(": ");
      Serial.print(om_avg);
      Serial.print("-> ");
      Serial.println(pwm_out);
    }
//    float omega = bldc_1.getOmega();
    //Serial.println(omega);//inf
//  Serial.println(omega);
//  Serial.print(' ');
//  int cmd = bldc_1.calculateCommand(omega_des);
//  Serial.print(cmd);
//  Serial.print(' ');
//  float error = omega_des - omega;
//  Serial.print(error);
}

void updatePulseTime0() {
    //Serial.println("**0**");
    bldc_1.hall_arr[0]->updatePulseTime();
//    Serial.println(bldc_1.hall_arr[0]->pulse_time);
//    Serial.println(bldc_1.hall_arr[0]->prev_pulse_time);
//    omega = bldc_1.hall_arr[0]->omega;
    //Serial.println(omega);
}
void updatePulseTime1() {
    //Serial.println("**1**");
    bldc_1.hall_arr[1]->updatePulseTime();
//    Serial.println(bldc_1.hall_arr[1]->pulse_time);
//    Serial.println(bldc_1.hall_arr[1]->prev_pulse_time);
//    omega1 = bldc_1.hall_arr[1]->omega;
    //Serial.println(omega1);
}
void updatePulseTime2() {
    //Serial.println("**2**");
    bldc_1.hall_arr[2]->updatePulseTime();
//    Serial.println(bldc_1.hall_arr[2]->pulse_time);
//    Serial.println(bldc_1.hall_arr[2]->prev_pulse_time);
//    omega2 = bldc_1.hall_arr[2]->omega;

}
