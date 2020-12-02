
#include "Arduino.h"
#include "ar_bldc.h"
#include "math.h"

// ------------- BLDC MOTOR FUNCTIONS -------------

BLDC::BLDC(int output_pin, int hall_sensors_pins[3], float controller_gains[3], int num_pulses_2pi, float max_omega, int MAX_PWM, int MIN_PWM) {
    // controller gains and time tracking
    this->k_p = controller_gains[0];
    this->k_i = controller_gains[1];
    this->k_d = controller_gains[2];
    this->prev_cmd_time = -1;
    this->integral_error = 0;

    // motor constants
    this->num_pulses_2pi = num_pulses_2pi;
    this->max_omega = max_omega;
    this->MIN_PWM = MIN_PWM;
    this->MAX_PWM = MAX_PWM;

    // data io pins & init
    this->prev_pulse_time = millis();
    this->hall_sensor_pins = hall_sensor_pins;
    this->num_halls = sizeof(this->hall_sensor_pins) / sizeof(this->hall_sensor_pins[0]);
    this->output_pin = output_pin;

    // setup hall sensors
    for (int i = 0; i < this->num_halls; i++) {
        this->hall_arr[i] = HallSensor(this->hall_sensor_pins[i], this->num_pulses_2pi);
    }

    // setup output pin
    pinMode(output_pin, OUTPUT);
}

float BLDC::getOmega() {
    float avg_omega = 0.0;

    for (int i = 0; i < this->num_halls; i++) {
        this->hall_arr[i].updateOmega();
        avg_omega += this->hall_arr[i].omega;
    }
    avg_omega = avg_omega / this->num_halls;

    return avg_omega;
}

int BLDC::calculateCommand(float omega_des) {
    long cmd_time = millis();
    long dt = 0.0;
    if (this->prev_cmd_time != -1) { // check time is initialized
        dt = (cmd_time - this->prev_cmd_time) * 1000; // convertt millis to seconds
    }

    // retrieve controller inputs
    float omega = this->getOmega();
    float error = omega_des - omega;
    float u_bias = this->MAX_PWM * omega_des / this->max_omega;

    // calculate controller output
    int pwm = u_bias + (this->k_p * error) + this->k_i * (this->integral_error + (error * dt));
    pwm = round(constrain(pwm, this->MIN_PWM, this->MAX_PWM)); // constrain pwm to min and max pwm and round to int
    if (pwm != this->MIN_PWM and pwm != this->MAX_PWM) { // anti-reset windup: only add to integral term if not saturated
        this->integral_error += error * dt;
    }
    this->prev_cmd_time = cmd_time;

    return pwm;
}

void BLDC::commandBLDC(float omega_des) {
    analogWrite(this->output_pin, this->calculateCommand(omega_des));
}


// -------------- HALL SENSOR FUNCTIONS --------------

HallSensor::HallSensor(int hall_pin, int num_pulses_2pi) {
    // init sensor io data
    this->pulse_time = -1;
    this->prev_pulse_time = -1;
    this->omega = 0;

    // motor constants
    this->num_pulses_2pi = num_pulses_2pi;
}

void HallSensor::updatePulseTime() {
    this->prev_pulse_time = this->pulse_time;
    this->pulse_time = millis();
}

void HallSensor::updateOmega() {
    if ((this->prev_pulse_time == -1) or (this->pulse_time == -1)) {
        this->omega = 0;
    } else {
        long delta_time = (this->pulse_time - this->prev_pulse_time) * 1000; // convert millis to seconds
        this->omega = (2 * PI) / (delta_time * this->num_pulses_2pi); // rad/s
    }
}
