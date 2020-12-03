
#include "Arduino.h"
#include "ar_bldc.h"

// ------------- BLDC MOTOR FUNCTIONS -------------

BLDC::BLDC(int output_pin, int hall_sensors_pins[3], int NUM_HALLS, float controller_gains[3], int num_pulses_2pi, float max_omega, int MAX_PWM, int MIN_PWM, float ZERO_OMEGA_TIME) {
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
    this->num_halls = NUM_HALLS;
    this->output_pin = output_pin;

    // setup hall sensors
    this->hall_arr[0] = new HallSensor(this->num_pulses_2pi, ZERO_OMEGA_TIME);
    this->hall_arr[1] = new HallSensor(this->num_pulses_2pi, ZERO_OMEGA_TIME);
    this->hall_arr[2] = new HallSensor(this->num_pulses_2pi, ZERO_OMEGA_TIME);

    // setup output pin
    pinMode(output_pin, OUTPUT);
}

float BLDC::getOmega() {
    float avg_omega = 0.0;

    for (int i = 0; i < this->num_halls; i++) {
        this->hall_arr[i]->updateOmega();
        avg_omega += this->hall_arr[i]->omega;
    }
    avg_omega = avg_omega / this->num_halls;

    return avg_omega;
}

int BLDC::calculateCommand(float omega_des) {
    long cmd_time = millis();
    double dt = 0.0;
    if (this->prev_cmd_time != -1) { // check time is initialized
        dt = (cmd_time - this->prev_cmd_time) / 1000.0; // convertt millis to seconds
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

HallSensor::HallSensor(int num_pulses_2pi, float ZERO_OMEGA_TIME) {
    // init sensor io data
    this->pulse_time = -1;
    this->prev_pulse_time = -1;
    this->omega = 0;
    this->ZERO_OMEGA_TIME = ZERO_OMEGA_TIME;

    // motor constants
    this->num_pulses_2pi = num_pulses_2pi;
}

void HallSensor::updatePulseTime() {
    this->prev_pulse_time = this->pulse_time;
    this->pulse_time = millis();
}

void HallSensor::updateOmega() {
    if ((this->prev_pulse_time != -1) and (this->pulse_time != -1)) {
        double dt = (this->pulse_time - this->prev_pulse_time) / 1000.0; // convert millis to seconds
        this->omega = (2 * PI) / (dt * this->num_pulses_2pi); // rad/s
    }

    if ( ((millis() - this->pulse_time) / 1000.0) > this->ZERO_OMEGA_TIME ) { // reset omega to zero if no ticks recieved in time window
        this->omega = 0;
    }
}
