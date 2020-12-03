
#ifndef BLDC_h
#define BLDC_h

class HallSensor {
    public:
        float omega;            // rad/s
        int hall_pin;
        long prev_pulse_time;   // time of last pulse (millis)
        long pulse_time;        // time of current pulse (millis)
        bool new_meas;
        float ZERO_OMEGA_TIME;  // time without a sensor tick to set omega to 0 (s)

        HallSensor(int num_pulses_2pi, float ZERO_OMEGA_TIME); // constructor
        void updateOmega(); // update omega estimate based on interrupt pulse
        void updatePulseTime();

    private:
        int num_pulses_2pi;     // number of pulses in 2 pi (number of magnets in outer ring of hub motor)
};


class BLDC {
    public:
        long prev_pulse_time;   // previous pulse time
        HallSensor* hall_arr[3];   // array of hall sensors for bldc motor

        BLDC(int output_pin, int *hall_sensors, int NUM_HALLS, float *controller_gains, int num_pulses_2pi, float max_omega, int MAX_PWM, int MIN_PWM, float ZERO_OMEGA_TIME); // constructor
        void commandBLDC(float omega_des); // command omega to motor
        int calculateCommand(float omega_des); // calculate pwm out based on omega error
        float getOmega(); // retrieve average of estiamte of omega from hall sensors

    private:
        float k_p;              // proportional gain
        float k_i;              // integral gain
        float k_d;              // derivative gain
        float integral_error;   // sum error over time
        long prev_cmd_time;     // previous command time (ms)

        int num_pulses_2pi;     // number of pulses in 2 pi (number of magnets in outer ring of hub motor)
        int output_pin;         // output pwm throttle pin
        int *hall_sensor_pins;  // hall sensor pins
        int num_halls;          // number of halls sensors
        float max_omega;        // rad/s
        int MIN_PWM;            // minimum duty cycle
        int MAX_PWM;            // maximum duty cycle
        float ZERO_OMEGA_TIME;  // time (s) with no hall measurements to reset omega to 0 rad/s
};

#endif