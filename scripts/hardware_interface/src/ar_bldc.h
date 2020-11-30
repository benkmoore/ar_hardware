
#ifndef BLDC_h
#define BLDC_h

class HallSensor {
    public:
        float omega;            // rad/s

        HallSensor(int hall_pin, int num_pulses_2pi); // constructor
        void updateOmega(); // update omega estimate based on interrupt pulse
        void updatePulseTime();

    private:
        long prev_pulse_time;   // time of last pulse (millis)
        long pulse_time;        // time of current pulse (millis)
        int num_pulses_2pi;     // number of pulses in 2 pi (number of magnets in outer ring of hub motor)
};


class BLDC {
    public:
        BLDC(int output_pin, int *hall_sensors, float *controller_gains, int num_pulses_2pi); // constructor

        void commandBLDC(float omega_des); // command omega to motor

    private:
        float k_p;              // proportional gain
        float k_i;              // integral gain
        float k_d;              // derivative gain
        long prev_pulse_time;   // previous pulse time
        int num_pulses_2pi;     // number of pulses in 2 pi (number of magnets in outer ring of hub motor)
        int output_pin;         // output pwm throttle pin
        int *hall_sensor_pins;  // hall sensor pins
        int num_halls;          // number of halls sensors
        HallSensor *hall_arr;   // array of hall sensors for bldc motor

        int calculateCommand(float omega_des); // calculate pwm out based on omega error
        float getOmega(); // retrieve average of estiamte of omega from hall sensors
};

#endif