
#ifndef Stepper_h
#define Stepper_h

/*
 * Stepper motor class for Analytical Robotics. Derived from the
 * following arduino libraries:
 *      https://github.com/pololu/high-power-stepper-driver-arduino
 *      https://github.com/arduino-libraries/Stepper
 */
class Stepper {
  public:
    // constructors:
    Stepper(int cs_pin, int stepsIn2pi, float phi_step, int steps_threshold,
            int max_vel, int min_vel, int max_milliamps);

    // calculate steps from encoder data pos to desired phi
    int calculateSteps(int encoder_data, int phi_des);

    // control stepper speed to decelerate to stop
    void controlSpeed(int steps);

    // speed setter method (RPM):
    void setSpeedRPM(long whatSpeed);

    // command number of steps (cw/ccw) for each stepper
    void commandStepper(int enc_pos, int phi_des);

    // mover method:
    void step(int stepsIn2pi);

    int version(void);

  private:
    void stepMotor(int this_step);
    void writeCTRL();
    void writeTORQUE();
    void writeOFF();
    void writeBLANK();
    void writeDECAY();
    void writeSTALL();
    void writeDRIVE();


    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int stepsIn2pi;           // total number of steps this motor can take
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on
    float phi_step;           // degrees per stepper step (deg/step)
    int steps_threshold;      // number of steps to begin deceleration to a stop (step)
    int max_vel;              // max continous veloicty for stepper rotation (steps/s)
    int min_vel;              // min velocity which stepper will decelerate to (steps/s)
    int max_milliamps;        // max current stepper can draw through driver (mA)

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;
    int motor_pin_5;          // Only 5 phase motor

    unsigned long last_step_time; // time stamp in us of when the last step was taken



};

/*
 * Driver class for DRV8711, handles all communications from teensy to driver.
 */
class Driver {
    public:
        void setCSPin(uint8_t cs_pin);
        void writeReg(StepperRegAddr address, uint16_t value);
        void resetSettings();
        uint16_t transferToSPI(uint16_t value);

    private:
        void writeReg(uint8_t address, uint16_t value);
        SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE0);
}

#endif
