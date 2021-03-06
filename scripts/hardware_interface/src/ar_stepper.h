/*
 * This file includes a Stepper class and a Driver class.
 * Derived from the following arduino libraries:
 *      https://github.com/pololu/high-power-stepper-driver-arduino
 *      https://github.com/arduino-libraries/Stepper
 *
 * The stepper class handles higher level stepper commands.
 * This class receives the desired location and uses encoder
 * data to calculate the number of steps. This class also
 * handles stepper motor deceleration.
 *
 * The driver class handles lower level communication of these
 * commands to the registers which ultimately actuate the stepper
 * motors.
 */

#ifndef Stepper_h
#define Stepper_h
#include "SPI.h"


/*
 * Driver adresses and stepper modes
 */

// Addresses of control and status registers.
enum class StepperRegAddr : uint8_t
{
  CTRL   = 0x00,
  TORQUE = 0x01,
  OFF    = 0x02,
  BLANK  = 0x03,
  DECAY  = 0x04,
  STALL  = 0x05,
  DRIVE  = 0x06,
  STATUS = 0x07,
};

// Possible arguments to set decay mode.
enum class StepperDecayMode : uint8_t
{
  Slow                = 0b000,
  SlowIncMixedDec     = 0b001,
  Fast                = 0b010,
  Mixed               = 0b011,
  SlowIncAutoMixedDec = 0b100,
  AutoMixed           = 0b101,
};



/*
 * Driver class for DRV8711
 */
class Driver {
    public:
        Driver();

        // set CS pin, acts as stepper motor ID
        void setCSPin(uint8_t cs_pin);

        // write data to a particular address on the bus
        void writeReg(StepperRegAddr address, uint16_t value);

        // reset stepper register settings
        void resetSettings();

        uint16_t readReg(uint8_t address);

        void clearFaults();

        // register address identifiers
        uint16_t ctrl, torque, off, blank, decay, stall, drive;

    private:
        void writeReg(uint8_t address, uint16_t value);
        void selectChip();
        void deselectChip();
        uint16_t transferToSPI(uint16_t value);


        uint8_t cs_pin;
        SPISettings settings;
};



/*
 * Stepper motor class
 */
class Stepper {
  public:
    int revolutions;                 // total revolutions moved by stepper positive or negative since leaving IDLE mode

    Stepper(int stepsIn2pi, float phi_step, int steps_threshold, int max_phi_delta,
            int max_vel, int min_vel, int max_milliamps, int micro_step_size, StepperDecayMode decay_mode);

     // turn stepper until steps + revolution compared to last IDLE mode = 0.
    void unwind(int encoder_data);
    void checkRevolutions(int encoder_data);

    // calculate steps from encoder data pos to desired phi
    int calculateSteps(int encoder_data, int phi_des);

    // control stepper speed to decelerate to stop
    void controlSpeed(int steps);

    // speed setter method (RPM):
    void setSpeedRPM(long whatSpeed);

    // command number of steps (cw/ccw) for each stepper
    void commandStepper(int enc_pos, int phi_des);

    // setup driver communication
    void setupDriver(int cs_pin);

    // get motor direction of rotation
    bool getDirection();
    void checkDirection(int steps);

    uint8_t readStatus();

    // instantiate stepper motor driver
    Driver driver;

    // indicate if phi des and phi curr are above phi_threshold, stop DC motors
    bool phi_flag;

  private:
    void step(int steps_to_move);
    void setStepMode(uint16_t mode);
    void setDecayMode(StepperDecayMode mode);
    void setMaxCurrent(uint16_t current);
    void setDirection(bool value);
    void enableDriver();

    int direction;                  // tracks motor rotation direction
    long step_delay;       // delay between steps, in us, based on speed
    long last_step_time;   // time stamp in us of when the last step was taken
    int stepsIn2pi;                 // total number of steps this motor can take
    int micro_step_size;            // micro step size: 1,2,4,...,256
    float phi_step;                 // degrees per stepper step (deg/step)
    int steps_threshold;            // number of steps to begin deceleration to a stop (step)
    int max_phi_delta;              // number of steps between des and curr to stop DCs
    int max_vel;                    // max continous veloicty for stepper rotation (steps/s)
    int min_vel;                    // min velocity which stepper will decelerate to (steps/s)
    int max_milliamps;              // max current stepper can draw through driver (mA)
    int prev_encoder_data;
    StepperDecayMode decay_mode;    // decay mode on PWM signals
};



/*
 * Encoder class for AMT21 encoder
 */
class AMTEncoder {
    public:
        AMTEncoder(int Re, int De);
        void RS485Transmit();           // sets the 'data enable' pin high and 'receive enable' pin low
        void RS485Receive();            // sets the 'receive enable' pin high and 'data enable' pin low
        int checkEncoder(int address);  // checks position of desired encoder

    private:
        long response;                  // output 12 bit number from encoder, goes from 0 to 4095 depending on shaft position
        int byteOut;
        uint8_t byteIn[3];
        int i;
        int Re, De;                      // Data and Receive enable pins
};


#endif
