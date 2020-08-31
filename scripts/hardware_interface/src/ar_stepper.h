
// ensure this library description is only included once
#ifndef Stepper_h
#define Stepper_h

// library interface description
class Stepper {
  public:
    // constructors:
    Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, float phi_step, int steps_threshold, int max_vel, int min_vel);	

    // speed setter method:
    void setSpeed(long whatSpeed);

    // mover method:
    void step(int number_of_steps);

    // calculate steps from encoder data pos to desired phi
    int calculateSteps(int encoder_data, int phi_des);

    // Set stepper speed to decelerate to stop
    void setStepperSpeed(int steps);

    //Command number of steps (cw/ccw) for each stepper
    void commandStepper(int enc_pos, int phi_des);

    int version(void);

  private:
    void stepMotor(int this_step);

    int direction;            // Direction of rotation
    unsigned long step_delay; // delay between steps, in ms, based on speed
    int number_of_steps;      // total number of steps this motor can take
    int pin_count;            // how many pins are in use.
    int step_number;          // which step the motor is on
    float phi_step;           // degrees per stepper step (deg/step)
    int steps_threshold;      // number of steps to begin deceleration to a stop (step)
    int max_vel;              // max continous veloicty for stepper rotation (steps/s)
    int min_vel;              // min velocity which stepper will decelerate to (steps/s)

    // motor pin numbers:
    int motor_pin_1;
    int motor_pin_2;
    int motor_pin_3;
    int motor_pin_4;
    int motor_pin_5;          // Only 5 phase motor

    unsigned long last_step_time; // time stamp in us of when the last step was taken
};

#endif

