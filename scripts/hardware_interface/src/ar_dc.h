
class DC_Motors{
  public:

    DC_Motors(int* reverseFlags, int* DC_reverse, int* DC_throttlepins, int N_DCMotors, int* flip);
    void PowerDC(int aPin, int PWMspeed, int index);
    void flipDirection();    
    int flipFlag = 0;
    int N_DCMotors;
   
  private:
    int *flip;
    int *reverse;
    int *throttlePins;
    int *reverseFlags;
};
