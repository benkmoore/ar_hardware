
class DC_Motors{
  public:

    DC_Motors(int* reverseFlags, int* DC_reverse, int* DC_throttlepins);

    void PowerDC(int PWMspeed, int aPin);

    void flipDirection(int reversePin);

  private:
    int *reverse;
    int *throttlePins;
    int *reverseFlags;

};