
class DC_Motors{
  public:

    DC_Motors(int* reverseFlags, int* DC_reverse, int* DC_throttlepins);

    void PowerDC(int aPin, int PWMspeed, int index);

    void flipDirection();

  private:
    int *reverse;
    int *throttlePins;
    int *reverseFlags;
    int flip[4] = {0,0,0,0};

};
