class Robot {
  public:
    short speed;
    float target;
    float dir;
    float yaw;
    bool line_state[4];

  private:
    const byte motors_pwm[4] = {2, 12, 8, 6};
    const byte motors_in1[4] = {36, 42, 44, 34};
    const byte motors_in2[4] = {38, 40, 46, 32};

    void init();
    void setSpeed(byte port, short motor_speed);
    void move();
    void setLED(byte port, bool state);
    bool checkLine();
    bool btnLeft();
    bool btnRight();
    bool btnCenter();
    float updateIMU();
}
