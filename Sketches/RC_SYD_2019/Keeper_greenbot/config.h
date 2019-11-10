// ***********************
// SETUP AREA
// ***********************

#define BASIC_SPEED 340 // 0 - 360
#define BOOST_SPEED 340 // 0 - 360
#define K_YAW 190 // heading coefficient

#define ENEMY_GOAL 2 // 2 - yellow, 3 - blue

#define WHITE_LINE 950

#define GYRO_TYPE 1 // 1 - sparkfun, 2 - mpu6050

#define CAM_CENTER_X 153
#define CAM_CENTER_Y 128

// ***********************
// END OF SETUP AREA
// ***********************

#define MOTOR_A 0 // cw
#define MOTOR_B 1 // cw
#define MOTOR_C 2 // ccw
#define MOTOR_D 3 // cw
#define L_STANDBY 4
#define R_STANDBY 10
#define RIGHT_BTN 5
#define CENTER_BTN 9
#define LEFT_BTN 13
#define RIGHT_LED 3
#define CENTER_LED 7
#define LEFT_LED 11
#define MPL_1 33
#define MPL_2 35
#define RSIDE 1
#define BSIDE 2
#define FSIDE 3
#define LSIDE 4

#if GYRO_TYPE == 1
#define GYRO_CALIB_TIME 9000
#endif // GYRO_TYPE == 1
#if GYRO_TYPE == 2
#define GYRO_CALIB_TIME 9000
#endif // GYRO_TYPE == 2

#if ENEMY_GOAL == 3
  #define HOME_GOAL 2 // 2 - yellow, 3 - blue
#endif // ENEMY_GOAL == 3
#if ENEMY_GOAL == 2
  #define HOME_GOAL 3 // 2 - yellow, 3 - blue
#endif // ENEMY_GOAL == 2

template <typename T>
T sign(T value);
void startMenu();
void setSpeed(byte port, short motor_speed);
void move();
void setLED(byte port, bool state);
bool checkLights();
void checkButtons();
float calcAngle(short block_id);
float calcDist(short block_id);
void updateIMU();
void followBall();
