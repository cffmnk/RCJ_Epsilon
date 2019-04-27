#define BASIC_SPEED 340 // 0 - 360
#define BOOST_SPEED 340 // 0 - 360
#define K_YAW 200 // heading coefficient

#define ENEMY_GOAL 2   // 2 - yellow, 3 - blue

#define WHITE_LINE 300

#define CAM_CENTER_X 155
#define CAM_CENTER_Y 123

#define GYRO_CALIB_TIME 5000
#define GYRO_OFFSET_XG 199
#define GYRO_OFFSET_YG -9
#define GYRO_OFFSET_ZG -8
#define GYRO_OFFSET_ZA 3990

#define MOTOR_A 0 // cw
#define MOTOR_B 1 // cw
#define MOTOR_C 2 // ccw
#define MOTOR_D 3 // cw
#define KICKER 48
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

#define CCC_SIG1 1 
#define CCC_SIG2 2
#define CCC_SIG3 4

#if ENEMY_GOAL == 3
  #define HOME_GOAL 2 // 2 - yellow, 3 - blue
#endif // ENEMY_GOAL == 3
#if ENEMY_GOAL == 2
  #define HOME_GOAL 3 // 2 - yellow, 3 - blue
#endif // ENEMY_GOAL == 2


struct camBlock {
  int x;
  int y;
  int width;
  int height;
  float dir;
  float dist;
};
