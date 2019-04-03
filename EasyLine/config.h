#define BASIC_SPEED 200 // 0 - 360
#define BOOST_SPEED 300 // 0 - 360
#define K_YAW 150

#define ENEMY_GOAL 3  // 2 - yellow, 3 - blue
#define HOME_GOAL 2 // 2 - yellow, 3 - blue

#define WHITE_LINE 800

#define CAM_CENTER_X 160
#define CAM_CENTER_Y 130

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

void setSpeed(byte port, short motor_speed);
void move();
void setLED(byte port, bool state);
bool checkLights();
void checkButtons();
float calcAngle(short block_id);
float updateIMU();
void follow_ball();
