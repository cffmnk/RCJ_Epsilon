#define BASIC_SPEED 200// 0 - 360
#define BOOST_SPEED 300 // 0 - 360
#define K_YAW 150

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

void initHardware();
void setSpeed();
void move();
void set_led();
void follow_ball();
void set_target(int target_id);

/*
    if (abs(dir) < PI / 2) {
      dir *= 2;
    } else if (x > 90 && x < 118 && y > 100 && y < 166) {
      dir = (1 - 2 * (dir >= 0)) * PI / 2;
    } else {
      speed = BOOST_SPEED;
      dir = PI;
    }*/
/*
    if (y > 115 && y < 145) {
      if (x > 180 && x < 200) {
        speed = BOOST_SPEED;
        set_led(CENTER_LED, HIGH);
      } else {
        set_led(CENTER_LED, LOW);
      }
    }
  } else {
    pixy.setLED(0, 0, 0);
    if (millis() - ball_found > 1000)
      dir = PI;
    speed = BOOST_SPEED;
  }*/
