#include "config.h"
#include "Pixy2.h"

Pixy2 pixy;

short speed;
double dir = 0;
double target = 0;
double imu_angle = 0;

bool btn_left = false;
bool btn_center = false;
bool btn_right = false;

const byte motors_pwm[4] = {2, 12, 8, 6};
const byte motors_in1[4] = {36, 42, 44, 34};
const byte motors_in2[4] = {38, 40, 46, 32};

bool line_state[4];

uint32_t ball_found;

void move() {
  int u = target * K_YAW;

  set_speed(MOTOR_A,  -speed * cos((dir + 0.785398163397448)) + u);
  set_speed(MOTOR_B,   speed * cos((dir - 0.785398163397448)) + u);
  set_speed(MOTOR_C,  -speed * cos((dir + 0.785398163397448)) - u);
  set_speed(MOTOR_D,  -speed * cos((dir - 0.785398163397448)) + u);
}

void set_speed(byte port, short motor_speed) {
  if (motor_speed > 255)
    motor_speed = 255;
  else if (motor_speed < -255)
    motor_speed = -255;

  analogWrite(motors_pwm[port], abs(motor_speed));
  digitalWrite(motors_in1[port], (motor_speed >= 0));
  digitalWrite(motors_in2[port], !(motor_speed >= 0));
}

void set_led(byte port, bool state) {
  digitalWrite(port, state);
}

void check_light() {
  /*
     /------------------------\ /-------------------------\
     |  side  | port | return | | sensor  | mpl_1 | mpl_2 |
     |--------|------|--------| |---------|-------|-------|
     |  None  |  --  |   0    | | Central |   0   |   0   |
     |  Right |  A0  |   1    | | Lower   |   0   |   1   |
     |  Back  |  A1  | . 2    | | Left    |   1   |   0   |
     |  Front |  A2  | . 3    | | Right   |   1   |   1   |
     |  Left  |  A3  |   4    | \-------------------------/
     \------------------------/
  */

  for (int i = 0; i < 4; ++i) {
    digitalWrite(MPL_1, i & 1);
    digitalWrite(MPL_2, i >> 1);
    line_state[0] = (analogRead(A0) > WHITE_LINE);
    line_state[1] = (analogRead(A1) > WHITE_LINE);
    line_state[2] = (analogRead(A2) > WHITE_LINE);
    line_state[3] = (analogRead(A3) > WHITE_LINE);
  }
}

void initHardware() {
  // setup motors
  pinMode(L_STANDBY, OUTPUT);
  digitalWrite(L_STANDBY, HIGH);
  pinMode(R_STANDBY, OUTPUT);
  digitalWrite(R_STANDBY, HIGH);
  for (int i = 0; i < 4; ++i) {
    pinMode(motors_pwm[i], OUTPUT);
    pinMode(motors_in1[i], OUTPUT);
    pinMode(motors_in2[i], OUTPUT);
  }

  // setup light sensors
  pinMode(MPL_1, OUTPUT);
  pinMode(MPL_2, OUTPUT);

  // setup buttons
  pinMode(RIGHT_BTN, INPUT_PULLUP);
  pinMode(CENTER_BTN, INPUT_PULLUP);
  pinMode(LEFT_BTN, INPUT_PULLUP);

  // setup LEDs
  pinMode(RIGHT_LED, OUTPUT);
  pinMode(CENTER_LED, OUTPUT);
  pinMode(LEFT_LED, OUTPUT);
}

void follow_ball(int ball_id, int target_id, int home_id) {
  //  int home = PI;
  //  if (home_id >= 0) {
  //    int home_x = pixy.ccc.blocks[home_id].m_x;
  //    int home_y = pixy.ccc.blocks[home_id].m_y;
  //    home = -acos((home_x - CAM_CENTER_X) / sqrt(pow((home_y - CAM_CENTER_Y), 2) + pow((home_x - CAM_CENTER_X), 2))) * (1 - 2 * (home_y > CAM_CENTER_Y));
  //  }

  if (ball_id >= 0) {
    pixy.setLED(0, 255, 0);
    ball_found = millis();
    int x = pixy.ccc.blocks[ball_id].m_x;
    int y = pixy.ccc.blocks[ball_id].m_y;
    dir = -acos((x - CAM_CENTER_X) / sqrt(pow(y - CAM_CENTER_Y, 2) + pow(x - CAM_CENTER_X, 2))) * (1 - 2 * (y > CAM_CENTER_Y));

    if (abs(dir) < PI / 6) {
      dir *= 3;
    } else if (abs(dir) < PI / 2) {
      dir *= 2;
    } else {
      if (x > 90 && x < 118 && y > 100 && y < 166) {
        dir = (1 - 2 * (dir >= 0)) * PI / 2;
      } else {
        speed = BOOST_SPEED;
        dir = PI;
      }
    }

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
  }
}

void setup() {
  speed = 0;
  dir = 0;
  target = 0;

  initHardware();

  pixy.init();

  Serial.begin(115200);
  Serial3.begin(115200);
  delay(10000);
  Serial3.write('c');

  pixy.setLED(0, 0, 0);
}

void set_target(int target_id) {
  if (target_id >= 0) {
    int target_x = pixy.ccc.blocks[target_id].m_x;
    int target_y = pixy.ccc.blocks[target_id].m_y;
    target = -acos((target_x - CAM_CENTER_X) / sqrt(pow((target_y - CAM_CENTER_Y), 2) + pow((target_x - CAM_CENTER_X), 2))) * (1 - 2 * (target_y > CAM_CENTER_Y));
  }
}

void check_btn() {
  btn_left = digitalRead(LEFT_BTN);
  btn_center = digitalRead(CENTER_BTN);
  btn_right = digitalRead(RIGHT_BTN);
}

void loop() {
  check_btn();
  speed = BASIC_SPEED;
  if (btn_right) {
    Serial3.write('c');
  }
  int buffer_size = Serial3.available();
  if (buffer_size) {
    for (int i = 0; i < buffer_size; ++i)
      imu_angle = (Serial3.read() - 120) * PI / 180;
    //Serial.println(imu_angle);
  }
  target = imu_angle;
  pixy.ccc.getBlocks();
  int ball_id = -1;
  int target_id = -1;
  int home_id = -1;
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; ++i) {
      if (pixy.ccc.blocks[i].m_signature == 1 && (ball_id == -1 || pixy.ccc.blocks[i].m_age > pixy.ccc.blocks[ball_id].m_age)) {
        ball_id = i;
      }
      if (pixy.ccc.blocks[i].m_signature == 3) {
        target_id = i;
      }
      if (pixy.ccc.blocks[i].m_signature == 2) {
        home_id = i;
      }
    }
  }

      set_target(target_id);
  follow_ball(ball_id, target_id, home_id);
  move();
}
