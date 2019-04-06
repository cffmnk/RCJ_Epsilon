#include "config.h"
#include <Pixy2.h>

#if GYRO_TYPE == 2
#include "MPU6050.h"
MPU6050 mpu;
#endif

Pixy2 pixy;

short speed = 0;
float dir = 0;
float target  = 0;
float yaw = 0;

bool btn_left = false;
bool btn_center = false;
bool btn_right = false;

const byte motors_pwm[4] = {2, 12, 8, 6};
const byte motors_in1[4] = {36, 42, 44, 34};
const byte motors_in2[4] = {38, 40, 46, 32};

bool line_state[4];
bool line_found = false;
bool robot_on_line = false;
uint32_t line_timer;

short ball_id = -1;
short target_id = -1;
short home_id = -1;

uint32_t ball_found;

uint32_t gyro_timer = 0;
float timeStep = 0.02;

uint32_t timer = 0;
byte arc_start = 0;

template <typename T>
T sign(T value) {
  return T((value > 0) - (value < 0));
}

void setSpeed(byte port, short motor_speed) {
  if (motor_speed > 255)
    motor_speed = 255;
  else if (motor_speed < -255)
    motor_speed = -255;

  analogWrite(motors_pwm[port], abs(motor_speed));
  digitalWrite(motors_in1[port], (motor_speed >= 0));
  digitalWrite(motors_in2[port], !(motor_speed >= 0));
}

void move() {
  int u = target * K_YAW;
  setSpeed(MOTOR_A,  -speed * cos((dir + 0.785398163397448)) + u);
  setSpeed(MOTOR_B,   speed * cos((dir - 0.785398163397448)) + u);
  setSpeed(MOTOR_C,  -speed * cos((dir + 0.785398163397448)) - u);
  setSpeed(MOTOR_D,  -speed * cos((dir - 0.785398163397448)) + u);
}

void setLED(byte port, bool state) {
  digitalWrite(port, state);
}

bool checkLine() {
  /*
     /---------------\ /-------------------------\
     |  side  | port | | sensor  | mpl_1 | mpl_2 |
     |--------|------| |---------|-------|-------|
     |  Right |  A0  | | Central |   0   |   0   |
     |  Back  |  A1  | | Lower   |   0   |   1   |
     |  Front |  A2  | | Left    |   1   |   0   |
     |  Left  |  A3  | | Right   |   1   |   1   |
     \---------------/ \-------------------------/
  */

  for (int i = 0; i < 4; ++i)
    line_state[i] = false;

  for (int i = 0; i < 4; ++i) {
    digitalWrite(MPL_1, i & 1);
    digitalWrite(MPL_2, i >> 1);
    line_state[0] |= (analogRead(A0) > WHITE_LINE);
    line_state[1] |= (analogRead(A1) > WHITE_LINE);
    line_state[2] |= (analogRead(A2) > WHITE_LINE);
    line_state[3] |= (analogRead(A3) > WHITE_LINE);
  }

  return line_state[0] | line_state[1] | line_state[2] | line_state[3];
}

void checkButtons() {
  btn_left = digitalRead(LEFT_BTN);
  btn_center = digitalRead(CENTER_BTN);
  btn_right = digitalRead(RIGHT_BTN);
}

float calcAngle(short block_id) {
  if (block_id >= 0) {
    int x = pixy.ccc.blocks[block_id].m_x;
    int y = pixy.ccc.blocks[block_id].m_y;
    return (-acos((x - CAM_CENTER_X) / sqrt(pow(y - CAM_CENTER_Y, 2) + pow(x - CAM_CENTER_X, 2))) * (1 - 2 * (y > CAM_CENTER_Y)));
  }
  return 0;
}

float calcDist(short block_id) {
  if (block_id >= 0) {
    int x = pixy.ccc.blocks[block_id].m_x;
    int y = pixy.ccc.blocks[block_id].m_y;
    return sqrt(pow(x - CAM_CENTER_X, 2) + pow(y - CAM_CENTER_Y, 2));
  }
  return 0;
}

float updateIMU() {
#if GYRO_TYPE == 1
  if (btn_right && btn_left && btn_center) {
    Serial3.write('c');
  }

  int buffer_size = Serial3.available();
  float imu_angle = 0;
  if (buffer_size) {
    for (int i = 0; i < buffer_size; ++i) {
      imu_angle = Serial3.read();
    }
    imu_angle = (imu_angle - 120) * PI / 180;
  }
  Serial.println(imu_angle);
  return imu_angle;
#endif
#if GYRO_TYPE == 2
  if (btn_right && btn_left)
    yaw = 0;
  gyro_timer = millis();
  VectorMPU norm = mpu.readNormalizeGyro();
  yaw = yaw + DEG_TO_RAD(norm.ZAxis * timeStep);
  return yaw;
#endif
}

void followBall() {
  float home = PI;
  if (home_id >= 0) {
    home = calcAngle(home_id);
  }

  if (ball_id >= 0) {
    pixy.setLED(0, 255, 0);
    ball_found = millis();
    int x = pixy.ccc.blocks[ball_id].m_x;
    int y = pixy.ccc.blocks[ball_id].m_y;

    dir = calcAngle(ball_id);
    float dist = calcDist(ball_id);
    //Serial.println(dist);
    if (dist < 65) {
      if (abs(dir) < PI / 2) {
        dir *= 2;
      } else {
        speed = BOOST_SPEED;
        dir = home;
      }
    }

    if (y > 100 && y < 165 && x < CAM_CENTER_X && dir < 80) {
      dir = (1 - 2 * (dir >= 0)) * PI / 2;
    }

    if (x > 180 && x < 200 && y > 115 && y < 145) {
      speed = BOOST_SPEED;
      setLED(CENTER_LED, HIGH);
    } else {
      setLED(CENTER_LED, LOW);
    }
  } else {
    pixy.setLED(0, 0, 0);
    if (millis() - ball_found > 1000) {
      dir = home;
      speed = BOOST_SPEED;
    }
  }
}

void startMenu() {
  while (42) {
    checkButtons();
    if (btn_left) {
      arc_start = 3;
      break;
    }

    if (btn_right) {
      arc_start = 2;
      break;
    }

    if (btn_center) {
      arc_start = 1;
      break;
    }
  }
  setLED(CENTER_LED, HIGH);
  delay(1000);
  setLED(CENTER_LED, LOW);
  checkButtons();
  while (btn_center) checkButtons();
  timer = millis();
}

void arcStart() {
  while (millis() - timer < 1500) {
    updateIMU();
    dir = 0;
    if (arc_start == 2) {
      target = yaw - PI / 3;
    }
    if (arc_start == 3) {
      target = yaw + PI / 3;
    }
  }
}

void setup() {
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

  pixy.init();
  Serial.begin(115200);
  Serial3.begin(115200);

#if GYRO_TYPE == 2
  //setup GYRO
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro(100);
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
#endif
  setLED(LEFT_LED, true);
  setLED(CENTER_LED, true);
  setLED(RIGHT_LED, true);
  delay(GYRO_CALIB_TIME);
  for (int i = 0; i < 3; ++i) {
    setLED(LEFT_LED, (i % 2));
    setLED(CENTER_LED, (i % 2));
    setLED(RIGHT_LED, (i % 2));
    delay(200);
  }

  pixy.setLED(0, 0, 0);

  startMenu();
}

void loop() {
  speed = BASIC_SPEED;
  line_found = checkLine();
  checkButtons();
  target = updateIMU();
  ball_id = -1;
  target_id = -1;
  home_id = -1;
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; ++i) {
      if (pixy.ccc.blocks[i].m_signature == 1 && (ball_id == -1 || pixy.ccc.blocks[i].m_age > pixy.ccc.blocks[ball_id].m_age))
        ball_id = i;
      if (pixy.ccc.blocks[i].m_signature == ENEMY_GOAL)
        target_id = i;
      if (pixy.ccc.blocks[i].m_signature == HOME_GOAL)
        home_id = i;
    }
  }
  if (target_id >= 0)
    target = calcAngle(target_id);

  if (line_found) {
    robot_on_line = true;
    line_timer = millis();
  } else {
    if (millis() - line_timer > 60) {
      robot_on_line = false;
    }
  }

  if (!robot_on_line) {
    followBall();
    setLED(LEFT_LED, LOW);
  } else {
    setLED(LEFT_LED, HIGH);
    int block_id = -1;
    if (home_id >= 0) {
      block_id = home_id;
    } else if (target_id >= 0) {
      block_id = target_id;
    }

    if (block_id >= 0) {
      int x = pixy.ccc.blocks[block_id].m_x;
      int y = pixy.ccc.blocks[block_id].m_y;
      dir = calcAngle(block_id);
    } else {
      target = yaw;
      dir = PI;
      speed = 200;
    }
  }
  move();
  delay((timeStep * 1000) - (millis() - gyro_timer));
}
