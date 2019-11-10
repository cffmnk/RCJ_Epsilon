#include "config.h"
#include "math.h"
#include <MedianFilterLib.h>

#include <Pixy2.h>

#if GYRO_TYPE == 2
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define INTERRUPT_PIN 21  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
#endif // GYRO_TYPE == 2

Pixy2 pixy;

MedianFilter<float> gyroFilter(3);

short speed = 0;
float dir = 0;
float target  = 0;
float yaw = 0;
float yaw_offset = 0;

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
short line_return = 1;

short ball_id = -1;
short target_id = -1;
short home_id = -1;
float ball_dist = 1000;
float home_dist = 1000;
float target_dist = 1000;
float ball_dir = 0;
float home_dir = 0;
float target_dir = 0;

uint32_t ball_t = 0, target_t = 0, home_t = 0;

bool turn_on_yaw = false;

uint32_t ball_found;

uint32_t gyro_timer = 0;
float timeStep = 0.02;

uint32_t cam_timer = 0;
uint32_t timer = 0;
byte arc_start = 0;

template <typename T>
T sign(T value) {
  return T((value > 0) - (value < 0));
}

double constrainAngle(double x) {
  x = fmod(x + PI, 2 * PI);
  if (x < 0)
    x += 2 * PI;
  return x - PI;
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
  if (target > PI / 2)
    speed = 0;
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
    line_state[2] |= (analogRead(A2) > 600);
    line_state[3] |= (analogRead(A3) > WHITE_LINE);
  }
  // Serial.println(analogRead(A0));
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

void updateIMU() {
#if GYRO_TYPE == 1

  int buffer_size = Serial3.available();
  float imu_angle = 0;
  if (buffer_size) {
    for (int i = 0; i < buffer_size; ++i) {
      imu_angle = Serial3.read();
    }
    imu_angle = (imu_angle - 126) * PI / 180;
    if (btn_left && btn_right) {
      yaw_offset = imu_angle;
    }
    yaw = gyroFilter.AddValue((constrainAngle(imu_angle - yaw_offset)));
  }
#endif // GYRO_TYPE == 1
#if GYRO_TYPE == 2
  if (!dmpReady) {
    yaw = 0;
  } else {
    while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop
        fifoCount = mpu.getFIFOCount();
      }
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      if (btn_left && btn_right) {
        yaw_offset = ypr[0];
      }
      yaw = -(constrainAngle(ypr[0] - yaw_offset));
    }
  }
#endif // GYRO_TYPE == 2
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

    float ang = constrainAngle(ball_dir - yaw);

    dir = calcAngle(ball_id);
    //Serial.println(dist);
    if (ball_dist < 75) {
      if (abs(dir) < PI / 2) {
        if (ball_dist < 50 && abs(dir) < PI / 4)
          dir *= 4;
        else
          dir *= 2;
      } else {
        speed = BOOST_SPEED;
        dir = home;
      }
    }

    if (abs(ang) > 2.6 && ball_dist < 55) {
      dir = ang + PI / 2;
      setLED(CENTER_LED, HIGH);
    } else {
      setLED(CENTER_LED, LOW);
    }

    if (x > 180 && x < 200 && y > 115 && y < 145) {
      speed = BOOST_SPEED;
      setLED(CENTER_LED, HIGH);
    } else {
      setLED(CENTER_LED, LOW);
    }
  } else {
    pixy.setLED(0, 0, 0);
    if (millis() - ball_found > 500) {
      dir = home;
      speed = BOOST_SPEED;
    }
  }
}

void startMenu() {
  target = 0;
  speed = 0;
  move();
  while (!btn_right && !btn_left && !btn_center) checkButtons();

  while (42) {
    checkButtons();
    if (btn_left) {
      arc_start = 2;
      break;
    }

    if (btn_right) {
      arc_start = 3;
      break;
    }

    if (btn_center) {
      arc_start = 1;
      break;
    }
  }
  setLED(CENTER_LED, HIGH);
  delay(1000);
  checkButtons();
  while (!btn_center) checkButtons();
  while (btn_center) checkButtons();
  setLED(CENTER_LED, LOW);
  timer = millis();
  arcStart();
}

void arcStart() {
  while (millis() - timer < 400) {
    updateIMU();
    target = yaw;
    if (arc_start == 2) {
      dir = PI / 6;
    }
    if (arc_start == 3) {
      dir = -PI / 6;
    }
    speed = 360;
    move();
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
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(199);
  mpu.setYGyroOffset(-17);
  mpu.setZGyroOffset(-9);
  mpu.setZAccelOffset(3989);

  if (!devStatus) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
#endif // GYRO_TYPE == 2
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

  // startMenu();
}

void loop() {

  speed = BASIC_SPEED;
  line_found = checkLine();
  checkButtons();
  updateIMU();
  target = yaw;

  ball_id = -1;
  target_id = -1;
  home_id = -1;

  pixy.ccc.getBlocks(true, CCC_SIG1 | CCC_SIG2 | CCC_SIG3);
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; ++i) {
      if (pixy.ccc.blocks[i].m_signature == 1 && (ball_id == -1 || pixy.ccc.blocks[i].m_age > pixy.ccc.blocks[ball_id].m_age)) {
        ball_id = i;
        ball_dist = calcDist(ball_id);
        ball_dir = calcAngle(ball_id);
        ball_t = millis();
      }
      if (pixy.ccc.blocks[i].m_signature == ENEMY_GOAL) {
        target_id = i;
        target_dist = calcDist(target_id);
        target_dir = calcAngle(target_id);
        target = target_dir;
        target_t = millis();
      }
      if (pixy.ccc.blocks[i].m_signature == HOME_GOAL && (home_id == -1 || pixy.ccc.blocks[i].m_height * pixy.ccc.blocks[i].m_width > pixy.ccc.blocks[home_id].m_height * pixy.ccc.blocks[i].m_width)) {
        home_id = i;
        home_dist = calcDist(home_id);
        home_dir = calcAngle(home_id);
        home_t = millis();
      }
    }
  }

  if (line_found) {
    robot_on_line = true;
    line_timer = millis();
  } else {
    if (millis() - line_timer > line_return) {
      robot_on_line = false;
      line_return = 1;
    }
  }

  if (!robot_on_line) {
    followBall();
    setLED(LEFT_LED, LOW);
  } else {
    setLED(LEFT_LED, HIGH);
    int block_id = -1;
    if (home_id >= 0 && calcDist(home_id) > 50) {
      block_id = home_id;
    } else if (target_id >= 0 && calcDist(target_id) > 83) {
      block_id = target_id;
    }

    if (block_id >= 0) {
      int x = pixy.ccc.blocks[block_id].m_x;
      int y = pixy.ccc.blocks[block_id].m_y;
      dir = calcAngle(block_id);
    } else {
      speed = 300;
      line_return = 300;
      dir = PI;
    }
  }

  if (home_id >= 0 && home_dist < 80 && (abs(dir) > abs(yaw) + 2) && !robot_on_line) {
    speed = 0;
    target = yaw;
    pixy.setLED(255, 255, 255);
  }
  Serial.println(yaw);
  move();
  //    if (btn_left && btn_right && btn_center)
  //      startMenu();
}
