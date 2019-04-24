#include "config.h"
#include <math.h>
#include <Pixy2.h>
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

//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

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

Pixy2 pixy;

unsigned long gyroTimer = 0;
float timeStep = 0.015;
float yaw = 0;
short speed = 0;
float dir = 0;
float heading = 0;
float yaw_offset = 0;

bool buttonLeft = false;
bool buttonCenter = false;
bool buttonRight = false;

const byte motors_pwm[4] = {2, 12, 8, 6};
const byte motors_in1[4] = {36, 42, 44, 34};
const byte motors_in2[4] = {38, 40, 46, 32};

bool lineState[4];
bool lineFound = false;
uint32_t line_t = millis();

bool ledLeft, ledRight, ledCenter;

uint32_t ball_t = 0, target_t = 0, home_t = 0;
bool ballFound = false, targetFound = false, homeFound = false;

camBlock ball, target, home;

byte lineSide = -1;

template <typename T>
T sign(T value) {
  return T((value > 0) - (value < 0));
}

template <typename T>
T trunc(T number, T low, T high) {
  return max(low, min(number, high));
}

double constrainAngle(double x) {
  x = fmod(x + PI, 2 * PI);
  if (x < 0)
    x += 2 * PI;
  return x - PI;
}

float calcAngle(short block_id) {
  if (block_id >= 0) {
    int x = pixy.ccc.blocks[block_id].m_x;
    int y = pixy.ccc.blocks[block_id].m_y;
    return (-acos((x - CAM_CENTER_X) / sqrt(pow(y - CAM_CENTER_Y, 2) + pow(x - CAM_CENTER_X, 2))) * (1 - 2 * (y > CAM_CENTER_Y)));
  }
  return 0;
}

float calcAngle(int x, int y) {
  return (-acos((x - CAM_CENTER_X) / sqrt(pow(y - CAM_CENTER_Y, 2) + pow(x - CAM_CENTER_X, 2))) * (1 - 2 * (y > CAM_CENTER_Y)));
}

float calcDist(short block_id) {
  if (block_id >= 0) {
    int x = pixy.ccc.blocks[block_id].m_x;
    int y = pixy.ccc.blocks[block_id].m_y;
    return sqrt(pow(x - CAM_CENTER_X, 2) + pow(y - CAM_CENTER_Y, 2));
  }
  return 0;
}

bool checkTimer(uint32_t timer, int dt) {
  return (millis() - timer < dt);
}

void updateGyro() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
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
    Serial.println(F("FIFO overflow!"));

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

    if (buttonLeft && buttonRight)
      yaw_offset = ypr[0];
    yaw = -constrainAngle(ypr[0] - yaw_offset);
  }
}

void checkButtons() {
  buttonLeft = digitalRead(LEFT_BTN);
  buttonCenter = digitalRead(CENTER_BTN);
  buttonRight = digitalRead(RIGHT_BTN);
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
  int u = heading * K_YAW;
  if (heading > PI / 2)
    speed = 0;
  setSpeed(MOTOR_A,  -speed * cos((dir + 0.785398163397448)) + u);
  setSpeed(MOTOR_B,   speed * cos((dir - 0.785398163397448)) + u);
  setSpeed(MOTOR_C,  -speed * cos((dir + 0.785398163397448)) - u);
  setSpeed(MOTOR_D,  -speed * cos((dir - 0.785398163397448)) + u);
}

void setLED(byte port, bool state) {
  digitalWrite(port, state);
}

bool updateLights() {
  for (int i = 0; i < 4; ++i) {
    lineState[i] = false;
  }

  for (int i = 0; i < 4; ++i) {
    digitalWrite(MPL_1, i & 1);
    digitalWrite(MPL_2, i >> 1);
    lineState[0] |= (analogRead(A0) > WHITE_LINE);
    lineState[1] |= (analogRead(A1) > WHITE_LINE);
    lineState[2] |= (analogRead(A2) > WHITE_LINE);
    lineState[3] |= (analogRead(A3) > WHITE_LINE);
  }

  return lineState[0] | lineState[1] | lineState[2] | lineState[3];
}

void updateCamera() {
  int ball_id = -1;
  int target_id = -1;
  int home_id = -1;

  homeFound = false;
  targetFound = false;
  homeFound = false;

  pixy.ccc.getBlocks(false, CCC_SIG1 | CCC_SIG2 | CCC_SIG3);
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; ++i) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        ball_id = i;
        ball.dist = calcDist(ball_id);
        ball.dir = calcAngle(ball_id);
        ballFound = true;
        ball.x = pixy.ccc.blocks[ball_id].m_x;
        ball.y = pixy.ccc.blocks[ball_id].m_y;
        ball.width = pixy.ccc.blocks[ball_id].m_width;
        ball.height = pixy.ccc.blocks[ball_id].m_height;
        ball_t = millis();
      }
      if (pixy.ccc.blocks[i].m_signature == ENEMY_GOAL) {
        target_id = i;
        target.dist = calcDist(target_id);
        target.dir = calcAngle(target_id);
        targetFound = true;
        target.x = pixy.ccc.blocks[target_id].m_x;
        target.y = pixy.ccc.blocks[target_id].m_y;
        target.width = pixy.ccc.blocks[target_id].m_width;
        target.height = pixy.ccc.blocks[target_id].m_height;
        target_t = millis();
      }
      if (pixy.ccc.blocks[i].m_signature == HOME_GOAL) {
        home_id = i;
        home.dist = calcDist(home_id);
        home.dir = calcAngle(home_id);
        homeFound = true;
        home.x = pixy.ccc.blocks[home_id].m_x;
        home.y = pixy.ccc.blocks[home_id].m_y;
        home.width = pixy.ccc.blocks[home_id].m_width;
        home.height = pixy.ccc.blocks[home_id].m_height;
        home_t = millis();
      }
    }
  }

  ballFound = checkTimer(ball_t, 600);
  homeFound = checkTimer(home_t, 300);
  targetFound = checkTimer(target_t, 300);
}

void followBall() {
  float homeDir = PI;
  if (homeFound)
    homeDir = home.dir;

  if (ballFound) {
    dir = ball.dir;

    if (ball.dist < 85) {
      if (abs(ball.dir) < PI / 2) {
        dir = ball.dir * 2;
        if (abs(ball.dir) < PI / 4.5 && ball.dist < 65)
          dir = ball.dir * 4;
      } else {
        dir = homeDir;
      }
    }

    float ang = constrainAngle(ball.dir - yaw);
    if (abs(ang) > 2.3 && ball.dist < 60)
      dir = ang + PI / 2 * sign(ang);

    if (ball.x > 195 && ball.x < 204 && ball.y > 122 && ball.y < 138) {
      dir = 0;
      heading = calcAngle(target.x, target.y);
      speed = BOOST_SPEED;
    }
  } else {
    dir = homeDir;
    heading = yaw;
    speed = 300;
  }
}

void setup() {
  setLED(CENTER_LED, HIGH);
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
  pixy.setLED(0, 0, 0);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(199);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(-8);
  mpu.setZAccelOffset(3990); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  setLED(CENTER_LED, LOW);
}

void loop() {
  speed = BASIC_SPEED;
  ledLeft = false;
  ledRight = false;
  ledCenter = false;
  lineFound = updateLights();
  if (lineFound) {
    if (!checkTimer(line_t, 1000)) {
      if (lineState[1])
        lineSide = 1;
      if (lineState[2])
        lineSide = 2;
      if (lineState[3])
        lineSide = 3;
      if (lineState[0])
        lineSide = 0;
    }
    line_t = millis();
  } else {
    if (checkTimer(line_t, 300))
      lineFound = true;
  }
  checkButtons();
  updateCamera();
  updateGyro();
  heading = yaw;
  //
  //  if (targetFound) {
  //    //    if (target.dist < 100) {
  //    //      heading = calcAngle(target.x + target.width / 3, target.y + target.height / 3);
  //    //    }
  //    heading = calcAngle(target.x, target.y);
  //    // heading = calcAngle(target.x, target.y);
  //    //heading = calcAngle(target.x + target.width / 3, target.y + target.height / 3);
  //    // heading = calcAngle(target.x - target.width / 3, target.y - target.height / 3);
  //  }
  followBall();
  if (targetFound && target.dist < 90) {
    heading = target.dir;
  }
    
  if (!lineFound) {
    speed = BASIC_SPEED;
  } else {
    speed = 120;
    if ((homeFound && home.dist > 80 && targetFound && target.dist > 80)) {
      if (lineSide == 0) {
        if (lineState[3]) {
          speed = 60;
          dir = PI / 2;
          heading = yaw;
        }
      }
      if (lineSide == 3) {
        if (lineState[0]) {
          speed = 60;
          dir = - PI / 2;
          heading = yaw;
        }
      }
      if (lineSide == 1 && lineFound) {
        dir = target.dir;
        heading = yaw;
      }
      if (lineSide == 2 && lineFound) {
        dir = home.dir;
        heading = yaw;
      }
    } else {
      if (homeFound) {
        dir = home.dir;
        heading = yaw;
      } else {
        dir = PI;
        heading = yaw;
      }
    }
  }

  if (homeFound && abs(home.dir) > 2 && abs(dir) > 2 && home.dist < 80) {
    heading = yaw;
    speed = 0;
  }

  move();
  setLED(LEFT_LED, ledLeft);
  setLED(RIGHT_LED, ledRight);
  setLED(CENTER_LED, ledCenter);
}
