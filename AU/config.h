#ifndef CONFIG_H
#define CONFIG_H

#define BASIC_SPEED 300
#define SNOOZE_DRIBBLER_SPEED 0
#define BASIC_DRIBBLER_SPEED 255
#define BASIC_K 170

#define ENEMY_GOAL 2

#define WHITE_LINE 300

#define CAM_CENTER_X 166
#define CAM_CENTER_Y 122

#define GYRO_OFFSET_XG 192
#define GYRO_OFFSET_YG -2
#define GYRO_OFFSET_ZG -33
#define GYRO_OFFSET_ZA 531

#define INTERRUPT_PIN 21
#define DRIBBLER_SPEED 3
#define DRIBBLER_DIR 14
#define KICKER 47
#define BALL_SENSOR 43

#define LEFT_LED 11
#define CENTER_LED 7
#define RIGHT_LED 17

#define LEFT_BTN 13
#define CENTER_BTN 9
#define RIGHT_BTN 5

#define MOTOR_A 0 // cw
#define MOTOR_B 1 // cw
#define MOTOR_C 2 // ccw
#define MOTOR_D 3 // cw
#define L_STANDBY 4
#define R_STANDBY 10
#define MPL_1 33
#define MPL_2 35

#if ENEMY_GOAL == 3
#define HOME_GOAL 2 // 2 - yellow, 3 - blue
#endif // ENEMY_GOAL == 3
#if ENEMY_GOAL == 2
#define HOME_GOAL 3 // 2 - yellow, 3 - blue
#endif // ENEMY_GOAL == 2

#define CCC_SIG1 1
#define CCC_SIG2 2
#define CCC_SIG3 4

#include <math.h>
#include <Pixy2.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
Pixy2 pixy;

struct camBlock {
  int x;
  int y;
  int w;
  int h;
  float dir;
  float dist;
  bool found;
  uint32_t t;
};

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

volatile bool mpuInterrupt = false; 

const byte motors_pwm[4] = {2, 12, 8, 6};
const byte motors_in1[4] = {36, 42, 44, 34};
const byte motors_in2[4] = {38, 40, 46, 32};

template <typename T>
T sign(T value);

double constrainAngle(double x);
float calcAngle(int x, int y);
float calcAngle(camBlock b);
float calcDist(camBlock b);
void dribbler(short speed);

#endif
