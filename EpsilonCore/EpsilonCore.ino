#include "EpsilonCore.h"

class Robot {
  public:
    Robot() {
      speed = 0;
      target = 0;
      dir = 0;
      yaw = 0;
      for (int i = 0; i < 4; ++i)
        line_state = false;
    }

    void init() {
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

    bool btnLeft() {
      return digitalRead(LEFT_BTN);
    }

    bool btnRight() {
      return digitalRead(RIGHT_BTN);
    }

    bool btnCenter() {
      return digitalRead(CENTER_BTN);
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
};
