#define MPL_1 33
#define MPL_2 35
#define MOTOR_A 0 // cw
#define MOTOR_B 1 // cw
#define MOTOR_C 2 // ccw
#define MOTOR_D 3 // cw
#define L_STANDBY 4
#define R_STANDBY 10
#define K_YAW 200

int lights_min[4][4];
int lights_max[4][4];

float heading = 0;
int speed = 0;
float dir = 0;


const byte motors_pwm[4] = {2, 12, 8, 6};
const byte motors_in1[4] = {36, 42, 44, 34};
const byte motors_in2[4] = {38, 40, 46, 32};

uint32_t timer;

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

  pinMode(MPL_1, OUTPUT);
  pinMode(MPL_2, OUTPUT);

  Serial.begin(115200);
  delay(1000);
  timer = millis();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      lights_min[j][i] = 1e3;
      lights_max[j][i] = 0;
    }
  }
  pinMode(13, INPUT_PULLUP);

  while (!digitalRead(13)){}
  while (digitalRead(13)){}

  timer = millis();
}


void loop() {
  speed = 0;
  heading = 0.7;
  move();
  for (int i = 0; i < 4; ++i) {
    digitalWrite(MPL_1, i & 1);
    digitalWrite(MPL_2, i >> 1);
    while (true) {
      lights_min[0][i] = min(analogRead(A0), lights_min[0][i]);
      lights_max[0][i] = max(analogRead(A0), lights_max[0][i]);
      lights_min[1][i] = min(analogRead(A1), lights_min[1][i]);
      lights_max[1][i] = max(analogRead(A1), lights_max[1][i]);
      lights_min[2][i] = min(analogRead(A2), lights_min[2][i]);
      lights_max[2][i] = max(analogRead(A2), lights_max[2][i]);
      lights_min[3][i] = min(analogRead(A3), lights_min[3][i]);
      lights_max[3][i] = max(analogRead(A3), lights_max[3][i]);
      if (millis() - timer > 1000)
        break;
    }

    timer = millis();
  }

  speed = 0;
  heading = 0;
  move();

  for (int i = 0; i < 4; ++i) {
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < 4; ++j) {
      Serial.print(lights_min[i][j]);
      Serial.print('/');
      Serial.print(lights_max[i][j]);
      Serial.print(' ');
    }
    Serial.print('\n');
  }
  while (1){}
}
