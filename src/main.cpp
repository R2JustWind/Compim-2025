#include <Arduino.h>  //DIR1 = INA  DIR2 = INB
// Roda frontal esquerda
#define RFE_DIR1 22
#define RFE_DIR2 23
#define RFE_PWM 5
// Roda frontal direita
#define RFD_DIR1 24
#define RFD_DIR2 25
#define RFD_PWM 6
// Roda traseira esquerda
#define RTE_DIR1 26
#define RTE_DIR2 27
#define RTE_PWM 9
// Roda traseira direita
#define RTD_DIR1 28
#define RTD_DIR2 29
#define RTD_PWM 10
// Encoder roda frontal esquerda
#define EFE_A 2
#define EFE_B 3


void setMotor(int dir1, int dir2, int pwm, int speed);
void moveForward(int speed);
void moveBackward(int speed);
void moveLeft(int speed);
void moveRight(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop();

void setup() {
  pinMode(RFE_DIR1, OUTPUT);
  pinMode(RFE_DIR2, OUTPUT);
  pinMode(RFE_PWM, OUTPUT);

  pinMode(RFD_PWM, OUTPUT);
  pinMode(RFD_PWM, OUTPUT);
  pinMode(RFD_PWM, OUTPUT);

  pinMode(RTE_PWM, OUTPUT);
  pinMode(RTE_PWM, OUTPUT);
  pinMode(RTE_PWM, OUTPUT);

  pinMode(RTD_PWM, OUTPUT);
  pinMode(RTD_PWM, OUTPUT);
  pinMode(RTD_PWM, OUTPUT);
}

void loop() {
  moveForward(128);
  delay(2000);
}

void setMotor(int dir1, int dir2, int pwm, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(pwm, -speed);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    analogWrite(pwm, 0);
  }
}

void moveForward(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
}

void moveBackward(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
}

void moveLeft(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
}

void moveRight(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
}

void turnLeft(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
}

void turnRight(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
}

void stop() {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, 0);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, 0);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, 0);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, 0);
}
