#include <Arduino.h>
// Roda frontal esquerda
#define RFE_DIR1 2
#define RFE_DIR2 3
#define RFE_PWM 5
// Roda frontal direita
#define RFD_DIR1 4
#define RFD_DIR2 7
#define RFD_PWM 6
// Roda traseira esquerda
#define RTE_DIR1 8
#define RTE_DIR2 9
#define RTE_PWM 10
// Roda traseira direita
#define RTD_DIR1 12
#define RTD_DIR2 13
#define RTD_PWM 11


int setMotor(int dir1, int dir2, int pwm, int speed);

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
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, 200);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, 200);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, 200);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, 200);
  delay(2000);
}


int setMotor(int dir1, int dir2, int pwm, int speed) {
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