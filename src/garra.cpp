#include <Arduino.h>
#include <util/atomic.h>

#define GARRA_DIR1 49
#define GARRA_DIR2 48
#define GARRA_PWM 12

void setup() {
  pinMode(GARRA_DIR1, OUTPUT);
  pinMode(GARRA_DIR2, OUTPUT);
  pinMode(GARRA_PWM, OUTPUT);

}

void loop() {

  digitalWrite(GARRA_DIR1, HIGH);
  digitalWrite(GARRA_DIR2, LOW);
  analogWrite(GARRA_PWM, 150);

  delay(3000);

  // trás
  digitalWrite(GARRA_DIR1, LOW);
  digitalWrite(GARRA_DIR2, HIGH);
  analogWrite(GARRA_PWM, 150);

  delay(3000);

  // parar
  digitalWrite(GARRA_DIR1, LOW);
  digitalWrite(GARRA_DIR2, LOW);
  analogWrite(GARRA_PWM, 0);

  delay(3000);
}
