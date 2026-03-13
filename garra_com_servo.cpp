#include <Arduino.h>
#include <Servo.h>
#include <util/atomic.h>

Servo garraServo;

#define GARRA_DIR1 49
#define GARRA_DIR2 48
#define GARRA_PWM 12

#define SERVO_PIN 44

#define EGARRA_A 20
#define EGARRA_B 37



volatile long pulseCountGarra = 0;

int primeiro_andar = 7200;
int segundo_andar = 14200;
int alvo = 0;

int altura = 0;
int pulses_garra = 0;

long currT;
unsigned long lastTime = 0;
float deltaT;

int SobeGarra(int pulses);
int DesceGarra(int pulses);
void isrEGARRA();

void setup() {

  Serial.begin(9600);

  pinMode(GARRA_DIR1, OUTPUT);
  pinMode(GARRA_DIR2, OUTPUT);
  pinMode(GARRA_PWM, OUTPUT);

  pinMode(EGARRA_A, INPUT_PULLUP);
  pinMode(EGARRA_B, INPUT_PULLUP);

  garraServo.attach(SERVO_PIN);

  attachInterrupt(digitalPinToInterrupt(EGARRA_A), isrEGARRA, RISING);
}

void loop() {
  SobeGarra(primeiro_andar);
  while (pulseCountGarra < primeiro_andar){
    delay(1);
  }
  garraServo.write(80);
  altura += pulses_garra;
  delay(2000);
  garraServo.write(90);
  SobeGarra(segundo_andar);
  while (pulseCountGarra < segundo_andar){
    delay(1);
  }
  garraServo.write(105);
  altura += pulses_garra;
  delay(2000);
  garraServo.write(90);
  DesceGarra(0);
  altura -= pulses_garra;
  delay(1000);
}


int SobeGarra(int pulses) {
  digitalWrite(GARRA_DIR1, HIGH);
  digitalWrite(GARRA_DIR2, LOW);
  analogWrite(GARRA_PWM, 255);

  volatile long count = 0;
  while (count < pulses){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      count = pulseCountGarra;
    }

    Serial.print(count);
    Serial.println(' ');

    lastTime = currT;
  }

  analogWrite(GARRA_PWM, 0);
  pulses_garra = - count;
  return count;
}

int DesceGarra(int pulses) {
  digitalWrite(GARRA_DIR1, LOW);
  digitalWrite(GARRA_DIR2, HIGH);
  analogWrite(GARRA_PWM, 255);

  volatile long count = pulseCountGarra;
  while (count > pulses){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      count = pulseCountGarra;
    }

    Serial.print(count);
    Serial.println(' ');

    lastTime = currT;
  }

  analogWrite(GARRA_PWM, 0);
  pulses_garra = - count;
  return count;
}

void isrEGARRA() {
  if(digitalRead(EGARRA_B) == HIGH) {
    pulseCountGarra--;
  } else {
    pulseCountGarra++;
  }
}