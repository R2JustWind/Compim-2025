#include <Arduino.h> 
//DIR1 = INA  DIR2 = INB
// Driver1A - Roda frontal direita - M1
#define RFD_DIR1 22
#define RFD_DIR2 23
#define RFD_PWM 4
// Driver1B - Roda frontal esquerda - M2
#define RFE_DIR1 24
#define RFE_DIR2 25
#define RFE_PWM 7
// Driver2A - Roda traseira direita - M3
#define RTD_DIR1 28
#define RTD_DIR2 29
#define RTD_PWM 10
// Driver2B - Roda traseira esquerda - M4
#define RTE_DIR1 26
#define RTE_DIR2 27 
#define RTE_PWM 13
// Encoder roda frontal esquerda
#define EFE_A 2
#define EFE_B 3

// Sensores IR
#define IR_E A0 // Esquerdo
#define IR_C A1 // Centro
#define IR_D A2 // Direito

#define THRESHOLD 90 // Limiar para detecção de linha
#define THRESHOLD_CENTER 40
// THRESHOLD = (preto + branco) / 2;

#define BASE_SPEED 35 // Velocidade base
#define CORRECTION 35 // Correção lateral (metade da velocidade base)

int readLine(int pin);

void setMotor(int dir1, int dir2, int pwm, int speed);
void moveForward(int speed);
void moveBackward(int speed);
void moveLeft(int speed);
void moveRight(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stop();

void setup() {
  Serial.begin(9600);

  // Motores
  pinMode(RFE_DIR1, OUTPUT);
  pinMode(RFE_DIR2, OUTPUT);
  pinMode(RFE_PWM, OUTPUT);

  pinMode(RFD_DIR1, OUTPUT);
  pinMode(RFD_DIR2, OUTPUT);
  pinMode(RFD_PWM, OUTPUT);

  pinMode(RTE_DIR1, OUTPUT);
  pinMode(RTE_DIR2, OUTPUT);
  pinMode(RTE_PWM, OUTPUT);

  pinMode(RTD_DIR1, OUTPUT);
  pinMode(RTD_DIR2, OUTPUT);
  pinMode(RTD_PWM, OUTPUT);

  // Sensores IR
  pinMode(IR_E, INPUT);
  pinMode(IR_C, INPUT);
  pinMode(IR_D, INPUT);
}

void loop() {
  // Sensores IR de teste
  int sE = readLine(IR_E); //Sensor esquerdo
  int sC = readLine(IR_C); //Sensor centro
  int sD = readLine(IR_D); //Sensor direito

  // Linha no centro → segue reto
  if (sC == HIGH && sE == LOW && sD == LOW) {
    moveForward(BASE_SPEED);
  }
  // Linha puxando para esquerda → corrige esquerda
  else if (sE == HIGH && sC == LOW && sD == LOW) {
    moveLeft(CORRECTION);
  }
  // Linha puxando para direita → corrige direita
  else if (sD == HIGH && sC == LOW && sE == LOW) {
    moveRight(CORRECTION);
  }
  // Centro + lado → curva suave
  else if (sE == HIGH && sC == HIGH && sD == LOW) {
    turnLeft(CORRECTION);
  }
  else if (sD == HIGH && sC == HIGH && sE == LOW) {
    turnRight(CORRECTION);
  }
  // Linha perdida
  else if (sE == LOW && sC == LOW && sD == LOW) {
    stop();
  }

  Serial.print("E: ");
  Serial.println(analogRead(IR_E));
  Serial.print(" C: ");
  Serial.println(analogRead(IR_C));
  Serial.print(" D: ");
  Serial.println(analogRead(IR_D));
}

int readLine(int pin) {
  int value = analogRead(pin);

  if(pin == IR_C) {
    if (value > THRESHOLD_CENTER) {
      return 1;   // linha amarela (preto)
    } else {
      return 0;   // fundo cinza (branco)
    }
  } else {
    if (value > THRESHOLD) {
        return 1;   // linha amarela (preto)
      } else {
        return 0;   // fundo cinza (branco)
      }
    }
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

void moveBackward(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
}

void moveForward(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
}

void moveLeft(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
}

void moveRight(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
}

void turnLeft(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
}

void turnRight(int speed) {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
}

void stop() {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, 0);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, 0);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, 0);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, 0);
}
