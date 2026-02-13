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
#define RTD_DIR1 53
#define RTD_DIR2 52
#define RTD_PWM 10
// Driver2B - Roda traseira esquerda - M4
#define RTE_DIR1 51
#define RTE_DIR2 50
#define RTE_PWM 13
// Encoder roda frontal direita
#define EFD_A 2
#define EFD_B 27
// Encoder roda frontal esquerda
#define EFE_A 3
#define EFE_B 26
// Encoder roda traseira direita
#define ETD_A 19
#define ETD_B 29
// Encoder roda traseira direita
#define ETE_A 18
#define ETE_B 28

// Sensores IR
#define IR_E A0 // Esquerdo
#define IR_C A1 // Centro
#define IR_D A2 // Direito

#define THRESHOLD 90 // Limiar para detecção de linha
#define THRESHOLD_CENTER 40
// THRESHOLD = (preto + branco) / 2;

#define TARGET 30 // Velocidade base
#define CORRECTION 25 // Correção lateral (metade da velocidade base)

volatile long pulseCountEFD = 0;
volatile long pulseCountEFE = 0;
volatile long pulseCountETD = 0;
volatile long pulseCountETE = 0;

unsigned long lastTime = 0;
long lastPulseEFD = 0, lastPulseEFE = 0, lastPulseETD = 0, lastPulseETE = 0;

float eprevEFD, eprevEFE, eprevETD, eprevETE;
float eintegralEFD, eintegralEFE, eintegralETD, eintegralETE;

int correction;

void updateVelocity();

int readLine(int pin);

void setMotor(int dir1, int dir2, int pwm, int speed);
void moveForward(int speed);
void moveForwardEncoder(int speed);
void moveBackward(int speed);
void moveBackwardEncoder(int speed);
void moveLeft(int speed);
void moveLeftEncoder(int speed);
void moveRight(int speed);
void moveRightEncoder(int speed);
void turnLeft(int speed);
void turnLeftEncoder(int speed);
void turnRight(int speed);
void turnRightEncoder(int speed);
void stop();
void isrEFE();
void isrEFD();
void isrETE();
void isrETD();
void moveMFE(int target);
void moveMFD(int target);
void moveMTE(int target);
void moveMTD(int target);

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

  pinMode(EFE_A, INPUT_PULLUP);
  pinMode(EFE_B, INPUT_PULLUP);
  pinMode(EFD_A, INPUT_PULLUP);
  pinMode(EFD_B, INPUT_PULLUP);
  pinMode(ETE_A, INPUT_PULLUP);
  pinMode(ETE_B, INPUT_PULLUP);
  pinMode(ETD_A, INPUT_PULLUP);
  pinMode(ETD_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EFE_A), isrEFE, RISING);
  attachInterrupt(digitalPinToInterrupt(EFD_A), isrEFD, RISING);
  attachInterrupt(digitalPinToInterrupt(ETE_A), isrETE, RISING);
  attachInterrupt(digitalPinToInterrupt(ETD_A), isrETD, RISING);
}

void loop() {
  moveMFE(TARGET);
  moveMFD(TARGET);
  moveMTE(TARGET);
  moveMTD(TARGET);
  delay(1000);
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

// void moveBackwardEncoder(int speed) {
//   updateVelocity();

//   int pwmEFE = TARGET + correction * (speed - velEFE);
//   int pwmEFD = TARGET + correction * (speed - velEFD);
//   int pwmETE = TARGET + correction * (speed - velETE);
//   int pwmETD = TARGET + correction * (speed - velETD);

//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -pwmEFD);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, pwmEFE);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -pwmETE);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, pwmETD);
// }

// void moveForward(int speed) {
//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
// }

// void moveForwardEncoder(int speed) {
//   updateVelocity();

//   int e = pulseCountEFD - TARGET;

//   float dedt = (e - eprev)/(deltaT);
// }

// void moveLeftEncoder(int speed) {
//   updateVelocity();

//   int pwmEFE = TARGET + correction * (speed - velEFE);
//   int pwmEFD = TARGET + correction * (speed - velEFD);
//   int pwmETE = TARGET + correction * (speed - velETE);
//   int pwmETD = TARGET + correction * (speed - velETD);

//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -pwmEFD);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -pwmEFE);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, pwmETE);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, pwmETD);
// }

// void moveRight(int speed) {
//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
// }

// void moveRightEncoder(int speed) {
//   updateVelocity();

//   int pwmEFE = TARGET + correction * (speed - velEFE);
//   int pwmEFD = TARGET + correction * (speed - velEFD);
//   int pwmETE = TARGET + correction * (speed - velETE);
//   int pwmETD = TARGET + correction * (speed - velETD);

//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, pwmEFD);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, pwmEFE);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -pwmETE);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -pwmETD);
// }

// void turnLeft(int speed) {
//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -speed);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -speed);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -speed);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -speed);
// }

// void turnLeftEncoder(int speed) {
//   updateVelocity();

//   int pwmEFE = TARGET + correction * (speed - velEFE);
//   int pwmEFD = TARGET + correction * (speed - velEFD);
//   int pwmETE = TARGET + correction * (speed - velETE);
//   int pwmETD = TARGET + correction * (speed - velETD);

//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, -pwmEFD);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, -pwmEFE);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, -pwmETE);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, -pwmETD);
// }

// void turnRight(int speed) {
//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, speed);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, speed);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, speed);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, speed);
// }

// void turnRightEncoder(int speed) {
//   updateVelocity();

//   int pwmEFE = TARGET + correction * (speed - velEFE);
//   int pwmEFD = TARGET + correction * (speed - velEFD);
//   int pwmETE = TARGET + correction * (speed - velETE);
//   int pwmETD = TARGET + correction * (speed - velETD);

//   setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, pwmEFD);
//   setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, pwmEFE);
//   setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, pwmETE);
//   setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, pwmETD);
// }

void stop() {
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, 0);
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, 0);
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, 0);
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, 0);
}

void isrEFD() {
  if(digitalRead(EFD_B) == HIGH) {
    pulseCountEFD--;
  } else {
    pulseCountEFD++;
  }
}

void isrEFE() {
  if(digitalRead(EFE_B) == HIGH) {
    pulseCountEFE++;
  } else {
    pulseCountEFE--;
  }
}

void isrETD() {
  if(digitalRead(ETD_B) == HIGH) {
    pulseCountETD--;
  } else {
    pulseCountETD++;
  }
}

void isrETE() {
  if(digitalRead(ETE_B) == HIGH) {
    pulseCountETE++;
  } else {
    pulseCountETE--;
  }
}

void updateVelocity() {
  unsigned long now = micros();

  float deltaT = ((float)(now - lastTime)) / 1.0e6;

  lastTime = now;
}

void moveMFE(int target) {
  float kp = 1;
  float ki = 0;
  float kd = 0;

  long currentTime = micros();

  float deltaT = ((float)(currentTime - lastTime)) / 1.0e6;
  lastTime = currentTime;

  int e = pulseCountEFE - target;

  float dedt = (e - eprevEFE) / deltaT;

  eintegralEFE += e*deltaT;

  float u = kp*e + ki*eintegralEFE + kd*dedt;

  float pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, pwr);

  eprevEFE = e;

}

void moveMFD(int target) {
  float kp = 1;
  float ki = 0;
  float kd = 0;

  long currentTime = micros();

  float deltaT = ((float)(currentTime - lastTime)) / 1.0e6;
  lastTime = currentTime;

  int e = pulseCountEFD - target;

  float dedt = (e - eprevEFD) / deltaT;

  eintegralEFD += e*deltaT;

  float u = kp*e + ki*eintegralEFD + kd*dedt;

  float pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, pwr);

  eprevEFD = e;

}

void moveMTE(int target) {
  float kp = 1;
  float ki = 0;
  float kd = 0;

  long currentTime = micros();

  float deltaT = ((float)(currentTime - lastTime)) / 1.0e6;
  lastTime = currentTime;

  int e = pulseCountETE - target;

  float dedt = (e - eprevETE) / deltaT;

  eintegralETE += e*deltaT;

  float u = kp*e + ki*eintegralETE + kd*dedt;

  float pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, pwr);

  eprevETE = e;

}

void moveMTD(int target) {
  float kp = 1;
  float ki = 0;
  float kd = 0;

  long currentTime = micros();

  float deltaT = ((float)(currentTime - lastTime)) / 1.0e6;
  lastTime = currentTime;

  int e = pulseCountETD - target;

  float dedt = (e - eprevETD) / deltaT;

  eintegralETD += e*deltaT;

  float u = kp*e + ki*eintegralETD + kd*dedt;

  float pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, pwr);

  eprevETD = e;

}