#include <Arduino.h>
#include <util/atomic.h>

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
#define RTD_DIR1 50
#define RTD_DIR2 51
#define RTD_PWM 10
// Driver2B - Roda traseira esquerda - M4
#define RTE_DIR1 52
#define RTE_DIR2 53
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

// Ultrassom
#define TRIG 42
#define ECHO 43

#define DIST 15

#define THRESHOLD 150 // Limiar para detecção de linha
#define THRESHOLD_RIGHT 120

#define BASE_SPEED 45 // Velocidade base
#define CORRECTION 35 //

// THRESHOLD = (preto + branco) / 2;

volatile long pulseCountEFD = 0, pulseCountEFE = 0, pulseCountETD = 0, pulseCountETE = 0;

unsigned long lastTime = 0;
long lastPulseEFD = 0, lastPulseEFE = 0, lastPulseETD = 0, lastPulseETE = 0;

float posprevEFD = 0, posprevEFE = 0, posprevETD = 0, posprevETE = 0;
float eintegralEFD = 0, eintegralEFE = 0, eintegralETD = 0, eintegralETE = 0;

float deltaT;
long currT;

int contador = 0;
int distancia;

int pulsosreto = 0;
int pulsosgiro = 0; //virar 90
int lista_rotacao[8] = {1, 0, 1, 1, 1, 1, 1, 1}; //lista com funcao sequencial de rotacoes determinadas pelo cubo
int tipo_rotacao = 0;
int indice_rotacao = 0;

int readLine(int pin);
void setMotor(int dir1, int dir2, int pwm, int speed);
void isrEFE();
void isrEFD();
void isrETE();
void isrETD();
float calculateSpeedEFD(float vt);
float calculateSpeedETD(float vt);
float calculateSpeedETE(float vt);
float calculateSpeedEFE(float vt);
float readUltrassonic();
void RotacaoDireita90();
void RotacaoEsquerda90();
void Rotacao180();
void ignoreRotacao();

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

  // Ultrassom
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

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
  currT = micros();
  deltaT = ((float) (currT - lastTime))/1.0e6;

  // Sensores IR de teste
  int sE = readLine(IR_E); //Sensor esquerdo
  int sC = readLine(IR_C); //Sensor centro
  int sD = readLine(IR_D); //Sensor direito

  // Linha no centro → segue reto
  if (sC == HIGH && sE == LOW && sD == LOW) {
    calculateSpeedEFE(30);
    calculateSpeedEFD(-30);
    calculateSpeedETD(-30);
    calculateSpeedETE(30);
  }
  // Linha puxando para esquerda → corrige esquerda
  else if (sE == HIGH && sC == LOW && sD == LOW) {
   while (sC == LOW) {
    currT = micros();
    deltaT = ((float) (currT - lastTime))/1.0e6;

    calculateSpeedEFE(-50);
    calculateSpeedEFD(-50);
    calculateSpeedETD(50);
    calculateSpeedETE(50);

    lastTime = currT;
    sC = readLine(IR_C); //Sensor centro
   }
  }
  // Linha puxando para direita → corrige direita
  else if (sD == HIGH && sC == LOW && sE == LOW) {
   while (sC == LOW) {
    currT = micros();
    deltaT = ((float) (currT - lastTime))/1.0e6;

    calculateSpeedEFE(50);
    calculateSpeedEFD(50);
    calculateSpeedETD(-50);
    calculateSpeedETE(-50);

    lastTime = currT;
    sC = readLine(IR_C); //Sensor centro
   }
  }

  // Linha perdida
  else if (sE == LOW && sC == LOW && sD == LOW) {
    calculateSpeedEFE(0);
    calculateSpeedEFD(0);
    calculateSpeedETD(0);
    calculateSpeedETE(0);
  }

  // Centro + lado → curva suave
  else if ((sE == HIGH && sC == HIGH && sD == LOW)||(sE == LOW && sC == HIGH && sD == HIGH)||(sE == HIGH && sC == HIGH && sD == HIGH)) {
   pulsosreto = pulseCountETD;
   while (abs(pulseCountETD) - abs(pulsosreto) < 50){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;
    lastTime = currT;
   }
   if ((sE == HIGH && sC == HIGH && sD == LOW)||(sE == LOW && sC == HIGH && sD == HIGH)||(sE == HIGH && sC == HIGH && sD == HIGH)) {
    tipo_rotacao = lista_rotacao[indice_rotacao];

    if (tipo_rotacao == 1) {
      RotacaoDireita90();
    }
    else if (tipo_rotacao == 2) {
      RotacaoEsquerda90();
    }
    else if (tipo_rotacao == 3) {
      Rotacao180();
    }
    else if (tipo_rotacao == 0) {
      ignoreRotacao();
    }

    indice_rotacao++;
  }
  } 

  //distancia = 100*(sin(currT/1e6));
  distancia = readUltrassonic();
    
  while (distancia < 20 && distancia > 0) {
    currT = micros();
    deltaT = ((float) (currT - lastTime))/1.0e6;
    
    calculateSpeedEFE(0);
    calculateSpeedEFD(0);
    calculateSpeedETD(0);
    calculateSpeedETE(0);

    Serial.print(distancia);
    Serial.print(' ');
    Serial.print(sE);
    Serial.print(' ');
    Serial.print(sC);
    Serial.print(' ');
    Serial.print(sD);
    Serial.println(' ');

    distancia = readUltrassonic();
    lastTime = currT;
    delay(20);
  }

  Serial.print(distancia);
  Serial.print(' ');
  Serial.print(sE);
  Serial.print(' ');
  Serial.print(sC);
  Serial.print(' ');
  Serial.print(sD);
  Serial.println(' ');

  lastTime = currT;
  delay(20);
}

int readLine(int pin) {
  int value = analogRead(pin);

  if(pin == IR_D) {
    if (value > THRESHOLD_RIGHT) {
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

void isrEFD() {
  if(digitalRead(EFD_B) == HIGH) {
    pulseCountEFD++;
  } else {
    pulseCountEFD--;
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
    pulseCountETD++;
  } else {
    pulseCountETD--;
  }
}

void isrETE() {
  if(digitalRead(ETE_B) == HIGH) {
    pulseCountETE++;
  } else {
    pulseCountETE--;
  }
}

float calculateSpeedEFE(float vt) {
  volatile long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pulseCountEFE;
  }

  float velocity1 = (pos - posprevEFE)/deltaT;

  posprevEFE = pos;

  float v1 = velocity1/480.0*60.0;

  float kp = 1;
  float ki = 3;
  float e = vt-v1;
  eintegralEFE = eintegralEFE + (e*deltaT);

  float u = kp*e + ki*eintegralEFE;

  int pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RFE_DIR1, RFE_DIR2, RFE_PWM, pwr);

  return v1;
}

// Negativo é para frente
float calculateSpeedEFD(float vt) {
  volatile long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pulseCountEFD;
  }

  float velocity1 = (pos - posprevEFD)/deltaT;

  posprevEFD = pos;

  float v2 = velocity1/480.0*60.0;

  float kp = 1.5;
  float ki = 3;
  float e = vt-v2;
  eintegralEFD = eintegralEFD + (e*deltaT);

  float u = kp*e + ki*eintegralEFD;

  int pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RFD_DIR1, RFD_DIR2, RFD_PWM, pwr);
  
  return v2;
}

// Negativo é para frente
float calculateSpeedETD(float vt) {
  volatile long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pulseCountETD;
  }

  float velocity1 = (pos - posprevETD)/deltaT;

  posprevETD = pos;

  float v3 = velocity1/480.0*60.0;

  float kp = 1.5;
  float ki = 3;
  float e = vt-v3;
  eintegralETD = eintegralETD + (e*deltaT);

  float u = kp*e + ki*eintegralETD;

  int pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RTD_DIR1, RTD_DIR2, RTD_PWM, pwr);

  return v3;
}
float calculateSpeedETE(float vt) {
  volatile long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pulseCountETE;
  }

  float velocity1 = (pos - posprevETE)/deltaT;

  posprevETE = pos;

  float v4 = velocity1/480.0*60.0;

  float kp = 1.5;
  float ki = 2.9;
  float e = vt-v4;
  eintegralETE = eintegralETE + (e*deltaT);

  float u = kp*e + ki*eintegralETE;

  int pwr = u;
  if(pwr > 255) {
    pwr = 255;
  } else if(pwr < -255) {
    pwr = -255;
  }
  setMotor(RTE_DIR1, RTE_DIR2, RTE_PWM, pwr);

  return v4;
}

float readUltrassonic() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 10000);

  int distance = duration * 0.034 / 2.0;

  return distance;

}

void RotacaoDireita90() {
  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro > -450){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(30);
    calculateSpeedEFD(-30);
    calculateSpeedETD(-30);
    calculateSpeedETE(30);

    lastTime = currT;
  }

  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro > -900){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(-50);
    calculateSpeedEFD(-50);
    calculateSpeedETD(-50);
    calculateSpeedETE(-50);

    lastTime = currT;
  }
}

void RotacaoEsquerda90() {
  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro > -400){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(50);
    calculateSpeedEFD(-50);
    calculateSpeedETD(-50);
    calculateSpeedETE(50);

    lastTime = currT;
  }

  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro < 900){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(50);
    calculateSpeedEFD(50);
    calculateSpeedETD(50);
    calculateSpeedETE(50);

    lastTime = currT;
  }
}

void Rotacao180() {
  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro > -400){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(50);
    calculateSpeedEFD(-50);
    calculateSpeedETD(-50);
    calculateSpeedETE(50);

    lastTime = currT;
  }

  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro > -900){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(-50);
    calculateSpeedEFD(-50);
    calculateSpeedETD(-50);
    calculateSpeedETE(-50);

    lastTime = currT;
  }
}

void ignoreRotacao() {
  pulsosgiro = pulseCountETD;
  while (pulseCountETD - pulsosgiro > -100){
    currT = micros();
    deltaT = ((float)(currT - lastTime)) / 1.0e6;

    calculateSpeedEFE(50);
    calculateSpeedEFD(-50);
    calculateSpeedETD(-50);
    calculateSpeedETE(50);

    lastTime = currT;
  }
}
