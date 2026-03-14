#include <Arduino.h>
#include <Servo.h>
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

// Altura da garra
#define GARRA_DIR1 49
#define GARRA_DIR2 48
#define GARRA_PWM 12
#define EGARRA_A 20
#define EGARRA_B 37

// Servo da garra
#define SERVO_PIN 44
Servo garraServo;

// Ultrassom
#define TRIG 42
#define ECHO 43
#define TRIG2 46
#define ECHO2 47

#define DIST 15

#define THRESHOLD 150 // Limiar para detecção de linha
#define THRESHOLD_RIGHT 120

#define BASE_SPEED 45 // Velocidade base
#define CORRECTION 35 //

// THRESHOLD = (preto + branco) / 2;

volatile int pulseCountGarra = 0;
int primeiro_andar = 12500;
int segundo_andar = 14200;
int alvo = -30000;
int altura = 0;
int pulses_garra = 0;

volatile long pulseCountEFD = 0, pulseCountEFE = 0, pulseCountETD = 0, pulseCountETE = 0;

unsigned long lastTime = 0;
long lastPulseEFD = 0, lastPulseEFE = 0, lastPulseETD = 0, lastPulseETE = 0;

float posprevEFD = 0, posprevEFE = 0, posprevETD = 0, posprevETE = 0;
float eintegralEFD = 0, eintegralEFE = 0, eintegralETD = 0, eintegralETE = 0;

float deltaT;
long currT;

int contador = 0;
int distancia;

int pulsosgiro = 0; //virar 90
int temp = 0; //virar 90

int index = 0; // posicao do index do vetor das tags
int tags[3] = {1,2,3}; // lista das tags que serao passadas -> posicao0, cor0, posicao1.... (1=esquerda, 2=centro, 3=direita) (1=verde, 2=azul, 3=vermelho)

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
float readUltrassonicGarra();
void tag1();
void tag2();
int SobeGarra(int pulses);
int DesceGarra(int pulses);
void isrEGARRA();
int movGarra(int pulses);

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
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);

  pinMode(EFE_A, INPUT_PULLUP);
  pinMode(EFE_B, INPUT_PULLUP);
  pinMode(EFD_A, INPUT_PULLUP);
  pinMode(EFD_B, INPUT_PULLUP);
  pinMode(ETE_A, INPUT_PULLUP);
  pinMode(ETE_B, INPUT_PULLUP);
  pinMode(ETD_A, INPUT_PULLUP);
  pinMode(ETD_B, INPUT_PULLUP);

  // Garra
  pinMode(GARRA_DIR1, OUTPUT);
  pinMode(GARRA_DIR2, OUTPUT);
  pinMode(GARRA_PWM, OUTPUT);

  pinMode(EGARRA_A, INPUT_PULLUP);
  pinMode(EGARRA_B, INPUT_PULLUP);

  garraServo.attach(SERVO_PIN);

  attachInterrupt(digitalPinToInterrupt(EGARRA_A), isrEGARRA, RISING);

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

  //distancia = 100*(sin(currT/1e6));
  distancia = readUltrassonic();
    
  if (distancia < 60 && distancia > 5) {
    
    calculateSpeedEFE(0);
    calculateSpeedEFD(0);
    calculateSpeedETD(0);
    calculateSpeedETE(0);

    movGarra(primeiro_andar);

    if (tags[0] == 0) {
      // Função de ler as tags
    }
    int ignorar = tags[index] - 1;
    pulsosgiro = pulseCountETD;
    while (pulseCountETD - pulsosgiro < 500){
      currT = micros();
      deltaT = ((float)(currT - lastTime)) / 1.0e6;
      calculateSpeedEFE(-30);
      calculateSpeedEFD(-30);
      calculateSpeedETD(30);
      calculateSpeedETE(30);
      lastTime = currT;
    }
    while(1) {
      currT = micros();
      deltaT = ((float)(currT - lastTime)) / 1.0e6;
      calculateSpeedEFE(20);
      calculateSpeedEFD(20);
      calculateSpeedETD(-20);
      calculateSpeedETE(-20);
      lastTime = currT;
      int distanciaGarra = readUltrassonicGarra();
      if (distanciaGarra > 0 && distanciaGarra < 40) {
        if(ignorar) {
          ignorar--;
          pulsosgiro = pulseCountETD;
          while (pulseCountETD - pulsosgiro > -200) {
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(30);
            calculateSpeedEFD(30);
            calculateSpeedETD(-30);
            calculateSpeedETE(-30);
            lastTime = currT;
          }
        }else {
          break;
        }
      }
    }
    pulsosgiro = pulseCountETD;
    float distance = readUltrassonicGarra();
    Serial.print(distance);
    Serial.println(' ');
    while(readUltrassonicGarra() > 4 || readUltrassonicGarra() == 0) {
      currT = micros();
      deltaT = ((float)(currT - lastTime)) / 1.0e6;
      distance = readUltrassonicGarra();
      calculateSpeedEFE(30);
      calculateSpeedEFD(-30);
      calculateSpeedETD(-30);
      calculateSpeedETE(30);
      lastTime = currT;
      Serial.print(distance);
      Serial.println(' ');
    }
    calculateSpeedEFE(0);
    calculateSpeedEFD(0);
    calculateSpeedETD(0);
    calculateSpeedETE(0);
    movGarra(7200);
    int percorrido = pulsosgiro - pulseCountETD;
    garraServo.write(100); // Descobrir o valor pra fechar o servo
    delay(2000);
    garraServo.write(90);
    lastTime = currT;
    movGarra(segundo_andar);

    delay (10000);
  }
  //   pulsosgiro = pulseCountETD;
  //   while (pulseCountETD - pulsosgiro < percorrido){
  //     currT = micros();
  //     deltaT = ((float)(currT - lastTime)) / 1.0e6;
  //     calculateSpeedEFE(-30);
  //     calculateSpeedEFD(30);
  //     calculateSpeedETD(30);
  //     calculateSpeedETE(-30);
  //     lastTime = currT;
  //   }
  //   pulsosgiro = pulseCountETD;
  //   while (pulseCountETD - pulsosgiro > -1800){ // ajustar para um giro 180
  //    currT = micros();
  //    deltaT = ((float)(currT - lastTime)) / 1.0e6;
  //    calculateSpeedEFE(-50);
  //    calculateSpeedEFD(-50);
  //    calculateSpeedETD(-50);
  //    calculateSpeedETE(-50);
  //    lastTime = currT;
  //   }
  //   if(tags[index] == 1) {
  //     sC = LOW;
  //     while (sC != HIGH) {
  //       currT = micros();
  //       deltaT = ((float)(currT - lastTime)) / 1.0e6;
  //       int sE = readLine(IR_E);
  //       int sC = readLine(IR_C); 
  //       int sD = readLine(IR_D); 

  //       calculateSpeedEFE(-50);
  //       calculateSpeedEFD(-50);
  //       calculateSpeedETD(50);
  //       calculateSpeedETE(50);

  //       lastTime = currT;
  //     }
  //     tag1();
  //     index++;
  //   } else if(tags[index] == 2) {
  //     sC = LOW;
  //     pulsosgiro = pulseCountETD;
  //     while (pulseCountEFD - pulsosgiro < 200) {
  //       currT = micros();
  //       deltaT = ((float)(currT - lastTime)) / 1.0e6;
  //       calculateSpeedEFE(-50);
  //       calculateSpeedEFD(-50);
  //       calculateSpeedETD(50);
  //       calculateSpeedETE(50);
  //       lastTime = currT;

  //       sE = readLine(IR_E);
  //       sC = readLine(IR_C);
  //       sD = readLine(IR_D);

  //       if (sC == HIGH) {
  //         calculateSpeedEFE(0);
  //         calculateSpeedEFD(0);
  //         calculateSpeedETD(0);
  //         calculateSpeedETE(0);
  //         break;
  //       }
  //     }
  //     if(sC == LOW) {
  //       pulsosgiro = pulseCountETD;
  //       while (pulseCountEFD - pulsosgiro > -400) {
  //         currT = micros();
  //         deltaT = ((float)(currT - lastTime)) / 1.0e6;
  //         calculateSpeedEFE(50);
  //         calculateSpeedEFD(50);
  //         calculateSpeedETD(-50);
  //         calculateSpeedETE(-50);
  //         lastTime = currT;

  //         sE = readLine(IR_E);
  //         sC = readLine(IR_C);
  //         sD = readLine(IR_D);

  //         if (sC == HIGH) {
  //           calculateSpeedEFE(0);
  //           calculateSpeedEFD(0);
  //           calculateSpeedETD(0);
  //           calculateSpeedETE(0);
  //           break;
  //         }
  //       }
  //     }
  //     movGarra(primeiro_andar);
  //     tag2();
  //     index++;
  //   } else if (tags[index] == 3) {
  //     sC = LOW;
  //     while (sC != HIGH) {
  //       currT = micros();
  //       deltaT = ((float)(currT - lastTime)) / 1.0e6;
  //       int sE = readLine(IR_E);
  //       int sC = readLine(IR_C); 
  //       int sD = readLine(IR_D); 

  //       calculateSpeedEFE(50);
  //       calculateSpeedEFD(50);
  //       calculateSpeedETD(-50);
  //       calculateSpeedETE(-50);

  //       lastTime = currT;
  //     }
  //     movGarra(segundo_andar);
  //     tag2();
  //     index++;
  //   }
  // }

  // Serial.print(distancia);
  // Serial.print(' ');
  // Serial.print(sE);
  // Serial.print(' ');
  // Serial.print(sC);
  // Serial.print(' ');
  // Serial.print(sD);
  // Serial.println(' ');


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

float readUltrassonicGarra() {
  digitalWrite(TRIG2, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);

  long duration = pulseIn(ECHO2, HIGH, 10000);

  int distance = duration * 0.034 / 2.0;

  return distance;
}

void tag1() {
    while (1) {
        currT = micros();
        deltaT = ((float) (currT - lastTime))/1.0e6;
        int sE = readLine(IR_E); //Sensor esquerdo
        int sC = readLine(IR_C); //Sensor centro
        int sD = readLine(IR_D); //Sensor direito

        // Linha no centro → segue reto
        if (sC == HIGH && sE == LOW && sD == LOW) {
            calculateSpeedEFE(50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(-50);
            calculateSpeedETE(50);
        }
        // Linha puxando para esquerda → corrige esquerda
        else if (sE == HIGH && sC == LOW && sD == LOW) {
            calculateSpeedEFE(-50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(50);
            calculateSpeedETE(50);
        }
        // Linha puxando para direita → corrige direita
        else if (sD == HIGH && sC == LOW && sE == LOW) {
            calculateSpeedEFE(50);
            calculateSpeedEFD(50);
            calculateSpeedETD(-50);
            calculateSpeedETE(-50);
        }
        // Centro + lado → curva suave, apenas para a esquerda
        else if (sE == HIGH && sC == HIGH) {
            pulsosgiro = pulseCountETD;
            while (pulseCountETD - pulsosgiro > -400){
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
            temp = 1;
            break;
        }

        lastTime = currT;
    }
    while (pulseCountETD - pulsosgiro > -425){ // Colocar o valor certo para alinhar com o centro
        currT = micros();
        deltaT = ((float)(currT - lastTime)) / 1.0e6;
        calculateSpeedEFE(30);
        calculateSpeedEFD(-30);
        calculateSpeedETD(-30);
        calculateSpeedETE(30);
        lastTime = currT;
    }

    // função para descer a garra pro zero
    // função para abrir a garra
    // função para subir a garra pro primeiro andar 

    while (pulseCountETD - pulsosgiro < 425){ // Colocar o valor certo para ir pra tras o suficiente
        currT = micros();
        deltaT = ((float)(currT - lastTime)) / 1.0e6;
        calculateSpeedEFE(-30);
        calculateSpeedEFD(30);
        calculateSpeedETD(30);
        calculateSpeedETE(-30);
        lastTime = currT;
    }

    while (pulseCountETD - pulsosgiro > -1000){ // Colocar o valor certo para rotacionar 90 graus
        currT = micros();
        deltaT = ((float)(currT - lastTime)) / 1.0e6;
        calculateSpeedEFE(-50);
        calculateSpeedEFD(-50);
        calculateSpeedETD(-50);
        calculateSpeedETE(-50);
        lastTime = currT;
    }
    return;
}

void tag2() {
    int ignorar = 1;
    // Colocar a garra no primeiro andar (Ou não, já tá no loop)

    while (1) {
        currT = micros();
        deltaT = ((float) (currT - lastTime))/1.0e6;
        int sE = readLine(IR_E); //Sensor esquerdo
        int sC = readLine(IR_C); //Sensor centro
        int sD = readLine(IR_D); //Sensor direito

        // Linha no centro → segue reto
        if (sC == HIGH && sE == LOW && sD == LOW) {
            calculateSpeedEFE(50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(-50);
            calculateSpeedETE(50);
        }
        // Linha puxando para esquerda → corrige esquerda
        else if (sE == HIGH && sC == LOW && sD == LOW) {
            calculateSpeedEFE(-50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(50);
            calculateSpeedETE(50);
        }
        // Linha puxando para direita → corrige direita
        else if (sD == HIGH && sC == LOW && sE == LOW) {
            calculateSpeedEFE(50);
            calculateSpeedEFD(50);
            calculateSpeedETD(-50);
            calculateSpeedETE(-50);
        }
        // Centro + lado → curva suave, Curva apenas para a direita
        else if (sD == HIGH && sC == HIGH) {
            if(ignorar) {
                ignorar = 0;
                continue;
            }
            else{
                pulsosgiro = pulseCountETD;
                while (pulseCountETD - pulsosgiro > -400){
                currT = micros();
                deltaT = ((float)(currT - lastTime)) / 1.0e6;
                calculateSpeedEFE(30);
                calculateSpeedEFD(-30);
                calculateSpeedETD(-30);
                calculateSpeedETE(30);
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
                temp = 1;
            }
        }
        distancia = readUltrassonic();
    
        if (distancia < 50 && distancia > 0) {
            calculateSpeedEFD(0);
            calculateSpeedEFE(0);
            calculateSpeedETD(0);
            calculateSpeedETE(0);
            break;
        }
        lastTime = currT;
    }
    if(tags[index] == 1) {
      pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro < 900){ // Descobrir o valor certo para alcançar o verde
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-30);
            calculateSpeedEFD(-30);
            calculateSpeedETD(30);
            calculateSpeedETE(30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -200){ // Descobrir o valor certo para ir para frente
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(30);
            calculateSpeedEFD(-30);
            calculateSpeedETD(-30);
            calculateSpeedETE(30);
            lastTime = currT;
        }
        garraServo.write(105);
        delay(2000);
        garraServo.write(90);
        lastTime = currT;
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro < 200){ // Voltar o mesmo tanto que foi pra frente
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-30);
            calculateSpeedEFD(30);
            calculateSpeedETD(30);
            calculateSpeedETE(-30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -900){ // Corrigir para o centro
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(30);
            calculateSpeedEFD(30);
            calculateSpeedETD(-30);
            calculateSpeedETE(-30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -2000){ // Colocar o valor certo para rotacionar 180 graus
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(-50);
            calculateSpeedETE(-50);
            lastTime = currT;
        }
      }else if(tags[index] == 2) {
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -200){ // Descobrir o valor certo para ir para frente
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(30);
            calculateSpeedEFD(-30);
            calculateSpeedETD(-30);
            calculateSpeedETE(30);
            lastTime = currT;
        }
        garraServo.write(105);
        delay(2000);
        garraServo.write(90);
        lastTime = currT;
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro < 200){ // Voltar o mesmo tanto que foi pra frente
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-30);
            calculateSpeedEFD(30);
            calculateSpeedETD(30);
            calculateSpeedETE(-30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -2000){ // Colocar o valor certo para rotacionar 180 graus
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(-50);
            calculateSpeedETE(-50);
            lastTime = currT;
        }
        
    }
    else if(tags[index] == 3) {
      pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro < 900){ // Descobrir o valor certo para alcançar o azul
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(30);
            calculateSpeedEFD(30);
            calculateSpeedETD(-30);
            calculateSpeedETE(-30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -200){ // Descobrir o valor certo para ir para frente
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(30);
            calculateSpeedEFD(-30);
            calculateSpeedETD(-30);
            calculateSpeedETE(30);
            lastTime = currT;
        }
        garraServo.write(105);
        delay(2000);
        garraServo.write(90);
        lastTime = currT;
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro < 200){ // Voltar o mesmo tanto que foi pra frente
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-30);
            calculateSpeedEFD(30);
            calculateSpeedETD(30);
            calculateSpeedETE(-30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -900){ // Corrigir para o centro
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-30);
            calculateSpeedEFD(-30);
            calculateSpeedETD(30);
            calculateSpeedETE(30);
            lastTime = currT;
        }
        pulsosgiro = pulseCountETD;
        while (pulseCountETD - pulsosgiro > -2000){ // Colocar o valor certo para rotacionar 180 graus
            currT = micros();
            deltaT = ((float)(currT - lastTime)) / 1.0e6;
            calculateSpeedEFE(-50);
            calculateSpeedEFD(-50);
            calculateSpeedETD(-50);
            calculateSpeedETE(-50);
            lastTime = currT;
        }
      }
  return;
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

  //  Serial.print(count);
  //  Serial.println(' ');

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

   // Serial.print(count);
   // Serial.println(' ');

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

int movGarra(int pulses) {
  if(pulses - pulseCountGarra > 0) {
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
  }else if (pulses - pulseCountGarra < 0) {
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
}
