/*
  ==========================================================================
   Firmware de controle do triângulo autoequilibrante com roda de reação
  ==========================================================================

  Este código implementa o sistema de controle embarcado da planta utilizada
  no trabalho. O firmware realiza a leitura do sensor inercial MPU6050, estima
  o ângulo da estrutura por meio de um filtro de Kalman, lê o encoder do motor
  da roda de reação, calcula os estados da planta e executa a lógica de controle
  composta pelos modos swing-up, freio e LQR.

  A estratégia de operação é dividida em três modos:
    modo 0: swing-up por energia, utilizado para elevar a planta;
    modo 1: freio, utilizado para reduzir a velocidade angular antes da captura;
    modo 2: LQR, utilizado para estabilizar a planta em torno da posição vertical.

  Durante a execução, o ESP32 envia pela porta serial as principais variáveis
  do ensaio experimental, separadas por vírgula, na seguinte ordem:

    modo, theta, thetaDot, omega, xi, energia, u

  em que:
    modo      -> modo de operação ativo;
    theta     -> ângulo de inclinação da planta [rad];
    thetaDot  -> velocidade angular do corpo [rad/s];
    omega     -> velocidade angular da roda de reação [rad/s];
    xi        -> estado associado à posição acumulada da roda [rad];
    energia   -> energia mecânica calculada [J];
    u         -> tensão equivalente de controle [V].

  Os dados enviados pela serial são coletados posteriormente por um script em
  Python e utilizados na geração dos gráficos experimentais no MATLAB.

  Testado com:
    Arduino IDE 2.3.8
    esp32 package by Espressif Systems 2.0.17

  ==========================================================================
*/

#include <Wire.h>
#include <ESP32Encoder.h>
#include <math.h>

// ===================== PINOS =====================

#define INTERNAL_LED 2

#define MPU_ADDR 0x68

#define PIN_BRAKE 18
#define PIN_PWM 19
#define PIN_DIR 23
#define PIN_ENC_A 5
#define PIN_ENC_B 13

#define PWM_CHANNEL 1
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 20000

// ===================== ENCODER =====================

ESP32Encoder reactionWheelEncoder;

// ===================== FILTRO DE KALMAN =====================

float Q_angle = 0.001;
float Q_bias  = 0.005;
float R_meas  = 1.0;

float rollAngle = 0.0;
float gyroBias  = 0.0;

float P[2][2] = { { 1, 0 }, { 0, 1 } };
float Kf[2]   = { 0, 0 };

// ===================== AJUSTE DO ÂNGULO =====================

const float ANGLE_OFFSET_DEG = 0.0;
const float ANGLE_POLARITY   = 1.0;

// ===================== GANHOS DO CONTROLADOR LQR =====================

float K1 = -247.9;
float K2 = -26.31;
float K3 = -0.118;
float K4 = 0.011;

// ===================== ESTADOS DA PLANTA =====================

float theta = 0.0;
float thetaDot = 0.0;
float omega = 0.0;
float xi = 0.0;
float xiPrevious = 0.0;

// ===================== SINAL DE CONTROLE =====================

float controlVoltage = 0.0;
int pwmCommand = 0;

// ===================== AMOSTRAGEM =====================

const float Ts = 0.01;
const unsigned long Ts_us = 10000;

unsigned long currentTime = 0;
unsigned long previousTime = 0;

// ===================== CONVERSÕES =====================

const float encoderToRad = 63.7;
const float voltageToPWM = 21.3;

const float maxVoltage = 12.0;
const int maxPWM = 255;

// ===================== PARÂMETROS FÍSICOS DA PLANTA =====================

const float m = 0.498;
const float g = 9.81;
const float l = 0.10;
const float I = 0.00498;

// ===================== CONTROLE SWING-UP POR ENERGIA ====================

const float thetaInicialSwing = 1.0;

const float Einicial = m * g * l * (cos(thetaInicialSwing) - 1.0);

const float Etopo = 0.0;

const float DeltaE = fabs(Einicial - Etopo);

const float excessoEnergia = 0.15;

const float Eref = Etopo + excessoEnergia * DeltaE;

const float kSwing = 0.7*12.2815;

const float polaridadeSwing = -1.0;

const float polaridadeFreio = 1.0;

float energia = 0.0;

// ===================== CRITÉRIOS DE CAPTURA E COMUTAÇÃO =====================

const float thetaLQR = 12.0 / 57.3;
const float thetaDotLQR = 120.0 / 57.3;

const float thetaCaptura = 25.0 / 57.3;

// ===================== MODO DE OPERAÇÃO =====================

int modoControle = 0;

bool lqrAtivo = false;

// ===================== ENVIO SERIAL =====================

int serialCounter = 0;
const int serialDivider = 5;

// ===================== FUNÇÕES AUXILIARES =====================

float normalizeAngle(float angleRad) {
  while (angleRad > PI) {
    angleRad -= 2.0 * PI;
  }

  while (angleRad < -PI) {
    angleRad += 2.0 * PI;
  }

  return angleRad;
}

float signFloat(float value) {
  if (value > 0.0) return 1.0;
  if (value < 0.0) return -1.0;
  return 0.0;
}

// ===================== CONFIGURAÇÃO INICIAL =====================

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(INTERNAL_LED, OUTPUT);

  setupIMU();
  setupReactionWheelMotor();

  digitalWrite(INTERNAL_LED, HIGH);
  delay(1500);

  for (int i = 0; i < 400; i++) {
    readIMU();
    delay(5);
  }

  reactionWheelEncoder.clearCount();

  theta = 0.0;
  thetaDot = 0.0;
  omega = 0.0;
  xi = 0.0;
  xiPrevious = 0.0;

  controlVoltage = 0.0;
  pwmCommand = 0;

  modoControle = 0;
  lqrAtivo = false;

  previousTime = micros();

  digitalWrite(INTERNAL_LED, LOW);

  Serial.println("Sistema iniciado.");
  Serial.println("modo,theta,thetaDot,omega,xi,energia,u");
}

// ===================== LAÇO PRINCIPAL =====================

void loop() {
  currentTime = micros();

  if ((currentTime - previousTime) >= Ts_us) {
    previousTime += Ts_us;

    readIMU();

    theta = ANGLE_POLARITY * ((rollAngle - ANGLE_OFFSET_DEG) / 57.3);
    theta = normalizeAngle(theta);

    digitalWrite(PIN_BRAKE, HIGH);

    xi = -reactionWheelEncoder.getCount() / encoderToRad;

    omega = -(xi - xiPrevious) / Ts;
    xiPrevious = xi;

    energia = 0.5 * I * thetaDot * thetaDot + m * g * l * (cos(theta) - 1.0);

    if (!lqrAtivo) {
      if ((fabs(theta) < thetaLQR) && (fabs(thetaDot) < thetaDotLQR)) {
        ativarLQR();
      }
    }

    if (lqrAtivo) {
      calcularControleLQR();
      modoControle = 2;
    } else {
      calcularControleSwingOuFreio();
    }

    controlVoltage = constrain(controlVoltage, -maxVoltage, maxVoltage);

    pwmCommand = controlVoltage * voltageToPWM;

    applyMotorCommand(pwmCommand);

    sendSerialData();
  }
}

// ===================== ATIVAÇÃO DO MODO LQR =====================

void ativarLQR() {
  lqrAtivo = true;
  modoControle = 2;

  reactionWheelEncoder.clearCount();

  xi = 0.0;
  xiPrevious = 0.0;
  omega = 0.0;

  controlVoltage = 0.0;
}

// ===================== CONTROLE LQR =====================

void calcularControleLQR() {
  controlVoltage = -(K1 * theta + K2 * thetaDot + K3 * omega + K4 * xi);
}

// ===================== CONTROLE SWING-UP E MODO DE FREIO =====================

void calcularControleSwingOuFreio() {

  if ((fabs(theta) < thetaCaptura) && (fabs(thetaDot) >= thetaDotLQR)) {

    controlVoltage = 0.8*polaridadeFreio * maxVoltage * signFloat(thetaDot);

    controlVoltage = constrain(controlVoltage, -maxVoltage*0.8, maxVoltage*0.8);

    modoControle = 1;

  } else {

    float erroE = energia - Eref;

    float sinal = signFloat(thetaDot);

    controlVoltage = -polaridadeSwing * kSwing * erroE * sinal;

    modoControle = 0;
  }
}

// ===================== CONFIGURAÇÃO DO MPU6050 =====================

void setupIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(1 << 3);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

// ===================== LEITURA E FILTRAGEM DO MPU6050 =====================

void readIMU() {
  int16_t ax, ay, az, temp, gx, gy, gz;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  float accelAngle = atan2((float)ay, (float)az) * 57.3;

  float gyroRate = gx / 65.5;

  rollAngle += (gyroRate - gyroBias) * Ts;

  if (rollAngle > 180.0) rollAngle -= 360.0;
  if (rollAngle < -180.0) rollAngle += 360.0;

  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;

  Kf[0] = P[0][0] / (P[0][0] + R_meas);
  Kf[1] = P[1][0] / (P[0][0] + R_meas);

  float estimationError = accelAngle - rollAngle;

  if (estimationError > 180.0) estimationError -= 360.0;
  if (estimationError < -180.0) estimationError += 360.0;

  rollAngle += Kf[0] * estimationError;
  gyroBias += Kf[1] * estimationError;

  if (rollAngle > 180.0) rollAngle -= 360.0;
  if (rollAngle < -180.0) rollAngle += 360.0;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= Kf[0] * P00_temp;
  P[0][1] -= Kf[0] * P01_temp;
  P[1][0] -= Kf[1] * P00_temp;
  P[1][1] -= Kf[1] * P01_temp;

  thetaDot = ANGLE_POLARITY * ((gyroRate - gyroBias) / 57.3);
}

// ===================== CONFIGURAÇÃO DO MOTOR E DO ENCODER =====================

void setupReactionWheelMotor() {
  pinMode(PIN_BRAKE, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  digitalWrite(PIN_BRAKE, HIGH);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PIN_PWM, PWM_CHANNEL);

  applyMotorCommand(0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  reactionWheelEncoder.attachFullQuad(PIN_ENC_B, PIN_ENC_A);
  reactionWheelEncoder.clearCount();
}

// ===================== ACIONAMENTO DO MOTOR =====================

void applyMotorCommand(int command) {
  if (command < 0) {
    digitalWrite(PIN_DIR, HIGH);
    command = -command;
  } else {
    digitalWrite(PIN_DIR, LOW);
  }

  command = constrain(command, 0, maxPWM);

  ledcWrite(PWM_CHANNEL, maxPWM - command);
}

// ===================== TRANSMISSÃO DOS DADOS PELA SERIAL =====================

void sendSerialData() {
  serialCounter++;

  if (serialCounter >= serialDivider) {
    serialCounter = 0;

    Serial.print(modoControle);
    Serial.print(",");
    Serial.print(theta, 6);
    Serial.print(",");
    Serial.print(thetaDot, 6);
    Serial.print(",");
    Serial.print(omega, 6);
    Serial.print(",");
    Serial.print(xi, 6);
    Serial.print(",");
    Serial.print(energia, 6);
    Serial.print(",");
    Serial.println(controlVoltage, 6);
  }
}