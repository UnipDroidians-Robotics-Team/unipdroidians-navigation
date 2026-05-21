// ============================================================
//  UDH1 — Motor + Odometria + Controle de Velocidade
//  UnipDroidians Robotics Team
//
//  COMO CALIBRAR — ordem sugerida:
//
//  1. MIN_PWM: roda que demora a sair → aumente o MIN dela (passos de 0.5)
//
//  2. MAX_PWM: roda mais rápida em regime → diminua o MAX dela (passos de 0.5)
//
//  3. FF_GANHO: roda que sai forte demais na partida → diminua o FF dela
//              roda que demora a arrancar            → aumente o FF dela
//              (passos de 1.0)
//
//  4. KP: roda que acelera demais durante a rampa → diminua o KP dela
//         roda que responde devagar               → aumente o KP dela
//         (passos de 0.2)
//
//  5. MAX_PWM_GIRO: giro puxando para um lado → diminua o GIRO desse lado
// ============================================================

#include <PinChangeInterrupt.h>

// --- PINOS MOTORES ---
#define PIN_L_DIR   4
#define PIN_L_PWM   5
#define PIN_L_BREAK 11
#define PIN_R_DIR   7
#define PIN_R_PWM   9
#define PIN_R_BREAK 13

// --- PINOS ODOMETRIA ---
#define ODO_L   10   // canal B esquerda (direção)
#define ODO_R   6    // canal B direita  (direção)
#define ODO_A_L 2    // canal A esquerda (contagem)
#define ODO_A_R 3    // canal A direita  (contagem)

// ============================================================
// PARÂMETROS FÍSICOS
// ============================================================
const float METROS_POR_PULSO = 0.156 * 3.14159 / (12.95 * 4.0);

// ============================================================
// CALIBRAÇÃO DE PWM — escala 0.0 a 255.0
// Resolução: 1.0 = ~0.39% duty / 0.5 = ~0.20% / 0.1 = ~0.04%
// ============================================================

// -- Mínimo por roda (limiar de partida) --
const float MIN_PWM_ESQ      = 20.8;
const float MIN_PWM_DIR      = 21.0;

// -- Máximo por roda — linha reta --
const float MAX_PWM_ESQ      = 54.8;
const float MAX_PWM_DIR      = 55.0;

// -- Máximo por roda — giro no próprio eixo --
const float MAX_PWM_GIRO_ESQ = 89.8;
const float MAX_PWM_GIRO_DIR = 90.0;

// ============================================================
// CALIBRAÇÃO DO CONTROLADOR POR RODA
//
// KP — ganho do integrador (resposta durante aceleração):
//   → Roda ESQUERDA acelera demais na rampa → diminua KP_ESQ (passos de 0.2)
//   → Roda DIREITA  responde devagar        → aumente KP_DIR (passos de 0.2)
//
// FF_GANHO — feed-forward de partida (impulso inicial):
//   → Roda ESQUERDA sai forte demais        → diminua FF_GANHO_ESQ (passos de 1.0)
//   → Roda DIREITA  demora a arrancar       → aumente FF_GANHO_DIR (passos de 1.0)
// ============================================================

const float KP_ESQ      = 3.9;   // ← diminua se esquerda acelera demais na rampa
const float KP_DIR      = 4.0;   // ← diminua se direita   acelera demais na rampa

const float FF_GANHO_ESQ = 28.0; // ← diminua se esquerda sai forte demais na partida
const float FF_GANHO_DIR = 28.0; // ← diminua se direita   sai forte demais na partida

// ============================================================

// --- ODOMETRIA ---
volatile long pulsosEsq = 0;
volatile long pulsosDir = 0;
long pulsosEsqAnt       = 0;
long pulsosDirAnt       = 0;
unsigned long tempoAnterior = 0;

const unsigned long CTRL_INTERVAL_MS = 200;

float velEsq = 0.0;
float velDir = 0.0;
float pwmEsq = 0.0;
float pwmDir = 0.0;

// --- COMANDOS ROS ---
float setPointEsq = 0.0;
float setPointDir = 0.0;
unsigned long lastCmdTime        = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

// ============================================================
// mapF() — versão float do map() padrão do Arduino
// ============================================================
float mapF(float valor, float entMin, float entMax, float saiMin, float saiMax) {
  return saiMin + (valor - entMin) * (saiMax - saiMin) / (entMax - entMin);
}

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_L_DIR,   OUTPUT);
  pinMode(PIN_R_DIR,   OUTPUT);
  pinMode(PIN_L_BREAK, OUTPUT);
  pinMode(PIN_R_BREAK, OUTPUT);

  pinMode(ODO_L,   INPUT_PULLUP);
  pinMode(ODO_R,   INPUT_PULLUP);
  pinMode(ODO_A_L, INPUT_PULLUP);
  pinMode(ODO_A_R, INPUT_PULLUP);

  attachPinChangeInterrupt(digitalPinToPCINT(ODO_A_L), countEsqA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_L),   countEsqB, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_A_R), countDirA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_R),   countDirB, CHANGE);

  digitalWrite(PIN_L_BREAK, HIGH);
  digitalWrite(PIN_R_BREAK, HIGH);
}

// ============================================================
void loop() {

  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("CMD:")) {
      int sep = input.indexOf(';', 4);
      if (sep != -1) {
        float novoEsq = input.substring(4, sep).toFloat();
        float novoDir = input.substring(sep + 1).toFloat();
        lastCmdTime   = millis();

        if (abs(novoEsq) < 0.001 && abs(novoDir) < 0.001) {
          setPointEsq = 0.0;
          setPointDir = 0.0;
          pwmEsq      = 0.0;
          pwmDir      = 0.0;
          moveMotor(0.0, 0.0);

        } else {
          // Feed-forward separado por roda — partida simétrica
          if (abs(novoEsq - setPointEsq) > 0.05) {
            pwmEsq = (novoEsq >= 0 ? 1 : -1) * max(abs(novoEsq) * FF_GANHO_ESQ, 6.0f);
          }
          if (abs(novoDir - setPointDir) > 0.05) {
            pwmDir = (novoDir >= 0 ? 1 : -1) * max(abs(novoDir) * FF_GANHO_DIR, 6.0f);
          }

          setPointEsq = novoEsq;
          setPointDir = novoDir;
        }
      }
    }
  }

  if (millis() - lastCmdTime >= CMD_TIMEOUT_MS) {
    setPointEsq = 0.0;
    setPointDir = 0.0;
    pwmEsq      = 0.0;
    pwmDir      = 0.0;
    moveMotor(0.0, 0.0);
  }

  unsigned long agora = millis();
  unsigned long dt_ms = agora - tempoAnterior;

  if (dt_ms >= CTRL_INTERVAL_MS) {
    float dt_s = dt_ms * 0.001f;

    noInterrupts();
    long pEsq = pulsosEsq;
    long pDir = pulsosDir;
    interrupts();

    long dEsq = pEsq - pulsosEsqAnt;
    long dDir = pDir - pulsosDirAnt;
    pulsosEsqAnt = pEsq;
    pulsosDirAnt = pDir;

    // Velocidade medida (esquerda negada: encoder montado invertido)
    velEsq = -(dEsq * METROS_POR_PULSO) / dt_s;
    velDir =  (dDir * METROS_POR_PULSO) / dt_s;

    // Controlador I — KP separado por roda
    if (abs(setPointEsq) < 0.001) {
      pwmEsq = 0.0;
    } else {
      pwmEsq += (setPointEsq - velEsq) * KP_ESQ;
      pwmEsq  = constrain(pwmEsq, -100.0, 100.0);
    }

    if (abs(setPointDir) < 0.001) {
      pwmDir = 0.0;
    } else {
      pwmDir += (setPointDir - velDir) * KP_DIR;
      pwmDir  = constrain(pwmDir, -100.0, 100.0);
    }

    Serial.print("ODO:");
    Serial.print(pEsq);  Serial.print(";");
    Serial.print(pDir);  Serial.print(";");
    Serial.println(dt_ms);

    moveMotor(pwmDir, pwmEsq);
    tempoAnterior = agora;
  }
}

// ============================================================
// ACIONAMENTO DOS MOTORES
// velD / velE: −100.0 a +100.0
// ============================================================
void moveMotor(float velD, float velE) {

  if (abs(velE) < 5.0) velE = 0.0;
  if (abs(velD) < 5.0) velD = 0.0;

  bool girando = (velE != 0.0 && velD != 0.0) && ((velE > 0) != (velD > 0));

  float limE = girando ? MAX_PWM_GIRO_ESQ : MAX_PWM_ESQ;
  float limD = girando ? MAX_PWM_GIRO_DIR : MAX_PWM_DIR;

  // Motor ESQUERDO
  if (velE != 0.0) {
    int pwmE = (int)round(mapF(abs(velE), 5.0, 100.0, MIN_PWM_ESQ, limE));
    pwmE = constrain(pwmE, 0, 255);
    digitalWrite(PIN_L_BREAK, LOW);
    digitalWrite(PIN_L_DIR,   velE > 0);
    analogWrite(PIN_L_PWM, pwmE);
  } else {
    digitalWrite(PIN_L_BREAK, HIGH);
    analogWrite(PIN_L_PWM, 0);
  }

  // Motor DIREITO
  if (velD != 0.0) {
    int pwmD = (int)round(mapF(abs(velD), 5.0, 100.0, MIN_PWM_DIR, limD));
    pwmD = constrain(pwmD, 0, 255);
    digitalWrite(PIN_R_BREAK, LOW);
    digitalWrite(PIN_R_DIR,   velD < 0);
    analogWrite(PIN_R_PWM, pwmD);
  } else {
    digitalWrite(PIN_R_BREAK, HIGH);
    analogWrite(PIN_R_PWM, 0);
  }
}

// ============================================================
// QUADRATURA 4x — ISRs
// ============================================================

void countEsqA() {
  bool a = digitalRead(ODO_A_L);
  bool b = digitalRead(ODO_L);
  if (a ^ b) pulsosEsq++; else pulsosEsq--;
}

void countEsqB() {
  bool a = digitalRead(ODO_A_L);
  bool b = digitalRead(ODO_L);
  if (a == b) pulsosEsq++; else pulsosEsq--;
}

void countDirA() {
  bool a = digitalRead(ODO_A_R);
  bool b = digitalRead(ODO_R);
  if (a ^ b) pulsosDir++; else pulsosDir--;
}

void countDirB() {
  bool a = digitalRead(ODO_A_R);
  bool b = digitalRead(ODO_R);
  if (a == b) pulsosDir++; else pulsosDir--;
}
