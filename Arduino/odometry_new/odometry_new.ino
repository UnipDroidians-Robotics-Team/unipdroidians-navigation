#include <PinChangeInterrupt.h>

// --- PINOS MOTORES ---
#define PIN_L_DIR   4
#define PIN_L_PWM   5
#define PIN_L_BREAK 11
#define PIN_R_DIR   7
#define PIN_R_PWM   9
#define PIN_R_BREAK 13

// --- PINOS ODOMETRIA ---
#define ODO_L   10  // canal B esquerda (direção)
#define ODO_R   6   // canal B direita  (direção)
#define ODO_A_L 2   // canal A esquerda (contagem)
#define ODO_A_R 3   // canal A direita  (contagem)

// --- PARÂMETROS FÍSICOS (CALIBRADOS) ---
// 12.95 pulsos mecânicos por revolução; quadratura 4x → 51.8 pulsos efetivos.
// METROS_POR_PULSO usado no controlador interno de velocidade (Arduino).
const float METROS_POR_PULSO = 0.165 * 3.14159 / (12.95 * 4.0);

// --- POTÊNCIA ---
const int MAX_PWM = 28;
const int MIN_PWM = 13;

// --- TRIM — ajuste para andar em linha reta ---
// Robô deriva ESQUERDA → TRIM_ESQ > 0 (mais PWM no motor esquerdo)
// Robô deriva DIREITA  → TRIM_ESQ < 0 ou TRIM_DIR > 0
// Passo de ajuste sugerido: 0.02 por vez
const float TRIM_ESQ = 0.07;
const float TRIM_DIR = 0.0;

// --- ODOMETRIA ---
volatile long pulsosEsq = 0;
volatile long pulsosDir = 0;
long pulsosEsqAnt = 0;
long pulsosDirAnt = 0;
unsigned long tempoAnterior = 0;

// --- CONTROLE DE VELOCIDADE (controlador I) ---
const float KP     = 4.0;
const unsigned long CTRL_INTERVAL_MS = 200;

float velEsq   = 0.0;
float velDir   = 0.0;
float pwmEsq   = 0.0;
float pwmDir   = 0.0;

// --- COMANDOS ROS ---
float setPointEsq = 0.0;
float setPointDir = 0.0;
unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

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

  // Quadratura 4x: CHANGE em canal A e canal B de cada encoder.
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_A_L), countEsqA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_L),   countEsqB, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_A_R), countDirA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_R),   countDirB, CHANGE);
}

void loop() {
  // --- LEITURA SERIAL (ROS → Arduino) ---
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("CMD:")) {
      int sep = input.indexOf(';', 4);
      if (sep != -1) {
        setPointEsq = input.substring(4, sep).toFloat();
        setPointDir = input.substring(sep + 1).toFloat();
        lastCmdTime = millis();
        if (abs(setPointEsq) < 0.001 && abs(setPointDir) < 0.001) {
          pwmEsq = 0.0;
          pwmDir = 0.0;
          moveMotor(0, 0);
        }
      }
    }
  }

  // --- WATCHDOG: para se ROS parar de enviar ---
  if (millis() - lastCmdTime >= CMD_TIMEOUT_MS) {
    setPointEsq = 0.0;
    setPointDir = 0.0;
    pwmEsq      = 0.0;
    pwmDir      = 0.0;
  }

  // --- CICLO DE CONTROLE + ODOMETRIA ---
  unsigned long agora  = millis();
  unsigned long dt_ms  = agora - tempoAnterior;

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

    // Velocidade medida (esquerda negada: encoder conta invertido)
    velEsq = -(dEsq * METROS_POR_PULSO) / dt_s;
    velDir =  (dDir * METROS_POR_PULSO) / dt_s;

    if (abs(setPointEsq) < 0.001) {
      pwmEsq = 0.0;
    } else {
      pwmEsq += (setPointEsq - velEsq) * KP;
      pwmEsq  = constrain(pwmEsq, -100.0, 100.0);
    }

    if (abs(setPointDir) < 0.001) {
      pwmDir = 0.0;
    } else {
      pwmDir += (setPointDir - velDir) * KP;
      pwmDir  = constrain(pwmDir, -100.0, 100.0);
    }

    Serial.print("ODO:");
    Serial.print(pEsq); Serial.print(";");
    Serial.print(pDir); Serial.print(";");
    Serial.println(dt_ms);

    moveMotor((int)pwmDir, (int)pwmEsq);
    tempoAnterior = agora;
  }
}

void moveMotor(int velD, int velE) {
  if (abs(velE) < 5) velE = 0;
  if (abs(velD) < 5) velD = 0;

  int magE = constrain((int)(abs(velE) * (1.0 + TRIM_ESQ)), 0, 100);
  int magD = constrain((int)(abs(velD) * (1.0 + TRIM_DIR)), 0, 100);

  if (magE != 0) {
    int pwmE = map(magE, 5, 100, MIN_PWM, MAX_PWM);
    digitalWrite(PIN_L_BREAK, LOW);
    digitalWrite(PIN_L_DIR,   velE > 0);
    analogWrite(PIN_L_PWM, pwmE);
  } else {
    digitalWrite(PIN_L_BREAK, HIGH);
    analogWrite(PIN_L_PWM, 0);
  }

  if (magD != 0) {
    int pwmD = map(magD, 5, 100, MIN_PWM, MAX_PWM);
    digitalWrite(PIN_R_BREAK, LOW);
    digitalWrite(PIN_R_DIR,   velD < 0);
    analogWrite(PIN_R_PWM, pwmD);
  } else {
    digitalWrite(PIN_R_BREAK, HIGH);
    analogWrite(PIN_R_PWM, 0);
  }
}

// --- QUADRATURA 4x: canal A e canal B como interrupção (CHANGE) ---
// Lógica derivada de odometrypulse-malha: ODO_L/ODO_R definem direção.
// countEsqA/countDirA: acionadas por qualquer borda do canal A.
// countEsqB/countDirB: acionadas por qualquer borda do canal B.

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
