#include <PinChangeInterrupt.h>

// ================= PINOS MOTORES =================
#define PIN_L_DIR   4
#define PIN_L_PWM   5
#define PIN_L_BREAK 11

#define PIN_R_DIR   7
#define PIN_R_PWM   9
#define PIN_R_BREAK 13

// ================= PINOS ODOMETRIA =================
#define ODO_L   10
#define ODO_R   6
#define ODO_A_L 2
#define ODO_A_R 3

// ================= PARÂMETROS FÍSICOS =================
const float DIAMETRO_RODA = 0.165;
const float METROS_POR_PULSO = DIAMETRO_RODA * 3.14159 / (12.95 * 4.0);

// ================= POTÊNCIA =================
// Mais faixa para o controlador corrigir diferenças entre motores
const int MAX_PWM = 100;
const int MIN_PWM = 25;

// ================= TRIM =================
// Ajuste inicial para compensar diferença (~11%) entre motores
const float TRIM_ESQ = 0.11;
const float TRIM_DIR = 0.00;

// ================= ODOMETRIA =================
volatile long pulsosEsq = 0;
volatile long pulsosDir = 0;

long pulsosEsqAnt = 0;
long pulsosDirAnt = 0;

unsigned long tempoAnterior = 0;

// ================= CONTROLE =================
const float KP = 8.0;

// antes era 200 ms → muito lento
const unsigned long CTRL_INTERVAL_MS = 50;

float velEsq = 0.0;
float velDir = 0.0;

float pwmEsq = 0.0;
float pwmDir = 0.0;

// ================= COMANDOS ROS =================
float setPointEsq = 0.0;
float setPointDir = 0.0;

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_L_BREAK, OUTPUT);

  pinMode(PIN_R_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_R_BREAK, OUTPUT);

  pinMode(ODO_L, INPUT_PULLUP);
  pinMode(ODO_R, INPUT_PULLUP);
  pinMode(ODO_A_L, INPUT_PULLUP);
  pinMode(ODO_A_R, INPUT_PULLUP);

  // Encoder quadratura 4x
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_A_L), countEsqA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_L),   countEsqB, CHANGE);

  attachPinChangeInterrupt(digitalPinToPCINT(ODO_A_R), countDirA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(ODO_R),   countDirB, CHANGE);

  tempoAnterior = millis();
}

void loop() {

  // ================= RECEBE CMD DO ROS =================
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("CMD:")) {
      int sep = input.indexOf(';', 4);

      if (sep != -1) {
        setPointEsq = input.substring(4, sep).toFloat();
        setPointDir = input.substring(sep + 1).toFloat();

        lastCmdTime = millis();

        if (abs(setPointEsq) < 0.001 &&
            abs(setPointDir) < 0.001) {
          pwmEsq = 0.0;
          pwmDir = 0.0;
          moveMotor(0, 0);
        }
      }
    }
  }

  // ================= WATCHDOG =================
  if (millis() - lastCmdTime >= CMD_TIMEOUT_MS) {
    setPointEsq = 0.0;
    setPointDir = 0.0;
    pwmEsq = 0.0;
    pwmDir = 0.0;
  }

  // ================= LOOP DE CONTROLE =================
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

    // encoder esquerdo invertido
    velEsq = -(dEsq * METROS_POR_PULSO) / dt_s;
    velDir =  (dDir * METROS_POR_PULSO) / dt_s;

    // ================= CONTROLADOR P =================
    if (abs(setPointEsq) < 0.001) {
      pwmEsq = 0.0;
    } else {
      pwmEsq += (setPointEsq - velEsq) * KP;
      pwmEsq = constrain(pwmEsq, -100.0, 100.0);
    }

    if (abs(setPointDir) < 0.001) {
      pwmDir = 0.0;
    } else {
      pwmDir += (setPointDir - velDir) * KP;
      pwmDir = constrain(pwmDir, -100.0, 100.0);
    }

    // envia odometria pro ROS
    Serial.print("ODO:");
    Serial.print(pEsq);
    Serial.print(";");
    Serial.print(pDir);
    Serial.print(";");
    Serial.println(dt_ms);

    moveMotor((int)pwmDir, (int)pwmEsq);

    tempoAnterior = agora;
  }
}

// ===================================================
// CONTROLE DOS MOTORES
// ===================================================
void moveMotor(int velD, int velE) {

  if (abs(velE) < 5) velE = 0;
  if (abs(velD) < 5) velD = 0;

  int magE = constrain(
    (int)(abs(velE) * (1.0 + TRIM_ESQ)),
    0,
    100
  );

  int magD = constrain(
    (int)(abs(velD) * (1.0 + TRIM_DIR)),
    0,
    100
  );

  // -------- motor esquerdo --------
  if (magE != 0) {
    int pwmE = map(magE, 5, 100, MIN_PWM, MAX_PWM);

    digitalWrite(PIN_L_BREAK, LOW);
    digitalWrite(PIN_L_DIR, velE > 0);

    analogWrite(PIN_L_PWM, pwmE);
  } else {
    digitalWrite(PIN_L_BREAK, HIGH);
    analogWrite(PIN_L_PWM, 0);
  }

  // -------- motor direito --------
  if (magD != 0) {
    int pwmD = map(magD, 5, 100, MIN_PWM, MAX_PWM);

    digitalWrite(PIN_R_BREAK, LOW);

    // seu driver direito está invertido
    digitalWrite(PIN_R_DIR, velD < 0);

    analogWrite(PIN_R_PWM, pwmD);
  } else {
    digitalWrite(PIN_R_BREAK, HIGH);
    analogWrite(PIN_R_PWM, 0);
  }
}

// ===================================================
// ENCODER QUADRATURA 4x
// ===================================================
void countEsqA() {
  bool a = digitalRead(ODO_A_L);
  bool b = digitalRead(ODO_L);

  if (a ^ b)
    pulsosEsq++;
  else
    pulsosEsq--;
}

void countEsqB() {
  bool a = digitalRead(ODO_A_L);
  bool b = digitalRead(ODO_L);

  if (a == b)
    pulsosEsq++;
  else
    pulsosEsq--;
}

void countDirA() {
  bool a = digitalRead(ODO_A_R);
  bool b = digitalRead(ODO_R);

  if (a ^ b)
    pulsosDir++;
  else
    pulsosDir--;
}

void countDirB() {
  bool a = digitalRead(ODO_A_R);
  bool b = digitalRead(ODO_R);

  if (a == b)
    pulsosDir++;
  else
    pulsosDir--;
}