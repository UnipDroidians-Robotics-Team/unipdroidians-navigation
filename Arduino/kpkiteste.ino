#include <PinChangeInterrupt.h>

// ======================================================
// PINOS MOTORES
// ======================================================

#define PIN_L_DIR   4
#define PIN_L_PWM   5
#define PIN_L_BREAK 11

#define PIN_R_DIR   7
#define PIN_R_PWM   9
#define PIN_R_BREAK 13

// ======================================================
// PINOS ODOMETRIA
// ======================================================

#define ODO_L   10   // Canal B esquerda
#define ODO_R   6    // Canal B direita

#define ODO_A_L 2    // Canal A esquerda
#define ODO_A_R 3    // Canal A direita

// ======================================================
// PARÂMETROS FÍSICOS
// ======================================================

// roda = 16.5 cm
// encoder = 12.95 pulsos mecânicos
// quadratura 4x = 51.8 pulsos efetivos

const float METROS_POR_PULSO =
  (0.165 * 3.14159) / (12.95 * 4.0);

// ======================================================
// PWM
// ======================================================

const int MAX_PWM = 28;
const int MIN_PWM = 13;

// ======================================================
// TRIM
// ======================================================

const float TRIM_ESQ = 0.07;
const float TRIM_DIR = 0.00;

// ======================================================
// ODOMETRIA
// ======================================================

volatile long pulsosEsq = 0;
volatile long pulsosDir = 0;

long pulsosEsqAnt = 0;
long pulsosDirAnt = 0;

unsigned long tempoAnterior = 0;

// ======================================================
// CONTROLE PI
// ======================================================

const float KP = 4.0;
const float KI = 1.2;

// intervalo do controlador
const unsigned long CTRL_INTERVAL_MS = 50;

// velocidades medidas
float velEsq = 0.0;
float velDir = 0.0;

// saída do controlador
float pwmEsq = 0.0;
float pwmDir = 0.0;

// integral acumulada
float erroIntEsq = 0.0;
float erroIntDir = 0.0;

// ======================================================
// COMANDOS ROS
// ======================================================

float setPointEsq = 0.0;
float setPointDir = 0.0;

unsigned long lastCmdTime = 0;

const unsigned long CMD_TIMEOUT_MS = 500;

// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  // ------------------------
  // motores
  // ------------------------

  pinMode(PIN_L_DIR,   OUTPUT);
  pinMode(PIN_R_DIR,   OUTPUT);

  pinMode(PIN_L_BREAK, OUTPUT);
  pinMode(PIN_R_BREAK, OUTPUT);

  // ------------------------
  // encoders
  // ------------------------

  pinMode(ODO_L,   INPUT_PULLUP);
  pinMode(ODO_R,   INPUT_PULLUP);

  pinMode(ODO_A_L, INPUT_PULLUP);
  pinMode(ODO_A_R, INPUT_PULLUP);

  // ------------------------
  // interrupções quadratura 4x
  // ------------------------

  attachPinChangeInterrupt(
    digitalPinToPCINT(ODO_A_L),
    countEsqA,
    CHANGE
  );

  attachPinChangeInterrupt(
    digitalPinToPCINT(ODO_L),
    countEsqB,
    CHANGE
  );

  attachPinChangeInterrupt(
    digitalPinToPCINT(ODO_A_R),
    countDirA,
    CHANGE
  );

  attachPinChangeInterrupt(
    digitalPinToPCINT(ODO_R),
    countDirB,
    CHANGE
  );
}

// ======================================================
// LOOP PRINCIPAL
// ======================================================

void loop() {

  // ====================================================
  // LEITURA SERIAL
  // ====================================================

  while (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');

    if (input.startsWith("CMD:")) {

      int sep = input.indexOf(';', 4);

      if (sep != -1) {

        setPointEsq = input.substring(4, sep).toFloat();
        setPointDir = input.substring(sep + 1).toFloat();

        lastCmdTime = millis();

        // parada total

        if (abs(setPointEsq) < 0.001 &&
            abs(setPointDir) < 0.001) {

          pwmEsq = 0.0;
          pwmDir = 0.0;

          erroIntEsq = 0.0;
          erroIntDir = 0.0;

          moveMotor(0, 0);
        }
      }
    }
  }

  // ====================================================
  // WATCHDOG
  // ====================================================

  if (millis() - lastCmdTime >= CMD_TIMEOUT_MS) {

    setPointEsq = 0.0;
    setPointDir = 0.0;

    pwmEsq = 0.0;
    pwmDir = 0.0;

    erroIntEsq = 0.0;
    erroIntDir = 0.0;
  }

  // ====================================================
  // CONTROLE + ODOMETRIA
  // ====================================================

  unsigned long agora = millis();

  unsigned long dt_ms = agora - tempoAnterior;

  if (dt_ms >= CTRL_INTERVAL_MS) {

    float dt_s = dt_ms * 0.001f;

    // --------------------------------------------------
    // leitura segura dos encoders
    // --------------------------------------------------

    noInterrupts();

    long pEsq = pulsosEsq;
    long pDir = pulsosDir;

    interrupts();

    // --------------------------------------------------
    // delta pulsos
    // --------------------------------------------------

    long dEsq = pEsq - pulsosEsqAnt;
    long dDir = pDir - pulsosDirAnt;

    pulsosEsqAnt = pEsq;
    pulsosDirAnt = pDir;

    // --------------------------------------------------
    // velocidade medida
    // --------------------------------------------------

    velEsq = -(dEsq * METROS_POR_PULSO) / dt_s;
    velDir =  (dDir * METROS_POR_PULSO) / dt_s;

    // ==================================================
    // CONTROLE PI ESQUERDA
    // ==================================================

    if (abs(setPointEsq) < 0.001) {

      pwmEsq = 0.0;
      erroIntEsq = 0.0;

    } else {

      float erroEsq = setPointEsq - velEsq;

      // integral
      erroIntEsq += erroEsq * dt_s;

      // anti-windup
      erroIntEsq =
        constrain(erroIntEsq, -30.0, 30.0);

      // PI
      pwmEsq =
        (KP * erroEsq) +
        (KI * erroIntEsq);

      // limite
      pwmEsq =
        constrain(pwmEsq, -100.0, 100.0);
    }

    // ==================================================
    // CONTROLE PI DIREITA
    // ==================================================

    if (abs(setPointDir) < 0.001) {

      pwmDir = 0.0;
      erroIntDir = 0.0;

    } else {

      float erroDir = setPointDir - velDir;

      // integral
      erroIntDir += erroDir * dt_s;

      // anti-windup
      erroIntDir =
        constrain(erroIntDir, -30.0, 30.0);

      // PI
      pwmDir =
        (KP * erroDir) +
        (KI * erroIntDir);

      // limite
      pwmDir =
        constrain(pwmDir, -100.0, 100.0);
    }

    // ==================================================
    // ENVIA ODOMETRIA
    // ==================================================

    Serial.print("ODO:");
    Serial.print(pEsq);
    Serial.print(";");

    Serial.print(pDir);
    Serial.print(";");

    Serial.println(dt_ms);

    // ==================================================
    // MOVE MOTORES
    // ==================================================

    moveMotor((int)pwmDir, (int)pwmEsq);

    tempoAnterior = agora;
  }
}

// ======================================================
// CONTROLE DOS MOTORES
// ======================================================

void moveMotor(int velD, int velE) {

  // deadzone

  if (abs(velE) < 5) velE = 0;
  if (abs(velD) < 5) velD = 0;

  // trim

  int magE =
    constrain(
      (int)(abs(velE) * (1.0 + TRIM_ESQ)),
      0,
      100
    );

  int magD =
    constrain(
      (int)(abs(velD) * (1.0 + TRIM_DIR)),
      0,
      100
    );

  // ====================================================
  // MOTOR ESQUERDO
  // ====================================================

  if (magE != 0) {

    int pwmE =
      map(
        magE,
        5,
        100,
        MIN_PWM,
        MAX_PWM
      );

    digitalWrite(PIN_L_BREAK, LOW);

    digitalWrite(
      PIN_L_DIR,
      velE > 0
    );

    analogWrite(PIN_L_PWM, pwmE);

  } else {

    digitalWrite(PIN_L_BREAK, HIGH);

    analogWrite(PIN_L_PWM, 0);
  }

  // ====================================================
  // MOTOR DIREITO
  // ====================================================

  if (magD != 0) {

    int pwmD =
      map(
        magD,
        5,
        100,
        MIN_PWM,
        MAX_PWM
      );

    digitalWrite(PIN_R_BREAK, LOW);

    digitalWrite(
      PIN_R_DIR,
      velD < 0
    );

    analogWrite(PIN_R_PWM, pwmD);

  } else {

    digitalWrite(PIN_R_BREAK, HIGH);

    analogWrite(PIN_R_PWM, 0);
  }
}

// ======================================================
// QUADRATURA 4X
// ======================================================

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
