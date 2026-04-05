// ═══════════════════════════════════════════════════════════════════════
//  DIFF DRIVE v5 — DUMB EXECUTOR  (servo pen-lift edition)
//  Arduino only drives wheels + servo. Python does ALL path computation.
//
//  Commands:
//    STEPS L:<n> R:<n>   move wheels (signed step counts)
//    SPEED <us>          draw step delay µs  (100-5000)
//    TSPEED <us>         travel step delay µs (100-5000, default 120)
//    PEN DOWN            lower pen (servo to penDownAngle)
//    PEN UP              raise pen (servo to penUpAngle)
//    SERVO <pin> <down°> <up°>   reconfigure servo at runtime
//    STOP                halt immediately, disable motors, raise pen
//    ENABLE              re-enable motors
//    STATUS              report state
//    PING                reply OK PONG
//    PID Kp Ki Kd DZ     tune straight-line PID
//
//  Wiring:
//    STEP_L=2  DIR_L=3  STEP_R=6  DIR_R=7
//    EN_ALL=8  (active LOW, shared)
//    FLT_L=9   FLT_R=10 (INPUT_PULLUP, LOW=fault)
//    SERVO_PIN=11 (configurable via SERVO command)
//
//  Motor direction flags — toggle if a wheel runs backward:
//    INVERT_L true   INVERT_R false
//
//  Speed note: executeSteps() automatically uses travelDelay when pen is
//  up and stepDelay when pen is down — no explicit speed-switch needed.
// ═══════════════════════════════════════════════════════════════════════

#include <Servo.h>

#define STEP_L  2
#define DIR_L   3
#define STEP_R  6
#define DIR_R   7
#define EN_ALL  8
#define FLT_L   10
#define FLT_R   11

#define INVERT_L true
#define INVERT_R false

#define SPEED_MIN    100
#define SPEED_MAX   5000
#define SPEED_DEF    300
#define TSPEED_DEF   120    // fast travel speed (pen up)

#define RAMP_STEPS  60
#define RAMP_START 1500

// Servo defaults (overridable via SERVO command)
#define SERVO_PIN_DEF  13
#define PEN_DOWN_DEF   30
#define PEN_UP_DEF     90

int stepDelay   = SPEED_DEF;
int travelDelay = TSPEED_DEF;
volatile bool stopFlag = false;
bool motorsEnabled = false;
String buf = "";

// ── Servo / pen ────────────────────────────────────────────────────────
Servo penServo;
int servoPin     = SERVO_PIN_DEF;
int penDownAngle = PEN_DOWN_DEF;
int penUpAngle   = PEN_UP_DEF;
bool penIsDown   = false;

void attachServo() {
  if (!penServo.attached()) penServo.attach(servoPin);
}

void penDown() {
  attachServo();
  penServo.write(penDownAngle);
  penIsDown = true;
  delay(80);
}

void penUp() {
  attachServo();
  penServo.write(penUpAngle);
  penIsDown = false;
  delay(80);
}

// ── Motor helpers ──────────────────────────────────────────────────────
void enableMotors()  { digitalWrite(EN_ALL, LOW);  motorsEnabled = true; }
void disableMotors() { digitalWrite(EN_ALL, HIGH); motorsEnabled = false; }

void setDirL(bool fwd) { digitalWrite(DIR_L, (fwd ^ INVERT_L) ? HIGH : LOW); }
void setDirR(bool fwd) { digitalWrite(DIR_R, (fwd ^ INVERT_R) ? HIGH : LOW); }

bool checkFault() {
  bool fl = digitalRead(FLT_L) == LOW;
  bool fr = digitalRead(FLT_R) == LOW;
  if (fl || fr) {
    disableMotors(); stopFlag = true;
    penUp();
    Serial.print(F("ERR FAULT"));
    if (fl) Serial.print(F(" LEFT"));
    if (fr) Serial.print(F(" RIGHT"));
    Serial.println(F(" — power cycle 12V to reset"));
    return true;
  }
  return false;
}

// Ramp delay for step i of total, using provided cruise speed
int rampDelay(long i, long total, int cruise) {
  int start = max(RAMP_START, cruise * 5);
  long ramp  = min((long)RAMP_STEPS, total / 2);
  if (ramp == 0) return cruise;
  if (i < ramp)
    return (int)(start - (long)(start - cruise) * i / ramp);
  if (i >= total - ramp)
    return (int)(start - (long)(start - cruise) * (total - 1 - i) / ramp);
  return cruise;
}

// ── Straight-line PID ──────────────────────────────────────────────────
float pid_kp = 1.0f, pid_ki = 0.02f, pid_kd = 0.05f, pid_dz = 2.0f;
float pid_intg = 0.0f, pid_prev = 0.0f;

int pidUpdate(float err) {
  if (fabsf(err) < pid_dz) { pid_intg = 0; pid_prev = err; return 0; }
  pid_intg = constrain(pid_intg + err, -400.0f, 400.0f);
  float d = err - pid_prev; pid_prev = err;
  return (int)(pid_kp * err + pid_ki * pid_intg + pid_kd * d);
}

// ── Core: execute L and R step counts ─────────────────────────────────
// Speed is chosen automatically: travelDelay when pen up, stepDelay when down.
void executeSteps(long stL, long stR) {
  if (stL == 0 && stR == 0) { Serial.println(F("OK")); return; }
  if (checkFault()) return;
  enableMotors();

  int cruise = penIsDown ? stepDelay : travelDelay;

  setDirL(stL >= 0);
  setDirR(stR >= 0);
  long absL = abs(stL), absR = abs(stR);
  long total = max(absL, absR);

  long errL = total / 2;
  long errR = total / 2;

  bool straight = (stL == stR && stL != 0);
  pid_intg = 0; pid_prev = 0;
  int pidAdj = 0;
  long doneL = 0, doneR = 0;

  for (long i = 0; i < total; i++) {
    if (stopFlag) { disableMotors(); return; }
    if (i % 200 == 0 && i > 0 && checkFault()) return;

    int d = rampDelay(i, total, cruise);

    errL += absL;
    if (errL >= total) {
      errL -= total;
      digitalWrite(STEP_L, HIGH);
      delayMicroseconds(max(2, d - pidAdj - 2));
      digitalWrite(STEP_L, LOW);
      doneL++;
    }

    errR += absR;
    if (errR >= total) {
      errR -= total;
      digitalWrite(STEP_R, HIGH);
      delayMicroseconds(max(2, d + pidAdj - 2));
      digitalWrite(STEP_R, LOW);
      doneR++;
    }

    delayMicroseconds(2);

    if (straight && (i & 0x3F) == 0 && i > 0)
      pidAdj = constrain(pidUpdate((float)(doneL - doneR)), -(d/2), (d/2));
  }

  Serial.println(F("OK"));
}

// ── Setup ──────────────────────────────────────────────────────────────
void setup() {
  pinMode(STEP_L, OUTPUT); pinMode(DIR_L,  OUTPUT);
  pinMode(STEP_R, OUTPUT); pinMode(DIR_R,  OUTPUT);
  pinMode(EN_ALL, OUTPUT);
  pinMode(FLT_L,  INPUT_PULLUP);
  pinMode(FLT_R,  INPUT_PULLUP);
  digitalWrite(STEP_L, LOW); digitalWrite(STEP_R, LOW);
  disableMotors();

  attachServo();
  penUp();   // always start with pen raised

  Serial.begin(115200);
  Serial.println(F("DIFF DRIVE v5 — servo pen-lift"));
  Serial.print(F("SERVO pin=")); Serial.print(servoPin);
  Serial.print(F(" down="));    Serial.print(penDownAngle);
  Serial.print(F(" up="));      Serial.println(penUpAngle);
  Serial.print(F("SPEED_DEF=")); Serial.print(SPEED_DEF);
  Serial.print(F(" TSPEED_DEF=")); Serial.println(TSPEED_DEF);
  Serial.println(F("OK READY"));
}

// ── serialEvent: STOP fires instantly ─────────────────────────────────
void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      if (buf.length() > 0) {
        String up = buf; up.toUpperCase();
        if (up.startsWith("STOP")) {
          stopFlag = true;
          disableMotors();
          penUp();
          Serial.println(F("OK STOP"));
          buf = ""; return;
        }
        handleCmd(buf); buf = "";
      }
    } else if (buf.length() < 64) {
      buf += c;
    }
  }
}

void loop() {
  // Nothing — all work in serialEvent + executeSteps
}

// ── Position tracking ────────────────────────────────────────────────
float currentX = 0.0, currentY = 0.0;
float stp_per_mm = 5.0; // Placeholder, must match Python's STEPS_PER_MM

// ── Kinematics helper ────────────────────────────────────────────────
void moveTo(float targetX, float targetY) {
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  // This is a simplified kinematics for differential drive.
  // Full GRBL requires a proper inverse kinematics model.
  // For now, let's just convert coordinate change to crude step counts
  // stL = ..., stR = ...
  long stL = (long)((dy - dx) * stp_per_mm);
  long stR = (long)((dy + dx) * stp_per_mm);
  executeSteps(stL, stR);
  currentX = targetX;
  currentY = targetY;
}

// ── Command handler (updated for G-code) ────────────────────────────
void handleCmd(String cmd) {
  cmd.trim();
  String cu = cmd; cu.toUpperCase();
  stopFlag = false;

  // ── G-code (G0/G1) ──────────────────────────────────────────────────
  if (cu.startsWith("G0") || cu.startsWith("G1")) {
    bool isG1 = cu.startsWith("G1");
    int xi = cu.indexOf('X');
    int yi = cu.indexOf('Y');
    int fi = cu.indexOf('F');
    if (xi < 0 || yi < 0) {
      Serial.println(F("ERR G-code needs X and Y")); return;
    }
    float tx = cu.substring(xi + 1).toFloat();
    float ty = cu.substring(yi + 1).toFloat();
    if (isG1 && fi > 0) {
      stepDelay = cu.substring(fi + 1).toInt();
    }
    moveTo(tx, ty);
    Serial.println(F("OK"));
  }

  // ── PEN ────────────────────────────────────────────────────────────
  else if (cu.startsWith("PEN")) {
    String arg = cu.substring(3); arg.trim();
    if (arg.startsWith("DOWN")) {
      penDown();
      Serial.println(F("OK PEN DOWN"));
    } else if (arg.startsWith("UP")) {
      penUp();
      Serial.println(F("OK PEN UP"));
    } else {
      Serial.println(F("ERR PEN DOWN|UP"));
    }
  }

  // ── SERVO cfg ──────────────────────────────────────────────────────
  else if (cu.startsWith("SERVO")) {
    // SERVO <pin> <downAngle> <upAngle>
    String a = cu.substring(5); a.trim();
    int s1 = a.indexOf(' ');
    if (s1 < 0) { Serial.println(F("ERR SERVO <pin> <down> <up>")); return; }
    int newPin  = a.substring(0, s1).toInt();
    a = a.substring(s1 + 1); a.trim();
    int s2 = a.indexOf(' ');
    if (s2 < 0) { Serial.println(F("ERR SERVO <pin> <down> <up>")); return; }
    int newDown = a.substring(0, s2).toInt();
    int newUp   = a.substring(s2 + 1).toInt();
    if (penServo.attached()) penServo.detach();
    servoPin     = newPin;
    penDownAngle = constrain(newDown, 0, 180);
    penUpAngle   = constrain(newUp,   0, 180);
    attachServo();
    penUp();   // move to safe position immediately
    Serial.print(F("OK SERVO pin=")); Serial.print(servoPin);
    Serial.print(F(" down="));        Serial.print(penDownAngle);
    Serial.print(F(" up="));          Serial.println(penUpAngle);
  }

  // ── TSPEED ─────────────────────────────────────────────────────────
  else if (cu.startsWith("TSPEED")) {
    int v = cu.substring(6).toInt();
    if (v < SPEED_MIN || v > SPEED_MAX) {
      Serial.print(F("ERR TSPEED ")); Serial.print(SPEED_MIN);
      Serial.print(F("-")); Serial.println(SPEED_MAX); return;
    }
    travelDelay = v;
    Serial.print(F("OK TSPEED ")); Serial.println(travelDelay);
  }

  // ── SPEED ──────────────────────────────────────────────────────────
  else if (cu.startsWith("SPEED")) {
    int v = cu.substring(5).toInt();
    if (v < SPEED_MIN || v > SPEED_MAX) {
      Serial.print(F("ERR SPEED ")); Serial.print(SPEED_MIN);
      Serial.print(F("-")); Serial.println(SPEED_MAX); return;
    }
    stepDelay = v;
    Serial.print(F("OK SPEED ")); Serial.println(stepDelay);
  }

  // ── ENABLE ─────────────────────────────────────────────────────────
  else if (cu.startsWith("ENABLE")) {
    stopFlag = false; enableMotors();
    Serial.println(F("OK ENABLED"));
  }

  // ── STATUS ─────────────────────────────────────────────────────────
  else if (cu.startsWith("STATUS")) {
    Serial.print(F("OK EN:"));    Serial.print(motorsEnabled ? "ON" : "OFF");
    Serial.print(F(" PEN:"));     Serial.print(penIsDown ? "DOWN" : "UP");
    Serial.print(F(" SPD:"));     Serial.print(stepDelay);
    Serial.print(F(" TSPD:"));    Serial.print(travelDelay);
    Serial.print(F(" SERVO:"));   Serial.print(servoPin);
    Serial.print(F(" DN:"));      Serial.print(penDownAngle);
    Serial.print(F(" UP:"));      Serial.print(penUpAngle);
    Serial.print(F(" FLT_L:"));   Serial.print(digitalRead(FLT_L)==LOW?"FAULT":"OK");
    Serial.print(F(" FLT_R:"));   Serial.println(digitalRead(FLT_R)==LOW?"FAULT":"OK");
  }

  // ── PING ───────────────────────────────────────────────────────────
  else if (cu.startsWith("PING")) {
    Serial.println(F("OK PONG"));
  }

  // ── PID ────────────────────────────────────────────────────────────
  else if (cu.startsWith("PID")) {
    String a = cmd.substring(3); a.trim();
    int s1 = a.indexOf(' ');
    if (s1 > 0) {
      pid_kp = a.substring(0, s1).toFloat();
      a = a.substring(s1+1); a.trim(); s1 = a.indexOf(' ');
      if (s1 > 0) {
        pid_ki = a.substring(0, s1).toFloat();
        a = a.substring(s1+1); a.trim(); s1 = a.indexOf(' ');
        if (s1 > 0) {
          pid_kd = a.substring(0, s1).toFloat();
          pid_dz = a.substring(s1+1).toFloat();
        }
      }
    }
    pid_intg = 0; pid_prev = 0;
    Serial.print(F("OK PID Kp=")); Serial.print(pid_kp, 3);
    Serial.print(F(" Ki="));       Serial.print(pid_ki, 4);
    Serial.print(F(" Kd="));       Serial.print(pid_kd, 3);
    Serial.print(F(" DZ="));       Serial.println(pid_dz, 1);
  }

  else {
    Serial.print(F("ERR unknown: ")); Serial.println(cmd);
  }
}
