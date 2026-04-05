/*
 * DIFF DRIVE PLOTTER - Arduino Firmware
 * 
 * Hardware:
 * - Arduino Uno (or compatible)
 * - 2x Stepper motors with A4988/TMC2208 drivers
 * - 1x Servo for pen lift
 * 
 * Wiring:
 * - Left stepper:  STEP=2, DIR=3, ENABLE=4
 * - Right stepper: STEP=5, DIR=6, ENABLE=7
 * - Servo:         PWM=11
 * - Motor fault:   FAULT_L=8, FAULT_R=9 (optional)
 * 
 * Serial Protocol (115200 baud):
 * - STEPS L:<n> R:<n>   Execute steps (responds OK/ERR)
 * - PEN DOWN            Lower pen
 * - PEN UP              Raise pen
 * - SPEED <us>          Set draw speed (microseconds per step)
 * - TSPEED <us>         Set travel speed (microseconds per step)
 * - SERVO <pin> <down> <up>  Configure servo
 * - PING                Check connection (responds OK)
 * - STOP                Emergency stop
 */

#include <Servo.h>

// ═══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════
#define STEP_L   2
#define DIR_L    3
#define EN_L     4
#define STEP_R   5
#define DIR_R    6
#define EN_R     7
#define FAULT_L  8
#define FAULT_R  9
#define SERVO_PIN 11

// ═══════════════════════════════════════════════════════════════
//  DEFAULTS
// ═══════════════════════════════════════════════════════════════
#define DEFAULT_DRAW_US   300
#define DEFAULT_TRAVEL_US 120
#define DEFAULT_PEN_DOWN  30
#define DEFAULT_PEN_UP    90

// ═══════════════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════════════
Servo penServo;
int servoPin     = SERVO_PIN;
int penDownAngle = DEFAULT_PEN_DOWN;
int penUpAngle   = DEFAULT_PEN_UP;
bool penIsDown   = false;
bool penConfigured = false;

volatile long drawStepDelay  = DEFAULT_DRAW_US;
volatile long travelStepDelay = DEFAULT_TRAVEL_US;
volatile bool stopRequested  = false;

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  
  // Stepper pins
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(EN_R, OUTPUT);
  
  // Fault pins (optional, pull-up if not used)
  pinMode(FAULT_L, INPUT_PULLUP);
  pinMode(FAULT_R, INPUT_PULLUP);
  
  // Disable motors initially
  disableMotors();
  
  // Servo
  penServo.attach(servoPin);
  penServo.write(penUpAngle);
  delay(500);
  
  Serial.println(F("OK Diff Drive Plotter Ready"));
}

// ═══════════════════════════════════════════════════════════════
//  MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════
void enableMotors() {
  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
}

void disableMotors() {
  digitalWrite(EN_L, HIGH);
  digitalWrite(EN_R, HIGH);
}

bool checkFault() {
  if (!digitalRead(FAULT_L)) {
    Serial.println(F("ERR Left motor fault"));
    disableMotors();
    return true;
  }
  if (!digitalRead(FAULT_R)) {
    Serial.println(F("ERR Right motor fault"));
    disableMotors();
    return true;
  }
  return false;
}

void executeSteps(long stL, long stR) {
  if (stL == 0 && stR == 0) {
    Serial.println(F("OK"));
    return;
  }
  
  if (checkFault()) return;
  
  enableMotors();
  
  // Set directions
  digitalWrite(DIR_L, stL >= 0 ? HIGH : LOW);
  digitalWrite(DIR_R, stR >= 0 ? HIGH : LOW);
  
  long absL = abs(stL);
  long absR = abs(stR);
  long total = max(absL, absR);
  
  // Bresenham-like interpolation for smooth movement
  long errL = 0, errR = 0;
  long stepL = 0, stepR = 0;
  
  for (long i = 0; i < total; i++) {
    if (stopRequested) {
      stopRequested = false;
      disableMotors();
      Serial.println(F("WARN Stopped"));
      return;
    }
    
    errL += absL;
    errR += absR;
    
    if (errL >= total) {
      errL -= total;
      stepL++;
      digitalWrite(STEP_L, HIGH);
    }
    if (errR >= total) {
      errR -= total;
      stepR++;
      digitalWrite(STEP_R, HIGH);
    }
    
    delayMicroseconds(drawStepDelay);
    
    digitalWrite(STEP_L, LOW);
    digitalWrite(STEP_R, LOW);
  }
  
  disableMotors();
  Serial.println(F("OK"));
}

// ═══════════════════════════════════════════════════════════════
//  SERVO CONTROL
// ═══════════════════════════════════════════════════════════════
void setPen(bool down) {
  if (!penConfigured) {
    penServo.attach(servoPin);
    penConfigured = true;
  }
  penServo.write(down ? penDownAngle : penUpAngle);
  penIsDown = down;
  delay(150);
}

// ═══════════════════════════════════════════════════════════════
//  SERIAL PARSER
// ═══════════════════════════════════════════════════════════════
void processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("STEPS")) {
    // Parse: STEPS L:100 R:200
    long stL = 0, stR = 0;
    int lIdx = cmd.indexOf("L:");
    int rIdx = cmd.indexOf("R:");
    
    if (lIdx >= 0 && rIdx >= 0) {
      String lStr = cmd.substring(lIdx + 2, rIdx);
      String rStr = cmd.substring(rIdx + 2);
      stL = lStr.toInt();
      stR = rStr.toInt();
      executeSteps(stL, stR);
    } else {
      Serial.println(F("ERR Bad STEPS format"));
    }
    
  } else if (cmd == "PEN DOWN") {
    setPen(true);
    Serial.println(F("OK"));
    
  } else if (cmd == "PEN UP") {
    setPen(false);
    Serial.println(F("OK"));
    
  } else if (cmd.startsWith("SPEED")) {
    // Parse: SPEED 300
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx >= 0) {
      long us = cmd.substring(spaceIdx + 1).toInt();
      if (us > 0) {
        drawStepDelay = us;
        Serial.println(F("OK"));
      } else {
        Serial.println(F("ERR Bad SPEED value"));
      }
    }
    
  } else if (cmd.startsWith("TSPEED")) {
    // Parse: TSPEED 120
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx >= 0) {
      long us = cmd.substring(spaceIdx + 1).toInt();
      if (us > 0) {
        travelStepDelay = us;
        Serial.println(F("OK"));
      } else {
        Serial.println(F("ERR Bad TSPEED value"));
      }
    }
    
  } else if (cmd.startsWith("SERVO")) {
    // Parse: SERVO 11 30 90
    int p1 = cmd.indexOf(' ');
    if (p1 >= 0) {
      String rest = cmd.substring(p1 + 1);
      int p2 = rest.indexOf(' ');
      int p3 = rest.lastIndexOf(' ');
      
      if (p2 >= 0 && p3 > p2) {
        servoPin     = rest.substring(0, p2).toInt();
        penDownAngle = rest.substring(p2 + 1, p3).toInt();
        penUpAngle   = rest.substring(p3 + 1).toInt();
        
        penServo.attach(servoPin);
        penConfigured = true;
        penServo.write(penIsDown ? penDownAngle : penUpAngle);
        
        Serial.println(F("OK"));
      } else {
        Serial.println(F("ERR Bad SERVO format"));
      }
    }
    
  } else if (cmd == "PING") {
    Serial.println(F("OK"));
    
  } else if (cmd == "STOP") {
    stopRequested = true;
    disableMotors();
    Serial.println(F("OK Stopped"));
    
  } else {
    Serial.println(F("ERR Unknown command"));
  }
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}
