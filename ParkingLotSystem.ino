// AI Declaration:
// GitHub Copilot was used to help draft code, explain parts of the assignment by drawing parallels to PLC programming and robotics principles, and help fix errors and refine the code.
// Technical background: PLC programming and robotics systems.

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

// LCD (I2C address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Ultrasonic pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 6;

// Sensors
const int TOUCH_PIN = 9;       // Touch sensor
const int LIMIT_PIN = 11;      // Limit switch
const int BUZZER_BUTTON_PIN = 8; // Buzzer mute toggle button
const int OVERRIDE_PIN = 43;   // Joystick switch (override)
const int BUZZER_PIN = 2;      // Buzzer
const int EXIT_TRIGGER_PIN = 44; // Exit limit switch (active LOW)

// LEDs
const int LED_RED = 3;
const int LED_YELLOW = 4;
const int LED_GREEN = 5;

// Servo
Servo gateServo;
Servo exitGateServo;
const int SERVO_PIN = 10;
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE = 90;
const unsigned long SERVO_MOVE_TIME_MS = 5000;
const int EXIT_SERVO_PIN = 45;
const int EXIT_SERVO_CLOSED_ANGLE = 0;
const int EXIT_SERVO_OPEN_ANGLE = 90;
const unsigned long EXIT_JOYSTICK_CLOSE_COUNTDOWN_MS = 8000;
const unsigned long EXIT_SERVO_MOVE_TIME_MS = 5000;
int currentServoAngle = SERVO_CLOSED_ANGLE;
int servoTargetAngle = SERVO_CLOSED_ANGLE;
bool servoMoving = false;
unsigned long servoStepIntervalMs = 20;
unsigned long servoLastStepMs = 0;
unsigned long servoMoveStartMs = 0;
unsigned long servoMoveDurationMs = 0;
int currentExitServoAngle = EXIT_SERVO_CLOSED_ANGLE;
int exitServoTargetAngle = EXIT_SERVO_CLOSED_ANGLE;
bool exitServoMoving = false;
unsigned long exitServoStepIntervalMs = 20;
unsigned long exitServoLastStepMs = 0;
unsigned long exitServoMoveStartMs = 0;
unsigned long exitServoMoveDurationMs = 0;
bool exitGateOpen = false;
bool exitCloseCountdownActive = false;
unsigned long exitCloseTimer = 0;
unsigned long exitSensorsReleaseStartMs = 0;
unsigned long exitLimitPressStartMs = 0;
bool exitLimitCycleSeen = false;
bool lastJoystickPressed = false;
bool lastExitLimitPressed = false;
bool exitDisplayRequestActive = false;
bool entryWaitDisplayLatched = false;
char lastExitMatrixLeft = '\0';
char lastExitMatrixRight = '\0';
bool entryMoveQueued = false;
int entryQueuedTarget = SERVO_CLOSED_ANGLE;
unsigned long entryQueuedDurationMs = SERVO_MOVE_TIME_MS;
bool exitMoveQueued = false;
int exitQueuedTarget = EXIT_SERVO_CLOSED_ANGLE;
unsigned long exitQueuedDurationMs = EXIT_SERVO_MOVE_TIME_MS;

// Gate state
bool gateOpen = false;
bool waitingToClose = false;
unsigned long closeTimer = 0;
unsigned long limitPressStartMs = 0;
unsigned long sensorsReleaseStartMs = 0;
bool proceedConfirmPending = false;
bool reclosePromptActive = false;
unsigned long proceedConfirmStartMs = 0;
unsigned long touchHoldStartMs = 0;
bool pullThroughPromptActive = false;

const unsigned long LIMIT_HOLD_TO_OPEN_MS = 800;
const unsigned long RELEASE_HOLD_TO_CLOSE_MS = 700;
const unsigned long NORMAL_CLOSE_COUNTDOWN_MS = 5000;
const unsigned long RECLOSE_COUNTDOWN_MS = 6000;
const unsigned long PROCEED_CONFIRM_TIMEOUT_MS = 10000;
const unsigned long TOUCH_HELD_PROMPT_MS = 15000;
const float OPEN_ZONE_MIN_CM = 3.5;
const float OPEN_ZONE_MAX_CM = 6.5;
const float CAUTION_ZONE_MAX_CM = 10.0;

enum ProximityZone {
  ZONE_NEXT_CAR,
  ZONE_PROCEED,
  ZONE_CAUTION,
  ZONE_SLOW,
  ZONE_STOP
};

// Hysteresis overlap to avoid rapid zone flipping near boundaries.
const float SLOW_TO_STOP_CM = OPEN_ZONE_MIN_CM;
const float STOP_TO_SLOW_CM = 5.5;
const float CAUTION_TO_SLOW_CM = 10.0;
const float SLOW_TO_CAUTION_CM = 11.5;
const float CAUTION_TO_PROCEED_CM = 12.0;
const float PROCEED_TO_NEXT_CAR_CM = 26.0;
const float NEXT_CAR_TO_PROCEED_CM = 23.0;

ProximityZone currentZone = ZONE_NEXT_CAR;

// Buzzer state
unsigned long buzzerTimer = 0;
bool buzzerState = false;
bool buzzerEnabled = true;
bool lastButtonPressed = false;
unsigned long lastDebugPrintMs = 0;
unsigned long lastPullThroughChirpMs = 0;

// Remote Uno telemetry cache
float remoteTemp = -1000;
int remoteLdr = -1;
unsigned long lastRemoteUpdate = 0;

const unsigned long DEBUG_PRINT_INTERVAL_MS = 400;
const unsigned long PULL_THROUGH_CHIRP_INTERVAL_MS = 2000;
const unsigned int PULL_THROUGH_CHIRP_FREQ_HZ = 1500;
const unsigned int PULL_THROUGH_CHIRP_MS = 80;
const unsigned long REMOTE_SERIAL_BAUD = 9600;
const int REMOTE_MSG_BUF_SIZE = 48;
const int DARK_LDR_ENTER_THRESHOLD = 180;
const int DARK_LDR_EXIT_THRESHOLD = 220;
const unsigned long DARK_MODE_ARM_MS = 20000;
const unsigned long BRIGHT_MODE_CLEAR_MS = 20000;
const unsigned long REMOTE_TELEMETRY_TIMEOUT_MS = 5000;

// LCD anti-flicker
String lastLCDLine1 = "";
String lastLCDLine2 = "";
char remoteMsgBuffer[REMOTE_MSG_BUF_SIZE];
int remoteMsgPos = 0;
unsigned long darkConditionStartMs = 0;
unsigned long brightConditionStartMs = 0;
bool darkModeActive = false;
bool ldrDarkState = false;
bool nightGracePending = false;
bool nightGraceActive = false;
int filteredRemoteLdr = -1;

String centerText16(String s) {
  if (s.length() >= 16) {
    return s.substring(0, 16);
  }

  int totalPad = 16 - s.length();
  int leftPad = totalPad / 2;
  int rightPad = totalPad - leftPad;

  String out = "";
  for (int i = 0; i < leftPad; i++) out += " ";
  out += s;
  for (int i = 0; i < rightPad; i++) out += " ";
  return out;
}

const byte LCD_ICON_SOUND_ON = 0;
const byte LCD_ICON_SOUND_OFF = 1;
const byte LCD_ICON_DAY = 2;
const byte LCD_ICON_NIGHT = 3;

byte lcdSoundOnGlyph[8] = {
  B00010,
  B00110,
  B01110,
  B11110,
  B01110,
  B00110,
  B00010,
  B00000
};

byte lcdSoundOffGlyph[8] = {
  B10001,
  B01010,
  B00100,
  B00100,
  B00100,
  B01010,
  B10001,
  B00000
};

byte lcdDayGlyph[8] = {
  B00100,
  B10101,
  B01110,
  B11111,
  B01110,
  B10101,
  B00100,
  B00000
};

byte lcdNightGlyph[8] = {
  B00110,
  B01100,
  B11000,
  B11100,
  B11100,
  B11000,
  B01100,
  B00110
};

void initLcdIcons() {
  lcd.createChar(LCD_ICON_SOUND_ON, lcdSoundOnGlyph);
  lcd.createChar(LCD_ICON_SOUND_OFF, lcdSoundOffGlyph);
  lcd.createChar(LCD_ICON_DAY, lcdDayGlyph);
  lcd.createChar(LCD_ICON_NIGHT, lcdNightGlyph);
}

void lcdSoundIcon(bool soundOn) {
  lcd.setCursor(15, 1);
  lcd.write(soundOn ? LCD_ICON_SOUND_ON : LCD_ICON_SOUND_OFF);
}

void lcdDayNightIcon(bool nightMode) {
  lcd.setCursor(14, 1);
  lcd.write(nightMode ? LCD_ICON_NIGHT : LCD_ICON_DAY);
}

void parseRemoteMessage(const char* line) {
  const char* tPtr = strstr(line, "TEMP=");
  const char* lPtr = strstr(line, "LDR=");
  if (tPtr == NULL || lPtr == NULL) {
    return;
  }

  remoteTemp = atof(tPtr + 5);
  remoteLdr = atoi(lPtr + 4);
  lastRemoteUpdate = millis();
}

void updateRemoteTelemetry() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (remoteMsgPos > 0) {
        remoteMsgBuffer[remoteMsgPos] = '\0';
        parseRemoteMessage(remoteMsgBuffer);
        remoteMsgPos = 0;
      }
      continue;
    }

    if (remoteMsgPos < (REMOTE_MSG_BUF_SIZE - 1)) {
      remoteMsgBuffer[remoteMsgPos++] = c;
    } else {
      remoteMsgPos = 0;
    }
  }
}

void lcdMessage(String line1, String line2 = "") {
  if (line1 != lastLCDLine1 || line2 != lastLCDLine2) {
    lcd.setCursor(0, 0);
    lcd.print(centerText16(line1));

    lcd.setCursor(0, 1);
    lcd.print(centerText16(line2));

    lastLCDLine1 = line1;
    lastLCDLine2 = line2;
  }
}

void runBuzzerSelfTest() {
  Serial.println("Buzzer self-test: ON");
  tone(BUZZER_PIN, 2800);
  delay(250);
  noTone(BUZZER_PIN);
  delay(120);
  tone(BUZZER_PIN, 3400);
  delay(250);
  noTone(BUZZER_PIN);
  Serial.println("Buzzer self-test: OFF");
}

void runLedSelfTest() {
  Serial.println("LED self-test: START");

  digitalWrite(LED_GREEN, HIGH);
  delay(180);
  digitalWrite(LED_GREEN, LOW);

  digitalWrite(LED_YELLOW, HIGH);
  delay(180);
  digitalWrite(LED_YELLOW, LOW);

  digitalWrite(LED_RED, HIGH);
  delay(180);
  digitalWrite(LED_RED, LOW);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(220);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.println("LED self-test: END");
}

String buildCountdownText(int remaining) {
  if (remaining < 1) remaining = 1;
  return String(remaining);
}

String buildDriverDirection(float d) {
  if (d < 0) return "AWAITING CAR";
  if (currentZone == ZONE_NEXT_CAR) return "NEXT CAR";
  if (currentZone == ZONE_PROCEED) return "PROCEED";
  if (currentZone == ZONE_CAUTION) return "CAUTION";
  if (currentZone == ZONE_SLOW) return "SLOW";
  return "STOP";
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;

  return (duration * 0.0343f) / 2.0f;
}

void startServoMoveTo(int targetAngle, unsigned long durationMs) {
  servoTargetAngle = targetAngle;

  if (servoTargetAngle == currentServoAngle) {
    servoMoving = false;
    return;
  }

  int steps = abs(servoTargetAngle - currentServoAngle);
  if (steps <= 0) {
    servoMoving = false;
    return;
  }

  servoStepIntervalMs = durationMs / (unsigned long)steps;
  if (servoStepIntervalMs < 1) {
    servoStepIntervalMs = 1;
  }

  servoMoving = true;
  servoLastStepMs = 0;
  servoMoveStartMs = millis();
  servoMoveDurationMs = durationMs;
}

void updateServoMotion() {
  if (!servoMoving) {
    return;
  }

  if (servoLastStepMs != 0 && (millis() - servoLastStepMs) < servoStepIntervalMs) {
    return;
  }
  servoLastStepMs = millis();

  int direction = (servoTargetAngle > currentServoAngle) ? 1 : -1;
  currentServoAngle += direction;
  gateServo.write(currentServoAngle);

  if (currentServoAngle == servoTargetAngle) {
    servoMoving = false;
  }
}

void startExitServoMoveTo(int targetAngle, unsigned long durationMs) {
  exitServoTargetAngle = targetAngle;

  if (exitServoTargetAngle == currentExitServoAngle) {
    exitServoMoving = false;
    return;
  }

  int steps = abs(exitServoTargetAngle - currentExitServoAngle);
  if (steps <= 0) {
    exitServoMoving = false;
    return;
  }

  exitServoStepIntervalMs = durationMs / (unsigned long)steps;
  if (exitServoStepIntervalMs < 1) {
    exitServoStepIntervalMs = 1;
  }

  exitServoMoving = true;
  exitServoLastStepMs = 0;
  exitServoMoveStartMs = millis();
  exitServoMoveDurationMs = durationMs;
}

void updateExitServoMotion() {
  if (!exitServoMoving) {
    return;
  }

  if (exitServoLastStepMs != 0 && (millis() - exitServoLastStepMs) < exitServoStepIntervalMs) {
    return;
  }
  exitServoLastStepMs = millis();

  int direction = (exitServoTargetAngle > currentExitServoAngle) ? 1 : -1;
  currentExitServoAngle += direction;
  exitGateServo.write(currentExitServoAngle);

  if (currentExitServoAngle == exitServoTargetAngle) {
    exitServoMoving = false;
  }
}

void requestEntryServoMove(int targetAngle, unsigned long durationMs) {
  if (exitServoMoving) {
    entryMoveQueued = true;
    entryQueuedTarget = targetAngle;
    entryQueuedDurationMs = durationMs;
    return;
  }

  startServoMoveTo(targetAngle, durationMs);
}

void requestExitServoMove(int targetAngle, unsigned long durationMs) {
  if (servoMoving) {
    exitMoveQueued = true;
    exitQueuedTarget = targetAngle;
    exitQueuedDurationMs = durationMs;
    return;
  }

  startExitServoMoveTo(targetAngle, durationMs);
}

void serviceQueuedServoMoves() {
  if (servoMoving || exitServoMoving) {
    return;
  }

  if (entryMoveQueued) {
    entryMoveQueued = false;
    startServoMoveTo(entryQueuedTarget, entryQueuedDurationMs);
    return;
  }

  if (exitMoveQueued) {
    exitMoveQueued = false;
    startExitServoMoveTo(exitQueuedTarget, exitQueuedDurationMs);
  }
}

void updateZoneByDistance(float d) {
  if (d < 0) {
    return;
  }

  if (currentZone == ZONE_NEXT_CAR) {
    if (d < NEXT_CAR_TO_PROCEED_CM) currentZone = ZONE_PROCEED;
  }
  else if (currentZone == ZONE_PROCEED) {
    if (d >= PROCEED_TO_NEXT_CAR_CM) currentZone = ZONE_NEXT_CAR;
    else if (d <= CAUTION_ZONE_MAX_CM) currentZone = ZONE_CAUTION;
  }
  else if (currentZone == ZONE_CAUTION) {
    if (d >= CAUTION_TO_PROCEED_CM) currentZone = ZONE_PROCEED;
    else if (d <= CAUTION_TO_SLOW_CM) currentZone = ZONE_SLOW;
  }
  else if (currentZone == ZONE_SLOW) {
    if (d >= SLOW_TO_CAUTION_CM) currentZone = ZONE_CAUTION;
    else if (d <= SLOW_TO_STOP_CM) currentZone = ZONE_STOP;
  }
  else if (currentZone == ZONE_STOP) {
    if (d > STOP_TO_SLOW_CM) currentZone = ZONE_SLOW;
  }
}

void sendMatrix(const char* msg) {
  Serial2.println(msg);
}

int getExitMoveDisplayCount() {
  int totalTravel = abs(EXIT_SERVO_OPEN_ANGLE - EXIT_SERVO_CLOSED_ANGLE);
  if (totalTravel <= 0) {
    return 1;
  }

  int remainingTravel = abs(exitServoTargetAngle - currentExitServoAngle);
  if (remainingTravel <= 0) {
    return 1;
  }

  int segments = 5;
  int remaining = (remainingTravel * segments + totalTravel - 1) / totalTravel;
  if (remaining < 1) remaining = 1;
  if (remaining > segments) remaining = segments;
  return remaining;
}

int getMoveTimeDisplayCount(unsigned long moveStartMs, unsigned long moveDurationMs) {
  if (moveDurationMs == 0) {
    return 1;
  }

  unsigned long elapsed = millis() - moveStartMs;
  if (elapsed >= moveDurationMs) {
    return 1;
  }

  unsigned long remainingMs = moveDurationMs - elapsed;
  int remaining = (int)((remainingMs + 999UL) / 1000UL);
  if (remaining < 1) remaining = 1;
  if (remaining > 5) remaining = 5;
  return remaining;
}

int getEntryCloseDisplayCount() {
  bool gateClosingMotion = servoMoving && (servoTargetAngle == SERVO_CLOSED_ANGLE);
  if (gateClosingMotion) {
    return getMoveTimeDisplayCount(servoMoveStartMs, servoMoveDurationMs);
  }

  return 1;
}

int getExitCloseDisplayCount() {
  bool exitClosingMotion = exitServoMoving && (exitServoTargetAngle == EXIT_SERVO_CLOSED_ANGLE);
  if (exitClosingMotion) {
    return getMoveTimeDisplayCount(exitServoMoveStartMs, exitServoMoveDurationMs);
  }

  if (exitCloseCountdownActive && exitGateOpen) {
    unsigned long activeCloseMs = exitLimitCycleSeen ? NORMAL_CLOSE_COUNTDOWN_MS : EXIT_JOYSTICK_CLOSE_COUNTDOWN_MS;
    unsigned long elapsed = millis() - exitCloseTimer;
    if (elapsed >= activeCloseMs) {
      return 1;
    }

    int remaining = (int)((activeCloseMs - elapsed + 999UL) / 1000UL);
    if (remaining < 1) remaining = 1;
    if (remaining > 9) remaining = 9;
    return remaining;
  }

  return 1;
}

void sendExitMatrixStatus() {
  char nextLeft = 'C';
  char nextRight = '0';
  bool exitInteractionActive = exitDisplayRequestActive || exitGateOpen || exitServoMoving || exitCloseCountdownActive || exitMoveQueued || exitLimitCycleSeen;
  bool waitingOtherGate = (entryMoveQueued || exitMoveQueued) && (servoMoving || exitServoMoving);
  bool entryActiveBlockingExit = exitInteractionActive && !isEntryGateClosedForExit() && !exitGateOpen && !exitServoMoving;

  if (entryActiveBlockingExit) {
    nextLeft = 'W';
    bool entryCloseState = servoMoving && (servoTargetAngle == SERVO_CLOSED_ANGLE);
    if (entryCloseState) {
      int entryCount = getEntryCloseDisplayCount();
      nextRight = (char)('0' + entryCount);
    } else {
      nextRight = '?';
    }
  } else if (waitingOtherGate) {
    int waitCount = 1;
    if (entryMoveQueued && exitServoMoving) {
      waitCount = getMoveTimeDisplayCount(exitServoMoveStartMs, exitServoMoveDurationMs);
    } else if (exitMoveQueued && servoMoving) {
      waitCount = getMoveTimeDisplayCount(servoMoveStartMs, servoMoveDurationMs);
    }
    nextLeft = 'W';
    nextRight = (char)('0' + waitCount);
  } else if (exitServoMoving) {
    int remaining = getExitMoveDisplayCount();

    nextLeft = (exitServoTargetAngle == EXIT_SERVO_OPEN_ANGLE) ? 'O' : 'C';
    nextRight = (char)('0' + remaining);
  } else if (exitGateOpen || currentExitServoAngle == EXIT_SERVO_OPEN_ANGLE) {
    nextLeft = 'O';
    nextRight = '0';
  }

  if (nextLeft == lastExitMatrixLeft && nextRight == lastExitMatrixRight) {
    return;
  }

  char msg[3] = { nextLeft, nextRight, '\0' };
  sendMatrix(msg);
  lastExitMatrixLeft = nextLeft;
  lastExitMatrixRight = nextRight;
}

bool isEntryGateClosedForExit() {
  return !gateOpen &&
         !waitingToClose &&
         !servoMoving &&
         (currentServoAngle == SERVO_CLOSED_ANGLE);
}

bool isExitGateClosedForEntry() {
  return !exitGateOpen &&
         !exitCloseCountdownActive &&
         !exitServoMoving &&
         (currentExitServoAngle == EXIT_SERVO_CLOSED_ANGLE);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(REMOTE_SERIAL_BAUD);
  Serial2.begin(9600);

  lcd.init();
  lcd.backlight();
  initLcdIcons();
  lcd.clear();
  lcd.print("System Ready");
  delay(800);
  lcd.clear();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(TOUCH_PIN, INPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(BUZZER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(OVERRIDE_PIN, INPUT_PULLUP);
  pinMode(EXIT_TRIGGER_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  runLedSelfTest();
  runBuzzerSelfTest();

  gateServo.attach(SERVO_PIN);
  gateServo.write(SERVO_CLOSED_ANGLE);  // gate closed
  currentServoAngle = SERVO_CLOSED_ANGLE;
  servoTargetAngle = SERVO_CLOSED_ANGLE;

  exitGateServo.attach(EXIT_SERVO_PIN);
  exitGateServo.write(EXIT_SERVO_CLOSED_ANGLE);
  currentExitServoAngle = EXIT_SERVO_CLOSED_ANGLE;
  exitServoTargetAngle = EXIT_SERVO_CLOSED_ANGLE;
}

void loop() {
  updateServoMotion();
  updateExitServoMotion();
  serviceQueuedServoMoves();

  float d = getDistance();
  updateZoneByDistance(d);

  bool touchPressed = digitalRead(TOUCH_PIN) == HIGH;
  bool limitPressed = digitalRead(LIMIT_PIN) == LOW;
  bool buzzerButtonPressed = (digitalRead(BUZZER_BUTTON_PIN) == LOW);
  bool joystickPressed = (digitalRead(OVERRIDE_PIN) == LOW);
  bool exitLimitPressed = (digitalRead(EXIT_TRIGGER_PIN) == LOW);

  updateRemoteTelemetry();

  if (limitPressed) {
    if (limitPressStartMs == 0) limitPressStartMs = millis();
  } else {
    limitPressStartMs = 0;
  }

  bool limitStablePressed = (limitPressStartMs != 0) && ((millis() - limitPressStartMs) >= LIMIT_HOLD_TO_OPEN_MS);
  bool gateOpeningMotion = servoMoving && (servoTargetAngle == SERVO_OPEN_ANGLE);
  bool gateClosingMotion = servoMoving && (servoTargetAngle == SERVO_CLOSED_ANGLE);
  bool carInOpenRange = (currentZone == ZONE_SLOW || currentZone == ZONE_STOP);
  bool exitLimitStablePressed = false;

  if (exitLimitPressed) {
    if (exitLimitPressStartMs == 0) {
      exitLimitPressStartMs = millis();
    }
  } else {
    exitLimitPressStartMs = 0;
  }
  exitLimitStablePressed = (exitLimitPressStartMs != 0) && ((millis() - exitLimitPressStartMs) >= LIMIT_HOLD_TO_OPEN_MS);

  bool telemetryFresh = (lastRemoteUpdate != 0) && ((millis() - lastRemoteUpdate) <= REMOTE_TELEMETRY_TIMEOUT_MS);
  bool remoteLdrValid = telemetryFresh && (remoteLdr >= 0);

  if (remoteLdrValid) {
    if (filteredRemoteLdr == -1) filteredRemoteLdr = remoteLdr;
    else filteredRemoteLdr = (filteredRemoteLdr * 3 + remoteLdr) / 4;

    if (!ldrDarkState && filteredRemoteLdr <= DARK_LDR_ENTER_THRESHOLD) {
      ldrDarkState = true;
    } else if (ldrDarkState && filteredRemoteLdr >= DARK_LDR_EXIT_THRESHOLD) {
      ldrDarkState = false;
    }
  }

  bool darkReading = remoteLdrValid && ldrDarkState;
  bool brightReading = remoteLdrValid && !ldrDarkState;
  if (darkReading) {
    brightConditionStartMs = 0;
    if (darkConditionStartMs == 0) {
      darkConditionStartMs = millis();
      // One-entry grace: if a vehicle was already at gate trigger when darkness started.
      nightGracePending = gateOpen || waitingToClose || limitStablePressed || touchPressed;
    }
    if (!darkModeActive && (millis() - darkConditionStartMs >= DARK_MODE_ARM_MS)) {
      darkModeActive = true;
      nightGraceActive = nightGracePending;
      nightGracePending = false;
      Serial.println("Night mode ACTIVE");
    }
  } else if (brightReading) {
    darkConditionStartMs = 0;
    nightGracePending = false;

    if (darkModeActive) {
      if (brightConditionStartMs == 0) {
        brightConditionStartMs = millis();
      }
      if (millis() - brightConditionStartMs >= BRIGHT_MODE_CLEAR_MS) {
        darkModeActive = false;
        nightGraceActive = false;
        brightConditionStartMs = 0;
        Serial.println("Night mode CLEARED");
      }
    }
  } else {
    // No valid remote LDR reading: do not arm or clear mode transitions.
    brightConditionStartMs = 0;
    if (!darkModeActive) {
      darkConditionStartMs = 0;
      nightGracePending = false;
    }
  }

  if (!telemetryFresh) {
    filteredRemoteLdr = -1;
    ldrDarkState = false;
  }

  // Exit gate logic: independent of ultrasonic/day-night and timed like the entry gate.
  if (joystickPressed && !lastJoystickPressed) {
    exitDisplayRequestActive = true;
    if (!isEntryGateClosedForExit()) {
      Serial.println("Exit open blocked: entry gate not closed");
    }
    else if (!exitGateOpen) {
      exitGateOpen = true;
      requestExitServoMove(EXIT_SERVO_OPEN_ANGLE, EXIT_SERVO_MOVE_TIME_MS);
      Serial.println("Exit gate OPEN (joystick)");
      // Only reset cycle tracking on a fresh open, not mid-cycle.
      exitCloseCountdownActive = false;
      exitSensorsReleaseStartMs = 0;
      exitCloseTimer = 0;
      exitLimitCycleSeen = false;
    } else {
      // Gate already open: restart countdown from now without resetting limit cycle state.
      exitCloseCountdownActive = false;
      exitSensorsReleaseStartMs = 0;
      exitCloseTimer = 0;
      Serial.println("Exit countdown restarted (joystick)");
    }
  }

  if (exitLimitStablePressed) {
    exitDisplayRequestActive = true;
    if (!isEntryGateClosedForExit() && !exitGateOpen) {
      Serial.println("Exit open blocked: entry gate not closed");
    }
    else {
      exitCloseCountdownActive = false;
      exitSensorsReleaseStartMs = 0;
      exitLimitCycleSeen = true;
      if (!exitGateOpen) {
        exitGateOpen = true;
        requestExitServoMove(EXIT_SERVO_OPEN_ANGLE, EXIT_SERVO_MOVE_TIME_MS);
        Serial.println("Exit gate OPEN (limit)");
      }
    }
  }

  bool exitFullyOpen = exitGateOpen && !exitServoMoving && (currentExitServoAngle == EXIT_SERVO_OPEN_ANGLE);

  // Joystick-only: start countdown once servo is fully open.
  // Guard: only start the hold timer if the servo just arrived (exitCloseTimer == 0 means fresh open).
  if (!exitLimitCycleSeen && exitFullyOpen && !exitCloseCountdownActive && exitCloseTimer == 0) {
    if (exitSensorsReleaseStartMs == 0) {
      exitSensorsReleaseStartMs = millis();
    }
    if ((millis() - exitSensorsReleaseStartMs) >= RELEASE_HOLD_TO_CLOSE_MS) {
      exitCloseCountdownActive = true;
      exitCloseTimer = millis();
      exitSensorsReleaseStartMs = 0;
      Serial.println("Exit close countdown started (joystick)");
    }
  }

  // Limit cycle: countdown starts after servo is fully open and limit is released.
  if (exitLimitCycleSeen && exitFullyOpen && !exitLimitPressed && !exitCloseCountdownActive) {
    if (exitSensorsReleaseStartMs == 0) {
      exitSensorsReleaseStartMs = millis();
    }

    if ((millis() - exitSensorsReleaseStartMs) >= RELEASE_HOLD_TO_CLOSE_MS) {
      exitCloseCountdownActive = true;
      exitCloseTimer = millis();
      exitSensorsReleaseStartMs = 0;
      Serial.println("Exit close countdown started (limit)");
    }
  }

  if (exitGateOpen && exitLimitPressed && exitSensorsReleaseStartMs != 0) {
    exitSensorsReleaseStartMs = 0;
  }

  unsigned long exitActiveCloseCountdownMs = exitLimitCycleSeen ? NORMAL_CLOSE_COUNTDOWN_MS : EXIT_JOYSTICK_CLOSE_COUNTDOWN_MS;
  if (exitGateOpen && exitCloseCountdownActive && (millis() - exitCloseTimer >= exitActiveCloseCountdownMs)) {
    exitGateOpen = false;
    exitCloseCountdownActive = false;
    exitSensorsReleaseStartMs = 0;
    exitLimitCycleSeen = false;
    requestExitServoMove(EXIT_SERVO_CLOSED_ANGLE, EXIT_SERVO_MOVE_TIME_MS);
    Serial.println("Exit gate CLOSED");
  }

  bool exitIdleClosed = !exitGateOpen && !exitServoMoving && !exitCloseCountdownActive && !exitLimitCycleSeen && !exitMoveQueued && (currentExitServoAngle == EXIT_SERVO_CLOSED_ANGLE);
  if (exitIdleClosed && isEntryGateClosedForExit() && !joystickPressed && !exitLimitPressed) {
    exitDisplayRequestActive = false;
  }

  sendExitMatrixStatus();

  lastJoystickPressed = joystickPressed;
  lastExitLimitPressed = exitLimitPressed;

  // If touch stays active too long while gate is open, prompt driver to continue through.
  if (gateOpen && touchPressed) {
    if (touchHoldStartMs == 0) {
      touchHoldStartMs = millis();
    }
    pullThroughPromptActive = (millis() - touchHoldStartMs >= TOUCH_HELD_PROMPT_MS);
  } else {
    touchHoldStartMs = 0;
    pullThroughPromptActive = false;
  }

  // D8 button toggles run mute on/off for the buzzer.
  if (buzzerButtonPressed && !lastButtonPressed) {
    buzzerEnabled = !buzzerEnabled;
    buzzerState = false;
    noTone(BUZZER_PIN);
    Serial.println(buzzerEnabled ? "Buzzer ON" : "Buzzer muted");
  }
  lastButtonPressed = buzzerButtonPressed;

  // Throttled debug output for readability in Serial Monitor.
  if (millis() - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMs = millis();

    Serial.print("Distance: ");
    Serial.print(d);
    Serial.print(" | TOUCH: ");
    Serial.print(touchPressed ? "PRESSED" : "RELEASED");
    Serial.print(" | LIMIT: ");
    Serial.print(limitPressed ? "PRESSED" : "RELEASED");
    Serial.print(limitStablePressed ? "(STABLE)" : "");
    Serial.print(" | BZR_BTN: ");
    Serial.print(buzzerButtonPressed ? "PRESSED" : "RELEASED");
    Serial.print(" | JOY: ");
    Serial.print(joystickPressed ? "PRESSED" : "RELEASED");
    Serial.print(" | EXIT_LIMIT: ");
    Serial.print(exitLimitPressed ? "PRESSED" : "RELEASED");
    Serial.print(" | BUZZER: ");
    Serial.print(buzzerEnabled ? "ENABLED" : "MUTED");
    Serial.print(" | WAITING: ");
    Serial.print(waitingToClose ? "YES" : "NO");
    Serial.print(" | GATE: ");
    Serial.print(gateOpen ? "OPEN" : "CLOSED");
    Serial.print(" | LDR: ");
    Serial.print(remoteLdr);
    Serial.print(" | NIGHT: ");
    Serial.println(darkModeActive ? "YES" : "NO");
  }

  // -------------------------
  // LED ZONES
  // -------------------------
  if (d < 0) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
  }
  else if (currentZone == ZONE_NEXT_CAR || currentZone == ZONE_PROCEED) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
  }
  else if (currentZone == ZONE_CAUTION) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, LOW);
  }
  else if (currentZone == ZONE_SLOW) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, HIGH);
  }
  else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH);
  }

  // -------------------------
  // LCD MESSAGES
  // -------------------------
  String directionText = buildDriverDirection(d);
  String gateText = "CLOSED";

  // When the gate is available/opening, show a clear driver action.
  if (gateOpen || gateOpeningMotion) {
    directionText = "DRIVE FORWARD";
  }

  if (waitingToClose && reclosePromptActive) {
    int remaining = (int)((RECLOSE_COUNTDOWN_MS - (millis() - closeTimer) + 999UL) / 1000UL);
    if (remaining < 1) remaining = 1;
    if (remaining > 6) remaining = 6;
    directionText = "TOUCH TO GO";
    gateText = "REV OUT IN " + buildCountdownText(remaining);
  }
  else if (pullThroughPromptActive) {
    directionText = "PLEASE PULL";
    gateText = "THROUGH FULLY";
  }
  else if (waitingToClose) {
    int remaining = (int)((NORMAL_CLOSE_COUNTDOWN_MS - (millis() - closeTimer) + 999UL) / 1000UL);
    if (remaining < 1) remaining = 1;
    if (remaining > 5) remaining = 5;
    gateText = "CLOSING IN " + buildCountdownText(remaining);
  }
  else if (gateOpeningMotion) {
    unsigned long elapsed = millis() - servoMoveStartMs;
    int remaining = 1;
    if (servoMoveDurationMs > elapsed) {
      remaining = (int)((servoMoveDurationMs - elapsed + 999UL) / 1000UL);
      if (remaining < 1) remaining = 1;
      if (remaining > 5) remaining = 5;
    }
    gateText = "OPEN IN " + buildCountdownText(remaining);
  }
  else if (gateClosingMotion) {
    unsigned long elapsed = millis() - servoMoveStartMs;
    int remaining = 1;
    if (servoMoveDurationMs > elapsed) {
      remaining = (int)((servoMoveDurationMs - elapsed + 999UL) / 1000UL);
      if (remaining < 1) remaining = 1;
      if (remaining > 5) remaining = 5;
    }
    gateText = "CLOSED IN " + buildCountdownText(remaining);
  }
  else if (gateOpen) {
    gateText = proceedConfirmPending ? "WAIT TOUCH" : "OPEN";
  }
  else if (!gateOpen && !waitingToClose && !servoMoving) {
    if (remoteTemp > -100) {
      gateText = "TEMP " + String(remoteTemp, 1) + "C";
    }
  }

  if (darkModeActive && !nightGraceActive) {
    directionText = "LOT CLOSED";
    if (remoteTemp > -100) {
      gateText = "TEMP " + String(remoteTemp, 1) + "C";
    } else {
      gateText = "TIL MORNING";
    }
  }

  // If an entry-open attempt is blocked by exit activity, show immediate wait notice.
  bool entryAttemptFromApproach = (!gateOpen && (limitPressed || limitStablePressed || touchPressed));
  bool entryAttemptFromClosingReopen = gateClosingMotion && (touchPressed || limitStablePressed);
  bool entryReopenTrigger = reclosePromptActive ? touchPressed : (touchPressed || limitPressed);
  bool entryAttemptFromCountdownReopen = waitingToClose && entryReopenTrigger;
  bool entryAttemptPending = entryAttemptFromApproach || entryAttemptFromClosingReopen || entryAttemptFromCountdownReopen;
  if (!isExitGateClosedForEntry() && entryAttemptPending) {
    entryWaitDisplayLatched = true;
  }
  if (isExitGateClosedForEntry()) {
    entryWaitDisplayLatched = false;
  }
  bool entryWaitingOnExit = entryWaitDisplayLatched || (!isExitGateClosedForEntry() && entryAttemptPending);
  // If a gate command is queued, tell the driver why movement is delayed.
  bool waitingOtherGate = ((entryMoveQueued || exitMoveQueued) && (servoMoving || exitServoMoving)) || entryWaitingOnExit;
  if (waitingOtherGate) {
    directionText = "PLEASE WAIT";
    bool exitClosingMotion = exitServoMoving && (exitServoTargetAngle == EXIT_SERVO_CLOSED_ANGLE);
    if (entryWaitingOnExit && exitCloseCountdownActive) {
      gateText = "EXIT CLOSING IN " + buildCountdownText(getExitCloseDisplayCount());
    } else if (entryWaitingOnExit && exitClosingMotion) {
      gateText = "EXIT CLOSED IN " + buildCountdownText(getExitCloseDisplayCount());
    } else {
      gateText = "EXIT ACTIVE";
    }
  }

  // Top line: driver direction. Bottom line: gate condition.
  lcdMessage(directionText, gateText);

  // Bottom-right icon: speaker on/off status.
  lcdSoundIcon(buzzerEnabled);
  lcdDayNightIcon(darkModeActive);

  // -------------------------
  // BUZZER LOGIC (graded beeps)
  // -------------------------

  bool redZone = (currentZone == ZONE_STOP);
  bool slowZone = (currentZone == ZONE_SLOW);
  bool yellowZone = (currentZone == ZONE_CAUTION);
  bool greenZone = (currentZone == ZONE_NEXT_CAR || currentZone == ZONE_PROCEED);

  if (!buzzerEnabled) {
    noTone(BUZZER_PIN);
    buzzerState = false;
  }
  else if (pullThroughPromptActive) {
    if (millis() - lastPullThroughChirpMs >= PULL_THROUGH_CHIRP_INTERVAL_MS) {
      lastPullThroughChirpMs = millis();
      tone(BUZZER_PIN, PULL_THROUGH_CHIRP_FREQ_HZ, PULL_THROUGH_CHIRP_MS);
    }
    buzzerState = false;
  }
  else if (gateOpen || waitingToClose) {
    noTone(BUZZER_PIN);
    buzzerState = false;
  }
  else if (greenZone) {
    noTone(BUZZER_PIN);
    buzzerState = false;
  }
  else if (yellowZone) {
    if (millis() - buzzerTimer >= 600) {
      buzzerTimer = millis();
      buzzerState = !buzzerState;
      if (buzzerState) tone(BUZZER_PIN, 2400);
      else noTone(BUZZER_PIN);
    }
  }
  else if (slowZone) {
    if (millis() - buzzerTimer >= 350) {
      buzzerTimer = millis();
      buzzerState = !buzzerState;
      if (buzzerState) tone(BUZZER_PIN, 2800);
      else noTone(BUZZER_PIN);
    }
  }
  else if (redZone) {
    if (millis() - buzzerTimer >= 200) {
      buzzerTimer = millis();
      buzzerState = !buzzerState;
      if (buzzerState) tone(BUZZER_PIN, 3200);
      else noTone(BUZZER_PIN);
    }
  } else {
    noTone(BUZZER_PIN);
    buzzerState = false;
  }

  bool lotLockoutActive = darkModeActive && !nightGraceActive;
  if (lotLockoutActive) {
    if (servoMoving && servoTargetAngle == SERVO_OPEN_ANGLE) {
      // Let an already-triggered entry complete opening before enforcing close.
      return;
    }
    if (gateOpen || currentServoAngle != SERVO_CLOSED_ANGLE) {
      requestEntryServoMove(SERVO_CLOSED_ANGLE, SERVO_MOVE_TIME_MS);
    }
    gateOpen = false;
    waitingToClose = false;
    reclosePromptActive = false;
    proceedConfirmPending = false;
    sensorsReleaseStartMs = 0;
    return;
  }

  // -------------------------
  // GATE LOGIC WITH 5-SECOND DELAY
  // -------------------------

  // Re-open immediately if sensors trigger while gate is closing.
  if (gateClosingMotion && (touchPressed || limitStablePressed)) {
    if (!isExitGateClosedForEntry()) {
      entryWaitDisplayLatched = true;
      Serial.println("Entry open blocked: exit gate not closed");
    } else {
      gateOpen = true;
      waitingToClose = false;
      reclosePromptActive = false;
      proceedConfirmPending = false;
      sensorsReleaseStartMs = 0;
      requestEntryServoMove(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
      Serial.println("Gate re-opening: trigger detected during closing motion");
    }
  }

  // 1. OPEN GATE: trigger when car is in SLOW or STOP zone (close enough) with stable limit hold
  if (!gateOpen && carInOpenRange && limitStablePressed) {
    if (!isExitGateClosedForEntry()) {
      entryWaitDisplayLatched = true;
      Serial.println("Entry open blocked: exit gate not closed");
    } else {
      gateOpen = true;
      waitingToClose = false;
      reclosePromptActive = false;
      proceedConfirmPending = true;
      proceedConfirmStartMs = millis();
      sensorsReleaseStartMs = 0;
      requestEntryServoMove(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
    }
  }

  // 2. KEEP GATE OPEN
  if (gateOpen && (limitPressed || touchPressed)) {
    waitingToClose = false;
    reclosePromptActive = false;
    sensorsReleaseStartMs = 0;

    // Confirm proceed when vehicle reaches touch sensor after gate opens.
    if (proceedConfirmPending && touchPressed) {
      proceedConfirmPending = false;
      Serial.println("Proceed confirmed: continue forward");
    }

    if (!(servoMoving && servoTargetAngle == SERVO_OPEN_ANGLE)) {
      requestEntryServoMove(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
    }
  }

  // Prompt for second trigger; if no confirm, start reclose countdown.
  if (gateOpen && !waitingToClose && proceedConfirmPending && (millis() - proceedConfirmStartMs >= PROCEED_CONFIRM_TIMEOUT_MS)) {
    waitingToClose = true;
    reclosePromptActive = true;
    closeTimer = millis();
    sensorsReleaseStartMs = 0;
    Serial.println("No proceed confirm: touch sensor to proceed or reverse out");
  }

  // 3. START CLOSE COUNTDOWN (only after touch-confirm phase is complete)
  if (gateOpen && !proceedConfirmPending && !limitPressed && !touchPressed && !waitingToClose) {
    if (sensorsReleaseStartMs == 0) {
      sensorsReleaseStartMs = millis();
    }

    if ((millis() - sensorsReleaseStartMs) >= RELEASE_HOLD_TO_CLOSE_MS) {
      waitingToClose = true;
      closeTimer = millis();
      sensorsReleaseStartMs = 0;
    }
  }

  if (gateOpen && (limitPressed || touchPressed) && sensorsReleaseStartMs != 0) {
    sensorsReleaseStartMs = 0;
  }

  // 4. RESET COUNTDOWN / REOPEN
  // Reclose prompt requires touch confirmation, not limit hold.
  bool reopenTrigger = reclosePromptActive ? touchPressed : (touchPressed || limitPressed);
  if (waitingToClose && reopenTrigger) {
    if (!isExitGateClosedForEntry()) {
      entryWaitDisplayLatched = true;
      Serial.println("Entry open blocked: exit gate not closed");
    } else {
      waitingToClose = false;
      reclosePromptActive = false;
      proceedConfirmPending = false;
      sensorsReleaseStartMs = 0;
      if (!(servoMoving && servoTargetAngle == SERVO_OPEN_ANGLE)) {
        requestEntryServoMove(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
      }
    }
  }

  // 5. CLOSE GATE AFTER ACTIVE COUNTDOWN
  unsigned long activeCloseCountdownMs = reclosePromptActive ? RECLOSE_COUNTDOWN_MS : NORMAL_CLOSE_COUNTDOWN_MS;
  if (waitingToClose && (millis() - closeTimer >= activeCloseCountdownMs)) {
    gateOpen = false;
    waitingToClose = false;
    reclosePromptActive = false;
    proceedConfirmPending = false;
    requestEntryServoMove(SERVO_CLOSED_ANGLE, SERVO_MOVE_TIME_MS);

    if (nightGraceActive) {
      nightGraceActive = false;
    }
  }

  delay(50);
}
