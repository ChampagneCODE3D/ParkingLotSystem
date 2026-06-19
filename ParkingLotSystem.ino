#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD (I2C address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Ultrasonic pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 6;

// Sensors
const int TOUCH_PIN = 9;       // Touch sensor
const int LIMIT_PIN = 11;      // Limit switch
const int OVERRIDE_PIN = 8;    // Override button
const int BUZZER_PIN = 2;      // Buzzer

// LEDs
const int LED_RED = 3;
const int LED_YELLOW = 4;
const int LED_GREEN = 5;

// Servo
Servo gateServo;
const int SERVO_PIN = 10;
const int SERVO_CLOSED_ANGLE = 0;
const int SERVO_OPEN_ANGLE = 90;
const unsigned long SERVO_MOVE_TIME_MS = 5000;
int currentServoAngle = SERVO_CLOSED_ANGLE;
int servoTargetAngle = SERVO_CLOSED_ANGLE;
bool servoMoving = false;
unsigned long servoStepIntervalMs = 20;
unsigned long servoLastStepMs = 0;
unsigned long servoMoveStartMs = 0;
unsigned long servoMoveDurationMs = 0;

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
const float STOP_EXIT_CM = 7.5;
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
bool lastComboPressed = false;
bool buzzerPersistentMuted = false;
unsigned long lastDebugPrintMs = 0;
unsigned long lastPullThroughChirpMs = 0;

const unsigned long DEBUG_PRINT_INTERVAL_MS = 400;
const unsigned long PULL_THROUGH_CHIRP_INTERVAL_MS = 2000;
const unsigned int PULL_THROUGH_CHIRP_FREQ_HZ = 1500;
const unsigned int PULL_THROUGH_CHIRP_MS = 80;

// LCD anti-flicker
String lastLCDLine1 = "";
String lastLCDLine2 = "";

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

void initLcdIcons() {
  lcd.createChar(LCD_ICON_SOUND_ON, lcdSoundOnGlyph);
  lcd.createChar(LCD_ICON_SOUND_OFF, lcdSoundOffGlyph);
}

void lcdSoundIcon(bool soundOn) {
  lcd.setCursor(15, 1);
  lcd.write(soundOn ? LCD_ICON_SOUND_ON : LCD_ICON_SOUND_OFF);
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

String buildCountdownText(int remaining) {
  if (remaining < 1) remaining = 1;
  return String(remaining);
}

String buildDriverDirection(float d) {
  if (d < 0) return "NO SIGNAL";
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

void setup() {
  Serial.begin(9600);

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
  pinMode(OVERRIDE_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  runBuzzerSelfTest();

  gateServo.attach(SERVO_PIN);
  gateServo.write(SERVO_CLOSED_ANGLE);  // gate closed
  currentServoAngle = SERVO_CLOSED_ANGLE;
  servoTargetAngle = SERVO_CLOSED_ANGLE;
}

void loop() {
  updateServoMotion();

  float d = getDistance();
  updateZoneByDistance(d);

  bool touchPressed = digitalRead(TOUCH_PIN) == HIGH;
  bool limitPressed = digitalRead(LIMIT_PIN) == LOW;
  bool buttonReleased = (digitalRead(OVERRIDE_PIN) == HIGH);
  bool buttonPressed = !buttonReleased;

  if (limitPressed) {
    if (limitPressStartMs == 0) limitPressStartMs = millis();
  } else {
    limitPressStartMs = 0;
  }

  bool limitStablePressed = (limitPressStartMs != 0) && ((millis() - limitPressStartMs) >= LIMIT_HOLD_TO_OPEN_MS);
  bool comboPressed = buttonPressed && touchPressed;
  bool gateOpeningMotion = servoMoving && (servoTargetAngle == SERVO_OPEN_ANGLE);
  bool gateClosingMotion = servoMoving && (servoTargetAngle == SERVO_CLOSED_ANGLE);

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

  // Two-input trigger: button + touch toggles persistent mute across multiple rounds.
  if (comboPressed && !lastComboPressed) {
    buzzerPersistentMuted = !buzzerPersistentMuted;
    if (!buzzerPersistentMuted) {
      buzzerEnabled = true;
    }
    buzzerState = false;
    noTone(BUZZER_PIN);
    Serial.println(buzzerPersistentMuted ? "Persistent mute ON" : "Persistent mute OFF");
  }
  lastComboPressed = comboPressed;

  // Button toggles run mute on/off.
  if (buttonPressed && !touchPressed && !lastButtonPressed) {
    buzzerEnabled = !buzzerEnabled;
    buzzerState = false;
    noTone(BUZZER_PIN);
    Serial.println(buzzerEnabled ? "Buzzer ON" : "Buzzer muted");
  }
  lastButtonPressed = buttonPressed;

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
    Serial.print(" | BTN: ");
    Serial.print(buttonPressed ? "PRESSED(MUTE)" : "RELEASED");
    Serial.print(" | BUZZER: ");
    if (buzzerPersistentMuted) Serial.print("MUTED(PERSIST)");
    else Serial.print(buzzerEnabled ? "ENABLED" : "MUTED(RUN)");
    Serial.print(" | WAITING: ");
    Serial.print(waitingToClose ? "YES" : "NO");
    Serial.print(" | GATE: ");
    Serial.println(gateOpen ? "OPEN" : "CLOSED");
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
    gateText = "OPENING IN " + buildCountdownText(remaining);
  }
  else if (gateClosingMotion) {
    unsigned long elapsed = millis() - servoMoveStartMs;
    int remaining = 1;
    if (servoMoveDurationMs > elapsed) {
      remaining = (int)((servoMoveDurationMs - elapsed + 999UL) / 1000UL);
      if (remaining < 1) remaining = 1;
      if (remaining > 5) remaining = 5;
    }
    gateText = "CLOSING IN " + buildCountdownText(remaining);
  }
  else if (gateOpen) {
    gateText = proceedConfirmPending ? "WAIT TOUCH" : "OPEN";
  }

  // Top line: driver direction. Bottom line: gate condition.
  lcdMessage(directionText, gateText);

  // Bottom-right icon: speaker on/off status.
  lcdSoundIcon(buzzerEnabled && !buzzerPersistentMuted);

  // -------------------------
  // BUZZER LOGIC (graded beeps)
  // -------------------------

  bool redZone = (currentZone == ZONE_STOP);
  bool slowZone = (currentZone == ZONE_SLOW);
  bool yellowZone = (currentZone == ZONE_CAUTION);
  bool greenZone = (currentZone == ZONE_NEXT_CAR || currentZone == ZONE_PROCEED);

  if (!buzzerEnabled || buzzerPersistentMuted) {
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

  // -------------------------
  // GATE LOGIC WITH 5-SECOND DELAY
  // -------------------------

  // Re-open immediately if sensors trigger while gate is closing.
  if (gateClosingMotion && (touchPressed || limitStablePressed)) {
    gateOpen = true;
    waitingToClose = false;
    reclosePromptActive = false;
    proceedConfirmPending = false;
    sensorsReleaseStartMs = 0;
    startServoMoveTo(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
    Serial.println("Gate re-opening: trigger detected during closing motion");
  }

  // 1. OPEN GATE: trigger when car is in SLOW or STOP zone (close enough) with stable limit hold
  bool carInOpenRange = (currentZone == ZONE_SLOW || currentZone == ZONE_STOP);
  if (!gateOpen && carInOpenRange && limitStablePressed) {
    gateOpen = true;
    waitingToClose = false;
    reclosePromptActive = false;
    proceedConfirmPending = true;
    proceedConfirmStartMs = millis();
    sensorsReleaseStartMs = 0;
    startServoMoveTo(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
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
      startServoMoveTo(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
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
    waitingToClose = false;
    reclosePromptActive = false;
    proceedConfirmPending = false;
    sensorsReleaseStartMs = 0;
    if (!(servoMoving && servoTargetAngle == SERVO_OPEN_ANGLE)) {
      startServoMoveTo(SERVO_OPEN_ANGLE, SERVO_MOVE_TIME_MS);
    }
  }

  // 5. CLOSE GATE AFTER ACTIVE COUNTDOWN
  unsigned long activeCloseCountdownMs = reclosePromptActive ? RECLOSE_COUNTDOWN_MS : NORMAL_CLOSE_COUNTDOWN_MS;
  if (waitingToClose && (millis() - closeTimer >= activeCloseCountdownMs)) {
    gateOpen = false;
    waitingToClose = false;
    reclosePromptActive = false;
    proceedConfirmPending = false;
    startServoMoveTo(SERVO_CLOSED_ANGLE, SERVO_MOVE_TIME_MS);
  }

  delay(50);
}
