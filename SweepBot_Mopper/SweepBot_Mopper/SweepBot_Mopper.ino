#include "arduino_secrets.h"

/*
  SweepBot_Mopper.ino
*/

#include <Servo.h>

// ======================= User Configuration =======================

// Pin mapping (adjust to your wiring)
const int PIN_TRIG          = 8;   // HC-SR04 trig
const int PIN_ECHO          = 9;   // HC-SR04 echo

// L298N channel for mop motor (one DC motor):
// ENA/ENB can be PWM; IN1/IN2 direction pins
const int PIN_MOP_EN        = 5;   // PWM-capable pin for speed control
const int PIN_MOP_IN1       = 6;
const int PIN_MOP_IN2       = 7;

// Pump control (MOSFET gate or relay IN)
const int PIN_PUMP          = 4;   // digital output

// Optional servo for mop lift
const bool USE_MOP_SERVO    = true;
const int  PIN_MOP_SERVO    = 10;
const int  SERVO_LIFT_POS   = 75;  // degrees: lifted/idle
const int  SERVO_DOWN_POS   = 110; // degrees: mopping

// Start trigger (from navigation block or button)
const int PIN_START_TRIGGER = 2;   // digital input; HIGH to start mopping

// Feedback LED (optional)
const int PIN_STATUS_LED    = 13;

// Mopping parameters
const int    MOP_SPEED_PWM       = 180;   // 0-255
const int    PUMP_ON_TIME_MS     = 3000;  // initial wetting
const int    MOP_PASS_TIME_MS    = 15000; // mop pass duration
const int    SAFETY_STOP_DIST_CM = 12;    // stop if obstacle closer than this
const int    RECHECK_INTERVAL_MS = 200;   // ultrasonic ping interval

// Debounce for trigger
const unsigned long DEBOUNCE_MS = 50;

// ========================= Internal State =========================

enum MopState {
  IDLE,
  WETTING,
  MOPPING,
  DONE,
  SAFETY_HALT
};

MopState mopState = IDLE;
unsigned long stateStart = 0;
unsigned long lastPing   = 0;
unsigned long lastDebounce = 0;
int lastTriggerStable = LOW;

Servo mopServo;

// ======================= Utility Functions ========================

long measureDistanceCM() {
  // Basic HC-SR04 distance measurement
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 30000UL); // timeout ~30ms
  if (duration == 0) {
    // No echo; treat as far
    return 400; // ~ max reliable distance
  }
  long distance = duration / 58; // us to cm
  return distance;
}

void mopMotorStop() {
  digitalWrite(PIN_MOP_IN1, LOW);
  digitalWrite(PIN_MOP_IN2, LOW);
  analogWrite(PIN_MOP_EN, 0);
}

void mopMotorStartCW(uint8_t pwm) {
  digitalWrite(PIN_MOP_IN1, HIGH);
  digitalWrite(PIN_MOP_IN2, LOW);
  analogWrite(PIN_MOP_EN, pwm);
}

void pumpOn() {
  digitalWrite(PIN_PUMP, HIGH);
}

void pumpOff() {
  digitalWrite(PIN_PUMP, LOW);
}

void mopLift() {
  if (USE_MOP_SERVO) {
    mopServo.write(SERVO_LIFT_POS);
    delay(400);
  }
}

void mopLower() {
  if (USE_MOP_SERVO) {
    mopServo.write(SERVO_DOWN_POS);
    delay(400);
  }
}

void setLED(bool on) {
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

bool readStartTriggerDebounced() {
  int reading = digitalRead(PIN_START_TRIGGER);
  static int lastReading = LOW;
  static int stableState = LOW;

  if (reading != lastReading) {
    lastDebounce = millis();
  }
  if ((millis() - lastDebounce) > DEBOUNCE_MS) {
    if (stableState != reading) {
      stableState = reading;
    }
  }
  lastReading = reading;
  lastTriggerStable = stableState;
  return stableState == HIGH;
}

// ============================ Setup ===============================

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_MOP_IN1, OUTPUT);
  pinMode(PIN_MOP_IN2, OUTPUT);
  pinMode(PIN_MOP_EN, OUTPUT);

  pinMode(PIN_PUMP, OUTPUT);
  pumpOff();

  pinMode(PIN_START_TRIGGER, INPUT_PULLUP); // Active LOW? If using external pull-down, change accordingly.
  // If using INPUT_PULLUP, trigger is LOW when pressed/active.
  // For simplicity, invert here if needed:
  // readStartTriggerDebounced() assumes HIGH=active. If using pull-up button to GND, invert in code below.

  pinMode(PIN_STATUS_LED, OUTPUT);
  setLED(false);

  if (USE_MOP_SERVO) {
    mopServo.attach(PIN_MOP_SERVO);
    mopLift();
  }

  mopMotorStop();
  mopState = IDLE;
  stateStart = millis();
}

// Helper to normalize trigger polarity when using INPUT_PULLUP button
bool isMopStartRequested() {
  bool raw = (digitalRead(PIN_START_TRIGGER) == HIGH); // If using external pulldown to GND: HIGH=active
  // If using INPUT_PULLUP button to GND, invert:
  // bool raw = (digitalRead(PIN_START_TRIGGER) == LOW);
  // For clarity, if you wire a navigation MCU/signal, keep HIGH=active.
  return raw;
}

// ============================= Loop ===============================

void loop() {
  // Debounced trigger read
  bool trigger = isMopStartRequested(); // or use readStartTriggerDebounced() if needed for noisy inputs

  switch (mopState) {
    case IDLE: {
      setLED(false);
      mopMotorStop();
      pumpOff();
      // Wait for navigation to signal arrival -> trigger mopping
      if (trigger) {
        mopLower();
        pumpOn();
        mopState = WETTING;
        stateStart = millis();
        setLED(true);
      }
      break;
    }

    case WETTING: {
      // Run pump to wet floor
      if (millis() - stateStart >= PUMP_ON_TIME_MS) {
        pumpOff();
        mopMotorStartCW(MOP_SPEED_PWM);
        mopState = MOPPING;
        stateStart = millis();
      }
      // Safety check periodically
      if (millis() - lastPing >= RECHECK_INTERVAL_MS) {
        lastPing = millis();
        long d = measureDistanceCM();
        if (d < SAFETY_STOP_DIST_CM) {
          mopMotorStop();
          pumpOff();
          mopLift();
          mopState = SAFETY_HALT;
        }
      }
      break;
    }

    case MOPPING: {
      // Run mop motor for a fixed pass time
      if (millis() - stateStart >= MOP_PASS_TIME_MS) {
        mopMotorStop();
        mopLift();
        mopState = DONE;
        stateStart = millis();
      }
      // Optional periodic safety check
      if (millis() - lastPing >= RECHECK_INTERVAL_MS) {
        lastPing = millis();
        long d = measureDistanceCM();
        if (d < SAFETY_STOP_DIST_CM) {
          mopMotorStop();
          pumpOff();
          mopLift();
          mopState = SAFETY_HALT;
        }
      }
      break;
    }

    case DONE: {
      // Completed one mopping cycle; wait for trigger to drop before returning to IDLE
      if (!trigger) {
        mopState = IDLE;
        setLED(false);
      }
      break;
    }

    case SAFETY_HALT: {
      // Wait until obstacle clears and trigger is released before returning to IDLE
      if (millis() - lastPing >= RECHECK_INTERVAL_MS) {
        lastPing = millis();
        long d = measureDistanceCM();
        if (d >= SAFETY_STOP_DIST_CM && !trigger) {
          mopState = IDLE;
          setLED(false);
        }
      }
      break;
    }
  }
}

