// =====================================================================
//  HYDRAWLICS - BUTTON TEST MODE
//  Simple valve testing with 3 buttons: Extend, Retract, Valve Select
//  LED blinks to indicate which valve is currently active (0-3)
// =====================================================================

#include <Arduino.h>
#include "LowFreqPWM.h"
#include "PumpManager.h"

// ============================
// PIN CONFIGURATION
// ============================

// Button pins
constexpr uint8_t BTN_EXTEND = 2;      // Extend button (activates extend valve)
constexpr uint8_t BTN_RETRACT = 3;     // Retract button (activates retract valve)
constexpr uint8_t BTN_VALVE_SELECT = 4; // Valve select button (cycles through joints 0-3)

// Status LED
constexpr uint8_t STATUS_LED = 13;

// Pump pin
constexpr uint8_t PUMP_PIN = 30;

// Relay polarity (from main.cpp)
constexpr bool RELAY_ACTIVE_LOW = true;

// Joint valve pins
constexpr uint8_t J0_VALVE_RETRACT = 22;
constexpr uint8_t J0_VALVE_EXTEND = 23;

constexpr uint8_t J1_VALVE_RETRACT = 24;
constexpr uint8_t J1_VALVE_EXTEND = 25;

constexpr uint8_t J2_VALVE_RETRACT = 26;
constexpr uint8_t J2_VALVE_EXTEND = 27;

constexpr uint8_t J3_VALVE_RETRACT = 28;
constexpr uint8_t J3_VALVE_EXTEND = 29;

// ============================
// GLOBAL VARIABLES
// ============================

// Current selected valve (0-3)
uint8_t selectedValve = 0;

// Button debouncing
unsigned long lastSelectPress = 0;
const unsigned long DEBOUNCE_DELAY = 200; // ms

// Valve arrays for easy indexing
const uint8_t EXTEND_PINS[] = {J0_VALVE_EXTEND, J1_VALVE_EXTEND, J2_VALVE_EXTEND, J3_VALVE_EXTEND};
const uint8_t RETRACT_PINS[] = {J0_VALVE_RETRACT, J1_VALVE_RETRACT, J2_VALVE_RETRACT, J3_VALVE_RETRACT};

// Pump manager
PumpManager pumpMgr;

// ============================
// FUNCTION DECLARATIONS
// ============================

void pumpWrite(bool on);
void blinkSelectedValve();
void activateValve(uint8_t pin, bool activate);
void updateValves();

// ============================
// SETUP
// ============================

void setup() {
  Serial.begin(115200);
  Serial.println("=== Hydrawlics Button Test Mode ===");
  Serial.println("BTN_EXTEND: Pin 2");
  Serial.println("BTN_RETRACT: Pin 3");
  Serial.println("BTN_VALVE_SELECT: Pin 4");
  Serial.println("Selected valve starts at: 0");
  Serial.println();

  // Configure button pins
  pinMode(BTN_EXTEND, INPUT_PULLUP);
  pinMode(BTN_RETRACT, INPUT_PULLUP);
  pinMode(BTN_VALVE_SELECT, INPUT_PULLUP);

  // Configure LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Configure all valve pins
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(EXTEND_PINS[i], OUTPUT);
    pinMode(RETRACT_PINS[i], OUTPUT);

    // Ensure all valves are off (depends on relay polarity)
    if (RELAY_ACTIVE_LOW) {
      digitalWrite(EXTEND_PINS[i], HIGH);
      digitalWrite(RETRACT_PINS[i], HIGH);
    } else {
      digitalWrite(EXTEND_PINS[i], LOW);
      digitalWrite(RETRACT_PINS[i], LOW);
    }
  }

  // Initialize pump manager
  pumpMgr.begin();

  // Blink LED to show initial selected valve (valve 0 = 1 blink)
  delay(500);
  blinkSelectedValve();
}

// ============================
// MAIN LOOP
// ============================

void loop() {
  // Check if valve select button is pressed
  if (digitalRead(BTN_VALVE_SELECT) == LOW) {
    if (millis() - lastSelectPress > DEBOUNCE_DELAY) {
      lastSelectPress = millis();

      // Cycle to next valve
      selectedValve = (selectedValve + 1) % 4;

      Serial.print("Selected valve: ");
      Serial.println(selectedValve);

      // Blink LED to indicate new selection
      blinkSelectedValve();
    }
  }

  // Update valve states based on button presses
  updateValves();

  // Small delay to prevent excessive serial output
  delay(10);
}

// ============================
// VALVE CONTROL
// ============================

void updateValves() {
  bool extendPressed = (digitalRead(BTN_EXTEND) == LOW);
  bool retractPressed = (digitalRead(BTN_RETRACT) == LOW);

  // Turn off all valves first
  for (uint8_t i = 0; i < 4; i++) {
    activateValve(EXTEND_PINS[i], false);
    activateValve(RETRACT_PINS[i], false);
  }

  // Activate selected valve based on button state
  bool demand = false;

  if (extendPressed && !retractPressed) {
    activateValve(EXTEND_PINS[selectedValve], true);
    demand = true;
    Serial.print("Extending valve ");
    Serial.println(selectedValve);
  } else if (retractPressed && !extendPressed) {
    activateValve(RETRACT_PINS[selectedValve], true);
    demand = true;
    Serial.print("Retracting valve ");
    Serial.println(selectedValve);
  } else if (extendPressed && retractPressed) {
    // Safety: if both buttons pressed, do nothing
    Serial.println("WARNING: Both extend and retract pressed - ignoring");
  }

  // Update pump based on valve demand
  pumpMgr.update(demand);
}

// ============================
// HELPER FUNCTIONS
// ============================

// Activate or deactivate a valve
void activateValve(uint8_t pin, bool activate) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, activate ? LOW : HIGH);
  } else {
    digitalWrite(pin, activate ? HIGH : LOW);
  }
}

// Blink LED to indicate selected valve
// Valve 0 = 1 blink, Valve 1 = 2 blinks, etc.
void blinkSelectedValve() {
  const int blinkCount = selectedValve + 1;
  const int blinkDuration = 200; // ms
  const int blinkPause = 150;    // ms

  for (int i = 0; i < blinkCount; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(blinkDuration);
    digitalWrite(STATUS_LED, LOW);

    if (i < blinkCount - 1) {
      delay(blinkPause);
    }
  }
}

// Control pump relay
void pumpWrite(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(PUMP_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(PUMP_PIN, on ? HIGH : LOW);
  }
}