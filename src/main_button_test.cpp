// =====================================================================
//  HYDRAWLICS - BUTTON TEST MODE
//
//  Mode selection:
//  - Define SELECT_BTN_MODE for 3-button mode (Extend, Retract, Select)
//  - Comment out for direct mode (one button pair per valve)
// =====================================================================

#include <Arduino.h>
#include "LowFreqPWM.h"
#include "PumpManager.h"

// ============================
// MODE CONFIGURATION
// ============================

// Uncomment the line below to enable select button mode
// #define SELECT_BTN_MODE

#define NUM_VALVES 4  // Number of valves

// ============================
// PIN CONFIGURATION
// ============================

#ifdef SELECT_BTN_MODE
// ---- SELECT BUTTON MODE ----
// Button pins for select mode (3 buttons total)
constexpr uint8_t BTN_EXTEND = 2;      // Extend button (activates extend valve)
constexpr uint8_t BTN_RETRACT = 3;     // Retract button (activates retract valve)
constexpr uint8_t BTN_VALVE_SELECT = 4; // Valve select button (cycles through joints 0-3)

#else
// ---- DIRECT BUTTON MODE ----
// Button pins - one pair per valve (8 buttons total)
constexpr uint8_t J0_BTN_EXTEND = 6;
constexpr uint8_t J0_BTN_RETRACT = 7;

constexpr uint8_t J1_BTN_EXTEND = 4;
constexpr uint8_t J1_BTN_RETRACT = 5;

constexpr uint8_t J2_BTN_EXTEND = 2;
constexpr uint8_t J2_BTN_RETRACT = 3;

constexpr uint8_t J3_BTN_EXTEND = 9;
constexpr uint8_t J3_BTN_RETRACT = 8;

#endif

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

#ifdef SELECT_BTN_MODE
// Variables for select button mode
uint8_t selectedValve = 0;
unsigned long lastSelectPress = 0;
constexpr unsigned long DEBOUNCE_DELAY = 200; // ms
#else
// Button pin arrays for direct mode
constexpr uint8_t BUTTON_EXTEND_PINS[NUM_VALVES] = {J0_BTN_EXTEND, J1_BTN_EXTEND, J2_BTN_EXTEND, J3_BTN_EXTEND};
constexpr uint8_t BUTTON_RETRACT_PINS[NUM_VALVES] = {J0_BTN_RETRACT, J1_BTN_RETRACT, J2_BTN_RETRACT, J3_BTN_RETRACT};
#endif

// Valve arrays for easy indexing
constexpr uint8_t EXTEND_PINS[NUM_VALVES] = {J0_VALVE_EXTEND, J1_VALVE_EXTEND, J2_VALVE_EXTEND, J3_VALVE_EXTEND};
constexpr uint8_t RETRACT_PINS[NUM_VALVES] = {J0_VALVE_RETRACT, J1_VALVE_RETRACT, J2_VALVE_RETRACT, J3_VALVE_RETRACT};

// Pump manager
PumpManager pumpMgr;

// ============================
// FUNCTION DECLARATIONS
// ============================

void pumpWrite(bool on);
void activateValve(uint8_t pin, bool activate);
void updateValves();

#ifdef SELECT_BTN_MODE
void blinkSelectedValve();
#endif

// ============================
// SETUP
// ============================

void setup() {
  Serial.begin(115200);
  Serial.println("=== Hydrawlics Button Test Mode ===");

#ifdef SELECT_BTN_MODE
  Serial.println("Mode: SELECT BUTTON");
  Serial.println("BTN_EXTEND: Pin 2");
  Serial.println("BTN_RETRACT: Pin 3");
  Serial.println("BTN_VALVE_SELECT: Pin 4");
  Serial.println("Selected valve starts at: 0");

  // Configure button pins for select mode
  pinMode(BTN_EXTEND, INPUT_PULLUP);
  pinMode(BTN_RETRACT, INPUT_PULLUP);
  pinMode(BTN_VALVE_SELECT, INPUT_PULLUP);
#else
  Serial.println("Mode: DIRECT BUTTON (one pair per valve)");
  Serial.println("Button pairs:");
  Serial.println("  J0: Extend=2, Retract=3");
  Serial.println("  J1: Extend=4, Retract=5");
  Serial.println("  J2: Extend=6, Retract=7");
  Serial.println("  J3: Extend=8, Retract=9");

  // Configure button pins for direct mode
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    pinMode(BUTTON_EXTEND_PINS[i], INPUT_PULLUP);
    pinMode(BUTTON_RETRACT_PINS[i], INPUT_PULLUP);
  }
#endif

  Serial.println();

  // Configure LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Configure all valve pins
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
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

#ifdef SELECT_BTN_MODE
  // Blink LED to show initial selected valve (valve 0 = 1 blink)
  delay(500);
  blinkSelectedValve();
#endif
}

// ============================
// MAIN LOOP
// ============================

void loop() {
#ifdef SELECT_BTN_MODE
  // Check if valve select button is pressed
  if (digitalRead(BTN_VALVE_SELECT) == LOW) {
    if (millis() - lastSelectPress > DEBOUNCE_DELAY) {
      lastSelectPress = millis();

      // Cycle to next valve
      selectedValve = (selectedValve + 1) % NUM_VALVES;

      Serial.print("Selected valve: ");
      Serial.println(selectedValve);

      // Blink LED to indicate new selection
      blinkSelectedValve();
    }
  }
#endif

  // Update valve states based on button presses
  updateValves();

  // Small delay to prevent excessive serial output
  delay(10);
}

// ============================
// VALVE CONTROL
// ============================

void updateValves() {
  // Turn off all valves first
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    activateValve(EXTEND_PINS[i], false);
    activateValve(RETRACT_PINS[i], false);
  }

  bool demand = false;

#ifdef SELECT_BTN_MODE
  // ---- SELECT BUTTON MODE ----
  // Check single extend/retract buttons for selected valve
  bool extendPressed = (digitalRead(BTN_EXTEND) == LOW);
  bool retractPressed = (digitalRead(BTN_RETRACT) == LOW);

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
    Serial.println("WARNING: Both extend and retract pressed - ignoring");
  }

#else
  // ---- DIRECT BUTTON MODE ----
  // Check each valve's button pair independently
  for (uint8_t i = 0; i < NUM_VALVES; i++) {
    bool extendPressed = (digitalRead(BUTTON_EXTEND_PINS[i]) == LOW);
    bool retractPressed = (digitalRead(BUTTON_RETRACT_PINS[i]) == LOW);

    if (extendPressed && !retractPressed) {
      activateValve(EXTEND_PINS[i], true);
      demand = true;
      Serial.print("Extending valve ");
      Serial.println(i);
    } else if (retractPressed && !extendPressed) {
      activateValve(RETRACT_PINS[i], true);
      demand = true;
      Serial.print("Retracting valve ");
      Serial.println(i);
    } else if (extendPressed && retractPressed) {
      Serial.print("WARNING: Both buttons pressed for valve ");
      Serial.print(i);
      Serial.println(" - ignoring");
    }
  }
#endif

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

#ifdef SELECT_BTN_MODE
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
#endif

// Control pump relay
void pumpWrite(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(PUMP_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(PUMP_PIN, on ? HIGH : LOW);
  }
}