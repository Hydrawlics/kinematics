// =====================================================================
//  HYDRAWLICS - SERIAL CONTROL TEST MODE
//
//  Serial command protocol:
//  - E<n> : Extend valve n (0-3)
//  - R<n> : Retract valve n (0-3)
//  - S<n> : Select valve n (0-3) for select mode
//  - X    : Stop all valves
//
//  Mode selection:
//  - Define SELECT_BTN_MODE for select mode (extend/retract selected valve)
//  - Comment out for direct mode (control each valve independently)
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
#endif

// Valve arrays for easy indexing
constexpr uint8_t EXTEND_PINS[NUM_VALVES] = {J0_VALVE_EXTEND, J1_VALVE_EXTEND, J2_VALVE_EXTEND, J3_VALVE_EXTEND};
constexpr uint8_t RETRACT_PINS[NUM_VALVES] = {J0_VALVE_RETRACT, J1_VALVE_RETRACT, J2_VALVE_RETRACT, J3_VALVE_RETRACT};

// Command state tracking
char lastCommand = '\0';
uint8_t lastValveNumber = 0;

// Pump manager
PumpManager pumpMgr;

// ============================
// FUNCTION DECLARATIONS
// ============================

void pumpWrite(bool on);
void activateValve(uint8_t pin, bool activate);
void updateValves();
void processSerialCommands();

#ifdef SELECT_BTN_MODE
void blinkSelectedValve();
#endif

// ============================
// SETUP
// ============================

void setup() {
  Serial.begin(115200);
  Serial.println("=== Hydrawlics Serial Control Mode ===");

#ifdef SELECT_BTN_MODE
  Serial.println("Mode: SELECT (use 'S' to select, 'E' to extend, 'R' to retract)");
  Serial.println("Commands: E (extend), R (retract), S<n> (select valve 0-3), X (stop)");
  Serial.print("Selected valve starts at: ");
  Serial.println(selectedValve);
#else
  Serial.println("Mode: DIRECT (control each valve independently)");
  Serial.println("Commands: E<n> (extend valve n), R<n> (retract valve n), X (stop all)");
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
  // Process serial commands
  processSerialCommands();

  // Update valve states based on commands
  updateValves();

  // Small delay
  delay(10);
}

// ============================
// SERIAL COMMAND PROCESSING
// ============================

void processSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();

    // Handle commands
    if (cmd == 'X' || cmd == 'x') {
      // Stop all valves
      lastCommand = '\0';
      Serial.println("STOP - All valves off");

    } else if (cmd == 'E' || cmd == 'e') {
      // Extend command
#ifdef SELECT_BTN_MODE
      lastCommand = 'E';
      Serial.print("EXTEND valve ");
      Serial.println(selectedValve);
#else
      // Wait for valve number in direct mode
      if (Serial.available()) {
        char valveChar = Serial.read();
        if (valveChar >= '0' && valveChar <= '3') {
          lastValveNumber = valveChar - '0';
          lastCommand = 'E';
          Serial.print("EXTEND valve ");
          Serial.println(lastValveNumber);
        }
      }
#endif

    } else if (cmd == 'R' || cmd == 'r') {
      // Retract command
#ifdef SELECT_BTN_MODE
      lastCommand = 'R';
      Serial.print("RETRACT valve ");
      Serial.println(selectedValve);
#else
      // Wait for valve number in direct mode
      if (Serial.available()) {
        char valveChar = Serial.read();
        if (valveChar >= '0' && valveChar <= '3') {
          lastValveNumber = valveChar - '0';
          lastCommand = 'R';
          Serial.print("RETRACT valve ");
          Serial.println(lastValveNumber);
        }
      }
#endif

    } else if (cmd == 'S' || cmd == 's') {
      // Select valve (only in SELECT_BTN_MODE)
#ifdef SELECT_BTN_MODE
      if (Serial.available()) {
        char valveChar = Serial.read();
        if (valveChar >= '0' && valveChar <= '3') {
          selectedValve = valveChar - '0';
          Serial.print("SELECTED valve ");
          Serial.println(selectedValve);
          blinkSelectedValve();
        }
      }
#endif
    }
  }
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
  // ---- SELECT MODE ----
  // Control the selected valve based on last command
  if (lastCommand == 'E') {
    activateValve(EXTEND_PINS[selectedValve], true);
    demand = true;
  } else if (lastCommand == 'R') {
    activateValve(RETRACT_PINS[selectedValve], true);
    demand = true;
  }

#else
  // ---- DIRECT MODE ----
  // Control specific valve based on last command and valve number
  if (lastCommand == 'E' && lastValveNumber < NUM_VALVES) {
    activateValve(EXTEND_PINS[lastValveNumber], true);
    demand = true;
  } else if (lastCommand == 'R' && lastValveNumber < NUM_VALVES) {
    activateValve(RETRACT_PINS[lastValveNumber], true);
    demand = true;
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