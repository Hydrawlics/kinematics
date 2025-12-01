// =====================================================================
//  HYDRAWLICS - JOINT & VALVE CONTROL SYSTEM
//  Hardware implementation matching the Unity simulation structure.
//  Includes LCD feedback, pump manager, self-test, and safety deadband.
// =====================================================================

#define SELFTEST_ON_START 1          // Run relay polarity test at startup (disable after confirmed)
#define RELAY_ACTIVE_LOW  true       // Set false if relay board is active-HIGH (depends on module type)
//#define VERBOSE                      // NOTE! Breaks communications with the python script. For debugging only when not communicating with flask
#define SLOW

#include <Arduino.h>
#include <Wire.h>

#include "Joint.h"
#include "PumpManager.h"
#include "ArmController.h"
#include "GCodeCommandQueue.h"

// --- I/O declerations ---
#ifdef LCD
#include <LiquidCrystal_I2C.h>
constexpr uint8_t LCD_ADDR = 0x27; // LCD setup
#endif

constexpr uint8_t STATUS_LED = 13; // 13 has PWM, fancy fading light, internal led
constexpr uint8_t CALIBRATION_BUTTON = 2;
constexpr uint8_t PUMP_PIN = 30; // dedicated pump relay pin

// Joint 0 pins
constexpr uint8_t J0_VALVE_EXTEND = 22;
constexpr uint8_t J0_VALVE_RETRACT = 23;
constexpr uint8_t J0_POTMETER = A3;

// Joint 1 pins
constexpr uint8_t J1_VALVE_EXTEND = 24;
constexpr uint8_t J1_VALVE_RETRACT = 25;
constexpr uint8_t J1_POTMETER = A4;

// Joint 2 pins
constexpr uint8_t J2_VALVE_EXTEND = 26;
constexpr uint8_t J2_VALVE_RETRACT = 27;
constexpr uint8_t J2_POTMETER = A5;

// Joint 3 pins
constexpr uint8_t J3_VALVE_EXTEND = 28;
constexpr uint8_t J3_VALVE_RETRACT = 29;
constexpr uint8_t J3_POTMETER = A6;

// Piston Lengths (used in Joints to calculate angles)
// the only piston type we have as of now
// Others can be added below if we ever make different lengths
constexpr float PISTON1_LEN_MIN = 0.1280;
constexpr float PISTON1_LEN_MAX = 0.1657;

// Forward declarations
void calibrateIfFlag();
void updateValves();
void serialRead();
void processCommandQueue();
uint8_t calculateChecksum(String &line);
void selfTestOnce();
void printFloatOrDash(float v, uint8_t d);
void calibrateBtnInterrupt();
void getStoredOffsets();

#ifdef LCD
void lcdClearLine(uint8_t row);
void lcdFeedback();

// --- Custom degree symbol for LCD ---
const uint8_t DEG_CHAR = 0;
byte degreeGlyph[8] = {
  B00110,B01001,B01001,B00110,
  B00000,B00000,B00000,B00000
};
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
#endif

PumpManager pumpMgr;
GCodeCommandQueue gcodeQueue;

// Calibration flag - set by interrupt, handled in loop
volatile bool calibrationRequested = false;

// --- Pump relay (state) ---
inline void pumpWrite(const bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(PUMP_PIN, on ? LOW : HIGH);
  else                  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

//  Instances
// Azimuth joint; Rotates the whole arm around the azimuth angle
Joint j0({
  J0_VALVE_EXTEND, J0_VALVE_RETRACT, 0,
  0.050, -180,   // base distance and angle - Defined the same way as j1!
  0.151, 0,   // end distance and angle
  PISTON1_LEN_MIN, PISTON1_LEN_MAX,
  false,  // invertPistonLengthRelationship
  false   // invertEncoderDirection
});
// Base-arm joint; Lifts the whole arm, first joint in arm
Joint j1({
  J1_VALVE_EXTEND, J1_VALVE_RETRACT, 1,
  0.111, 79.21,   // base distance and angle (79.21 because has an offset to the left of straight up that is 79.21)
  0.070, 0,   // end distance and angle (0 because straight up)
  PISTON1_LEN_MIN, PISTON1_LEN_MAX,
  false,  // invertPistonLengthRelationship
  false   // invertEncoderDirection
});
// Elbow joint; Topmost joint, elbow up.
Joint j2({
  J2_VALVE_EXTEND, J2_VALVE_RETRACT, 2,
  0.050, -180,   // base distance and angle
  0.151, 0,   // end distance and angle
  PISTON1_LEN_MIN, PISTON1_LEN_MAX,
  false,  // invertPistonLengthRelationship
  true // invertEncoderDirection - try without encoder inversion
});
// End-effector joint; Keeps the end-effector horizontal
Joint j3({
  J3_VALVE_EXTEND, J3_VALVE_RETRACT, 3,
  0.095, -180,   // base distance and angle
  0.070, 0,   // end distance and angle
  PISTON1_LEN_MIN, PISTON1_LEN_MAX,
  false,  // invertPistonLengthRelationship
  false   // invertEncoderDirection
});

ArmController armController(&j0, &j1, &j2, &j3);



//  Setup() - Equivalent to Start() in Unity
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);
  Serial.println("Connected");

  Wire.begin();  // Initialize I2C for rotary encoders and LCD

#ifdef LCD
  lcd.init(); lcd.backlight(); lcd.createChar(DEG_CHAR, degreeGlyph);
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Hydrawlics valve test");
  lcd.setCursor(0,1); lcd.print("Angle ctrl ready");
  delay(700); lcd.clear();
#endif

  pinMode(STATUS_LED, OUTPUT);
  pinMode(CALIBRATION_BUTTON, INPUT_PULLUP);

  digitalWrite(STATUS_LED, LOW);

  getStoredOffsets();

  attachInterrupt(digitalPinToInterrupt(CALIBRATION_BUTTON), calibrateBtnInterrupt, FALLING);

  pumpMgr.begin();
  selfTestOnce();  // One-time hardware verification
}

//  Loop() - Equivalent to Update() in Unity
void loop() {
  // -- COLD CALLS; unlikely to actually do something (requires a button press, etc) --
  // Handle calibration request from interrupt
  calibrateIfFlag();
  // processes incoming serial communication from the python script running on Raspberry Pi and enqueues into gcodequeue
  serialRead();
  // runs next gcode command in queue if the one executing currently is within tolerances
  processCommandQueue();
  //lcdFeedback();

  // -- HOT CALLS; somewhat processor consuming each iteration --
  // Run through the arm valves and update the output PWM duty based on PID value from error
  armController.update(pumpMgr);

#ifdef VERBOSE
  armController.printPistonLengths();
  armController.printJointAngles();
#endif
}


void calibrateIfFlag() {
  if (calibrationRequested) {
    calibrationRequested = false;

    armController.calibrateJoints();

    // LED fade indication (~2 seconds)
    for (int i = 0; i < 1000; i++) {
      float brightnessAmplitude = abs(sin(i * ((2 * M_PI) / 1000)));
      analogWrite(STATUS_LED, (int)(brightnessAmplitude * 255));
      delay(1);
    }
    digitalWrite(STATUS_LED, LOW);
  }
}



// define to emulate slowely filling beyond limit
//#define TIMEOUT_READY

//  Serial Communication with GCode Queue
//  Protocol: P sends 1 line, then A responds "OK <checksum>"
//  When queue < 5, A sends "Ready" to continue
void serialRead() {
  static bool lastSentReady = false;
#ifdef TIMEOUT_READY
  static unsigned long lastSentReadyTime = 0;
#endif

  // Check if we should send Ready signal
  if (
    (gcodeQueue.shouldSendReady() &&
    !lastSentReady)
#ifdef TIMEOUT_READY
    || millis() - lastSentReadyTime > 5000
#endif
    ) {
    Serial.println("Ready");
    lastSentReady = true;
#ifdef TIMEOUT_READY
    lastSentReadyTime = millis();
#endif
  } else if (!gcodeQueue.shouldSendReady()) {
    lastSentReady = false;
  }

  // Read one line at a time
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) return;

    // Parse and enqueue GCode command
    GCodeCommand cmd;
    switch (armController.parseGCodeLine(line, cmd)) {
      case GCodeParseResult::Success:
        if (gcodeQueue.enqueue(cmd)) {
          Serial.print("OK ");
          Serial.println(calculateChecksum(line));
        } else {
          Serial.println("ERR Queue Full");
        }
        break;
      case GCodeParseResult::InvalidCommand:
        Serial.println("ERR Invalid GCode");
      case GCodeParseResult::EmptyLine:
      case GCodeParseResult::ModeChange:
      default:
        Serial.print("OK ");
        Serial.println(calculateChecksum(line));
        break;
    }
    // last sent was error or OK
    lastSentReady = false;
  }
}

//  Process commands from the queue
void processCommandQueue() {
  if (!gcodeQueue.isEmpty() && armController.isAtTarget()) {
    GCodeCommand cmd;
    if (gcodeQueue.dequeue(cmd)) {
      armController.processGCodeCommand(cmd);

      #ifdef VERBOSE
      Serial.print("Processed: ");
      Serial.print(cmd.commandType);
      if (cmd.hasX) { Serial.print(" X"); Serial.print(cmd.x); }
      if (cmd.hasY) { Serial.print(" Y"); Serial.print(cmd.y); }
      if (cmd.hasZ) { Serial.print(" Z"); Serial.print(cmd.z); }
      Serial.print(" | Queue: ");
      Serial.println(gcodeQueue.count());
      #endif
    }
  }
}

//  calculateChecksum()
//  Returns simple XOR checksum for serial confirmation.
uint8_t calculateChecksum(String &line) {
  uint8_t checksum = 0; for (int i=0;i<line.length();i++) checksum ^= static_cast<uint8_t>(line[i]);
  return checksum;
}

#ifdef LCD
//  LCD for visuals
void lcdClearLine(uint8_t row) { lcd.setCursor(0,row); for (int i=0;i<16;i++) lcd.print(' '); lcd.setCursor(0,row); }
void printFloatOrDash(float v, uint8_t d){ if (isnan(v)||isinf(v)) lcd.print("--"); else lcd.print(v,d); }

void lcdFeedback () {
  static unsigned long tLCD = 0;
  if (millis() - tLCD > 200) {
    tLCD = millis();
    const float cur = joints[0]->getCurrentAngleDeg();
    const float tar = joints[0]->getTargetAngleDeg();
#ifdef VERBOSE
    Serial.print("cur:");
    Serial.println(cur);
#endif
    lcdClearLine(0); lcd.print("C:"); printFloatOrDash(cur,1); lcd.write(DEG_CHAR);
    lcd.setCursor(9,0); lcd.print("p:"); lcd.print(joints[0]->getLastPID());
    lcdClearLine(1); lcd.print("T:"); printFloatOrDash(tar,1); lcd.write(DEG_CHAR);
    lcd.setCursor(9,1); lcd.print("P:"); lcd.print(pumpMgr.isOn() ? '1' : '0');
  }
}
#endif

// Calibration interrupt - just set flag, handle in loop()
void calibrateBtnInterrupt() {
  calibrationRequested = true;
}

//  Self-Test
//  Verifies relay polarity and pin wiring at startup.
void selfTestOnce() {
#if SELFTEST_ON_START

#ifdef LCD
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Hydrawlics");
  lcd.setCursor(0,1); lcd.print("Pump+Valves check");
#endif

  pinMode(J0_VALVE_EXTEND, OUTPUT); pinMode(J0_VALVE_RETRACT, OUTPUT);

  // EXTEND
  if (RELAY_ACTIVE_LOW){ digitalWrite(J0_VALVE_EXTEND, LOW);  digitalWrite(J0_VALVE_RETRACT, HIGH); }
  else                 { digitalWrite(J0_VALVE_EXTEND, HIGH); digitalWrite(J0_VALVE_RETRACT, LOW);  }
  delay(1000);

  // OFF
  if (RELAY_ACTIVE_LOW){ digitalWrite(J0_VALVE_EXTEND, HIGH); digitalWrite(J0_VALVE_RETRACT, HIGH); }
  else                 { digitalWrite(J0_VALVE_EXTEND, LOW);  digitalWrite(J0_VALVE_RETRACT, LOW);  }
  delay(600);

  // RETRACT
  if (RELAY_ACTIVE_LOW){ digitalWrite(J0_VALVE_EXTEND, HIGH); digitalWrite(J0_VALVE_RETRACT, LOW); }
  else                 { digitalWrite(J0_VALVE_EXTEND, LOW);  digitalWrite(J0_VALVE_RETRACT, HIGH); }
  delay(1000);

  // OFF
  if (RELAY_ACTIVE_LOW){ digitalWrite(J0_VALVE_EXTEND, HIGH); digitalWrite(J0_VALVE_RETRACT, HIGH); }
  else                 { digitalWrite(J0_VALVE_EXTEND, LOW);  digitalWrite(J0_VALVE_RETRACT, LOW);  }
  delay(600);

#ifdef LCD
  lcd.clear();
#endif
#endif
}


