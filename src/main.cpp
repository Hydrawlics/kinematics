// =====================================================================
//  HYDRAWLICS - JOINT & VALVE CONTROL SYSTEM
//  Hardware implementation matching the Unity simulation structure.
//  Includes LCD feedback, pump manager, self-test, and safety deadband.
// =====================================================================

#define SELFTEST_ON_START 1          // Run relay polarity test at startup (disable after confirmed)
//#define VERBOSE                      // NOTE! Breaks communications with the python script. For debugging only when not communicating with flask
//#define SLOW

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>  // Watchdog timer for I2C hang recovery

#include "PinConfig.h"
#include "Joint.h"
#include "PumpManager.h"
#include "ArmController.h"
#include "GCodeCommandQueue.h"
#include "SerialCom.h"

// --- I/O declerations ---
#ifdef LCD
#include <LiquidCrystal_I2C.h>
constexpr uint8_t LCD_ADDR = 0x27; // LCD setup
#endif

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

// Immediate mode: when true, movement commands clear queue and override current target
// Enable with M1000, disable with M1001
bool immediateMode = false;

// Calibration button state tracking
volatile bool buttonPressed = false;  // FIXED: Made volatile for ISR safety
unsigned long buttonPressStartTime = 0;
constexpr unsigned long CALIBRATION_HOLD_TIME_MS = 1000; // Hold button for 1 second to calibrate
bool calibrationTriggered = false;

// --- Pump relay (state) ---
inline void pumpWrite(const bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(PUMP_PIN, on ? LOW : HIGH);
  else                  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

//  Instances
// Azimuth joint; Rotates the whole arm around the azimuth angle
Joint j0({
  J0_VALVE_EXTEND, J0_VALVE_RETRACT, 2,
  0.16014, 65.314,   // base distance and angle - Defined the same way as j1!
  0.0703, 0,   // end distance and angle
  0.1300, PISTON1_LEN_MAX,
  true,  // invertPistonLengthRelationship
  false   // invertEncoderDirection
});
// Base-arm joint; Lifts the whole arm, first joint in arm
Joint j1({
  J1_VALVE_EXTEND, J1_VALVE_RETRACT, 3,
  0.111, 79.21,   // base distance and angle (79.21 because has an offset to the left of straight up that is 79.21)
  0.070, 0,   // end distance and angle (0 because straight up)
  0.1340, PISTON1_LEN_MAX,
  false,  // invertPistonLengthRelationship
  false   // invertEncoderDirection
});
// Elbow joint; Topmost joint, elbow up.
Joint j2({
  J2_VALVE_EXTEND, J2_VALVE_RETRACT, 4,
  0.050, -180,   // base distance and angle
  0.151, 0,   // end distance and angle
  0.1330, PISTON1_LEN_MAX,
  false,  // invertPistonLengthRelationship
  true // invertEncoderDirection - try without encoder inversion
});
// End-effector joint; Keeps the end-effector horizontal
Joint j3({
  J3_VALVE_EXTEND, J3_VALVE_RETRACT, 5,
  0.095, -180,   // base distance and angle
  0.070, 0,   // end distance and angle
  PISTON1_LEN_MIN, PISTON1_LEN_MAX,
  true,  // invertPistonLengthRelationship
  false   // invertEncoderDirection
});

ArmController armController(&j0, &j1, &j2, &j3);



//  Setup() - Equivalent to Start() in Unity. Note! Restart when new serial connection is established,
// so this is in practice the same as saying onSerialConnect()
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);
  SerialCom::connected();

  Wire.begin();  // Initialize I2C for rotary encoders and LCD
  Wire.setClock(50000);  // Reduce I2C clock to 50kHz for better EMI resistance
  Wire.setWireTimeout(3000, true);  // 3ms timeout with auto-reset

  // Enable watchdog timer for auto-recovery from I2C hangs (2 second timeout)
  wdt_enable(WDTO_2S);

#ifdef LCD
  lcd.init(); lcd.backlight(); lcd.createChar(DEG_CHAR, degreeGlyph);
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Hydrawlics valve test");
  lcd.setCursor(0,1); lcd.print("Angle ctrl ready");
  delay(700); lcd.clear();
#endif

  pinMode(STATUS_LED, OUTPUT);
  pinMode(CALIBRATION_BUTTON, INPUT_PULLUP);

  digitalWrite(STATUS_LED, LOW);

  armController.getStoredOffsets();

  // Apply default position using 3D coordinates and IK
  // Coordinate system: X+ = forward (away from base), Y+ = up, Z+ = left
  // Position: 43cm forward, 10cm up, centered (0 sideways)
  Vector3 initialPosition3D(0.43f, 0.10f, 0.0f);  // X, Y, Z in meters
  JointAngles initialPosition = armController.moveToWorldSpace(initialPosition3D);

  if (initialPosition.valid) {
    armController.applyJointAngles(initialPosition);
  } else {
    Serial.println("Warning: Initial position not reachable, using fallback angles");
    // Fallback to known good position
    initialPosition = JointAngles();
    initialPosition.baseRotation = 0;
    initialPosition.joint1Angle = -29;
    initialPosition.joint2Angle = -104;
    initialPosition.joint3Angle = 60;
    initialPosition.valid = true;
    armController.applyJointAngles(initialPosition);
  }

  attachInterrupt(digitalPinToInterrupt(CALIBRATION_BUTTON), calibrateBtnInterrupt, FALLING);

  pumpMgr.begin();
  //selfTestOnce();  // One-time hardware verification
}

//  Loop() - Equivalent to Update() in Unity
void loop() {
  // Reset watchdog timer to prevent auto-reset (must be called every loop)
  wdt_reset();

  // FIXED: Detect millis() corruption from EMI (sanity check)
  static unsigned long lastMillis = 0;
  unsigned long now = millis();
  if (now < lastMillis && (lastMillis - now) > 100) {
    // millis() went backwards significantly - likely EMI corruption
    // Force a controlled reset rather than undefined behavior
    Serial.println("ERR: millis() corruption detected, resetting...");
    delay(100);  // Give time for message to send
    wdt_enable(WDTO_15MS);  // Set shortest watchdog
    while(1);  // Wait for watchdog reset
  }
  lastMillis = now;

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
  // Check current button state (LOW = pressed due to INPUT_PULLUP)
  bool currentlyPressed = (digitalRead(CALIBRATION_BUTTON) == LOW);

  // Button was just pressed (interrupt fired)
  if (buttonPressed && currentlyPressed && buttonPressStartTime == 0) {
    buttonPressStartTime = millis();
    calibrationTriggered = false;
  }

  // Button is being held - check if held long enough
  if (currentlyPressed && buttonPressStartTime > 0 && !calibrationTriggered) {
    unsigned long holdDuration = millis() - buttonPressStartTime;

    // Visual feedback while holding - pulse LED faster as you approach threshold
    if (holdDuration < CALIBRATION_HOLD_TIME_MS) {
      // Blink faster as we get closer to triggering
      int blinkRate = map(holdDuration, 0, CALIBRATION_HOLD_TIME_MS, 500, 100);
      digitalWrite(STATUS_LED, (millis() % blinkRate) < (blinkRate / 2) ? HIGH : LOW);
    }

    // Held long enough - trigger calibration
    if (holdDuration >= CALIBRATION_HOLD_TIME_MS) {
      calibrationTriggered = true;

      armController.calibrateJoints();

      // LED fade indication (~2 seconds)
      for (int i = 0; i < 1000; i++) {
        wdt_reset();  // FIXED: Feed watchdog during long delay to prevent timeout
        float brightnessAmplitude = abs(sin(i * ((2 * M_PI) / 1000)));
        analogWrite(STATUS_LED, static_cast<int>(brightnessAmplitude * 255));
        delay(1);
      }
      digitalWrite(STATUS_LED, LOW);
    }
  }

  // Button released - reset state
  if (!currentlyPressed) {
    buttonPressed = false;
    buttonPressStartTime = 0;
    calibrationTriggered = false;
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
    SerialCom::ready();
    lastSentReady = true;
#ifdef TIMEOUT_READY
    lastSentReadyTime = millis();
#endif
  } else if (!gcodeQueue.shouldSendReady()) {
    lastSentReady = false;
  }

  // Read one line at a time
  if (Serial.available()) {
    String line = SerialCom::readLine();
    if (line.length() == 0) return;

    // Check for immediate mode control commands
    if (line.startsWith("M1000")) {
      immediateMode = true;
      gcodeQueue.clear();  // Clear any pending commands
      Serial.println("Immediate mode enabled - targets will override immediately");
      SerialCom::ok(calculateChecksum(line));
      lastSentReady = false;
      return;
    } else if (line.startsWith("M1001")) {
      immediateMode = false;
      Serial.println("Immediate mode disabled - normal queued operation");
      SerialCom::ok(calculateChecksum(line));
      lastSentReady = false;
      return;
    }

    // Parse and enqueue GCode command
    GCodeCommand cmd;
    switch (armController.parseGCodeLine(line, cmd)) {
      case GCodeParseResult::Success:
        // In immediate mode, clear queue before adding movement commands
        if (immediateMode && (cmd.commandType == "G0" || cmd.commandType == "G1")) {
          gcodeQueue.clear();
        }

        if (gcodeQueue.enqueue(cmd)) {
          SerialCom::ok(calculateChecksum(line));
        } else {
          SerialCom::err("Queue Full");
        }
        break;
      case GCodeParseResult::InvalidCommand:
        SerialCom::err("Invalid GCode");
      case GCodeParseResult::EmptyLine:
      case GCodeParseResult::ModeChange:
      default:
        SerialCom::ok(calculateChecksum(line));
        break;
    }
    // last sent was error or OK
    lastSentReady = false;
  }
}

//  Process commands from the queue
void processCommandQueue() {
  // In immediate mode, execute commands right away without waiting for target
  // In normal mode, wait for arm to reach target before executing next command
  const bool shouldProcess = !gcodeQueue.isEmpty() && (immediateMode || armController.isAtTarget());

  if (shouldProcess) {
#ifdef VERBOSE
    if (armController.isAtTarget()) {Serial.println("Is at target! Moving to next point."); }
#endif

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

// Calibration interrupt - set flag when button pressed, actual handling in loop()
void calibrateBtnInterrupt() {
  buttonPressed = true;
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


