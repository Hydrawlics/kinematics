#include "Joint.h"
#include <math.h>

constexpr float DegToRad = M_PI / 180;
constexpr float RadToDeg = 180 / M_PI;

Joint::Joint(const JointConfig &config) {
  v = new Valve(config.pin_valve_e, config.pin_valve_r);
  re = new RotaryEncoder(config.multiplexI);

  /** The base attachment in parent segment space */
  pistonBaseDistance = config.pistonBaseDistance;
  pistonBaseAngleInParentSpace = config.pistonBaseAngleInParentSpace;

  /** The end attachment in child segment space (rotates with the joint) */
  pistonEndDistance = config.pistonEndDistance;
  pistonEndAngle = config.pistonEndAngle;

  /** Piston length constraints */
  minPistonLength = config.minPistonLength;
  maxPistonLength = config.maxPistonLength;

  /** Store inversion flags */
  invertPistonLengthRelationship = config.invertPistonLengthRelationship;
  invertEncoderDirection = config.invertEncoderDirection;

  /** Calculate angle boundaries from piston length constraints and geometry */
  float angle1, angle2;
  if (invertPistonLengthRelationship) {
    // Swap: min piston length corresponds to max angle, and vice versa
    angle1 = calculateJointAngle(maxPistonLength);
    angle2 = calculateJointAngle(minPistonLength);
  } else {
    angle1 = calculateJointAngle(minPistonLength);
    angle2 = calculateJointAngle(maxPistonLength);
  }
  // Ensure angle_min < angle_max regardless of geometry
  angle_min_deg = min(angle1, angle2);
  angle_max_deg = max(angle1, angle2);

  targetAngleDeg = (angle_min_deg + angle_max_deg) * 0.5f; // Start mid position
}

// Initializes the rotary encoder and loads offset from EEPROM
void Joint::beginEncoder() {
  re->begin();
}

float Joint::calculatePistonLength(const float jointAngle) const {
  // Calculate the angle in the triangle at the joint pivot
  // pistonBaseAngleInParentSpace: angle to base attachment (in parent's space, doesn't change with joint rotation)
  // pistonEndAngle: angle to end attachment (in joint's local space)
  // jointAngle: current rotation of the joint (in parent's space)

  // If inverted, negate jointAngle to flip the angle-length relationship
  const float effectiveJointAngle = invertPistonLengthRelationship ? -jointAngle : jointAngle;

  // The end attachment's angle in parent space is: pistonEndAngle + effectiveJointAngle
  // The triangle angle at the joint is the absolute difference between the two attachment angles
  const float triangleAngle = abs((pistonEndAngle + effectiveJointAngle) - pistonBaseAngleInParentSpace);
  const float triangleAngleRad = triangleAngle * DegToRad;

  // Use law of cosines: c² = a² + b² - 2ab*cos(C)
  // where c is the piston length, a and b are the attachment distances
  //Debug.Log(_pistonBaseDistance +" "+ _pistonEndDistance);

  const float length = sqrt(
      pistonBaseDistance * pistonBaseDistance +
      pistonEndDistance * pistonEndDistance -
      2 * pistonBaseDistance * pistonEndDistance * cos(triangleAngleRad)
  );

  return constrain(length, minPistonLength, maxPistonLength);
}

float Joint::calculateJointAngle(const float pistonLength) const {
  // Use law of cosines to find the triangle angle: c² = a² + b² - 2ab*cos(C)
  // Solving for C: cos(C) = (a² + b² - c²) / (2ab)
  const float a = pistonBaseDistance;
  const float b = pistonEndDistance;
  const float c = pistonLength;

  const float cosTriangleAngle = (a * a + b * b - c * c) / (2 * a * b);
  const float triangleAngleRad = acos(cosTriangleAngle);
  const float triangleAngle = triangleAngleRad * RadToDeg;

  // Convert triangle angle back to joint angle
  // Since forward uses: triangleAngle = abs((pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace)
  // We need to account for both possible cases. Based on typical joint geometry,
  // use the case where jointAngle < pistonBaseAngleInParentSpace:
  // jointAngle = pistonBaseAngleInParentSpace - triangleAngle - pistonEndAngle
  const float jointAngle = pistonBaseAngleInParentSpace - triangleAngle - pistonEndAngle;

  return jointAngle;
}

// Equivalent to Update() in Unity
// Reads the rotary encoder angle, compares it to the target angle,
// and adjusts valve outputs using PID control with a deadband
void Joint::update() {
  const long deltaTime = millis() - lastUpdate;

  // --- Step 1: Read current angle from rotary encoder ---
  float circularAngle = re->getAngleDeg();

  // Invert encoder direction if sensor is mounted backwards
  if (invertEncoderDirection) {
    circularAngle = 360.0f - circularAngle;
  }

  // the zero point should align with the joint zero point,
  // however, we need to fix so that values between 360 and 180 are negatives:
  // 359 => 359-360 == -1
  const float nonCircAngle = circularAngle > 180 ? (circularAngle - 360) : circularAngle;
  currentAngleDeg = constrain(nonCircAngle, angle_min_deg, angle_max_deg);
#ifdef VERBOSE
  Serial.print("[Joint] currentAngle:");
  Serial.print(currentAngleDeg);
  Serial.print(" targetAngle:");
  Serial.println(targetAngleDeg);
#endif

  // --- Step 2: Calculate target piston length for desired angle ---
  const float targetLength = calculatePistonLength(targetAngleDeg);
  const float currentPistonLength = calculatePistonLength(currentAngleDeg);

  // PID control to get desired piston velocity
  float error = targetLength - currentPistonLength;

  // Deadband: reset integral when very close to target (prevents lingering)
  constexpr float deadband = 0.001f; // 1mm tolerance
  if (abs(error) < deadband) {
      integralError = 0;
      error = 0; // Stop control signal completely within deadband
  }

  const float derivative = (error - previousError) / deltaTime;

  float pidOutput = kP * error + kI * integralError + kD * derivative;

  // Anti-windup: integrate if not saturated, OR if integrating would reduce saturation
  const bool saturatedHigh = pidOutput > 1.0;
  const bool saturatedLow = pidOutput < -1.0;
  const bool shouldIntegrate = (!saturatedHigh && !saturatedLow) ||
                         (saturatedHigh && error < 0) ||
                         (saturatedLow && error > 0);

  if (shouldIntegrate && abs(error) >= deadband) {
      integralError += error * deltaTime;
  }

  // Clamp output to valve range
  pidOutput = constrain(pidOutput, -1.0, 1.0);

  lastPID = pidOutput;

#ifdef VERBOSE
  Serial.print("[Joint] pid:");
  Serial.print(pidOutput);
  Serial.print(" integral:");
  Serial.print(integralError);
  Serial.print(" currentLength:");
  Serial.print(currentPistonLength, 6);
  Serial.print(" targetLength: ");
  Serial.println(targetLength, 6);
#endif
  previousError = error;

  lastUpdate = millis();

  // --- Step 3: Send control signal to valve ---
  v->UpdatePWM(pidOutput);
  v->update();
}

// Resets the joint's target angle to its mid position
void Joint::resetToInit() {
  setTargetAngle((angle_min_deg + angle_max_deg) * 0.5f);
}

// Accessors for current state and control values
void Joint::setTargetAngle(const float deg) {
  targetAngleDeg = constrain(deg, angle_min_deg, angle_max_deg);
}

float Joint::getTargetAngleDeg() const {
  return targetAngleDeg;
}

float Joint::getCurrentAngleDeg() const {
  return currentAngleDeg;
}

float Joint::getRawEncoderAngleDeg() const {
  return re->getAngleDeg();
}

float Joint::getCurrentPistonLength() const {
  return calculatePistonLength(currentAngleDeg);
}

uint8_t Joint::getExtendDuty() const {
  return v->getLastExtendDuty();
}

uint8_t Joint::getRetractDuty() const {
  return v->getLastRetractDuty();
}

float Joint::getLastPID() const {
  return lastPID;
}


// Configuration
void Joint::setOffsetToCurrentPhysicalRotation(const float currentPhysicalRotation) {
  // Use RAW sensor reading (without offset) to avoid issues with repeated calibration
  const uint16_t rawReading = re->getRawAngle();
  const float sensorReading = (static_cast<float>(rawReading) / 4096.0f) * 360.0f;

  // so we calibrate the joints mostly when the joints are straight,
  // meaning current rotation will be set as zero, however
  // some of the joints do not have sensor adapters that allow
  // straightening the joint fully out. Those joints will specify its angle here
  // e.g. j2 will probably have to be at -90 deg physically while calibrating
  // (Make sure that the sensors are increasing against the clock)

  // e.g. current read position is 21 deg, the physical position is -90 deg (quarter full rotation with the clock from straight position)
  // hence -90 deg is what we want to output when the actual sensor reads 21 deg. This is a difference of -111 deg.
  re->setOffsetDeg(currentPhysicalRotation - sensorReading);
};

bool Joint::isAtTarget(const float degreeTolerance) const {
  const float angleDiff = abs(targetAngleDeg - currentAngleDeg);
  return angleDiff <= degreeTolerance;
}