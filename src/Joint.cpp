#include "Joint.h"
#include <math.h>

constexpr float DegToRad = M_PI / 180;
constexpr float RadToDeg = 180 / M_PI;

Joint::Joint(const JointConfig &config) {
  v = new Valve(config.pin_valve_e, config.pin_valve_r);
  re = new RotaryEncoder(config.multiplexI, config.invertEncoderDirection);

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
  const float angle1 = calculateJointAngle(minPistonLength);
  const float angle2 = calculateJointAngle(maxPistonLength);
  // Ensure angle_min < angle_max regardless of geometry
  angle_min_deg = min(angle1, angle2);
  angle_max_deg = max(angle1, angle2);

  // Cached values used in update function
  pistonBaseDistance_sq = pistonBaseDistance * pistonBaseDistance;
  pistonEndDistance_sq = pistonEndDistance * pistonEndDistance;
  pistonBaseEnd_2ab = 2 * pistonBaseDistance * pistonEndDistance;

  // Set initial target angle to mid position and calculate corresponding target length
  setTargetAngle((angle_min_deg + angle_max_deg) * 0.5f);
}

// Initializes the rotary encoder and loads offset from EEPROM
void Joint::beginEncoder() const {
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

  // here we have some calculation we can truncate into cached values
  // const float length = sqrt(
  //     pistonBaseDistance * pistonBaseDistance +
  //     pistonEndDistance * pistonEndDistance -
  //     2 * pistonBaseDistance * pistonEndDistance * cos(triangleAngleRad)
  // );

  const float length = sqrt(pistonBaseDistance_sq + pistonEndDistance_sq - pistonBaseEnd_2ab * cos(triangleAngleRad));

  return constrain(length, minPistonLength, maxPistonLength);
}

float Joint::calculateJointAngle(const float pistonLength) const {
  // Use law of cosines to find the triangle angle: c² = a² + b² - 2ab*cos(C)
  // Solving for C: cos(C) = (a² + b² - c²) / (2ab)
  const float a = pistonBaseDistance;
  const float b = pistonEndDistance;
  const float c = pistonLength;

  float cosTriangleAngle = (a * a + b * b - c * c) / (2 * a * b);
  // Clamp to valid range for acos to handle floating-point errors
  cosTriangleAngle = constrain(cosTriangleAngle, -1.0f, 1.0f);
  const float triangleAngleRad = acos(cosTriangleAngle);
  const float triangleAngle = triangleAngleRad * RadToDeg;

  // Convert triangle angle back to joint angle
  // Since forward uses: triangleAngle = abs((pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace)
  // There are two cases depending on the sign of pistonBaseAngleInParentSpace:
  float jointAngle;
  if (pistonBaseAngleInParentSpace > 0) {
    // Case for positive pistonBaseAngleInParentSpace (e.g., J1 with 79.21°)
    // Formula: jointAngle = pistonBaseAngleInParentSpace - triangleAngle - pistonEndAngle
    jointAngle = pistonBaseAngleInParentSpace - triangleAngle - pistonEndAngle;
  } else {
    // Case for negative pistonBaseAngleInParentSpace (e.g., J0/J2/J3 with -180°)
    // Formula: jointAngle = triangleAngle + pistonBaseAngleInParentSpace - pistonEndAngle
    jointAngle = triangleAngle + pistonBaseAngleInParentSpace - pistonEndAngle;
  }

  // If piston length relationship is inverted, negate the result
  // This matches the forward calculation which uses -jointAngle when inverted
  if (invertPistonLengthRelationship) {
    jointAngle = -jointAngle;
  }

  return jointAngle;
}

// Equivalent to Update() in Unity
// Reads the rotary encoder angle, compares it to the target angle,
// and adjusts valve outputs using PID control with a deadband
void Joint::update() {
  const long deltaTime = millis() - lastUpdate;
  const unsigned long now = millis();

  // --- Step 1: Read current angle from rotary encoder with error detection ---
  bool sensorValid = false;
  float circularAngle = re->getAngleDeg(sensorValid);

  // the zero point should align with the joint zero point,
  // however, we need to fix so that values between 360 and 180 are negatives:
  // 359 => 359-360 == -1
  float nonCircAngle = circularAngle > 180 ? (circularAngle - 360) : circularAngle;
  float newAngle = constrain(nonCircAngle, angle_min_deg, angle_max_deg);

  // --- Step 2: Validate sensor reading for EMI corruption ---
  bool angleIsValid = sensorValid;  // Start with I2C validity

  // Additional validation: Rate-of-change check (only if we have a previous valid reading)
  if (angleIsValid && !isnan(lastValidAngleDeg) && lastValidReadTime > 0) {
    const unsigned long timeSinceLastValid = now - lastValidReadTime;
    const float angleDelta = abs(newAngle - lastValidAngleDeg);

    // Check for impossible rate of change (indicates corrupted data)
    if (timeSinceLastValid > 0) {
      const float angleRate = angleDelta / timeSinceLastValid;  // degrees per millisecond
      if (angleRate > MAX_ANGLE_RATE) {
        angleIsValid = false;  // Reject impossible movement
#ifdef VERBOSE
        Serial.print("[Joint] Rate-of-change violation: ");
        Serial.print(angleRate);
        Serial.print(" deg/ms (max: ");
        Serial.print(MAX_ANGLE_RATE);
        Serial.println(")");
#endif
      }
    }
  }

  // Handle valid vs invalid readings
  if (angleIsValid) {
    // Good reading - use it
    currentAngleDeg = newAngle;
    lastValidAngleDeg = newAngle;
    lastValidReadTime = now;
    consecutiveBadReadings = 0;
    sensorErrorState = false;
  } else {
    // Bad reading - use last valid value and increment error counter
    consecutiveBadReadings++;

    if (!isnan(lastValidAngleDeg)) {
      currentAngleDeg = lastValidAngleDeg;  // Hold last valid position
    }

    // Enter error state if too many consecutive failures
    if (consecutiveBadReadings >= MAX_CONSECUTIVE_BAD_READS) {
      sensorErrorState = true;
#ifdef VERBOSE
      Serial.println("[Joint] SENSOR ERROR STATE - Too many bad readings!");
#endif
    }
  }

#ifdef VERBOSE
  Serial.print("[Joint] currentAngle:");
  Serial.print(currentAngleDeg);
  Serial.print(" targetAngle:");
  Serial.print(targetAngleDeg);
  Serial.print(" valid:");
  Serial.println(angleIsValid ? "Y" : "N");
#endif

  // --- Step 3: Calculate target piston length for desired angle ---
  const float currentPistonLength = calculatePistonLength(currentAngleDeg); // Changes all the time

  // Safety: If in error state, disable valves
  if (sensorErrorState) {
    v->UpdatePWM(0);  // Stop all valve movement
    v->update();
    lastUpdate = millis();
    return;  // Skip PID when sensor is unreliable
  }

  // PID control to get desired piston velocity
  float error = targetLength - currentPistonLength;

  // Calculate derivative BEFORE applying deadband (to preserve proper derivative calculation)
  const float derivative = (error - previousError) / deltaTime;

  // Deadband: reset integral and zero output when very close to target (prevents lingering)
  constexpr float deadband = 0.001f; // 1mm tolerance
  bool withinDeadband = abs(error) < deadband;

  if (withinDeadband) {
      integralError = 0;
  }

  // Detect error sign change (crossing target) - aggressively reduce integral
  const bool errorSignChanged = (previousError * error) < 0;
  if (errorSignChanged) {
      integralError *= 0.3f;  // Reduce integral to 30% when crossing target
  }

  float pidOutput = kP * error + kI * integralError + kD * derivative;

  // Zero output if within deadband
  if (withinDeadband) {
      pidOutput = 0;
  }

  // Anti-windup: integrate if not saturated, OR if integrating would reduce saturation
  const bool saturatedHigh = pidOutput > 1.0;
  const bool saturatedLow = pidOutput < -1.0;
  const bool shouldIntegrate = (!saturatedHigh && !saturatedLow) ||
                         (saturatedHigh && error < 0) ||
                         (saturatedLow && error > 0);

  if (shouldIntegrate && !withinDeadband) {
      integralError += error * deltaTime;
      // Clamp integral to prevent excessive windup
      integralError = constrain(integralError, -integralMax, integralMax);
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

  // --- Step 4: Send control signal to valve ---
  v->UpdatePWM(pidOutput);
  v->update();
}

// Resets the joint's target angle to its mid position
void Joint::resetToInit() {
  setTargetAngle((angle_min_deg + angle_max_deg) * 0.5f);
}

// Accessors for current state and control values
void Joint::setTargetAngle(const float deg) {
  // as of now, the only reason we store this is for tolerances in isAtTarget
  targetAngleDeg = constrain(deg, angle_min_deg, angle_max_deg);

  // also calculate the target piston length to cache it
  targetLength = calculatePistonLength(targetAngleDeg);
}

float Joint::getTargetAngleDeg() const {
  return targetAngleDeg;
}

float Joint::getCurrentAngleDeg() const {
  return currentAngleDeg;
}

float Joint::getRawEncoderAngleDeg() const {
  bool isValid;
  return re->getAngleDeg(isValid);
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

// Error diagnostics
bool Joint::isSensorHealthy() const {
  return !sensorErrorState && (consecutiveBadReadings < 3);
}

uint16_t Joint::getSensorErrorCount() const {
  return consecutiveBadReadings;
}