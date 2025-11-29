#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>

#include "RotaryEncoder.h"
#include "Valve.h"

struct JointConfig {
  uint8_t pin_valve_e;
  uint8_t pin_valve_r;
  uint8_t multiplexI;

  /** The base attachment in parent segment space */
  float pistonBaseDistance;
  float pistonBaseAngleInParentSpace;

  /** The end attachment in child segment space (rotates with the joint) */
  float pistonEndDistance;
  float pistonEndAngle;

  /** Piston length constraints */
  float minPistonLength;
  float maxPistonLength;

  /** Invert the relationship between angle and piston length */
  bool invertPistonLengthRelationship;

  /** Invert encoder direction (for sensors mounted backwards) */
  bool invertEncoderDirection;
};

// Joint describes the whole joint, including;
// - Valve - controlling its angle
// - Piston - the piston the valve controles
// - RotaryEncoder - reads the angle of the joint as it is
class Joint {
private:
  Valve* v;
  RotaryEncoder* re;

  /** The base attachment in parent segment space */
  float pistonBaseDistance;
  float pistonBaseAngleInParentSpace;

  /** The end attachment in child segment space (rotates with the joint) */
  float pistonEndDistance;
  float pistonEndAngle;

  float minPistonLength;
  float maxPistonLength;

  // Invert the piston length relationship
  bool invertPistonLengthRelationship;

  // Invert encoder direction
  bool invertEncoderDirection;

  // Angle limits (calculated from piston length constraints)
  float angle_min_deg;
  float angle_max_deg;

  float kP = 2.0f;
  float kI = 0.05f;
  float kD = 2.0f;

  // PID state
  float integralError = 0;
  float previousError = 0;
  float lastPID = 0;

  float targetAngleDeg = 0.0f;
  float currentAngleDeg = NAN;

  // calculated when new target angle deg is given.. there isnt really a reason to store the angle itself
  float targetLength = 0.0f;

  long lastUpdate = 0;

  /* Cached values, used repeatedly in calculations */
  float pistonBaseDistance_sq;
  float pistonEndDistance_sq;
  float pistonBaseEnd_2ab;

  // Gets piston length from the active joint angle
  // Used when getting the current real-world angle and translating to piston length used in PID
  // depends on joint endpoint placements to use law of cosines
  float calculatePistonLength(float jointAngle) const;
  // Gets angle from piston lenghts.
  // Not really used actively in the main loop. Used initially to set the angle limits of the joint.
  // depends on joint endpoint placements to use law of cosines
  float calculateJointAngle(float pistonLength) const;

public:
  explicit Joint(const JointConfig &config);

  // Initializes the rotary encoder and loads offset from EEPROM
  void beginEncoder() const;

  // Equivalent to Update() in Unity
  // Reads the rotary encoder angle, compares it to the target angle,
  // and adjusts valve outputs using PID control with a deadband
  void update();

  // Resets the joint's target angle to its mid position
  void resetToInit();

  // Accessors for current state and control values
  void setTargetAngle(float deg);
  float getTargetAngleDeg() const;
  float getCurrentAngleDeg() const;
  float getRawEncoderAngleDeg() const;
  float getCurrentPistonLength() const;
  uint8_t getExtendDuty() const;
  uint8_t getRetractDuty() const;
  float getLastPID() const;

  // Configuration
  void setOffsetToCurrentPhysicalRotation(float currentPhysicalRotation);

  bool isAtTarget(float degreeTolerance) const;
};

#endif // JOINT_H
