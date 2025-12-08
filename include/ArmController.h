#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "Joint.h"
#include "PumpManager.h"

#ifdef NATIVE_TEST
    // Native testing mode - use standard C++ libraries
    #include <string>
    #include <cmath>
    #include <cstdlib>
    #include <cstring>
    #include <cctype>
    #include <cstdio>

    // Use std::string for native testing
    typedef std::string String;

    // Mock Serial for native testing
    class MockSerial {
    public:
        void begin(unsigned long) {}
        void println(const std::string& s) { printf("%s\n", s.c_str()); }
        void println(const char* s) { printf("%s\n", s); }
    };
    extern MockSerial Serial;
#else
    // Arduino mode - use Arduino libraries
    #include <Arduino.h>
#endif

// Simple 3D vector structure for positions
struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    float magnitude() const {
        return sqrt(x * x + y * y + z * z);
    }

    Vector3 normalized() const {
        float mag = magnitude();
        if (mag > 0.0001f) {
            return Vector3(x / mag, y / mag, z / mag);
        }
        return Vector3(0, 0, 0);
    }
};

// Structure representing a single G-Code command
struct GCodeCommand {
    String commandType;  // G00, G01, G02, G03, etc.
    bool hasX;
    bool hasY;
    bool hasZ;
    bool hasFeedRate;
    float x;
    float y;
    float z;

    GCodeCommand() : hasX(false), hasY(false), hasZ(false), hasFeedRate(false),
                     x(0), y(0), z(0) {}

    explicit GCodeCommand(const String& type) : commandType(type), hasX(false), hasY(false),
                                                hasZ(false), hasFeedRate(false),
                                                x(0), y(0), z(0) {}
};

// Result structure for joint angles
struct JointAngles {
    float baseRotation;  // theta_1 (rotation around vertical axis)
    float joint1Angle;   // theta_2 (first arm segment)
    float joint2Angle;   // theta_3 (second arm segment)
    float joint3Angle;   // end effector angle
    bool valid;          // whether IK solution is valid

    JointAngles() : baseRotation(0), joint1Angle(0), joint2Angle(0),
                    joint3Angle(0), valid(false) {}
};

enum class GCodeParseResult {
    Success,        // Valid movement command parsed
    EmptyLine,      // Nothing to parse (benign)
    ModeChange,     // G90/G91 handled (benign)
    InvalidCommand  // Actual parsing error
};

class ArmController {
public:
    // Constructor
    ArmController(Joint* j0, Joint* j1, Joint* j2, Joint* j3);

    // Configuration
    void setArmDimensions(float a1, float a2, float a3, float endEffectorLength);
    void setDrawingSpaceOffset(const Vector3 &offset);
    void setJointAngleTolerance(float tolerance);

#ifndef NATIVE_TEST

    // Joint management (Arduino only)

    // the bible of calibrateJoints function:
    // A function to calibrate all the joints by setting their offsets
    // This function should only be ran when the arm is in a known location
    // Each joint's zero position is with the next segment straight forward (..-O-.. type shi)

    // second joint should be rotated in a -90° position
    // third joint should be straight (0°)
    //     O2-----O3--- E
    //     |
    //     O1       // first joint should be rotated straight up (0°)
    // |-------|    // base should be rotated towards the middle of the drawing area
    void calibrateJoints();
    // get calibration data stored on the EEPROM
    void getStoredOffsets();

    void applyJointAngles(const JointAngles& angles);
    bool isAtTarget() const;
    void printJointAngles();
    void printPistonLengths();
#endif

    void update(PumpManager& pumpMgr);

    // G-Code processing
    GCodeParseResult parseGCodeLine(const String& line, GCodeCommand& outCommand);
    void processGCodeCommand(const GCodeCommand& cmd);
    void setAbsoluteMode(bool absolute);
    bool isAbsoluteMode() const { return absoluteMode; }

    // Movement functions
    JointAngles calculateJointAngles(const Vector3& position) const;
    JointAngles moveToDrawingSpace(const Vector3& gCodePos) const;
    JointAngles moveToWorldSpace(const Vector3& worldPos) const;
    Vector3 calculateForwardKinematics() const;  // Calculate position from current joint angles

    // State getters
    Vector3 getCurrentPosition() const { return currentPosition; }
    void setCurrentPosition(const Vector3& pos) { currentPosition = pos; }

    // Debug/logging
    void enableDebug(bool enable) { debugEnabled = enable; }

private:
    // Arm dimensions (in meters)
    float a1;  // Height of first joint above base
    float a2;  // Length of first arm segment
    float a3;  // Length of second arm segment
    float endEffectorMagnitude;  // Length of end effector

    float inputValueMultiplier = 1; // for translation from millimeters and such
    // default expected input is meters, hence 1

    // Drawing space configuration
    Vector3 drawSpaceOffset;

    // G-Code state
    Vector3 currentPosition;
    bool absoluteMode;  // true = G90 (absolute), false = G91 (relative)

    // Control parameters
    float jointAngleTolerance;

    // Debug
    bool debugEnabled;

#ifndef NATIVE_TEST
    // Joint management (Arduino only)
    Joint* j[4];
    JointAngles targetAngles;
    unsigned long lastAnglePrintTime = 0;
#endif

    // Helper functions
    JointAngles calculateInverseKinematics(const Vector3& endEffectorOriginPos) const;
    Vector3 adjustForEndEffector(const Vector3& tipPosition, const Vector3& basePosition) const;
    Vector3 translateToWorldSpace(const Vector3& gCodePos) const;

    // Utility
    void debugLog(const String& message) const;
    static float parseFloat(const String& str, bool& success);
    static String floatToString(float value, int decimals);
};

#endif // ARM_CONTROLLER_H