#include "ArmController.h"

#include "Joint.h"
#include "PumpManager.h"

#ifdef NATIVE_TEST
    #include <algorithm>
    #include <sstream>
    #include <iomanip>
    MockSerial Serial;  // Define global Serial mock for native testing
#endif

// Helper functions for string manipulation - work with both Arduino String and std::string
// NATIVE_TEST is defined in the platformio.ini file, see build-flags..
namespace {
    void stringToUpper(String& str) {
#ifdef NATIVE_TEST
        std::transform(str.begin(), str.end(), str.begin(), ::toupper);
#else
        str.toUpperCase();
#endif
    }

    void stringTrim(String& str) {
#ifdef NATIVE_TEST
        // Trim from start
        str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
        // Trim from end
        str.erase(std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), str.end());
#else
        str.trim();
#endif
    }

    bool stringStartsWith(const String& str, const char* prefix) {
#ifdef NATIVE_TEST
        return str.compare(0, strlen(prefix), prefix) == 0;
#else
        return str.startsWith(prefix);
#endif
    }

    int stringFind(const String& str, char ch) {
#ifdef NATIVE_TEST
        size_t pos = str.find(ch);
        return (pos != std::string::npos) ? (int)pos : -1;
#else
        return str.indexOf(ch);
#endif
    }

    String stringSubstr(const String& str, size_t start, size_t len) {
#ifdef NATIVE_TEST
        return str.substr(start, len);
#else
        return str.substring(start, start + len);
#endif
    }

    String stringSubstr(const String& str, size_t start) {
#ifdef NATIVE_TEST
        return str.substr(start);
#else
        return str.substring(start);
#endif
    }
}

ArmController::ArmController(Joint* j0, Joint* j1, Joint* j2, Joint* j3): j{j0, j1, j2, j3} {
    // Default arm dimensions from Unity simulation (in meters)
    a1 = 0.098f;
    a2 = 0.270f;
    a3 = 0.320f;
    endEffectorMagnitude = 0.07f;

    // Drawing space offset based on physical constraints
    // X: 340mm forward (moved 60mm closer), Z: 8×14.211mm = 113.688mm to the left
    drawSpaceOffset = Vector3(0.36f, 0, 0.11);

    // Initialize state
    currentPosition = Vector3(0, 0, 0);
    absoluteMode = true;  // G90 by default
    jointAngleTolerance = 1.5f;  // Matches PID deadband (~1mm ≈ 1-2° angular error)
    debugEnabled = false;
}

void ArmController::setArmDimensions(const float _a1, const float _a2, const float _a3, const float _endEffectorLength) {
    a1 = _a1;
    a2 = _a2;
    a3 = _a3;
    endEffectorMagnitude = _endEffectorLength;
}

void ArmController::setDrawingSpaceOffset(const Vector3 &offset) {
    drawSpaceOffset = offset;
}

void ArmController::setJointAngleTolerance(const float tolerance) {
    jointAngleTolerance = tolerance;
}

void ArmController::setAbsoluteMode(const bool absolute) {
    absoluteMode = absolute;
    debugLog(absolute ? "Switched to absolute positioning mode (G90)" :
                       "Switched to relative positioning mode (G91)");
}

void ArmController::debugLog(const String& message) const {
    if (debugEnabled) {
        // Debounce debug messages to prevent serial spam
        static unsigned long lastDebugTime = 0;
        const unsigned long DEBUG_INTERVAL = 200; // 200ms between debug messages

        if (millis() - lastDebugTime >= DEBUG_INTERVAL) {
            lastDebugTime = millis();
            Serial.println(message);
        }
    }
}

String ArmController::floatToString(float value, int decimals) {
#ifdef NATIVE_TEST
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(decimals) << value;
    return ss.str();
#else
    return String(value, decimals);
#endif
}

float ArmController::parseFloat(const String& str, bool& success) {
    if (str.length() == 0) {
        success = false;
        return 0.0f;
    }

    // Simple validation - check if string contains valid float characters
    bool hasDigit = false;
    for (size_t i = 0; i < str.length(); i++) {
        char c = str[i];
        if (isdigit(c)) {
            hasDigit = true;
        } else if (c != '.' && c != '-' && c != '+') {
            success = false;
            return 0.0f;
        }
    }

    success = hasDigit;
    return success ? atof(str.c_str()) : 0.0f;
}

// Load stored offsets from EEPROM for all joint encoders
void ArmController::getStoredOffsets() {
    for (Joint* joint : j) {
        joint->beginEncoder();
    }
}

void ArmController::update(PumpManager& pumpMgr) {
#ifdef SLOW
    static float lastRun = 0;
    if (millis() - lastRun < 1000) return;
    lastRun = millis();
#endif
    bool demand = false;
    for (Joint* i : j) {
        i->update();
        // check if this joint has any demand
        demand |= (i->getExtendDuty() > 0 || i->getRetractDuty() > 0);
    }
    // if any of the joints have any demand, close the pump circuit
    pumpMgr.update(demand);
}

GCodeParseResult ArmController::parseGCodeLine(const String& line, GCodeCommand& outCommand) {
    // Remove comments (everything after semicolon)
    const int commentPos = stringFind(line, ';');
    String cleanLine = (commentPos >= 0) ? stringSubstr(line, 0, commentPos) : line;
    stringTrim(cleanLine);

    if (cleanLine.length() == 0 || cleanLine[0] == '(') {
        return GCodeParseResult::EmptyLine;
    }

    // Split line into tokens (simple space-based tokenization)
    size_t tokenStart = 0;
    String tokens[10];  // Max 10 tokens per line
    int tokenCount = 0;
    const size_t lineLen = cleanLine.length();

    for (size_t i = 0; i <= lineLen && tokenCount < 10; i++) {
        if (i == lineLen || cleanLine[i] == ' ') {
            if (i > tokenStart) {
                tokens[tokenCount++] = stringSubstr(cleanLine, tokenStart, i - tokenStart);
            }
            tokenStart = i + 1;
        }
    }

    if (tokenCount == 0) {
        return GCodeParseResult::EmptyLine;
    }

    String commandType = tokens[0];
    stringToUpper(commandType);

    // Handle mode changes
    if (commandType == "G90") {
        setAbsoluteMode(true);
        return GCodeParseResult::ModeChange;  // Not a movement command
    } else if (commandType == "G91") {
        setAbsoluteMode(false);
        return GCodeParseResult::ModeChange;  // Not a movement command
    }

    // Handle G21 (means use millimeter units, default is meters)
    if (commandType == "G21") {
        // if set, we need to multiply the inputs with 1000
        inputValueMultiplier = 1000;
        return GCodeParseResult::ModeChange;
    }

    // Only process movement commands (G00, G01, G02, G03)
    if (!stringStartsWith(commandType, "G0") && !stringStartsWith(commandType, "G1")) {
        return GCodeParseResult::InvalidCommand;
    }

    outCommand = GCodeCommand(commandType);

    // Parse parameters
    for (int i = 1; i < tokenCount; i++) {
        String token = tokens[i];
        stringToUpper(token);

        if (token.length() < 2) continue;

        const char param = token[0];
        String valueStr = stringSubstr(token, 1);
        bool success;
        float value = parseFloat(valueStr, success);

        if (success) {
            value *= inputValueMultiplier;
            switch (param) {
                case 'X':
                    outCommand.x = value;
                    outCommand.hasX = true;
                    break;
                case 'Y':
                    outCommand.y = value;
                    outCommand.hasY = true;
                    break;
                case 'Z':
                    outCommand.z = value;
                    outCommand.hasZ = true;
                    break;
                default:
                    //ignore everything else
                    break;
            }
        } else {
            debugLog("Failed to parse GCode parameter: " + token);
        }
    }

    return GCodeParseResult::Success;
}

void ArmController::processGCodeCommand(const GCodeCommand& cmd) {
    // Calculate target position in GCode space
    Vector3 targetPos = currentPosition;

    if (absoluteMode) {
        // Absolute positioning
        if (cmd.hasX) targetPos.x = cmd.x;
        if (cmd.hasY) targetPos.y = cmd.y;
        if (cmd.hasZ) targetPos.z = cmd.z;
    } else {
        // Relative positioning
        if (cmd.hasX) targetPos.x += cmd.x;
        if (cmd.hasY) targetPos.y += cmd.y;
        if (cmd.hasZ) targetPos.z += cmd.z;
    }

    currentPosition = targetPos;

    debugLog("Executing " + cmd.commandType + " -> X:" + floatToString(targetPos.x, 3) +
             " Y:" + floatToString(targetPos.y, 3) + " Z:" + floatToString(targetPos.z, 3));

#ifndef NATIVE_TEST
    // Calculate inverse kinematics and apply to joints
    JointAngles angles = moveToDrawingSpace(targetPos);

    if (angles.valid) {
        applyJointAngles(angles);
        // Print target angles (from IK) and current angles (measured)
        Serial.print("TARGET: Base=");
        Serial.print(angles.baseRotation, 1);
        Serial.print(" J1=");
        Serial.print(angles.joint1Angle, 1);
        Serial.print(" J2=");
        Serial.print(angles.joint2Angle, 1);
        Serial.print(" J3=");
        Serial.println(angles.joint3Angle, 1);

        Serial.print("CURRENT: Base=");
        Serial.print(j[0]->getCurrentAngleDeg(), 1);
        Serial.print(" J1=");
        Serial.print(j[1]->getCurrentAngleDeg(), 1);
        Serial.print(" J2=");
        Serial.print(j[2]->getCurrentAngleDeg(), 1);
        Serial.print(" J3=");
        Serial.println(j[3]->getCurrentAngleDeg(), 1);

        debugLog("IK Success - Angles applied to joints");
    } else {
        // FIXED: When IK fails, maintain current target instead of leaving arm stuck
        // This prevents the system from blocking when unreachable coordinates are sent
        Serial.println("TARGET: IK_FAILED");
        Serial.print("CURRENT: Base=");
        Serial.print(j[0]->getCurrentAngleDeg(), 1);
        Serial.print(" J1=");
        Serial.print(j[1]->getCurrentAngleDeg(), 1);
        Serial.print(" J2=");
        Serial.print(j[2]->getCurrentAngleDeg(), 1);
        Serial.print(" J3=");
        Serial.println(j[3]->getCurrentAngleDeg(), 1);
        debugLog("IK Failed - Position unreachable, keeping current target");

        // Set target to current position to unblock queue
        // This allows the system to continue accepting commands instead of waiting forever
        targetAngles.baseRotation = j[0]->getCurrentAngleDeg();
        targetAngles.joint1Angle = j[1]->getCurrentAngleDeg();
        targetAngles.joint2Angle = j[2]->getCurrentAngleDeg();
        targetAngles.joint3Angle = j[3]->getCurrentAngleDeg();
        targetAngles.valid = true;
        applyJointAngles(targetAngles);
    }
#endif
}

Vector3 ArmController::translateToWorldSpace(const Vector3& gCodePos) const {
    // Coordinate translation from G-Code space to world space
    // Based on Unity implementation
    const auto gCodeTranslated = Vector3(
        gCodePos.y,
        gCodePos.z,
        gCodePos.x * -1.0f
    );

    const Vector3 targetWorldPos = drawSpaceOffset + gCodeTranslated;

    debugLog("GCode: (" + floatToString(gCodePos.x, 3) + ", " + floatToString(gCodePos.y, 3) +
             ", " + floatToString(gCodePos.z, 3) + ") -> World: (" +
             floatToString(targetWorldPos.x, 3) + ", " + floatToString(targetWorldPos.y, 3) +
             ", " + floatToString(targetWorldPos.z, 3) + ")");

    return targetWorldPos;
}

Vector3 ArmController::adjustForEndEffector(const Vector3& tipPosition, const Vector3& basePosition) const {
    // Calculate the position of the end effector origin by removing the end effector vector
    debugLog("MoveTip - Input Position: (" + floatToString(tipPosition.x, 3) + ", " +
             floatToString(tipPosition.y, 3) + ", " + floatToString(tipPosition.z, 3) + ")");

    Vector3 P_vector = tipPosition - basePosition;
    debugLog("MoveTip - P_vector before zeroing Y: (" + floatToString(P_vector.x, 3) + ", " +
             floatToString(P_vector.y, 3) + ", " + floatToString(P_vector.z, 3) + ")");

    P_vector.y = 0;
    debugLog("MoveTip - P_vector after zeroing Y: (" + floatToString(P_vector.x, 3) + ", " +
             floatToString(P_vector.y, 3) + ", " + floatToString(P_vector.z, 3) + ")");

    const Vector3 P_vector_normalized = P_vector.normalized();
    const Vector3 P_vector_normalized_scaled = P_vector_normalized * endEffectorMagnitude;
    const Vector3 endEffectorOriginPos = tipPosition - P_vector_normalized_scaled;

    debugLog("MoveTip - End Effector Origin: (" + floatToString(endEffectorOriginPos.x, 3) +
             ", " + floatToString(endEffectorOriginPos.y, 3) + ", " +
             floatToString(endEffectorOriginPos.z, 3) + ")");

    return endEffectorOriginPos;
}

JointAngles ArmController::calculateInverseKinematics(const Vector3& position) const {
    JointAngles result;

    // Renaming axis. Unity uses y up instead of z up. y and z are swapped.
    const float x_3 = position.x;
    const float y_3 = position.z;
    const float z_3 = position.y;

    debugLog("IK Input - x:" + floatToString(x_3, 3) + ", y:" + floatToString(y_3, 3) +
             ", z:" + floatToString(z_3, 3));

    // Calculate theta_1 (base rotation)
    const double theta_1_rad = M_PI + atan2(x_3, y_3);

    // Calculate r (distance in plane)
    const double r = sqrt(x_3 * x_3 + (z_3 - a1) * (z_3 - a1));

    // Check if position is reachable
    const double maxReach = a2 + a3;
    const double minReach = abs(a2 - a3);

    if (r > maxReach || r < minReach) {
        debugLog("Position unreachable! r=" + floatToString(r, 3) +
                 ", min=" + floatToString(minReach, 3) +
                 ", max=" + floatToString(maxReach, 3));
        result.valid = false;
        return result;
    }

    // Calculate phi angles using law of cosines
    const double phi_1_rad = acos((a2 * a2 + r * r - a3 * a3) / (2 * a2 * r));
    const double phi_2_rad = acos((a2 * a2 + a3 * a3 - r * r) / (2 * a2 * a3));
    const double phi_3_rad = atan((z_3 - a1) / x_3);

    // Calculate theta angles
    const double theta_2_rad = phi_3_rad + phi_1_rad;
    const double theta_3_rad = phi_2_rad - M_PI;

    // Convert to degrees
    const float phi_1 = (180.0f / M_PI) * phi_1_rad;
    const float phi_2 = (180.0f / M_PI) * phi_2_rad;
    const float phi_3 = (180.0f / M_PI) * phi_3_rad;

    const float theta_1 = (180.0f / M_PI) * theta_1_rad;
    const float theta_2 = (180.0f / M_PI) * theta_2_rad;
    const float theta_3 = (180.0f / M_PI) * theta_3_rad;

    // Store results (with Unity-specific adjustments)
    result.baseRotation = theta_1 + 90.0f;
    result.joint1Angle = theta_2 - 90.0f;
    result.joint2Angle = theta_3;
    result.joint3Angle = (180.0f - phi_1 - phi_2) - phi_3;

    // Normalize base rotation to -180 to 180 range
    while (result.baseRotation > 180.0f) result.baseRotation -= 360.0f;
    while (result.baseRotation < -180.0f) result.baseRotation += 360.0f;

    result.valid = true;

    debugLog("Phi_1: " + floatToString(phi_1, 2) + "° Phi_2: " + floatToString(phi_2, 2) +
             "° Phi_3: " + floatToString(phi_3, 2) + "°");
    debugLog("Theta_1: " + floatToString(theta_1, 2) + "° Theta_2: " + floatToString(theta_2, 2) +
             "° Theta_3: " + floatToString(theta_3, 2) + "°");

    return result;
}

JointAngles ArmController::calculateJointAngles(const Vector3& position) const {
    // Assumes position is already in world space and is the tip position
    // Adjust for end effector (assuming base is at origin for now)
    const auto basePosition = Vector3(0, 0, 0);
    const Vector3 endEffectorOriginPos = adjustForEndEffector(position, basePosition);

    // Calculate inverse kinematics
    return calculateInverseKinematics(endEffectorOriginPos);
}

JointAngles ArmController::moveToDrawingSpace(const Vector3& gCodePos) const {
    // Validate drawing space limits (from Unity simulation)
    if (gCodePos.x < 0 || gCodePos.x > 0.3264f) {
        debugLog("Warning: X out of drawing space range [0, 0.3264]");
    }
    if (gCodePos.y < 0 || gCodePos.y > 0.1846f) {
        debugLog("Warning: Y out of drawing space range [0, 0.1846]");
    }

    // Translate to world space
    const Vector3 targetWorldPos = translateToWorldSpace(gCodePos);

    // Calculate joint angles
    return calculateJointAngles(targetWorldPos);
}

JointAngles ArmController::moveToWorldSpace(const Vector3& worldPos) const {
    return calculateJointAngles(worldPos);
}

#ifndef NATIVE_TEST

void ArmController::calibrateJoints() {
    // Run through all the joints and set the offset to the correct thing given the current values of the joint

    j[0]->setOffsetToCurrentPhysicalRotation(0);
    j[1]->setOffsetToCurrentPhysicalRotation(0);
    j[2]->setOffsetToCurrentPhysicalRotation(-90);
    j[3]->setOffsetToCurrentPhysicalRotation(0);

}

bool ArmController::isAtTarget() const {
    // Loop through all joints and check that they are within tolerances
    for (const Joint* i : j) {
        if (!i->isAtTarget(jointAngleTolerance)) {
            return false;
        }
    }
    return true;
}

void ArmController::applyJointAngles(const JointAngles& angles) {
    // Store the target angles
    targetAngles = angles;

    // Apply to physical joints
    j[0]->setTargetAngle(angles.baseRotation);
    j[1]->setTargetAngle(angles.joint1Angle);
    j[2]->setTargetAngle(angles.joint2Angle);
    j[3]->setTargetAngle(angles.joint3Angle);
}

Vector3 ArmController::calculateForwardKinematics() const {
    // Get current joint angles (in degrees)
    const float base = j[0]->getCurrentAngleDeg();
    const float j1 = j[1]->getCurrentAngleDeg();
    const float j2 = j[2]->getCurrentAngleDeg();
    const float j3 = j[3]->getCurrentAngleDeg();

    // Convert to radians for calculation
    const float baseRad = base * (M_PI / 180.0f);
    const float j1Rad = j1 * (M_PI / 180.0f);
    const float j2Rad = j2 * (M_PI / 180.0f);
    const float j3Rad = j3 * (M_PI / 180.0f);

    // Forward kinematics - calculate end effector position
    // Using the same transformations as inverse kinematics but in reverse

    // Calculate reach in XZ plane (horizontal)
    const float reach = a2 * cos(j1Rad) + a3 * cos(j1Rad + j2Rad) + endEffectorMagnitude * cos(j1Rad + j2Rad + j3Rad);

    // Calculate height (Y)
    const float height = a1 + a2 * sin(j1Rad) + a3 * sin(j1Rad + j2Rad) + endEffectorMagnitude * sin(j1Rad + j2Rad + j3Rad);

    // Calculate X and Z from base rotation and reach
    const float x = reach * sin(baseRad);
    const float z = reach * cos(baseRad);

    return Vector3(x, height, z);
}

void ArmController::printJointAngles() {
    // Print joint angles and perceived position every 500ms
    if (millis() - lastAnglePrintTime >= 500) {
        lastAnglePrintTime = millis();

        Serial.print(millis()); Serial.print(" ");
        // Format: J0: raw(current)[target], J1: raw(current)[target], ...
        for (int i = 0; i < 4; i++) {
            Serial.print("J");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(j[i]->getRawEncoderAngleDeg(), 1);
            Serial.print("(");
            Serial.print(j[i]->getCurrentAngleDeg(), 1);
            Serial.print(")[");
            Serial.print(j[i]->getTargetAngleDeg(), 1);
            Serial.print("]");
            if (i < 3) {
                Serial.print(", ");
            }
        }

        // Calculate and print perceived position from forward kinematics
        Vector3 pos = calculateForwardKinematics();
        Serial.print(" | Pos: X=");
        Serial.print(pos.x * 1000, 1); // Convert to mm
        Serial.print("mm Y=");
        Serial.print(pos.y * 1000, 1);
        Serial.print("mm Z=");
        Serial.print(pos.z * 1000, 1);
        Serial.print("mm");

        Serial.println();
    }
}

void ArmController::printPistonLengths() {
    // Print calculated piston lengths at 10Hz (every 100ms)
    static unsigned long lastPistonPrintTime = 0;
    if (millis() - lastPistonPrintTime >= 500) {
        lastPistonPrintTime = millis();

        // Format: P0: length, P1: length, ...
        // Lengths in meters (multiply by 1000 for mm)
        for (int i = 0; i < 4; i++) {
            Serial.print("P");
            Serial.print(i);
            Serial.print(": ");
            Serial.print(j[i]->getCurrentPistonLength() * 1000, 1);
            Serial.print("mm");
            if (i < 3) {
                Serial.print(", ");
            }
        }
        Serial.println();
    }
}

#endif

