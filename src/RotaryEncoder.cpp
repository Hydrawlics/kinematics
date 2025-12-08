#include "RotaryEncoder.h"
#include <EEPROM.h>

// Angle register of AS5600 (high byte of angle value)
#define AS5600_ANGLE_REG 0x0C
#define REVERSE_DIR

/*
 * Constructor: stores I2C addresses and multiplexer channel.
 * Offset defaults to zero.
 */
RotaryEncoder::RotaryEncoder(uint8_t tcaChannel,
                             bool invertValues,
                             uint8_t tcaAddress,
                             uint8_t as5600Address)
    : _tcaAddr(tcaAddress),
      _invertValues(invertValues),
      _as5600Addr(as5600Address),
      _channel(tcaChannel),
      _offsetDeg(0.0f),
      _consecutiveErrors(0),
      _totalErrors(0),
      _lastValidAngle(0.0f)
{
}

/*
 * Must be called after Wire.begin().
 * Loads the offset from EEPROM if previously saved.
 */
void RotaryEncoder::begin() {
    // Load offset from EEPROM
    // Each channel needs 8 bytes: 4 for float + 4 for marker
    int eepromAddress = _channel * 8;
    loadOffsetFromEEPROM(eepromAddress);
}

/*
 * Selects the correct TCA9548A channel.
 * The TCA only activates one downstream channel at a time.
 */
void RotaryEncoder::selectTCAChannel() const {
    if (_channel > 7) return; // Invalid channel guard

    Wire.beginTransmission(_tcaAddr);
    Wire.write(1 << _channel);  // Enable only this channel
    Wire.endTransmission();
} 


/*
 * Reads the raw 12-bit angle register from AS5600 with retry logic.
 * Returns: value from 0 to 4095.
 * Sets success flag to indicate if read was successful.
 */
uint16_t RotaryEncoder::readRawAngleRegister(bool &success) {
    success = false;
    uint16_t angle = 0;

    // If too many consecutive errors, try to recover the I2C bus
    if (_consecutiveErrors >= 10) {
        Wire.end();
        delayMicroseconds(500);
        Wire.begin();
        Wire.setClock(50000);  // Match main.cpp I2C clock speed
        Wire.setWireTimeout(3000, true);
        _consecutiveErrors = 0;  // Reset after bus recovery attempt
    }

    // Retry loop for EMI resilience
    for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
        if (attempt > 0) {
            delayMicroseconds(RETRY_DELAY_US);  // Brief delay between retries
        }

        selectTCAChannel();  // Ensure correct downstream I2C device is active

        // Tell the AS5600 which register we want to read
        Wire.beginTransmission(_as5600Addr);
        Wire.write(AS5600_ANGLE_REG);
        uint8_t txStatus = Wire.endTransmission(false);   // Repeated start

        // Check if transmission was successful
        if (txStatus != 0) {
            continue;  // I2C error, retry
        }

        // Request 2 bytes: high and low part of angle value
        uint8_t bytesReceived = Wire.requestFrom((int)_as5600Addr, 2);

        if (bytesReceived >= 2 && Wire.available() >= 2) {
            uint8_t highByte = Wire.read();
            uint8_t lowByte  = Wire.read();

            // Combine bytes into one 12-bit number
            angle = ((uint16_t)highByte << 8) | lowByte;
            angle &= 0x0FFF; // Mask to keep only lowest 12 bits

            // Basic sanity check: AS5600 returns max 4095
            if (angle <= 4095) {
                if (_invertValues) angle = 4096 - angle;
                #ifdef REVERSE_DIR
                angle = 4096 - angle;
                #endif

                success = true;
                _consecutiveErrors = 0;  // Reset on success
                return angle;
            }
        }
    }

    // All retries failed
    _consecutiveErrors++;
    _totalErrors++;
    return 0;  // Return 0 but success is false
}

/*
 * Returns the raw sensor value without offset.
 */
uint16_t RotaryEncoder::getRawAngle() {
    bool success;
    return readRawAngleRegister(success);
}

/*
 * Returns angle in degrees with error detection, applying offset and normalizing to [0, 360).
 * If I2C read fails after retries, returns last valid angle and sets isValid to false.
 */
float RotaryEncoder::getAngleDeg(bool &isValid) {
    bool readSuccess;
    uint16_t raw = readRawAngleRegister(readSuccess);

    if (!readSuccess) {
        isValid = false;
        return _lastValidAngle;  // Return last known good value on failure
    }

    // Convert raw AS5600 0–4095 value to degrees
    float angle = (static_cast<float>(raw) / 4096.0f) * 360.0f;

    // Apply user-defined offset
    angle += _offsetDeg;

    // Normalize into the range [0, 360)
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;

    // Store as last valid reading
    _lastValidAngle = angle;
    isValid = true;
    return angle;
}

/*
 * Returns error statistics for diagnostics.
 */
uint16_t RotaryEncoder::getConsecutiveErrors() const {
    return _consecutiveErrors;
}

uint16_t RotaryEncoder::getTotalErrors() const {
    return _totalErrors;
}

void RotaryEncoder::resetErrorCounters() {
    _consecutiveErrors = 0;
    _totalErrors = 0;
}


/*
 * Sets the zero-calibration offset and saves it to EEPROM.
 * Each encoder channel gets its own EEPROM space (8 bytes per channel).
 */
void RotaryEncoder::setOffsetDeg(float offsetDeg) {
    _offsetDeg = offsetDeg;

    // Save to EEPROM automatically
    // Each channel needs 8 bytes: 4 for float + 4 for marker
    int eepromAddress = _channel * 8;
    saveOffsetToEEPROM(eepromAddress);
}


/*
 * Returns the current offset value.
 */
float RotaryEncoder::getOffsetDeg() const {
    return _offsetDeg;
}

/*
 * Stores the offset in EEPROM so it persists after power cycling.
 */
void RotaryEncoder::saveOffsetToEEPROM(int eepromAddress) const {
    EEPROM.put(eepromAddress, _offsetDeg);

    int markerAddress = eepromAddress + sizeof(_offsetDeg);
    uint32_t marker = 0xDEADBEEF; // Simple marker to indicate valid data
    EEPROM.put(markerAddress, marker);
}

/*
 * Reads a previously saved offset from EEPROM.
 * Only loads the offset if the validation marker is present.
 * Otherwise, keeps the default offset (0.0).
 */
void RotaryEncoder::loadOffsetFromEEPROM(int eepromAddress) {
    // Read the marker first to check if EEPROM has been initialized
    int markerAddress = eepromAddress + sizeof(_offsetDeg);
    uint32_t marker;
    EEPROM.get(markerAddress, marker);

    // Only load offset if marker is valid (indicates data was previously saved)
    if (marker == 0xDEADBEEF) {
        EEPROM.get(eepromAddress, _offsetDeg);
    } else {
        // EEPROM was never initialized, keep default offset
        _offsetDeg = 0.0f;
    }
}
