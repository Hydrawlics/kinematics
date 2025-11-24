#include "RotaryEncoder.h"
#include <EEPROM.h>

// Angle register of AS5600 (high byte of angle value)
#define AS5600_ANGLE_REG 0x0C

/*
 * Constructor: stores I2C addresses and multiplexer channel.
 * Offset defaults to zero.
 */
RotaryEncoder::RotaryEncoder(uint8_t tcaChannel,
                             uint8_t tcaAddress,
                             uint8_t as5600Address)
    : _tcaAddr(tcaAddress),
      _as5600Addr(as5600Address),
      _channel(tcaChannel),
      _offsetDeg(0.0f)
{
}

/*
 * Must be called after Wire.begin().
 * Currently does nothing, but can later detect sensor presence.
 */
void RotaryEncoder::begin() {
    // Placeholder for optional startup diagnostics
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
 * Reads the raw 12-bit angle register from AS5600.
 * Returns: value from 0 to 4095.
 * Returns 0 if the read fails.
 */
uint16_t RotaryEncoder::readRawAngleRegister() {
    selectTCAChannel();  // Ensure correct downstream I2C device is active

    // Tell the AS5600 which register we want to read
    Wire.beginTransmission(_as5600Addr);
    Wire.write(AS5600_ANGLE_REG);
    Wire.endTransmission(false);   // Repeated start (keeps connection open)

    // Request 2 bytes: high and low part of angle value
    Wire.requestFrom((int)_as5600Addr, 2);
    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte  = Wire.read();

        // Combine bytes into one 12-bit number
        uint16_t angle = ((uint16_t)highByte << 8) | lowByte;
        angle &= 0x0FFF; // Mask to keep only lowest 12 bits
        return angle;
    }

    // Read failed — return 0 (valid but typically indicates a communication issue)
    return 0;
}

/*
 * Returns the raw sensor value without offset.
 */
uint16_t RotaryEncoder::getRawAngle() {
    return readRawAngleRegister();
}

/*
 * Returns angle in degrees, applying offset and normalizing to [0, 360).
 */
float RotaryEncoder::getAngleDeg() {
    uint16_t raw = readRawAngleRegister();

    // Convert raw AS5600 0–4095 value to degrees
    float angle = (static_cast<float>(raw) / 4096.0f) * 360.0f;

    // Apply user-defined offset
    angle += _offsetDeg;

    // Normalize into the range [0, 360)
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;

    return angle;
}


/*
 * Sets the zero-calibration offset.
 */
void RotaryEncoder::setOffsetDeg(float offsetDeg) {
    _offsetDeg = offsetDeg;
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
