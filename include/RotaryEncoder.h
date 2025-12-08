#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include <Arduino.h>
#include <Wire.h>

class RotaryEncoder {
public:
    /*
     * Constructor
     * tcaChannel:      Which multiplexer channel the AS5600 is connected to (0–7)
     * tcaAddress:      I2C address of TCA9548A (default: 0x70)
     * as5600Address:   I2C address of AS5600   (default: 0x36)
     */
    
    RotaryEncoder(uint8_t tcaChannel,
                  bool invertValues = false,
                  uint8_t tcaAddress   = 0x70,
                  uint8_t as5600Address = 0x36);

    /*
     * Call this in setup() AFTER Wire.begin().
     * Could later be expanded with sensor-detection logic.
     */
    void begin();

    /*
     * Returns the raw 12-bit angle value from the AS5600 (0–4095).
     * Returns 0 if the read fails.
     */
    uint16_t getRawAngle();

    /*
     * Returns the angle in degrees (0–360), including stored offset.
     * Sets isValid to false if I2C read failed after retries.
     */
    float getAngleDeg(bool &isValid);

    /*
     * Returns error statistics for diagnostics.
     */
    uint16_t getConsecutiveErrors() const;
    uint16_t getTotalErrors() const;
    void resetErrorCounters();

    /*
     * Sets the angle offset (in degrees). Positive or negative values allowed.
     * Offset is useful when the "mechanical zero" is not aligned with sensor zero.
     */
    void setOffsetDeg(float offsetDeg);
    float getOffsetDeg() const;

    /*
     * Saves and loads the offset value to/from EEPROM.
     * eepromAddress must be within valid EEPROM bounds for your board.
     */
    void saveOffsetToEEPROM(int eepromAddress) const;
    void loadOffsetFromEEPROM(int eepromAddress);

private:
    // I2C addressing
    uint8_t _tcaAddr;
    bool _invertValues;
    uint8_t _as5600Addr;
    uint8_t _channel;

    // Offset (in degrees) applied to the angle reading
    float   _offsetDeg;

    // Error tracking for EMI detection
    uint16_t _consecutiveErrors;
    uint16_t _totalErrors;
    float    _lastValidAngle;

    // I2C retry configuration
    static constexpr uint8_t MAX_RETRIES = 3;
    static constexpr uint16_t RETRY_DELAY_US = 100;

    // Internal helpers
    void     selectTCAChannel() const;
    uint16_t readRawAngleRegister(bool &success);
};

#endif // ROTARY_ENCODER_H
