#include "LowFreqPWM.h"

#define RELAY_ACTIVE_LOW

LowFreqPWM::LowFreqPWM(const uint8_t _pin, const float frequency, const float maxActionRate) {
  pin = _pin;
  period = 1000.0 / frequency;  // PWM period in ms
  minStateTime = 1000.0 / maxActionRate;  // Min state duration in ms
  dutyCycle = 0;
  state = false;
  lastToggle = 0;
  pinMode(pin, OUTPUT);

  // Initialize pin to OFF state immediately to prevent relay trigger on startup
#ifdef RELAY_ACTIVE_LOW
  digitalWrite(pin, HIGH);  // Active-low: HIGH = relay OFF
#else
  digitalWrite(pin, LOW);   // Active-high: LOW = relay OFF
#endif
}

void LowFreqPWM::setDutyCycle(const uint8_t dc) {
  // Clamp to valid ranges
  // Note! These could be adjusted to include some small values
  // in the sims it was 0.002. This is likely needed because the PID will
  // will never not move. Converge but never match.
  if (dc == 0 || dc == 100) {
    dutyCycle = dc;  // Full on/off is allowed
  } else {
    // Calculate min/max valid duty cycles
    const uint8_t minDC = (minStateTime * 100) / period;
    const uint8_t maxDC = 100 - minDC;

    // Clamp to valid range
    dutyCycle = constrain(dc, minDC, maxDC);

    #ifdef VERBOSE
    // notify if clamped, and verbose
    if (dutyCycle != dc) {
      static unsigned long lastClampDebug = 0;
      if (millis() - lastClampDebug >= 1000) {  // 1 second debounce
        lastClampDebug = millis();
        Serial.print(" c ");
        Serial.print(dc);
        Serial.print(" => ");
        Serial.println(dutyCycle);
      }
    }
    #endif
  }
}

void LowFreqPWM::update() {
  const unsigned long now = millis();
  const unsigned long elapsed = (now - lastToggle) % period;
  const unsigned long onTime = (period * dutyCycle) / 100;

  #ifdef VERBOSE
  // Debounce PWM debug output
  static unsigned long lastPWMDebug = 0;
  if (millis() - lastPWMDebug >= 1000) {  // 1 second debounce
    lastPWMDebug = millis();
    Serial.print(elapsed);
    Serial.print(" ");
    Serial.print(onTime);
    Serial.print(" ");
    Serial.println(dutyCycle);
  }
  #endif

  const bool shouldBeOn = (elapsed < onTime) && (dutyCycle > 0);

  if (shouldBeOn != state && ((now - lastToggle) >= minStateTime)) {
    state = shouldBeOn;
    lastToggle = millis();

    // Invert for active-low relays only when writing to pin
#ifdef RELAY_ACTIVE_LOW
    digitalWrite(pin, !state);
#else
    digitalWrite(pin, state);
#endif
  }
}
