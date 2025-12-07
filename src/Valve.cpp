#include "Valve.h"

Valve::Valve(const uint8_t _pin_e, const uint8_t _pin_r) {
  // Create PWM controllers for both directions
  v_e = new LowFreqPWM(_pin_e, 1.5, 7.5); // 1Hz PWM, 5Hz max action rate
  v_r = new LowFreqPWM(_pin_r, 1.5, 7.5);
}

// move ∈ [-1, 1]  (− = retract, + = extend, 0 = hold)
void Valve::updatePWMs(const float move) {
  //Serial.println(move);
  const float normalized = constrain(move, -1.0, 1.0);
  const auto dc = static_cast<uint8_t>((fabsf(normalized) * 100.0f));

  if (normalized < 0) {             // RETRACT
    v_e->setDutyCycle(0);
    v_r->setDutyCycle(dc);
    lastDcE = 0; lastDcR = dc;
  } else if (normalized > 0) {      // EXTEND
    v_e->setDutyCycle(dc);
    v_r->setDutyCycle(0);
    lastDcE = dc; lastDcR = 0;
  } else {                          // IDLE
    v_e->setDutyCycle(0);
    v_r->setDutyCycle(0);
    lastDcE = 0; lastDcR = 0;
  }
}

void Valve::UpdatePWM(const float pidOutput) {
  updatePWMs(pidOutput);
}

void Valve::update() const {
  v_e->update();
  v_r->update();
}

uint8_t Valve::getLastExtendDuty() const {
  return lastDcE;
}

uint8_t Valve::getLastRetractDuty() const {
  return lastDcR;
}
