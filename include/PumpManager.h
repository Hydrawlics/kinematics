#ifndef PUMP_MANAGER_H
#define PUMP_MANAGER_H

#include <Arduino.h>

//  CLASS: PumpManager
//  Controls the hydraulic pump. The pump is ON only while a valve is active.
//  Holds for a short delay after movement to stabilize pressure.
class PumpManager {
private:
  bool isOn_ = false;
  unsigned long lastDemandMs_ = 0;
  const unsigned long holdMs_ = 200; // Hold time after motion stops (ms)

public:
  void begin();
  void update(bool demand);
  bool isOn() const;
};

#endif // PUMP_MANAGER_H
