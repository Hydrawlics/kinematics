//
// Created by syvers on 16.12.2025.
//

#ifndef HYDRAWLICS_KINEMATICS_PINCONFIG_H
#define HYDRAWLICS_KINEMATICS_PINCONFIG_H

#pragma once
#include <Arduino.h>

// ============================
// PIN CONFIGURATION
// ============================

// Status and control
constexpr uint8_t STATUS_LED = 13;
constexpr uint8_t CALIBRATION_BUTTON = 2;
constexpr uint8_t PUMP_PIN = 30;

// Relay polarity (true = relay board is active-LOW)
constexpr bool RELAY_ACTIVE_LOW = true;

// Joint 0 (Azimuth)
constexpr uint8_t J0_VALVE_RETRACT = 24;
constexpr uint8_t J0_VALVE_EXTEND = 25;

// Joint 1 (Base-arm)
constexpr uint8_t J1_VALVE_RETRACT = 22;
constexpr uint8_t J1_VALVE_EXTEND = 23;

// Joint 2 (Elbow)
constexpr uint8_t J2_VALVE_RETRACT = 26;
constexpr uint8_t J2_VALVE_EXTEND = 27;

// Joint 3 (End-effector)
constexpr uint8_t J3_VALVE_RETRACT = 28;
constexpr uint8_t J3_VALVE_EXTEND = 29;


#endif //HYDRAWLICS_KINEMATICS_PINCONFIG_H