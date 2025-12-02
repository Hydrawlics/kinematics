//
// Created by syvers on 02.12.2025.
//

#ifndef HYDRAWLICS_KINEMATICS_SERIALCOM_H
#define HYDRAWLICS_KINEMATICS_SERIALCOM_H
#include <Arduino.h>

// using namespace to signify no internal state
namespace SerialCom {
    void ready();
    void ok(uint8_t checksum);
    void err(const char* reason);
    String readLine();
}


#endif //HYDRAWLICS_KINEMATICS_SERIALCOM_H
