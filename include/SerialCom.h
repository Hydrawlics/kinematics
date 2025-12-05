//
// Created by syvers on 02.12.2025.
//

#ifndef HYDRAWLICS_KINEMATICS_SERIALCOM_H
#define HYDRAWLICS_KINEMATICS_SERIALCOM_H
#include <Arduino.h>

// using namespace to signify no internal state
namespace SerialCom {
    /* Signal that the Arduino has restarted and is connected */
    void connected();
    /* Signal that there is enough room in the queue to send more commands */
    void ready();

    /* Read and trim one line from the Serial buffer */
    String readLine();

    /* Signal that the command was received without issue */
    void ok(uint8_t checksum);
    /* Signal that an error has occurred during parsing or execution of command */
    void err(const char* reason);
}


#endif //HYDRAWLICS_KINEMATICS_SERIALCOM_H
