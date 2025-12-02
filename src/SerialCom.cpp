//
// Created by syvers on 02.12.2025.
//

#include "SerialCom.h"

void SerialCom::connected() { Serial.println("Connected"); }
void SerialCom::ready() { Serial.println("Ready"); }
void SerialCom::ok(uint8_t checksum) { Serial.print("OK "); Serial.println(checksum); }
void SerialCom::err(const char* reason) { Serial.print("ERR "); Serial.println(reason); }
String SerialCom::readLine() {
    String line = Serial.readStringUntil('\n');
    line.trim();
    return line;
}
