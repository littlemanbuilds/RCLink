/**
 * MIT License
 *
 * @brief Run guided S.BUS receiver calibration on an isolated ESP32-S3.
 *
 * Connect the configured receiver S.BUS signal to RX_PIN and share ground.
 * Use compatible signal levels with actuators disconnected. Open Serial Monitor
 * at 115200 baud, press a key to begin, visit every control position, then stop.
 * Suggestions use converted raw values; verify estimated centres before use.
 *
 * @file 03_SbusCalibration.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-11-17
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#define RCLINK_NO_LEGACY_GLOBAL_ALIASES
#include <RCLink.h>

#include <calibration/Sbus_Calibrate.hpp>
#include <Arduino.h>

using namespace rc;

const int RX_PIN = 18;
const int TX_PIN = -1; // Receive only; no UART TX connection to the receiver.
const uint32_t SBUS_BAUD = 100000; // Fixed S.BUS baud; helper configures inverted 8E2.

void setup()
{
    Serial.begin(115200);
    delay(50);
    calibrate::run_sbus(Serial2, RX_PIN, TX_PIN, SBUS_BAUD);
}

void loop()
{
    // Not used; the guided calibration session runs inside setup().
}
