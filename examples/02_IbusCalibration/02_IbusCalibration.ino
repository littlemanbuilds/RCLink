/**
 * MIT License
 *
 * @brief Run the iBUS commissioning calibration helper on an ESP32-S3 receiver input.
 *
 * @file 02_IbusCalibration.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#include <calibration/Ibus_Calibrate.hpp>
#include <RCLink.h>

#include <Arduino.h>

// ---- Wiring configuration ---- //

const int RX_PIN = 18;             // iBUS signal from the receiver.
const int TX_PIN = -1;             // TX is not used for iBUS.
const uint32_t IBUS_BAUD = 115200; // Typical iBUS baud rate.

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Uncomment this only if your board needs a defined idle input.
    // pinMode(RX_PIN, INPUT_PULLDOWN);

    calibrate::run_ibus(Serial2, /*rx*/ RX_PIN, /*tx*/ TX_PIN, /*baud*/ IBUS_BAUD);
}

void loop()
{
    // Not used; calibrator runs inside setup().
}
