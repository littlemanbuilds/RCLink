/**
 * MIT License
 *
 * @brief Run the iBUS commissioning calibration helper on an ESP32-S3 receiver input.
 *
 * Connect receiver RX and common ground using compatible signal levels, with
 * actuators disconnected. At 115200-baud Serial Monitor, press a key to begin,
 * visit all control positions, then press a key to stop. Verify provisional
 * centre and classification suggestions before applying them to role settings.
 *
 * @file 02_IbusCalibration.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#define RCLINK_NO_LEGACY_GLOBAL_ALIASES // Use the namespace API in this self-contained sketch.
#include <RCLink.h>

#include <calibration/Ibus_Calibrate.hpp>
#include <Arduino.h>

using namespace rc;

// ---- Wiring configuration ---- //

const int RX_PIN = 18;             // iBUS signal from the receiver.
const int TX_PIN = -1;             // TX is not used for iBUS.
const uint32_t IBUS_BAUD = 115200; // Exact-width UART baud argument; 115200 exceeds a 16-bit int.

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
