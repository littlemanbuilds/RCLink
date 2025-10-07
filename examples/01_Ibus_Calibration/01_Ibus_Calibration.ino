/**
 * @file 01_Ibus_Calibration.ino
 *
 * @brief Simple iBUS calibrator for up to 14 channel RCs.
 */

#include <Arduino.h>
#include <RCLink.h>
#include <calibration/Ibus_Calibrate.hpp> ///< iBUS calibrator (header-only).

// ---- Wiring configuration ---- //
static constexpr int RX_PIN = 18;             ///< iBUS signal input GPIO.
static constexpr int TX_PIN = -1;             ///< TX not used for iBUS.
static constexpr uint32_t IBUS_BAUD = 115200; ///< iBUS baud rate (typical: 115200).

void setup()
{
    Serial.begin(115200); ///< Initialize serial port.
    delay(50);

    // pinMode(RX_PIN, INPUT_PULLDOWN); ///< Uncomment if your board needs a defined idle.
    calibrate::run_ibus(Serial2, /*rx*/ RX_PIN, /*tx*/ TX_PIN, /*baud*/ IBUS_BAUD); ///< // Run the calibrator.
}

void loop()
{
    // Not used; calibrator runs inside setup().
}