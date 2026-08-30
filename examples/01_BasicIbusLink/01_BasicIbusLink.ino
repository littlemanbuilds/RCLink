/**
 * MIT License
 *
 * @brief Read a centred axis, one-sided axis, and switch from an iBUS receiver.
 *
 * Connect the receiver iBUS signal to RX_PIN and share receiver/board ground.
 * Open Serial Monitor at 115200 baud and move the controls to see safe values.
 *
 * @file 01_BasicIbusLink.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-14
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#include <RCLink.h>

#include <Arduino.h>

// ---- Hardware configuration ---- //

const int RX_PIN = 18;
const int TX_PIN = -1; // iBUS reception does not use TX.
const uint32_t IBUS_BAUD = 115200;

// ---- Application roles ---- //

#define BASIC_ROLES(_)                                                                                                 \
    _(Steering)                                                                                                        \
    _(Throttle)                                                                                                        \
    _(Mode)
RC_DECLARE_ROLES(BasicRole, BASIC_ROLES)

// The transport owns iBUS byte reception. RcLink validates, maps, shapes, and
// exposes those receiver values as the application roles above.
rc::RcIbusTransport transport;
rc::RcLink<rc::RcIbusTransport, BasicRole> rclink(transport);

// ---- Setup ---- //

void setup()
{
    Serial.begin(115200);
    rclink.begin(Serial2, IBUS_BAUD, RX_PIN, TX_PIN);

    RC_CONFIG(BasicRole, config);
    config.map(BasicRole::Steering, 0);
    config.map(BasicRole::Throttle, 2);
    config.map(BasicRole::Mode, 6);

    config.axis(BasicRole::Steering).raw(1000, 2000, 1500).deadband_us(8).out(-100.0f, 100.0f).done();
    config.axis(BasicRole::Throttle).raw(1000, 2000, 1000).out(0.0f, 100.0f).done();
    config.sw(BasicRole::Mode).raw_levels({1000, 2000}).values({0.0f, 1.0f}).done();

    const rc::RcConfigResult result = rclink.apply_config(config);
    if (!result)
    {
        Serial.println(F("RCLink configuration rejected."));
        while (true)
            delay(1000);
    }
}

// ---- Main loop ---- //

void loop()
{
    // Polling update() frequently is required to drain receiver bytes and track freshness.
    rclink.update();

    static bool printed_healthy_frame = false;
    if (rclink.healthy() && (!printed_healthy_frame || rclink.changed()))
    {
        Serial.print(F("steering="));
        Serial.print(rclink.read(BasicRole::Steering));
        Serial.print(F(" throttle="));
        Serial.print(rclink.read(BasicRole::Throttle));
        Serial.print(F(" mode="));
        Serial.println(rclink.read(BasicRole::Mode));
        printed_healthy_frame = true;
    }
    else if (!rclink.healthy())
        printed_healthy_frame = false;
}
