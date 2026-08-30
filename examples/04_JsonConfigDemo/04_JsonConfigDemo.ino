/**
 * MIT License
 *
 * @brief Load an RCLink role configuration from JSON after declaring the role enum.
 *
 * @file 04_JsonConfigDemo.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#include <RCLink.h>

#include <Arduino.h>

// ---- Wiring configuration ---- //

const int RX_PIN = 18;
const int TX_PIN = -1;
const uint32_t IBUS_BAUD = 115200;

// ---- Roles (same as previous demo) ---- //

#define RC_ROLES(_)                                                                                                    \
    _(Ch1)                                                                                                             \
    _(Ch2)                                                                                                             \
    _(Ch3)                                                                                                             \
    _(Ch4)                                                                                                             \
    _(Ch5)                                                                                                             \
    _(Ch6)                                                                                                             \
    _(Ch7)                                                                                                             \
    _(Ch8)                                                                                                             \
    _(Ch9)                                                                                                             \
    _(Ch10)
RC_DECLARE_ROLES(RC, RC_ROLES)

#include <Json.hpp> // Must come after RC_DECLARE_ROLES so role names can be matched.

// ---- Transport and link ---- //

RcIbusTransport transport;
RcLink<RcIbusTransport, RC> rclink(transport);

// JSON config (kept inline for demo, could also be in SPIFFS/SD/Serial).
// Replace this with your own remote control configuration output.
static const char kJsonConfig[] PROGMEM = R"(
{
  "map": {
    "Ch1": 0,
    "Ch2": 1,
    "Ch3": 2,
    "Ch4": 3,
    "Ch5": 4,
    "Ch6": 5,
    "Ch7": 6,
    "Ch8": 7,
    "Ch9": 8,
    "Ch10": 9
  },
  "axes": {
    "Ch1": { "raw": [1000,2000,1500], "deadband_us": 24, "out": [-100,100], "expo": 0.00 },
    "Ch2": { "raw": [1000,2000,1500], "deadband_us": 24, "out": [-100,100], "expo": 0.00 },
    "Ch3": { "raw": [1000,2000,1500], "deadband_us": 22, "out": [-100,100], "expo": 0.00 },
    "Ch4": { "raw": [1000,2000,1500], "deadband_us": 24, "out": [-100,100], "expo": 0.00 },
    "Ch5": { "raw": [1000,2000,1500], "deadband_us": 22, "out": [-100,100], "expo": 0.00 },
    "Ch6": { "raw": [1000,2000,1500], "deadband_us": 23, "out": [-100,100], "expo": 0.00 }
  },
  "switches": {
    "Ch7": { "raw_levels": [1000,2000], "values": [0,1] },
    "Ch8": { "raw_levels": [1000,2000], "values": [0,1] },
    "Ch9": { "raw_levels": [1000,1500,2000], "values": [0,1,2] },
    "Ch10": { "raw_levels": [1000,2000], "values": [0,1] }
  }
}
)";

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Uncomment this only if your board needs a defined idle input.
    // pinMode(RX_PIN, INPUT_PULLDOWN);

    rclink.begin(Serial2, IBUS_BAUD, RX_PIN, TX_PIN);

    RC_CONFIG(RC, cfg);
    const bool json_loaded = rc::load_json(RC{}, cfg, kJsonConfig);
    const rc::RcConfigResult result = json_loaded ? rclink.apply_config(cfg) : rc::RcConfigResult(false);

    if (!json_loaded || !result)
    {
        Serial.println(F("RCLink JSON configuration rejected. Check names, channel numbers and value ranges."));
        while (true)
            delay(1000);
    }

    Serial.println(F("RCLink JSON config loaded."));
}

void loop()
{
    rclink.update();
    RC_PRINT_ALL(rclink, RC);
    delay(10);
}
