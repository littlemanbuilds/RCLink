/**
 * @file 03_Json_Config_Demo.ino
 *
 * @brief Demo loading mapping/axes/switches from JSON using Json.hpp helpers.
 *        kJsonConfig taken directly from Ibus_Calibration.
 */

#include <Arduino.h>
#include <RCLink.h>

// Wiring configuration.
static constexpr int RX_PIN = 18;
static constexpr int TX_PIN = -1;
static constexpr uint32_t IBUS_BAUD = 115200;

// ---- Roles (same as previous demo) ----
#define RC_ROLES(_) \
  _(Ch1)            \
  _(Ch2)            \
  _(Ch3)            \
  _(Ch4)            \
  _(Ch5)            \
  _(Ch6)            \
  _(Ch7)            \
  _(Ch8)            \
  _(Ch9)            \
  _(Ch10)
RC_DECLARE_ROLES(RC, RC_ROLES)

#include <Json.hpp> ///< Must come *after* RC_DECLARE_ROLES so generated helpers are available.

// Transport + link.
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
  Serial.begin(115200); ///< Initialize serial port.
  delay(50);

  // pinMode(RX_PIN, INPUT_PULLDOWN); ///< Uncomment if your board needs a defined idle.
  rclink.begin(Serial2, IBUS_BAUD, RX_PIN, TX_PIN); ///< Initialize link/transport.

  RC_CONFIG(RC, cfg);                ///< Build configuration.
  load_json(RC{}, cfg, kJsonConfig); ///< Parse + apply JSON.
  rclink.apply_config(cfg);          ///< Apply configuration.

  Serial.println(F("RCLink JSON config loaded."));
}

void loop()
{
  rclink.update();          ///< Poll RX and refresh channels.
  RC_PRINT_ALL(rclink, RC); ///< Print all roles and values.
  delay(10);                ///< ~100 Hz.
}