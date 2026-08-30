/**
 * MIT License
 *
 * @brief Map a Flysky FS-i6X receiver into ten named axes and switches.
 *
 * @file 03_FlyskyFSi6XDemo.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#include <RCLink.h>

#include <Arduino.h>

// ---- Wiring configuration ---- //

const int RX_PIN = 18;             // iBUS signal from the receiver.
const int TX_PIN = -1;             // TX is not used for iBUS.
const uint32_t IBUS_BAUD = 115200; // Typical iBUS baud rate.

#define FLYSKY_ROLES(_)                                                                                                \
    _(Ch1_RH)   /* CH1: Right stick horizontal (roll) */                                                               \
    _(Ch2_RV)   /* CH2: Right stick vertical (pitch) */                                                                \
    _(Ch3_LV)   /* CH3: Left stick vertical (throttle) */                                                              \
    _(Ch4_LH)   /* CH4: Left stick horizontal (yaw) */                                                                 \
    _(Ch5_VrA)  /* CH5: Knob VrA (pot A) */                                                                            \
    _(Ch6_VrB)  /* CH6: Knob VrB (pot B) */                                                                            \
    _(Ch7_SwA)  /* CH7: Switch A */                                                                                    \
    _(Ch8_SwB)  /* CH8: Switch B */                                                                                    \
    _(Ch9_SwC)  /* CH9: Switch C (3-pos) */                                                                            \
    _(Ch10_SwD) /* CH10: Switch D */
RC_DECLARE_ROLES(Flysky, FLYSKY_ROLES)

// ---- Transport and link ---- //

RcIbusTransport transport;
RcLink<RcIbusTransport, Flysky> rclink(transport);

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Uncomment this only if your board needs a defined idle input.
    // pinMode(RX_PIN, INPUT_PULLDOWN);

    rclink.begin(Serial2, IBUS_BAUD, RX_PIN, TX_PIN);

    RC_CONFIG(Flysky, cfg);
    RC_CFG_MAP_DEFAULT(Flysky, cfg); // Role 0 uses channel 0, role 1 uses channel 1, and so on.

    // Axes (variable potentiometers included).
    cfg.axis(Flysky::Ch1_RH).out(-100.0f, 100.0f).done();
    cfg.axis(Flysky::Ch2_RV).out(-100.0f, 100.0f).done();
    cfg.axis(Flysky::Ch3_LV).out(0.0f, 100.0f).done();
    cfg.axis(Flysky::Ch4_LH).out(-100.0f, 100.0f).done();
    cfg.axis(Flysky::Ch5_VrA).out(0.0f, 100.0f).done();
    cfg.axis(Flysky::Ch6_VrB).out(0.0f, 100.0f).done();

    // Switches: app-visible values.
    cfg.sw(Flysky::Ch7_SwA).values({0.0f, 1.0f}).done();
    cfg.sw(Flysky::Ch8_SwB).values({0.0f, 1.0f}).done();
    cfg.sw(Flysky::Ch9_SwC).values({0.0f, 1.0f, 2.0f}).done();
    cfg.sw(Flysky::Ch10_SwD).values({0.0f, 1.0f}).done();

    const rc::RcConfigResult result = rclink.apply_config(cfg);
    if (!result)
    {
        Serial.println(F("RCLink configuration rejected. Check channel mapping and switch/axis settings."));
        while (true)
            delay(1000);
    }
}

void loop()
{
    rclink.update();
    RC_PRINT_ALL(rclink, Flysky);
    delay(10); // About 100 prints per second.
}
