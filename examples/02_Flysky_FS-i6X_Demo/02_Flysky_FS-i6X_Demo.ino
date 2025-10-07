/**
 * @file 02_Flysky_FS-i6X_Demo.ino
 *
 * @brief 10-channel demo that maps axes/switches and prints values at ~100 Hz.
 *        This example is specific to a Flysky FS-i6X remote control.
 */

#include <Arduino.h>
#include <RCLink.h>

// Wiring configuration.
static constexpr int RX_PIN = 18;             ///< iBUS signal input GPIO.
static constexpr int TX_PIN = -1;             ///< TX not used for iBUS.
static constexpr uint32_t IBUS_BAUD = 115200; ///< iBUS baud rate (typical: 115200).

#define FLYSKY_ROLES(_)                                     \
    _(Ch1_RH)   /**< CH1: Right stick horizontal (roll) */  \
    _(Ch2_RV)   /**< CH2: Right stick vertical (pitch) */   \
    _(Ch3_LV)   /**< CH3: Left stick vertical (throttle) */ \
    _(Ch4_LH)   /**< CH4: Left stick horizontal (yaw) */    \
    _(Ch5_VrA)  /**< CH5: Knob VrA (pot A) */               \
    _(Ch6_VrB)  /**< CH6: Knob VrB (pot B) */               \
    _(Ch7_SwA)  /**< CH7: Switch A */                       \
    _(Ch8_SwB)  /**< CH8: Switch B */                       \
    _(Ch9_SwC)  /**< CH9: Switch C (3-pos) */               \
    _(Ch10_SwD) /**< CH10: Switch D */
RC_DECLARE_ROLES(Flysky, FLYSKY_ROLES)

// Transport + link.
RcIbusTransport transport;                         ///< iBUS transport instance.
RcLink<RcIbusTransport, Flysky> rclink(transport); ///< Protocol-agnostic link.

void setup()
{
    Serial.begin(115200); ///< Initialize serial port.
    delay(50);

    // pinMode(RX_PIN, INPUT_PULLDOWN); ///< Uncomment if your board needs a defined idle.
    rclink.begin(Serial2, IBUS_BAUD, RX_PIN, TX_PIN); ///< Initialize link/transport.

    RC_CONFIG(Flysky, cfg);          ///< Build configuration.
    RC_CFG_MAP_DEFAULT(Flysky, cfg); ///< Map roles in declared order to channels.

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

    rclink.apply_config(cfg); ///< Apply configuration.
}

void loop()
{
    rclink.update();              ///< Poll the receiver and refresh channel values.
    RC_PRINT_ALL(rclink, Flysky); ///< Print all role names and their current values.
    delay(10);                    ///< ~100 Hz print rate.
}
