/**
 * MIT License
 *
 * @brief Demonstrate filtering, epsilon suppression, failsafe signatures, and status reporting.
 *
 * @file 05_AdvancedFeatureDemo.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#include <RCLink.h>

#include <Arduino.h>

// ---- Wiring (iBUS, same as earlier demos) ---- //

const int RX_PIN = 18;             // iBUS signal from the receiver.
const int TX_PIN = -1;             // TX is not used for iBUS.
const uint32_t IBUS_BAUD = 115200; // Typical iBUS baud rate.

// ---- Roles (10 channels) ---- //

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
    RC_CFG_MAP_DEFAULT(Flysky, cfg);
    cfg.setLinkTimeout(200); // Link is stale if no new frame arrives for 200 ms.

    // Epsilon = minimum change before reporting (units: % output).
    // Helps ignore tiny jitter, keeps values steady.
    cfg.setEpsilon(Flysky::Ch1_RH, 1); // 1% movement required.
    cfg.setEpsilon(Flysky::Ch2_RV, 1);
    cfg.setEpsilon(Flysky::Ch3_LV, 1);
    cfg.setEpsilon(Flysky::Ch4_LH, 1);
    cfg.setEpsilon(Flysky::Ch5_VrA, 2); // Knobs: 2% step to update.
    cfg.setEpsilon(Flysky::Ch6_VrB, 2);

    // EMA filter = smoothing factor (0.0-1.0, dimensionless).
    // 0.0 = no smoothing; lower nonzero values smooth more, higher values track faster.
    cfg.setAxisFilter(Flysky::Ch1_RH, 0.20f); // 20% blend with previous value.
    cfg.setAxisFilter(Flysky::Ch2_RV, 0.20f);
    cfg.setAxisFilter(Flysky::Ch3_LV, 0.10f); // Throttle: smoother but still responsive.
    cfg.setAxisFilter(Flysky::Ch4_LH, 0.20f);
    cfg.setAxisFilter(Flysky::Ch5_VrA, 0.10f);
    cfg.setAxisFilter(Flysky::Ch6_VrB, 0.10f);

    // Axes: raw calibration, deadband, output range, expo, invert.
    // Tip: raw() is optional if your defaults already match; shown here for clarity.
    cfg.axis(Flysky::Ch1_RH).raw(1000, 2000, 1500).deadband_us(8).out(-100.f, 100.f).expo(0.35f).done();
    cfg.axis(Flysky::Ch2_RV).raw(1000, 2000, 1500).deadband_us(8).out(-100.f, 100.f).expo(0.35f).done();
    cfg.axis(Flysky::Ch3_LV)
        .raw(1000, 2000, 1000)
        .deadband_us(4)
        .out(0.f, 100.f)
        .expo(0.15f)
        .done(); // Throttle (0..100).
    cfg.axis(Flysky::Ch4_LH).raw(1000, 2000, 1500).deadband_us(8).out(-100.f, 100.f).expo(0.35f).done();
    cfg.axis(Flysky::Ch5_VrA).raw(1000, 2000, 1500).deadband_us(2).out(0.f, 100.f).done();
    cfg.axis(Flysky::Ch6_VrB).raw(1000, 2000, 1500).deadband_us(2).out(0.f, 100.f).invert(true).done(); // Inverted.

    // Switches: app-visible values.
    cfg.sw(Flysky::Ch7_SwA).values({0.f, 1.f}).done();
    cfg.sw(Flysky::Ch8_SwB).values({0.f, 1.f}).done();
    cfg.sw(Flysky::Ch9_SwC).values({-1.f, 0.f, +1.f}).done(); // 3-position switch: center is 0.
    cfg.sw(Flysky::Ch10_SwD).values({0.f, 1.f}).done();

    // Commissioning alternative: configure a switch with .auto_levels(), apply
    // the config, then enable learning and physically visit every position.
    // finalize_switch_learning() returns false until every configured position
    // has valid receiver observations; seeded centroids alone never finalize.

    // Per-channel failsafe policies apply when required receiver evidence is unsafe,
    // including stale links, invalid required roles, and protocol failsafe indications.
    cfg.setFailsafePolicy(Flysky::Ch1_RH, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch2_RV, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch3_LV, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch4_LH, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch5_VrA, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch6_VrB, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch7_SwA, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch8_SwB, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch9_SwC, rc::Failsafe::Mode::Value, 0);
    cfg.setFailsafePolicy(Flysky::Ch10_SwD, rc::Failsafe::Mode::Value, 0);

    // Sets the receiver-failsafe signature (when the receiver keeps sending frames but in a
    // known pattern). This DOESN'T apply the output policy automatically.
    const rc::FsRule<Flysky> rx_failsafe_signature = rc::make_signature_selected(
        Flysky{}, /* tol */ 2, /* hold_ms */ 250,
        {{Flysky::Ch1_RH, 100}, {Flysky::Ch2_RV, 100}, {Flysky::Ch3_LV, 100}, {Flysky::Ch4_LH, -100}});
    const rc::RcSignatureResult signature_result = rclink.set_failsafe_signature(rx_failsafe_signature);
    if (!signature_result)
        Serial.println(F("Receiver failsafe signature was rejected."));

    // Control how receiver-failsafe signatures affect outputs. If enabled, outputs switch
    // to the configured failsafe policy, when a signature is detected. If disabled,
    // status().rx_failsafe_sig still reports the match, but outputs continue to follow live frames.
    rclink.apply_rxfs_outputs(false);

    const rc::RcConfigResult config_result = rclink.apply_config(cfg);
    if (!config_result)
    {
        Serial.println(F("RCLink configuration rejected. Check channel mapping and switch/axis settings."));
        while (true)
            delay(1000);
    }

    // One-time capability printout.
    const RcTransportCaps &caps = rclink.caps();
    Serial.print("Transport caps - protoFS: ");
    Serial.print(caps.has_proto_failsafe ? "yes" : "no");
    Serial.print(", linkStats: ");
    Serial.print(caps.has_link_stats ? "yes" : "no");
    Serial.print(", telemetry: ");
    Serial.print(caps.has_telemetry ? "yes" : "no");
    Serial.print(", halfDuplex: ");
    Serial.println(caps.half_duplex ? "yes" : "no");
}

void loop()
{
    rclink.update();

    // Periodic status heartbeat (every second).
    static uint32_t lastHB = 0;
    const uint32_t now = millis();
    const RcLinkStatus &st = rclink.status();

    if (now - lastHB >= 1000)
    {
        lastHB = now;

        Serial.print("[link ");
        Serial.print(st.link_ok ? "OK" : "STALE");
        Serial.print("] fps=");
        Serial.print(st.fps);
        Serial.print(" protoFS=");
        Serial.print(st.proto_failsafe ? "1" : "0");
        Serial.print(" frameLost=");
        Serial.print(st.frame_lost ? "1" : "0");
        Serial.print(" rxFSsig=");
        Serial.print(st.rx_failsafe_sig ? "1" : "0");
        Serial.print(" age=");
        Serial.print(st.last_frame_age);
        Serial.println("ms");
    }

    // Show the *applied* outputs, but only when they change OR a failsafe is active.
    static bool printedFailsafe = false;

    if (!st.link_ok || st.proto_failsafe || st.rx_failsafe_sig)
    {
        if (!printedFailsafe)
        {
            if (!st.link_ok || st.proto_failsafe)
                Serial.println(F("Failsafe: link lost/protocol failsafe."));
            else if (st.rx_failsafe_sig)
                Serial.println(F("Failsafe: receiver signature detected."));

            RC_PRINT_ALL(rclink, Flysky); // Show applied outputs once.
            printedFailsafe = true;       // Do not print again until the state clears.
        }
    }
    else
    {
        // Normal case: print only when outputs change.
        if (rclink.changed())
        {
            RC_PRINT_ALL(rclink, Flysky);
        }

        // Reset, so it can trigger.
        printedFailsafe = false;
    }

    // Example: use individual roles directly (here throttle as 0..100).
    const int16_t throttle = rclink.read(Flysky::Ch3_LV);
    (void)throttle; // Replace with your application logic.

    delay(10); // About 100 Hz loop rate.
}
