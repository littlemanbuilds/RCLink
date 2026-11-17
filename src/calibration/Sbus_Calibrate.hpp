/**
 * MIT License
 *
 * @brief Guided ESP32 S.BUS calibration using the shared commissioning engine.
 *
 * @file Sbus_Calibrate.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-11-17
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */
#pragma once

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include "calibration/detail/Calibrate.hpp"
#include "transport/Sbus_Esp32.hpp"

namespace rc
{
namespace calibrate
{
/**
 * @brief Run guided S.BUS sampling and provisional converted-raw config/JSON suggestions.
 *
 * Uses inverted 100000-baud 8E2 reception. Native codes are never normalized or
 * substituted for converted raw calibration values. Any lost/failsafe evidence
 * within a polling batch excludes that batch from calibration, even if its latest
 * frame is healthy. No application safety latch is added.
 *
 * @param uart Receiver UART.
 * @param rxPin Receiver RX pin.
 * @param txPin Unused receive-only TX pin.
 * @param baud Accepted for interface parity; S.BUS is fixed at 100000 baud.
 * @param maxChannels Channels to sample, limited to 16.
 * @param motionThreshUs Jitter motion threshold in converted raw units.
 * @param defaultDeadbandUs Fallback deadband suggestion.
 * @param useJitterDeadband Derive deadband from observed jitter when true.
 */
inline void run_sbus(::HardwareSerial &uart, int rxPin, int txPin, std::uint32_t baud,
                     std::uint8_t maxChannels = kSbusChannels, int motionThreshUs = 8, int defaultDeadbandUs = 20,
                     bool useJitterDeadband = true)
{
    RcSbusEsp32Transport bus;
    pinMode(rxPin, INPUT);
    bus.begin(uart, baud, rxPin, txPin);
    /**
     * @brief Acquire a new batch and exclude all observed protocol faults.
     *
     * @param now Current millisecond timestamp.
     * @return True only for a new batch with no current or superseded fault frames.
     */
    auto poll = [&](std::uint32_t now)
    {
        const auto lost = bus.lostFrames(), failsafe = bus.failsafeFrames();
        return bus.update(now) && !bus.frameLost() && !bus.protoFailsafe() &&
               bus.lostFrames() == lost && bus.failsafeFrames() == failsafe;
    };
    detail::run_calibration<kSbusChannels>(bus, poll, "S.BUS",
        "S.BUS signal -> RX pin, common GND, compatible levels, no TX. Inverted 100000 baud 8E2; keep actuators disconnected.\n"
        "Native codes are 11-bit diagnostics; suggestions use converted raw values. Stale/lost/failsafe batches are excluded.",
        maxChannels, motionThreshUs, defaultDeadbandUs, useJitterDeadband);
    Serial.printf("Protocol evidence: %lu accepted lost-flag frames, %lu failsafe-flag frames (not RF-loss incidents).\n",
                  static_cast<unsigned long>(bus.lostFrames()), static_cast<unsigned long>(bus.failsafeFrames()));
}
} // namespace calibrate
} // namespace rc

#ifndef RCLINK_NO_LEGACY_GLOBAL_ALIASES
namespace calibrate = rc::calibrate;
#endif
#endif
