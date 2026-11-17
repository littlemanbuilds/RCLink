/**
 * MIT License
 *
 * @brief Interactive iBUS calibrator (histogram + clustering -> config suggestions).
 *
 * @file Ibus_Calibrate.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "calibration/detail/Calibrate.hpp"
#include "transport/Ibus.hpp"

namespace rc
{
namespace calibrate
{
/**
 * @brief Run guided iBUS sampling and provisional config/JSON suggestions.
 *
 * This optional commissioning helper requires HardwareSerial, Serial.printf and
 * availableForWrite; the supplied learning examples target ESP32-S3. It is not
 * pulled into the portable runtime by RCLink.h.
 *
 * @param uart Receiver UART.
 * @param rxPin Receiver RX pin.
 * @param txPin Unused receive-only TX pin.
 * @param baud Receiver baud rate.
 * @param maxChannels Channels to sample, limited to 14.
 * @param motionThreshUs Jitter motion threshold in raw units.
 * @param defaultDeadbandUs Fallback deadband suggestion.
 * @param useJitterDeadband Derive deadband from observed jitter when true.
 */
inline void run_ibus(::HardwareSerial &uart, int rxPin, int txPin, std::uint32_t baud,
                     std::uint8_t maxChannels = kIbusMaxChannels, int motionThreshUs = 8, int defaultDeadbandUs = 20,
                     bool useJitterDeadband = true)
{
    RcIbusTransport bus;
    pinMode(rxPin, INPUT);
    bus.begin(uart, baud, rxPin, txPin);
    /**
     * @brief Accept only newly decoded iBUS batches, never cached samples.
     *
     * @param now Current millisecond timestamp.
     * @return True when a new valid batch is available.
     */
    auto poll = [&](std::uint32_t now) { return bus.update(now); };
    detail::run_calibration<kIbusMaxChannels>(bus, poll, "iBUS",
        "Receiver iBUS signal -> RX pin; share GND and use compatible signal levels. Keep actuators disconnected.",
        maxChannels, motionThreshUs, defaultDeadbandUs, useJitterDeadband);
}
} // namespace calibrate
} // namespace rc

#ifndef RCLINK_NO_LEGACY_GLOBAL_ALIASES
namespace calibrate = rc::calibrate;
#endif
