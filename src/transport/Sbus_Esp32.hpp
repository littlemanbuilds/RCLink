/**
 * MIT License
 *
 * @brief ESP32 SBUS transport using inverted 100000-baud 8E2 UART reception.
 *
 * @file Sbus_Esp32.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)

#include "transport/detail/SbusDecoder.hpp"
#include "Types.hpp"

#include <Arduino.h>

#include <cstdint>

namespace rc
{
/**
     * @brief ESP32 SBUS receiver transport.
     *
     * The UART inversion path is intentionally ESP32-specific. The decoder itself
     * remains hardware-neutral and is host-tested independently.
     */
class RcSbusEsp32Transport
{
  public:
    /**
         * @brief Start SBUS on an inverted ESP32 UART.
         *
         * @tparam SerialT ESP32 HardwareSerial-compatible type.
         * @param port Serial object used for receiver input.
         * @param baud Ignored; SBUS is fixed at 100000 baud.
         * @param rxPin RX GPIO.
         * @param txPin Unused for receive-only SBUS.
         */
    template <typename SerialT> void begin(SerialT &port, std::uint32_t baud, int rxPin, int txPin)
    {
        (void)baud;
        (void)txPin;
        port.begin(100000UL, SERIAL_8E2, rxPin, -1, true);
        port_ = &port;
        decoder_.reset();
    }

    /**
         * @brief Drain available SBUS bytes.
         *
         * @param now_ms Timestamp used for parser timeout accounting.
         * @return True when one or more valid frames were decoded.
         */
    bool update(std::uint32_t now_ms)
    {
        bool new_frame = false;
        decoder_.tick(now_ms);
        while (port_ && port_->available() > 0)
        {
            const int value = port_->read();
            if (value >= 0)
                new_frame = decoder_.feed(static_cast<std::uint8_t>(value), now_ms) || new_frame;
        }
        return new_frame;
    }

    /**
         * @brief Arduino convenience overload using millis().
         */
    bool update()
    {
        return update(static_cast<std::uint32_t>(millis()));
    }

    /// @brief Return the standard SBUS analogue channel count.
    int channels() const noexcept
    {
        return decoder_.channels();
    }

    /// @brief Read one decoded SBUS channel in approximate microseconds.
    int readRaw(int ch) const noexcept
    {
        return decoder_.readRaw(ch);
    }

    /// @brief Return the receiver failsafe flag from the last accepted frame.
    bool protoFailsafe() const noexcept
    {
        return decoder_.protoFailsafe();
    }

    /// @brief Return the frame-lost flag from the last accepted frame.
    bool frameLost() const noexcept
    {
        return decoder_.frameLost();
    }

    /// @brief Return the accepted frame count.
    std::uint32_t frames() const noexcept
    {
        return decoder_.frames();
    }

    /// @brief Return the invalid-footer count exposed as the checksum diagnostic.
    std::uint32_t crcErrors() const noexcept
    {
        return decoder_.crcErrors();
    }

    /// @brief Return the parser recovery count.
    std::uint32_t parseErrors() const noexcept
    {
        return decoder_.parseErrors();
    }

    /// @brief Return the partial-frame timeout count.
    std::uint32_t parserTimeouts() const noexcept
    {
        return decoder_.parserTimeouts();
    }

    /// @brief Return bytes discarded during parser recovery.
    std::uint32_t discardedBytes() const noexcept
    {
        return decoder_.discardedBytes();
    }

    /// @brief Return validity bits for the analogue channels.
    std::uint32_t channelValidMask() const noexcept
    {
        return decoder_.channelValidMask();
    }

    /// @brief Return the timestamp of the last accepted frame.
    std::uint32_t lastGoodMs() const noexcept
    {
        return decoder_.lastGoodMs();
    }

    /**
         * @brief Describe SBUS transport capabilities.
         */
    RcTransportCaps caps() const noexcept
    {
        RcTransportCaps c;
        c.has_proto_failsafe = true;
        c.has_link_stats = false;
        c.has_telemetry = false;
        c.half_duplex = false;
        return c;
    }

  private:
    Stream *port_{nullptr};         ///< Attached ESP32 byte stream.
    detail::SbusDecoder decoder_{}; ///< Hardware-neutral parser/decoder.
};
} // namespace rc

#endif // ARDUINO_ARCH_ESP32 || ESP32
