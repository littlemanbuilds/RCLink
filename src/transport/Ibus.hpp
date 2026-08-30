/**
 * MIT License
 *
 * @brief Polling iBUS transport backed by the hardware-neutral RCLink decoder.
 *
 * @file Ibus.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "transport/detail/IbusDecoder.hpp"
#include "Types.hpp"

#include <Arduino.h>

#include <cstdint>

namespace rc
{
/**
     * @brief iBUS receiver transport using an Arduino Stream-compatible serial port.
     *
     * ESP32 targets configure the requested RX pin explicitly. On other Arduino
     * architectures the serial object owns its physical pins, so rxPin/txPin are
     * accepted for source compatibility but ignored.
     */
class RcIbusTransport
{
  public:
    /**
         * @brief Start iBUS reception.
         *
         * @tparam SerialT Arduino serial type derived from Stream.
         * @param port Serial object used for receiver input.
         * @param baud Receiver baud rate, normally 115200.
         * @param rxPin RX GPIO on ESP32; ignored on fixed-pin serial implementations.
         * @param txPin Unused for RX-only iBUS.
         */
    template <typename SerialT> void begin(SerialT &port, std::uint32_t baud, int rxPin, int txPin)
    {
        (void)txPin;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        port.begin(static_cast<unsigned long>(baud), SERIAL_8N1, rxPin, -1);
#else
        (void)rxPin;
        port.begin(static_cast<unsigned long>(baud));
#endif
        port_ = &port;
        decoder_.reset();
    }

    /**
         * @brief Attach an already configured Stream without reconfiguring its UART.
         *
         * @param stream Stream to poll.
         */
    void attach(Stream &stream) noexcept
    {
        port_ = &stream;
        decoder_.reset();
    }

    /**
         * @brief Drain available bytes and update the decoder.
         *
         * @param now_ms Timestamp used for inter-byte timeout accounting.
         * @return True when one or more complete valid frames were decoded.
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

    /// @brief Return the channel count from the last accepted frame.
    int channels() const noexcept
    {
        return decoder_.channels();
    }

    /// @brief Read one raw receiver channel in microseconds.
    int readRaw(int ch) const noexcept
    {
        return decoder_.readRaw(ch);
    }

    /// @brief Return false because iBUS has no protocol failsafe bit.
    bool protoFailsafe() const noexcept
    {
        return false;
    }

    /// @brief Return false because iBUS has no frame-lost bit.
    bool frameLost() const noexcept
    {
        return false;
    }

    /// @brief Return the accepted frame count.
    std::uint32_t frames() const noexcept
    {
        return decoder_.frames();
    }

    /// @brief Return the checksum failure count.
    std::uint32_t crcErrors() const noexcept
    {
        return decoder_.crcErrors();
    }

    /// @brief Return the framing and format failure count.
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

    /// @brief Return validity bits for channels in the last accepted frame.
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
         * @brief Describe transport capabilities.
         *
         * @return iBUS capability flags.
         */
    RcTransportCaps caps() const noexcept
    {
        RcTransportCaps c;
        c.has_proto_failsafe = false;
        c.has_link_stats = false;
        c.has_telemetry = false;
        c.half_duplex = false;
        return c;
    }

  private:
    Stream *port_{nullptr};         ///< Attached Arduino byte stream.
    detail::IbusDecoder decoder_{}; ///< Hardware-neutral parser/decoder.
};
} // namespace rc
