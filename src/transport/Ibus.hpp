/**
 * MIT License
 *
 * @brief Robust polling iBUS transport (no heap, no IRQ).
 *
 * @file Ibus.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <Types.hpp>
#include <Constants.hpp>

namespace rc
{
    /**
     * @brief iBUS frame reader: [len][0x40][ch0_lo][ch0_hi] ... [chN_hi][crc_lo][crc_hi].
     */
    class RcIbusTransport
    {
    public:
        /**
         * @brief Begin iBUS on a standard UART (8N1).
         * @param port Hardware serial reference.
         * @param baud UART baud rate.
         * @param rxPin RX pin.
         * @param txPin TX pin (ignored; RX-only).
         */
        void begin(::HardwareSerial &port, std::uint32_t baud, int rxPin, int txPin)
        {
            (void)txPin; ///< RX-only.
            port_ = &port;
            port_->begin(baud, SERIAL_8N1, rxPin, -1);
            reset_frame();
        }

        /**
         * @brief Drain UART and parse bytes; indicates when a complete frame is ready.
         * @return true if a complete, valid iBUS frame was parsed; false otherwise.
         */
        bool update()
        {
            bool new_frame = false;
            while (port_ && port_->available())
            {
                const std::uint8_t b = static_cast<std::uint8_t>(port_->read());
                switch (state_)
                {
                case S::kWaitLen:
                    len_ = b; ///< Total frame length (including CRC).
                    buf_[0] = b;
                    idx_ = 1;
                    if (len_ < 4 || len_ > sizeof(buf_))
                    {
                        reset_frame();
                        break;
                    }
                    state_ = S::kWaitCmd;
                    break;

                case S::kWaitCmd:
                    cmd_ = b;
                    buf_[idx_++] = b;
                    if (cmd_ != 0x40) ///< 0x40 = channel data.
                    {
                        reset_frame();
                        break;
                    }
                    state_ = S::kPayload;
                    break;

                case S::kPayload:
                    buf_[idx_++] = b;
                    if (idx_ >= len_)
                    {
                        if (validate())
                        {
                            decode_channels();
                            last_good_ms_ = millis();
                            frames_++;
                            new_frame = true;
                        }
                        else
                        {
                            crc_errors_++;
                        }
                        reset_frame();
                    }
                    break;
                }
            }
            return new_frame;
        }

        /**
         * @brief Number of channels found in the last valid frame.
         * @return Channel count.
         */
        int channels() const { return ch_count_; }

        /**
         * @brief Read a raw channel value in microseconds.
         * @param ch Channel index [0..channels()-1].
         * @return Channel value in microseconds (~1000..2000), or 0 if out of range.
         */
        int readRaw(int ch) const { return (ch >= 0 && ch < ch_count_) ? ch_raw_[ch] : 0; }

        /**
         * @brief Protocol-level failsafe flag (iBUS does not expose one).
         * @return Always false for iBUS.
         */
        bool protoFailsafe() const { return false; }

        /**
         * @brief Protocol-level frame-lost flag (iBUS does not expose one).
         * @return Always false for iBUS.
         */
        bool frameLost() const { return false; }

        /**
         * @brief Capability flags for this transport.
         * @return Transport capability structure.
         */
        RcTransportCaps caps() const
        {
            RcTransportCaps c;
            c.has_proto_failsafe = false;
            c.has_link_stats = false;
            c.has_telemetry = false;
            c.half_duplex = false;
            return c;
        }

        /**
         * @brief Total number of valid frames parsed.
         * @return Frame count.
         */
        std::uint32_t frames() const { return frames_; }

        /**
         * @brief Number of CRC failures observed.
         * @return CRC error count.
         */
        std::uint32_t crcErrors() const { return crc_errors_; }

        /**
         * @brief Timestamp (ms) of the last valid frame.
         * @return Milliseconds since boot of last valid frame.
         */
        std::uint32_t lastGoodMs() const { return last_good_ms_; }

    private:
        enum class S : std::uint8_t
        {
            kWaitLen, ///< Waiting for length byte.
            kWaitCmd, ///< Waiting for command byte (0x40 for channel data).
            kPayload  ///< Reading payload bytes until full frame is collected.
        };

        /// @brief Reset parser state for the next frame.
        void reset_frame()
        {
            state_ = S::kWaitLen;
            idx_ = 0;
            len_ = 0;
            cmd_ = 0;
        }

        /// @brief Validate the buffer with the iBUS checksum.
        /// @return true if checksum matches.
        bool validate() const
        {
            // Checksum = 0xFFFF - sum(bytes[0..len-3]); last two are CRC.
            if (len_ < 4)
                return false;
            std::uint16_t sum = 0xFFFF;
            for (std::uint16_t i = 0; i < (len_ - 2); ++i)
                sum = static_cast<std::uint16_t>(sum - buf_[i]);
            const std::uint16_t crc =
                static_cast<std::uint16_t>(buf_[len_ - 2]) |
                static_cast<std::uint16_t>(buf_[len_ - 1] << 8);
            return sum == crc;
        }

        /// @brief Decode the channel payload into microseconds.
        void decode_channels()
        {
            // After [len][cmd], channels occupy (len - 2 /*crc*/ - 1 /*cmd*/) bytes.
            const std::uint16_t bytes_channels =
                static_cast<std::uint16_t>((len_ - 2) - 1);
            ch_count_ = static_cast<int>(bytes_channels / 2);
            if (ch_count_ > kIbusMaxChannels)
                ch_count_ = kIbusMaxChannels;

            std::uint16_t off = 2; ///< After len + cmd.
            for (int i = 0; i < ch_count_; ++i)
            {
                const std::uint16_t lo = buf_[off++], hi = buf_[off++];
                const std::uint16_t v = static_cast<std::uint16_t>((hi << 8) | lo); ///< ~1000..2000 µs typical.
                ch_raw_[i] = static_cast<int>(v);
            }
        }

    private:
        static constexpr int kBuf = 64;                 ///< Working buffer size in bytes.
        static constexpr int kMaxCh = kIbusMaxChannels; ///< Maximum iBUS channels supported.

        ::HardwareSerial *port_{nullptr}; ///< UART used by transport.
        S state_{S::kWaitLen};            ///< Parser state.
        std::uint8_t buf_[kBuf]{};        ///< Working buffer.
        std::uint16_t idx_{0};            ///< Buffer index.
        std::uint16_t len_{0};            ///< Expected frame length.
        std::uint8_t cmd_{0};             ///< Command byte.
        int ch_raw_[kMaxCh]{};            ///< Channels (µs).
        int ch_count_{0};                 ///< Channel count.
        std::uint32_t frames_{0};         ///< Valid frames.
        std::uint32_t crc_errors_{0};     ///< CRC error count.
        std::uint32_t last_good_ms_{0};   ///< Timestamp of last frame (ms).
    };

} ///< namespace rc.