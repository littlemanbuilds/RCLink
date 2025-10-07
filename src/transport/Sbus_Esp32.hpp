/**
 * MIT License
 *
 * @brief Minimal SBUS transport for ESP32/ESP32-S3 using inverted UART (100000 8E2).
 *
 * @file Sbus_Esp32.hpp
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
     * @brief SBUS transport (25-byte frames). Exposes protocol flags.
     */
    class RcSbusEsp32Transport
    {
    public:
        /**
         * @brief Initialize SBUS on an inverted UART.
         *
         * SBUS uses 100000 baud, 8E2, inverted logic.
         *
         * @param port Hardware serial reference.
         * @param baud Ignored; SBUS is fixed at 100000 8E2.
         * @param rxPin RX pin.
         * @param txPin TX pin (unused).
         */
        void begin(::HardwareSerial &port, std::uint32_t /*baud*/, int rxPin, int txPin)
        {
            (void)txPin; ///< TX unused.
            port_ = &port;
            // SBUS: 100000 baud, 8E2, inverted.
            port_->begin(100000, SERIAL_8E2, rxPin, -1, true);
            reset();
        }

        /**
         * @brief Parse UART input; return true when a complete new frame is ready.
         * @return true if a full 25-byte SBUS frame was parsed; false otherwise.
         */
        bool update()
        {
            bool new_frame = false;
            while (port_ && port_->available())
            {
                const std::uint8_t b = static_cast<std::uint8_t>(port_->read());
                switch (st_)
                {
                case S::kWaitStart:
                    if (b == 0x0F)
                    {
                        idx_ = 0;
                        buf_[idx_++] = b;
                        st_ = S::kPayload;
                    }
                    break;

                case S::kPayload:
                    buf_[idx_++] = b;
                    if (idx_ == kBuf)
                    {
                        parse_frame();
                        new_frame = true;
                        reset();
                    }
                    break;
                }
            }
            return new_frame;
        }

        /**
         * @brief Number of channels provided by SBUS.
         * @return Always 16 for standard SBUS.
         */
        int channels() const { return kSbusChannels; }

        /**
         * @brief Read a raw channel in microseconds.
         * @param ch Channel index [0..15].
         * @return Channel value in microseconds (~1000..2000), or 0 if out of range.
         */
        int readRaw(int ch) const { return (ch >= 0 && ch < kSbusChannels) ? ch_us_[ch] : 0; }

        /**
         * @brief Protocol-level failsafe flag as exposed by SBUS.
         * @return true if SBUS failsafe is active.
         */
        bool protoFailsafe() const { return fs_flag_; }

        /**
         * @brief Protocol-level frame-lost flag as exposed by SBUS.
         * @return true if SBUS signaled a lost frame.
         */
        bool frameLost() const { return fl_flag_; }

        /**
         * @brief Capability flags for this transport.
         * @return Transport capability structure.
         */
        RcTransportCaps caps() const
        {
            RcTransportCaps c;
            c.has_proto_failsafe = true;
            c.has_link_stats = false;
            c.has_telemetry = false;
            c.half_duplex = false;
            return c;
        }

    private:
        enum class S : std::uint8_t
        {
            kWaitStart, ///< Waiting for frame start byte 0x0F.
            kPayload    ///< Reading payload bytes until full frame is collected.
        };

        /// @brief Reset parser state for the next frame.
        void reset()
        {
            st_ = S::kWaitStart;
            idx_ = 0;
            fs_flag_ = false;
            fl_flag_ = false;
        }

        /// @brief Decode one complete SBUS frame in @p buf_ into @p ch_us_ and flags.
        void parse_frame()
        {
            // buf_[0] = 0x0F, total 25 bytes.
            const std::uint8_t *d = buf_ + 1;
            std::uint16_t ch[16];
            ch[0] = (d[0] | (d[1] << 8)) & 0x07FF;
            ch[1] = ((d[1] >> 3) | (d[2] << 5)) & 0x07FF;
            ch[2] = ((d[2] >> 6) | (d[3] << 2) | (d[4] << 10)) & 0x07FF;
            ch[3] = ((d[4] >> 1) | (d[5] << 7)) & 0x07FF;
            ch[4] = ((d[5] >> 4) | (d[6] << 4)) & 0x07FF;
            ch[5] = ((d[6] >> 7) | (d[7] << 1) | (d[8] << 9)) & 0x07FF;
            ch[6] = ((d[8] >> 2) | (d[9] << 6)) & 0x07FF;
            ch[7] = ((d[9] >> 5) | (d[10] << 3)) & 0x07FF;
            ch[8] = (d[11] | (d[12] << 8)) & 0x07FF;
            ch[9] = ((d[12] >> 3) | (d[13] << 5)) & 0x07FF;
            ch[10] = ((d[13] >> 6) | (d[14] << 2) | (d[15] << 10)) & 0x07FF;
            ch[11] = ((d[15] >> 1) | (d[16] << 7)) & 0x07FF;
            ch[12] = ((d[16] >> 4) | (d[17] << 4)) & 0x07FF;
            ch[13] = ((d[17] >> 7) | (d[18] << 1) | (d[19] << 9)) & 0x07FF;
            ch[14] = ((d[19] >> 2) | (d[20] << 6)) & 0x07FF;
            ch[15] = ((d[20] >> 5) | (d[21] << 3)) & 0x07FF;

            // Flags (d[22]).
            const std::uint8_t flags = d[22];
            fl_flag_ = (flags & (1u << 2)) != 0u; // Frame lost.
            fs_flag_ = (flags & (1u << 3)) != 0u; // Failsafe.

            // Convert 11-bit SBUS range (172..1811) to microseconds (~1000..2000).
            for (int i = 0; i < 16; ++i)
            {
                const int v = static_cast<int>(ch[i]);
                int us = 1000 + ((v - 172) * 1000) / 1639;
                if (us < 800)
                    us = 800;
                if (us > 2200)
                    us = 2200;
                ch_us_[i] = us;
            }
        }

    private:
        static constexpr int kBuf = 25; ///< SBUS frame size in bytes.

        ::HardwareSerial *port_{nullptr}; ///< UART used by transport.
        S st_{S::kWaitStart};             ///< Parser state.
        std::uint8_t buf_[kBuf]{};        ///< Frame buffer.
        std::uint8_t idx_{0};             ///< Buffer index.
        bool fs_flag_{false};             ///< Protocol failsafe bit.
        bool fl_flag_{false};             ///< Protocol frame-lost bit.
        int ch_us_[kSbusChannels]{};      ///< Channels in microseconds.
    };

} ///< namespace rc.