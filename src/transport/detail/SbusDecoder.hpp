/**
 * MIT License
 *
 * @brief Hardware-neutral SBUS byte-stream decoder with timeout and sliding re-synchronization.
 *
 * @file SbusDecoder.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "Constants.hpp"

#include <cstddef>
#include <cstdint>

namespace rc
{
namespace detail
{
/**
         * @brief Decode standard 25-byte SBUS frames without Arduino dependencies.
         */
class SbusDecoder
{
  public:
    /**
             * @brief Reset parser state and diagnostics.
             */
    void reset() noexcept
    {
        count_ = 0u;
        failsafe_ = false;
        frame_lost_ = false;
        frames_ = 0u;
        footer_errors_ = 0u;
        parse_errors_ = 0u;
        timeout_resets_ = 0u;
        discarded_bytes_ = 0u;
        last_good_ms_ = 0u;
        last_byte_ms_ = 0u;
        has_last_byte_ = false;
        for (std::size_t i = 0u; i < static_cast<std::size_t>(kSbusChannels); ++i)
            channels_us_[i] = 0;
    }

    /**
             * @brief Feed one SBUS byte.
             *
             * @param byte Next UART byte.
             * @param now_ms Timestamp associated with this parser service point.
             * @return True when a complete valid frame is decoded.
             */
    bool feed(std::uint8_t byte, std::uint32_t now_ms) noexcept
    {
        tick(now_ms);

        if (count_ == 0u && byte != kStartByte)
        {
            ++discarded_bytes_;
            return false;
        }

        if (count_ >= kFrameBytes)
        {
            ++parse_errors_;
            resync_after_error_();
        }

        buffer_[count_++] = byte;
        last_byte_ms_ = now_ms;
        has_last_byte_ = true;

        if (count_ < kFrameBytes)
            return false;

        if (valid_footer_(buffer_[kFrameBytes - 1u]))
        {
            parse_frame_();
            count_ = 0u;
            has_last_byte_ = false;
            ++frames_;
            last_good_ms_ = now_ms;
            return true;
        }

        ++footer_errors_;
        resync_after_error_();
        return false;
    }

    /**
             * @brief Reset a stalled partial frame after the inter-byte timeout.
             *
             * @param now_ms Current timestamp.
             */
    void tick(std::uint32_t now_ms) noexcept
    {
        if (count_ != 0u && has_last_byte_ &&
            static_cast<std::uint32_t>(now_ms - last_byte_ms_) > kSbusInterByteTimeoutMs)
        {
            discarded_bytes_ += static_cast<std::uint32_t>(count_);
            count_ = 0u;
            has_last_byte_ = false;
            ++timeout_resets_;
        }
    }

    /// @brief Return the standard SBUS analogue channel count.
    int channels() const noexcept
    {
        return kSbusChannels;
    }

    /**
             * @brief Read one decoded channel in approximate microseconds.
             *
             * @param channel Zero-based channel index [0,15].
             * @return Converted channel value, or zero when out of range.
             */
    int readRaw(int channel) const noexcept
    {
        return (channel >= 0 && channel < kSbusChannels) ? channels_us_[static_cast<std::size_t>(channel)] : 0;
    }

    /// @brief Return the failsafe flag from the last accepted frame.
    bool protoFailsafe() const noexcept
    {
        return failsafe_;
    }

    /// @brief Return the frame-lost flag from the last accepted frame.
    bool frameLost() const noexcept
    {
        return frame_lost_;
    }

    /// @brief Return validity bits for all standard analogue channels.
    std::uint32_t channelValidMask() const noexcept
    {
        return 0xFFFFu;
    }

    /// @brief Return the accepted frame count.
    std::uint32_t frames() const noexcept
    {
        return frames_;
    }

    /// @brief Return invalid footers through the transport checksum diagnostic.
    std::uint32_t crcErrors() const noexcept
    {
        return footer_errors_;
    }

    /// @brief Return the parser state recovery count.
    std::uint32_t parseErrors() const noexcept
    {
        return parse_errors_;
    }

    /// @brief Return the partial-frame timeout count.
    std::uint32_t parserTimeouts() const noexcept
    {
        return timeout_resets_;
    }

    /// @brief Return bytes discarded during parser recovery.
    std::uint32_t discardedBytes() const noexcept
    {
        return discarded_bytes_;
    }

    /// @brief Return the timestamp of the last accepted frame.
    std::uint32_t lastGoodMs() const noexcept
    {
        return last_good_ms_;
    }

  private:
    static constexpr std::size_t kFrameBytes = 25u;   ///< Standard SBUS frame size.
    static constexpr std::uint8_t kStartByte = 0x0Fu; ///< Standard SBUS frame marker.

    /// @brief Accept standard SBUS footer values used by receiver variants.
    static bool valid_footer_(std::uint8_t footer) noexcept
    {
        return footer == 0x00u || footer == 0x04u || footer == 0x14u || footer == 0x24u || footer == 0x34u;
    }

    /// @brief Recover the next buffered start marker after a malformed frame.
    void resync_after_error_() noexcept
    {
        std::size_t next_start = kFrameBytes;
        for (std::size_t i = 1u; i < count_; ++i)
        {
            if (buffer_[i] == kStartByte)
            {
                next_start = i;
                break;
            }
        }

        if (next_start >= count_)
        {
            discarded_bytes_ += static_cast<std::uint32_t>(count_);
            count_ = 0u;
            has_last_byte_ = false;
            return;
        }

        discarded_bytes_ += static_cast<std::uint32_t>(next_start);
        const std::size_t remaining = count_ - next_start;
        for (std::size_t i = 0u; i < remaining; ++i)
            buffer_[i] = buffer_[i + next_start];
        count_ = remaining;
    }

    /// @brief Unpack and convert one structurally valid SBUS frame.
    void parse_frame_() noexcept
    {
        const std::uint8_t *d = buffer_ + 1u;
        std::uint16_t ch[kSbusChannels]{};

        ch[0] = static_cast<std::uint16_t>(
            (static_cast<std::uint32_t>(d[0]) | (static_cast<std::uint32_t>(d[1]) << 8u)) & 0x07FFu);
        ch[1] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[1]) >> 3u) | (static_cast<std::uint32_t>(d[2]) << 5u)) & 0x07FFu);
        ch[2] = static_cast<std::uint16_t>(((static_cast<std::uint32_t>(d[2]) >> 6u) |
                                            (static_cast<std::uint32_t>(d[3]) << 2u) |
                                            (static_cast<std::uint32_t>(d[4]) << 10u)) &
                                           0x07FFu);
        ch[3] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[4]) >> 1u) | (static_cast<std::uint32_t>(d[5]) << 7u)) & 0x07FFu);
        ch[4] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[5]) >> 4u) | (static_cast<std::uint32_t>(d[6]) << 4u)) & 0x07FFu);
        ch[5] = static_cast<std::uint16_t>(((static_cast<std::uint32_t>(d[6]) >> 7u) |
                                            (static_cast<std::uint32_t>(d[7]) << 1u) |
                                            (static_cast<std::uint32_t>(d[8]) << 9u)) &
                                           0x07FFu);
        ch[6] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[8]) >> 2u) | (static_cast<std::uint32_t>(d[9]) << 6u)) & 0x07FFu);
        ch[7] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[9]) >> 5u) | (static_cast<std::uint32_t>(d[10]) << 3u)) & 0x07FFu);
        ch[8] = static_cast<std::uint16_t>(
            (static_cast<std::uint32_t>(d[11]) | (static_cast<std::uint32_t>(d[12]) << 8u)) & 0x07FFu);
        ch[9] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[12]) >> 3u) | (static_cast<std::uint32_t>(d[13]) << 5u)) & 0x07FFu);
        ch[10] = static_cast<std::uint16_t>(((static_cast<std::uint32_t>(d[13]) >> 6u) |
                                             (static_cast<std::uint32_t>(d[14]) << 2u) |
                                             (static_cast<std::uint32_t>(d[15]) << 10u)) &
                                            0x07FFu);
        ch[11] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[15]) >> 1u) | (static_cast<std::uint32_t>(d[16]) << 7u)) & 0x07FFu);
        ch[12] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[16]) >> 4u) | (static_cast<std::uint32_t>(d[17]) << 4u)) & 0x07FFu);
        ch[13] = static_cast<std::uint16_t>(((static_cast<std::uint32_t>(d[17]) >> 7u) |
                                             (static_cast<std::uint32_t>(d[18]) << 1u) |
                                             (static_cast<std::uint32_t>(d[19]) << 9u)) &
                                            0x07FFu);
        ch[14] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[19]) >> 2u) | (static_cast<std::uint32_t>(d[20]) << 6u)) & 0x07FFu);
        ch[15] = static_cast<std::uint16_t>(
            ((static_cast<std::uint32_t>(d[20]) >> 5u) | (static_cast<std::uint32_t>(d[21]) << 3u)) & 0x07FFu);

        const std::uint8_t flags = d[22];
        frame_lost_ = (flags & (1u << 2u)) != 0u;
        failsafe_ = (flags & (1u << 3u)) != 0u;

        for (std::size_t i = 0u; i < static_cast<std::size_t>(kSbusChannels); ++i)
        {
            const int value = static_cast<int>(ch[i]);
            int microseconds = 1000 + ((value - 172) * 1000) / 1639;
            if (microseconds < 800)
                microseconds = 800;
            if (microseconds > 2200)
                microseconds = 2200;
            channels_us_[i] = microseconds;
        }
    }

    std::uint8_t buffer_[kFrameBytes]{}; ///< Partial SBUS frame.
    std::size_t count_{0u};              ///< Bytes currently buffered.
    int channels_us_[kSbusChannels]{};   ///< Last accepted analogue channels.
    bool failsafe_{false};               ///< Last failsafe flag.
    bool frame_lost_{false};             ///< Last frame-lost flag.
    std::uint32_t frames_{0u};           ///< Accepted frames.
    std::uint32_t footer_errors_{0u};    ///< Invalid footer count.
    std::uint32_t parse_errors_{0u};     ///< Parser state recovery count.
    std::uint32_t timeout_resets_{0u};   ///< Partial-frame timeout resets.
    std::uint32_t discarded_bytes_{0u};  ///< Recovery discard count.
    std::uint32_t last_good_ms_{0u};     ///< Last accepted frame timestamp.
    std::uint32_t last_byte_ms_{0u};     ///< Last partial-frame byte timestamp.
    bool has_last_byte_{false};          ///< Explicit timestamp validity.
};
} // namespace detail
} // namespace rc
