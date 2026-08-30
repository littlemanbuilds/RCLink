/**
 * MIT License
 *
 * @brief Hardware-neutral iBUS byte-stream decoder with timeout and sliding re-synchronization.
 *
 * @file IbusDecoder.hpp
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
         * @brief Decode Flysky-style iBUS channel frames without Arduino dependencies.
         *
         * Feed bytes in arrival order. A partial frame is discarded after an inter-byte
         * timeout, while malformed data is re-synchronized one byte at a time so a valid
         * frame immediately following corruption can still be recovered.
         */
class IbusDecoder
{
  public:
    /**
             * @brief Reset parser state and all diagnostic counters.
             */
    void reset() noexcept
    {
        count_ = 0u;
        channel_count_ = 0u;
        channel_valid_mask_ = 0u;
        frames_ = 0u;
        crc_errors_ = 0u;
        parse_errors_ = 0u;
        timeout_resets_ = 0u;
        discarded_bytes_ = 0u;
        last_good_ms_ = 0u;
        last_byte_ms_ = 0u;
        has_last_byte_ = false;
        for (std::size_t i = 0u; i < static_cast<std::size_t>(kIbusMaxChannels); ++i)
            channels_[i] = 0;
    }

    /**
             * @brief Feed one byte into the parser.
             *
             * @param byte Next UART byte.
             * @param now_ms Timestamp associated with this parser service point.
             * @return True when this byte completes a valid channel frame.
             */
    bool feed(std::uint8_t byte, std::uint32_t now_ms) noexcept
    {
        tick(now_ms);

        if (count_ >= kBufferBytes)
        {
            discard_front_(1u);
            ++parse_errors_;
        }

        buffer_[count_++] = byte;
        last_byte_ms_ = now_ms;
        has_last_byte_ = true;

        bool decoded = false;
        for (;;)
        {
            if (count_ == 0u)
                break;

            const std::uint8_t length = buffer_[0];
            if (!plausible_length_(length))
            {
                ++parse_errors_;
                discard_front_(1u);
                continue;
            }

            if (count_ < 2u)
                break;

            if (buffer_[1] != kChannelCommand)
            {
                ++parse_errors_;
                discard_front_(1u);
                continue;
            }

            if (count_ < static_cast<std::size_t>(length))
                break;

            if (checksum_valid_(length))
            {
                decode_(length);
                consume_front_(static_cast<std::size_t>(length));
                ++frames_;
                last_good_ms_ = now_ms;
                decoded = true;
                continue;
            }

            ++crc_errors_;
            discard_front_(1u);
        }

        if (count_ == 0u)
            has_last_byte_ = false;
        return decoded;
    }

    /**
             * @brief Service the partial-frame timeout even when no new bytes arrived.
             *
             * @param now_ms Current timestamp.
             */
    void tick(std::uint32_t now_ms) noexcept
    {
        if (count_ != 0u && has_last_byte_ &&
            static_cast<std::uint32_t>(now_ms - last_byte_ms_) > kIbusInterByteTimeoutMs)
        {
            discarded_bytes_ += static_cast<std::uint32_t>(count_);
            count_ = 0u;
            has_last_byte_ = false;
            ++timeout_resets_;
        }
    }

    /**
             * @brief Number of channels in the last accepted frame.
             */
    int channels() const noexcept
    {
        return static_cast<int>(channel_count_);
    }

    /**
             * @brief Read a channel from the last accepted frame.
             *
             * @param channel Zero-based channel index.
             * @return Raw microseconds, or zero when unavailable.
             */
    int readRaw(int channel) const noexcept
    {
        return (channel >= 0 && static_cast<std::size_t>(channel) < static_cast<std::size_t>(channel_count_))
                   ? channels_[static_cast<std::size_t>(channel)]
                   : 0;
    }

    /// @brief Return validity bits for channels in the last accepted frame.
    std::uint32_t channelValidMask() const noexcept
    {
        return channel_valid_mask_;
    }

    /// @brief Return the accepted frame count.
    std::uint32_t frames() const noexcept
    {
        return frames_;
    }

    /// @brief Return the checksum failure count.
    std::uint32_t crcErrors() const noexcept
    {
        return crc_errors_;
    }

    /// @brief Return the framing and format failure count.
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
    static constexpr std::size_t kBufferBytes = 64u;       ///< Sliding parser capacity.
    static constexpr std::uint8_t kChannelCommand = 0x40u; ///< iBUS servo-channel command byte.
    static constexpr std::uint8_t kMinimumFrameBytes = 6u; ///< Smallest structurally valid channel frame.

    /// @brief Check whether a length byte can describe a supported channel frame.
    static bool plausible_length_(std::uint8_t length) noexcept
    {
        if (length < kMinimumFrameBytes || static_cast<std::size_t>(length) > kBufferBytes)
            return false;
        const std::uint8_t channel_bytes = static_cast<std::uint8_t>(length - 4u);
        return (channel_bytes % 2u) == 0u &&
               static_cast<std::size_t>(channel_bytes / 2u) <= static_cast<std::size_t>(kIbusMaxChannels);
    }

    /// @brief Validate the little-endian subtractive checksum of a buffered frame.
    bool checksum_valid_(std::uint8_t length) const noexcept
    {
        std::uint16_t checksum = 0xFFFFu;
        const std::size_t payload_end = static_cast<std::size_t>(length) - 2u;
        for (std::size_t i = 0u; i < payload_end; ++i)
            checksum = static_cast<std::uint16_t>(checksum - static_cast<std::uint16_t>(buffer_[i]));

        const std::uint16_t received =
            static_cast<std::uint16_t>(buffer_[static_cast<std::size_t>(length) - 2u]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(buffer_[static_cast<std::size_t>(length) - 1u])
                                       << 8u);
        return checksum == received;
    }

    /// @brief Decode channel words from a validated buffered frame.
    void decode_(std::uint8_t length) noexcept
    {
        const std::size_t channel_bytes = static_cast<std::size_t>(length) - 4u;
        std::size_t count = channel_bytes / 2u;
        if (count > static_cast<std::size_t>(kIbusMaxChannels))
            count = static_cast<std::size_t>(kIbusMaxChannels);

        channel_count_ = static_cast<std::uint8_t>(count);
        channel_valid_mask_ =
            (count >= 32u) ? 0xFFFFFFFFu : ((count == 0u) ? 0u : ((1u << static_cast<unsigned>(count)) - 1u));

        std::size_t offset = 2u;
        for (std::size_t i = 0u; i < count; ++i)
        {
            const std::uint16_t lo = static_cast<std::uint16_t>(buffer_[offset++]);
            const std::uint16_t hi = static_cast<std::uint16_t>(buffer_[offset++]);
            channels_[i] = static_cast<int>(static_cast<std::uint16_t>((hi << 8u) | lo));
        }
    }

    /// @brief Discard and diagnose bytes from the front of the sliding buffer.
    void discard_front_(std::size_t count) noexcept
    {
        if (count > count_)
            count = count_;
        discarded_bytes_ += static_cast<std::uint32_t>(count);
        shift_front_(count);
    }

    /// @brief Consume accepted bytes without counting them as recovery discards.
    void consume_front_(std::size_t count) noexcept
    {
        if (count > count_)
            count = count_;
        shift_front_(count);
    }

    /// @brief Shift remaining buffered bytes after removing a front prefix.
    void shift_front_(std::size_t count) noexcept
    {
        const std::size_t remaining = count_ - count;
        for (std::size_t i = 0u; i < remaining; ++i)
            buffer_[i] = buffer_[i + count];
        count_ = remaining;
    }

    std::uint8_t buffer_[kBufferBytes]{};  ///< Sliding parser buffer.
    std::size_t count_{0u};                ///< Bytes currently buffered.
    int channels_[kIbusMaxChannels]{};     ///< Last accepted channel values.
    std::uint8_t channel_count_{0u};       ///< Last accepted channel count.
    std::uint32_t channel_valid_mask_{0u}; ///< Valid channel bits.
    std::uint32_t frames_{0u};             ///< Accepted frames.
    std::uint32_t crc_errors_{0u};         ///< Checksum failures.
    std::uint32_t parse_errors_{0u};       ///< Framing/format failures.
    std::uint32_t timeout_resets_{0u};     ///< Partial-frame timeouts.
    std::uint32_t discarded_bytes_{0u};    ///< Recovery discard count.
    std::uint32_t last_good_ms_{0u};       ///< Last accepted frame timestamp.
    std::uint32_t last_byte_ms_{0u};       ///< Last partial-frame byte timestamp.
    bool has_last_byte_{false};            ///< Explicit partial-frame timestamp validity.
};
} // namespace detail
} // namespace rc
