/**
 * MIT License
 *
 * @brief Compile-time limits and protocol constants used by RCLink.
 *
 * @file Constants.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace rc
{
static constexpr std::uint8_t kInvalidChannel = 0xFFu;     ///< Sentinel for an unmapped receiver channel.
static constexpr std::size_t kRcMaxReceiverChannels = 32u; ///< Maximum receiver channels represented by status masks.
static constexpr int kNoSignalThresholdUs = 200; ///< Values at or below this are not accepted as channel data.
static constexpr int kRcMinAxisSideSpanUs = 20;  ///< Minimum calibrated span on each active side of an axis centre.

static constexpr int kIbusMaxChannels = 14;                 ///< Maximum iBUS channel count supported by RCLink.
static constexpr int kSbusChannels = 16;                    ///< Standard analogue SBUS channel count.
static constexpr std::uint32_t kIbusInterByteTimeoutMs = 4; ///< Partial iBUS frame reset timeout.
static constexpr std::uint32_t kSbusInterByteTimeoutMs = 4; ///< Partial SBUS frame reset timeout.

static constexpr std::uint32_t kInvalidFrameAgeMs = 0xFFFFFFFFu; ///< Age reported before the first accepted frame.
} // namespace rc
