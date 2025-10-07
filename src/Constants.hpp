/**
 * MIT License
 *
 * @brief Central constants.
 *
 * @file Constants.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>

namespace rc
{
    // Generic limits.
    constexpr std::size_t kBufferSize64 = 64; ///< Fixed buffer size of 64 elements.

    // Link/transport-agnostic signal thresholds.
    constexpr int kNoSignalThresholdUs = 200; ///< Raw µs below this → treat as “no signal”.
    constexpr int kIbusMaxChannels = 14;      ///< Max iBUS channel count.
    constexpr int kSbusChannels = 16;         ///< sBUS channel count.
} ///< namespace rc.