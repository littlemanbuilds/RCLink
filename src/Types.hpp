/**
 * MIT License
 *
 * @brief Core public datatypes for RCLink.
 *
 * @file Types.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>

namespace rc
{
    // Transport capability flags.
    struct RcTransportCaps
    {
        bool has_proto_failsafe{false}; ///< Protocol exposes failsafe bit(s).
        bool has_link_stats{false};     ///< Protocol exposes link stats (RSSI/LQ/etc.).
        bool has_telemetry{false};      ///< Supports telemetry back to TX.
        bool half_duplex{false};        ///< Uses half-duplex UART.
    };

    // Channel kinds & specs.
    enum class ChannelKind : std::uint8_t
    {
        Axis,  ///< Continuous axis channel.
        Switch ///< Discrete multi-position switch channel.
    };

    struct RcAxisSpec
    {
        // Raw calibration in microseconds (pre-scale).
        std::int16_t raw_lo{1000};     ///< Lower raw limit in µs.
        std::int16_t raw_hi{2000};     ///< Upper raw limit in µs.
        std::int16_t raw_center{1500}; ///< Expected center for deadband.
        std::int16_t deadband_us{0};   ///< Deadband around center, µs.

        // Output shaping (post-scale).
        float out_lo{-100.0f}; ///< Output lower bound.
        float out_hi{+100.0f}; ///< Output upper bound.
        float expo{0.0f};      ///< 0..1 cubic-ish response.
        bool invert{false};    ///< Invert axis if true.
    };

    static constexpr std::size_t kRcMaxSwitchVals = 8; ///< Maximum supported switch positions per channel.

    struct RcSwitchSpec
    {
        // ---- Logical (scaled) targets ---- //
        std::uint8_t count{0};          ///< Number of logical positions.
        float vals[kRcMaxSwitchVals]{}; ///< Scaled outputs to snap to.

        // ---- Optional explicit raw snap levels ---- //
        std::uint8_t raw_count{0};                   ///< Number of raw snap levels.
        std::int16_t raw_levels[kRcMaxSwitchVals]{}; ///< Raw snap points in µs.

        // ---- Auto-learning controls (used only when raw_count == 0) ---- //
        bool auto_levels{true};        ///< Learn raw µs centroids from live data. Defaults to on.
        std::uint16_t hyst_us{60};     ///< Hysteresis band (µs) to avoid chatter around boundaries.
        float learn_alpha{0.20f};      ///< EMA rate for centroid updates (0..1]. Higher = slower change.
        std::uint16_t min_sep_us{120}; ///< Minimum separation enforced between learned centroids, µs. ///< TODO: reserved for future enforcement.
    };

    struct RcChannelSpec
    {
        ChannelKind kind{ChannelKind::Axis}; ///< Channel type.
        RcAxisSpec axis{};                   ///< Axis spec (valid if kind==Axis).
        RcSwitchSpec sw{};                   ///< Switch spec (valid if kind==Switch).

        /// @brief Convenience builder for an axis spec.
        static RcChannelSpec makeAxis(const RcAxisSpec &a)
        {
            RcChannelSpec s;
            s.kind = ChannelKind::Axis;
            s.axis = a;
            return s;
        }

        /// @brief Convenience builder for a switch spec.
        static RcChannelSpec makeSwitch(const RcSwitchSpec &w)
        {
            RcChannelSpec s;
            s.kind = ChannelKind::Switch;
            s.sw = w;
            return s;
        }
    };

    // ---- Per-channel failsafe policy ---- //
    struct Failsafe
    {
        enum class Mode : std::uint8_t
        {
            Value,        ///< Apply numeric value.
            HoldLast,     ///< Hold last valid filtered output.
            ClampToOutLo, ///< Clamp to axis out_lo.
            ClampToOutHi  ///< Clamp to axis out_hi.
        };
        Mode mode{Mode::Value}; ///< Failsafe mode.
        std::int16_t value{0};  ///< Applied value when Mode::Value.
    };

    // ---- Link status (unified) ---- //
    struct RcLinkStatus
    {
        bool link_ok{false};                                           ///< Fresh frame within link timeout.
        bool proto_failsafe{false};                                    ///< Protocol-level failsafe flag (e.g., SBUS).
        bool frame_lost{false};                                        ///< Protocol “frame lost” (if provided).
        bool rx_failsafe_sig{false};                                   ///< Receiver failsafe signature match.
        std::uint16_t fps{0};                                          ///< Estimated frames per second.
        std::uint32_t frames{0};                                       ///< Total good frames.
        std::uint32_t crc_errors{0};                                   ///< Parse/CRC errors from transport.
        std::uint32_t last_frame_ms{0};                                ///< Timestamp of last good frame (ms).
        std::uint32_t last_frame_age{0};                               ///< Age since last good frame (ms).
        std::int8_t rssi_dbm{std::numeric_limits<std::int8_t>::min()}; ///< RSSI if known, else min.
        std::uint8_t lq{255};                                          ///< Link quality % (255 = unknown).
    };

    // ---- Snapshot frame ---- //
    template <std::size_t N>
    struct RcFrame
    {
        std::int16_t vals[N]{}; ///< Scaled/safe values for N logical channels.
    };

    // ---- Receiver failsafe signature rule ---- //
    template <std::size_t N>
    struct RcFailsafeRule
    {
        std::int16_t expected[N]{}; ///< Scaled values expected during RX failsafe.
        std::uint8_t check[N]{};    ///< 1 to include channel in the match.
        std::uint8_t tol{2};        ///< Allowed |delta| per channel (scaled units).
        std::uint16_t hold_ms{120}; ///< Must persist at least this long (ms).
    };

} ///< namespace rc.