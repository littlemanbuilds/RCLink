/**
 * MIT License
 *
 * @brief Protocol-agnostic RCLink mapper, validity model, shaping, filtering, and failsafe logic.
 *
 * @file Link.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "Config.hpp"
#include "Constants.hpp"
#include "Types.hpp"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rc
{
namespace detail
{
/// @brief Compute a wrapping unsigned sequence difference.
constexpr std::uint32_t sequence_delta(std::uint32_t current, std::uint32_t previous) noexcept
{
    return static_cast<std::uint32_t>(current - previous);
}

/// @brief Prefer a timestamp-aware transport update overload when available.
template <typename Transport>
auto transport_update(Transport &transport, std::uint32_t now_ms, int)
    -> decltype(static_cast<bool>(transport.update(now_ms)))
{
    return static_cast<bool>(transport.update(now_ms));
}

/// @brief Fall back to a transport update overload without a timestamp.
template <typename Transport> bool transport_update(Transport &transport, std::uint32_t, long)
{
    return static_cast<bool>(transport.update());
}

/// @brief Read the transport-reported channel count when available.
template <typename Transport>
auto transport_channels(const Transport &transport, int) -> decltype(static_cast<int>(transport.channels()))
{
    return static_cast<int>(transport.channels());
}

/// @brief Supply the library channel limit when a transport has no count diagnostic.
template <typename Transport> int transport_channels(const Transport &, long)
{
    return static_cast<int>(kRcMaxReceiverChannels);
}

/// @brief Read the transport channel-validity mask when available.
template <typename Transport>
auto transport_valid_mask(const Transport &transport, int)
    -> decltype(static_cast<std::uint32_t>(transport.channelValidMask()))
{
    return static_cast<std::uint32_t>(transport.channelValidMask());
}

/// @brief Infer contiguous valid channels when a transport has no validity mask.
template <typename Transport> std::uint32_t transport_valid_mask(const Transport &transport, long)
{
    int count = transport_channels(transport, 0);
    if (count <= 0)
        return 0u;
    if (count >= 32)
        return 0xFFFFFFFFu;
    return (1u << static_cast<unsigned>(count)) - 1u;
}

/// @brief Read the optional transport frame diagnostic when available.
template <typename Transport>
auto transport_frames(const Transport &transport, int) -> decltype(static_cast<std::uint32_t>(transport.frames()))
{
    return static_cast<std::uint32_t>(transport.frames());
}

/// @brief Supply zero when a transport has no frame diagnostic.
template <typename Transport> std::uint32_t transport_frames(const Transport &, long)
{
    return 0u;
}

/// @brief Read the optional transport checksum-error diagnostic when available.
template <typename Transport>
auto transport_crc_errors(const Transport &transport, int)
    -> decltype(static_cast<std::uint32_t>(transport.crcErrors()))
{
    return static_cast<std::uint32_t>(transport.crcErrors());
}

/// @brief Supply zero when a transport has no checksum-error diagnostic.
template <typename Transport> std::uint32_t transport_crc_errors(const Transport &, long)
{
    return 0u;
}

/// @brief Read the optional transport parse-error diagnostic when available.
template <typename Transport>
auto transport_parse_errors(const Transport &transport, int)
    -> decltype(static_cast<std::uint32_t>(transport.parseErrors()))
{
    return static_cast<std::uint32_t>(transport.parseErrors());
}

/// @brief Supply zero when a transport has no parse-error diagnostic.
template <typename Transport> std::uint32_t transport_parse_errors(const Transport &, long)
{
    return 0u;
}

/// @brief Read the optional parser-timeout diagnostic when available.
template <typename Transport>
auto transport_parser_timeouts(const Transport &transport, int)
    -> decltype(static_cast<std::uint32_t>(transport.parserTimeouts()))
{
    return static_cast<std::uint32_t>(transport.parserTimeouts());
}

/// @brief Supply zero when a transport has no parser-timeout diagnostic.
template <typename Transport> std::uint32_t transport_parser_timeouts(const Transport &, long)
{
    return 0u;
}

/// @brief Read the optional discarded-byte diagnostic when available.
template <typename Transport>
auto transport_discarded_bytes(const Transport &transport, int)
    -> decltype(static_cast<std::uint32_t>(transport.discardedBytes()))
{
    return static_cast<std::uint32_t>(transport.discardedBytes());
}

/// @brief Supply zero when a transport has no discarded-byte diagnostic.
template <typename Transport> std::uint32_t transport_discarded_bytes(const Transport &, long)
{
    return 0u;
}
} // namespace detail

/**
     * @brief Map receiver transport channels into validated logical application roles.
     *
     * RcLink deliberately separates transport freshness from logical-role validity.
     * A frame can be fresh while still being unusable because a mapped channel is
     * missing. Required-role failure therefore fails closed instead of synthesizing
     * a plausible neutral command.
     *
     * @tparam Transport Receiver transport providing update/readRaw/protocol status methods.
     * @tparam E Logical role enum ending in `Count`.
     */
template <class Transport, typename E> class RcLink
{
  public:
    static constexpr std::size_t N = static_cast<std::size_t>(E::Count); ///< Logical role count.

    static_assert(std::is_enum<E>::value, "RcLink<E>: E must be an enum with a trailing Count.");
    static_assert(static_cast<std::size_t>(E::Count) > 0u, "RcLink<E>: E::Count must be > 0.");
    static_assert(kRcMaxSwitchVals >= 2u && kRcMaxSwitchVals <= 16u, "RcLink: kRcMaxSwitchVals must be in [2..16].");

    /**
         * @brief Construct a link around a transport instance.
         *
         * @param transport Transport reference; RCLink does not own it.
         */
    explicit RcLink(Transport &transport) noexcept : tx_(transport) {}

    /**
         * @brief Initialize the transport using its existing begin() contract.
         *
         * @tparam SerialT Serial type accepted by the selected transport.
         * @param port Serial/UART object.
         * @param baud Requested baud rate.
         * @param rxPin Receiver input pin.
         * @param txPin Optional transmit pin.
         */
    template <typename SerialT> void begin(SerialT &port, std::uint32_t baud, int rxPin, int txPin)
    {
        tx_.begin(port, baud, rxPin, txPin);
        caps_ = tx_.caps();
    }

    /**
         * @brief Validate and activate a complete logical-role configuration.
         *
         * @param config Configuration to activate.
         * @return Detailed validation result.
         *
         * Existing code may continue to ignore the return value. Invalid
         * configurations remain fail-closed and expose zero until corrected.
         */
    RcConfigResult apply_config(const RcConfig<E> &config)
    {
        cfg_ = config;
        const RcConfigResult result = validate_config(cfg_);
        config_valid_ = result.ok;
        status_.config_valid = result.ok;
        reset_runtime_for_config_();
        zero_outputs_();

        if (!result.ok)
        {
            status_.frame_valid = false;
            status_.required_roles_valid = false;
            status_.healthy = false;
            return result;
        }

        for (std::size_t i = 0u; i < N; ++i)
        {
            axis_center_out_[i] = axis_center_output(cfg_.specs[i].axis);

            const RcChannelSpec &spec = cfg_.specs[i];
            if (spec.kind == ChannelKind::Switch && spec.sw.auto_levels && spec.sw.raw_count == 0u)
                seed_switch_centroids_(i, spec.axis.raw_lo, spec.axis.raw_hi, spec.sw.count);
        }

        // A newly activated mapping has not yet consumed a frame under its
        // own validity rules. Expose configured failsafe outputs until it does.
        for (std::size_t i = 0u; i < N; ++i)
            out_.vals[i] = apply_failsafe_(i);
        last_ = out_;
        return result;
    }

    /**
         * @brief Configure a receiver-failsafe signature.
         *
         * @param rule Signature rule in scaled application space.
         * @return Validation result; an empty check mask is rejected.
         */
    RcSignatureResult set_failsafe_signature(const RcFailsafeRule<N> &rule) noexcept
    {
        bool any = false;
        for (std::size_t i = 0u; i < N; ++i)
            any = any || (rule.check[i] != 0u);

        reset_signature_tracking_();
        if (!any)
        {
            fsig_set_ = false;
            return RcSignatureResult(false, RcSignatureError::EmptyMask);
        }

        fsig_ = rule;
        fsig_set_ = true;
        return RcSignatureResult(true, RcSignatureError::None);
    }

    /**
         * @brief Disable receiver-failsafe signature matching.
         */
    void clear_failsafe_signature() noexcept
    {
        fsig_set_ = false;
        reset_signature_tracking_();
    }

    /**
         * @brief Return the currently stored receiver-failsafe signature.
         */
    const RcFailsafeRule<N> &failsafe_signature() const noexcept
    {
        return fsig_;
    }

    /**
         * @brief Build the configured expected failsafe frame.
         *
         * @return Expected values for checked roles; current outputs elsewhere.
         */
    RcFrame<N> failsafe_expected_frame() const noexcept
    {
        RcFrame<N> frame{};
        for (std::size_t i = 0u; i < N; ++i)
            frame.vals[i] = (fsig_.check[i] != 0u) ? fsig_.expected[i] : out_.vals[i];
        return frame;
    }

    /**
         * @brief Select whether a confirmed receiver signature applies configured failsafe outputs.
         *
         * @param enable True to fail outputs on signature detection; false to report only.
         *
         * Link staleness, missing required roles, invalid configuration, and protocol
         * failsafe always fail closed regardless of this policy.
         */
    void apply_rxfs_outputs(bool enable) noexcept
    {
        rxfs_apply_outputs_ = enable;
    }

    /// @brief Return whether signature confirmation applies configured failsafe outputs.
    bool is_apply_rxfs_outputs() const noexcept
    {
        return rxfs_apply_outputs_;
    }

    /**
         * @brief Change an active axis EMA coefficient safely.
         *
         * @param role Logical role.
         * @param alpha Filter alpha in [0,1].
         * @return True when the update was accepted.
         */
    bool set_axis_filter(E role, float alpha) noexcept
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N || cfg_.specs[i].kind != ChannelKind::Axis || !detail::finite_float(alpha))
            return false;
        if (alpha < 0.0f)
            alpha = 0.0f;
        if (alpha > 1.0f)
            alpha = 1.0f;
        cfg_.axis_ema_alpha[i] = alpha;
        return true;
    }

    /**
         * @brief Enable or disable commissioning-time adaptive switch learning.
         *
         * @param enable True only while intentionally commissioning learnable switches.
         * @return True if learning was enabled or disabled successfully.
         *
         * `RcSwitchSpec::auto_levels` merely marks a role as learnable. No centroid
         * movement occurs in normal service unless this runtime gate is enabled.
         * Enabling starts a new commissioning session and clears prior position-
         * observation evidence for every learnable switch.
         */
    bool set_switch_learning_enabled(bool enable) noexcept
    {
        if (enable && !config_valid_)
            return false;

        if (enable)
        {
            for (std::size_t i = 0u; i < N; ++i)
            {
                if (cfg_.specs[i].kind == ChannelKind::Switch && cfg_.specs[i].sw.auto_levels)
                {
                    sw_observed_mask_[i] = 0u;
                    sw_active_idx_[i] = -1;
                }
            }
        }
        switch_learning_enabled_ = enable;
        return true;
    }

    /// @brief Return whether commissioning-time switch learning is enabled.
    bool switch_learning_enabled() const noexcept
    {
        return switch_learning_enabled_;
    }

    /**
         * @brief Stop adaptive learning and finalize observed switch positions.
         *
         * Every configured position of every learnable switch must have been
         * selected by valid receiver evidence during the current learning session,
         * and the resulting centroids must retain their configured separation.
         *
         * @return True when observation and separation requirements are satisfied.
         */
    bool finalize_switch_learning() noexcept
    {
        switch_learning_enabled_ = false;
        if (!config_valid_)
            return false;

        for (std::size_t i = 0u; i < N; ++i)
        {
            const RcChannelSpec &spec = cfg_.specs[i];
            if (spec.kind != ChannelKind::Switch || !spec.sw.auto_levels)
                continue;
            const std::uint16_t required_mask =
                static_cast<std::uint16_t>((static_cast<std::uint32_t>(1u) << spec.sw.count) - 1u);
            if ((sw_observed_mask_[i] & required_mask) != required_mask)
                return false;
            if (!switch_centroids_valid_(i, spec.sw.min_sep_us))
                return false;
        }
        return true;
    }

    /**
         * @brief Copy the current learned/frozen raw levels for one switch role.
         *
         * @param role Logical switch role.
         * @param out Caller-provided raw-level buffer.
         * @param capacity Number of elements available in @p out.
         * @return Number of copied levels; zero on invalid input.
         *
         * RCLink intentionally does not persist data itself. Applications may store
         * these values with any persistence layer and later feed them to raw_levels().
         */
    std::size_t copy_switch_levels(E role, std::int16_t *out, std::size_t capacity) const noexcept
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (!out || i >= N || cfg_.specs[i].kind != ChannelKind::Switch)
            return 0u;
        const std::size_t count = static_cast<std::size_t>(sw_centroid_count_[i]);
        if (capacity < count)
            return 0u;
        for (std::size_t k = 0u; k < count; ++k)
            out[k] = sw_centroid_[i][k];
        return count;
    }

    /**
         * @brief Poll the transport and refresh RCLink state using an explicit timestamp.
         *
         * @param now_ms Current millisecond timestamp.
         */
    void update(std::uint32_t now_ms)
    {
        const bool got_frame = detail::transport_update(tx_, now_ms, 0);

        if (got_frame)
        {
            ++status_.frame_sequence;
            status_.has_frame = true;
            status_.last_frame_ms = now_ms;
            status_.channel_count = sanitized_channel_count_();
            status_.channel_valid_mask = detail::transport_valid_mask(tx_, 0);
            status_.proto_failsafe = tx_.protoFailsafe();
            status_.frame_lost = tx_.frameLost();

            bool all_required_valid = true;
            for (std::size_t i = 0u; i < N; ++i)
            {
                const bool valid = source_role_valid_(i);
                role_source_valid_[i] = valid;
                if (cfg_.role_required[i] != 0u && !valid)
                    all_required_valid = false;
            }

            status_.required_roles_valid = config_valid_ && all_required_valid;
            status_.frame_valid = status_.required_roles_valid;

            for (std::size_t i = 0u; i < N; ++i)
            {
                const bool valid = role_source_valid_[i];

                if (!valid || !config_valid_)
                    continue;

                const int raw = tx_.readRaw(static_cast<int>(cfg_.role_to_channel[i]));
                scaled_.vals[i] = scale_channel_(i, raw);

                const float alpha = cfg_.axis_ema_alpha[i];
                if (alpha > 0.0f && cfg_.specs[i].kind == ChannelKind::Axis)
                {
                    const float previous = static_cast<float>(filtered_.vals[i]);
                    const float current = static_cast<float>(scaled_.vals[i]);
                    candidate_.vals[i] = round_int16_(previous + alpha * (current - previous));
                }
                else
                {
                    candidate_.vals[i] = scaled_.vals[i];
                }
            }

            advance_signature_on_frame_(now_ms);

            const bool acceptable_frame = status_.required_roles_valid && !status_.proto_failsafe &&
                                          !status_.frame_lost && !(rxfs_apply_outputs_ && fsig_matching_);
            if (acceptable_frame)
            {
                for (std::size_t i = 0u; i < N; ++i)
                {
                    if (!role_source_valid_[i])
                        continue;
                    filtered_.vals[i] = candidate_.vals[i];
                    last_good_.vals[i] = candidate_.vals[i];
                    observe_and_learn_switch_(i);
                }
            }
        }

        pull_transport_diagnostics_();

        status_.last_frame_age =
            status_.has_frame ? static_cast<std::uint32_t>(now_ms - status_.last_frame_ms) : kInvalidFrameAgeMs;
        status_.link_ok =
            status_.has_frame && status_.last_frame_age <= static_cast<std::uint32_t>(cfg_.link_timeout_ms);

        if (!status_.link_ok)
        {
            status_.rx_failsafe_sig = false;
            fsig_matching_ = false;
        }

        status_.healthy = config_valid_ && status_.link_ok && status_.required_roles_valid && !status_.proto_failsafe &&
                          !status_.frame_lost && !status_.rx_failsafe_sig;

        const bool global_failsafe = !config_valid_ || !status_.link_ok || !status_.required_roles_valid ||
                                     status_.proto_failsafe || status_.frame_lost ||
                                     (status_.rx_failsafe_sig && rxfs_apply_outputs_);

        if (global_failsafe)
        {
            for (std::size_t i = 0u; i < N; ++i)
                out_.vals[i] = config_valid_ ? apply_failsafe_(i) : 0;
        }
        else
        {
            for (std::size_t i = 0u; i < N; ++i)
            {
                // Optional missing roles fail individually while valid required roles remain usable.
                if (!role_source_valid_[i])
                {
                    out_.vals[i] = apply_failsafe_(i);
                    continue;
                }

                const std::int32_t previous = static_cast<std::int32_t>(out_.vals[i]);
                const std::int32_t current = static_cast<std::int32_t>(candidate_.vals[i]);
                const std::int32_t delta = current - previous;
                const std::uint32_t magnitude = static_cast<std::uint32_t>(delta < 0 ? -delta : delta);
                if (magnitude > static_cast<std::uint32_t>(cfg_.epsilon[i]))
                    out_.vals[i] = candidate_.vals[i];
            }
        }

        update_fps_(now_ms);
    }

#if defined(ARDUINO)
    /**
         * @brief Arduino convenience overload using millis().
         */
    void update()
    {
        update(static_cast<std::uint32_t>(millis()));
    }
#endif

    /// @brief Return the current safe application frame.
    RcFrame<N> frame() const noexcept
    {
        return out_;
    }

    /**
         * @brief Bounds-checked read by enum-like logical role.
         *
         * @param role Logical role.
         * @return Current safe output, or zero for an invalid role.
         */
    template <typename EnumLike> std::int16_t read(EnumLike role) const noexcept
    {
        return read_or(role, 0);
    }

    /**
         * @brief Bounds-checked read with caller-selected fallback.
         */
    template <typename EnumLike> std::int16_t read_or(EnumLike role, std::int16_t fallback) const noexcept
    {
        const std::size_t i = static_cast<std::size_t>(role);
        return (i < N) ? out_.vals[i] : fallback;
    }

    /**
         * @brief Bounds-checked read by numeric logical-role index.
         *
         * @param index Logical role index.
         * @return Current safe output, or zero when out of range.
         */
    std::int16_t read_by_index(std::size_t index) const noexcept
    {
        return read_by_index_or(index, 0);
    }

    /**
         * @brief Bounds-checked read by numeric index with fallback.
         */
    std::int16_t read_by_index_or(std::size_t index, std::int16_t fallback) const noexcept
    {
        return (index < N) ? out_.vals[index] : fallback;
    }

    /**
         * @brief Copy a safe output only when the logical role index is valid.
         *
         * @param role Logical role.
         * @param out Destination value.
         * @return True when @p role is within the configured enum range.
         */
    bool try_read(E role, std::int16_t &out) const noexcept
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
            return false;
        out = out_.vals[i];
        return true;
    }

    /**
         * @brief Report whether a role currently has fresh valid source data.
         *
         * @param role Logical role.
         * @return True only when the latest source data is valid and the link remains fresh.
         */
    bool role_valid(E role) const noexcept
    {
        const std::size_t i = static_cast<std::size_t>(role);
        return i < N && config_valid_ && status_.link_ok && role_source_valid_[i];
    }

    /// @brief Return unified link, validity, failsafe, and diagnostic status.
    const RcLinkStatus &status() const noexcept
    {
        return status_;
    }

    /// @brief Return the backward-compatible accepted-frame freshness state.
    bool ok() const noexcept
    {
        return status_.link_ok;
    }

    /// @brief Return full provider health including configuration and required roles.
    bool healthy() const noexcept
    {
        return status_.healthy;
    }

    /// @brief Return capability flags reported by the active transport.
    const RcTransportCaps &caps() const noexcept
    {
        return caps_;
    }

    /// @brief Return whether the active logical-role configuration passed validation.
    bool config_valid() const noexcept
    {
        return config_valid_;
    }

    /**
         * @brief Detect whether the exposed safe frame changed since the previous call.
         *
         * @return True when at least one role changed.
         */
    bool changed() noexcept
    {
        bool different = false;
        for (std::size_t i = 0u; i < N; ++i)
        {
            if (out_.vals[i] != last_.vals[i])
            {
                different = true;
                break;
            }
        }
        if (different)
            last_ = out_;
        return different;
    }

  private:
    /// @brief Clamp an integer to an inclusive range.
    static int clamp_int_(int value, int lo, int hi) noexcept
    {
        return (value < lo) ? lo : ((value > hi) ? hi : value);
    }

    /// @brief Clamp a normalized floating-point value to [-1,1].
    static float clamp_unit_(float value) noexcept
    {
        return (value < -1.0f) ? -1.0f : ((value > 1.0f) ? 1.0f : value);
    }

    /// @brief Apply the configured cubic-mix expo in normalized space.
    static float apply_expo_(float x, float expo) noexcept
    {
        if (expo <= 0.0001f)
            return x;
        if (expo >= 0.9999f)
            return x * x * x;
        return (1.0f - expo) * x + expo * (x * x * x);
    }

    /// @brief Round an in-range application value to the exposed int16 representation.
    static std::int16_t round_int16_(float value) noexcept
    {
        return static_cast<std::int16_t>(value >= 0.0f ? value + 0.5f : value - 0.5f);
    }

    /// @brief Bound the transport-reported channel count to RCLink capacity.
    std::uint8_t sanitized_channel_count_() const noexcept
    {
        int count = detail::transport_channels(tx_, 0);
        if (count < 0)
            count = 0;
        if (count > static_cast<int>(kRcMaxReceiverChannels))
            count = static_cast<int>(kRcMaxReceiverChannels);
        return static_cast<std::uint8_t>(count);
    }

    /// @brief Determine whether one mapped role has valid source data in the current frame.
    bool source_role_valid_(std::size_t index) const noexcept
    {
        if (!config_valid_ || index >= N || cfg_.role_mapped[index] == 0u)
            return false;

        const std::uint8_t channel = cfg_.role_to_channel[index];
        if (channel == kInvalidChannel || channel >= status_.channel_count)
            return false;
        if ((status_.channel_valid_mask & (1u << static_cast<unsigned>(channel))) == 0u)
            return false;

        const int raw = tx_.readRaw(static_cast<int>(channel));
        return raw > kNoSignalThresholdUs;
    }

    /// @brief Scale one valid raw role according to its configured channel kind.
    std::int16_t scale_channel_(std::size_t index, int raw)
    {
        const RcChannelSpec &spec = cfg_.specs[index];
        if (spec.kind == ChannelKind::Axis)
            return scale_axis_(index, raw);
        return scale_switch_(index, raw);
    }

    /// @brief Scale and shape one axis around its physical calibrated centre.
    std::int16_t scale_axis_(std::size_t index, int raw) const noexcept
    {
        const RcAxisSpec &axis = cfg_.specs[index].axis;
        const int lo = static_cast<int>(axis.raw_lo);
        const int hi = static_cast<int>(axis.raw_hi);
        const int center = static_cast<int>(axis.raw_center);
        const int deadband = static_cast<int>(axis.deadband_us);
        const int value = clamp_int_(raw, lo, hi);

        float normalized = 0.0f;
        const int lower_edge = center - deadband;
        const int upper_edge = center + deadband;

        if (value < lower_edge)
        {
            const int span = lower_edge - lo;
            normalized = (span > 0) ? -static_cast<float>(lower_edge - value) / static_cast<float>(span) : 0.0f;
        }
        else if (value > upper_edge)
        {
            const int span = hi - upper_edge;
            normalized = (span > 0) ? static_cast<float>(value - upper_edge) / static_cast<float>(span) : 0.0f;
        }

        normalized = clamp_unit_(normalized);
        if (axis.invert)
            normalized = -normalized;
        normalized = apply_expo_(normalized, axis.expo);

        const float center_out = axis_center_out_[index];
        const float output = (normalized < 0.0f) ? center_out + normalized * (center_out - axis.out_lo)
                                                 : center_out + normalized * (axis.out_hi - center_out);
        return round_int16_(output);
    }

    /// @brief Snap one switch input using explicit, learned, or evenly spaced levels.
    std::int16_t scale_switch_(std::size_t index, int raw)
    {
        const RcChannelSpec &spec = cfg_.specs[index];
        const RcSwitchSpec &sw = spec.sw;
        if (sw.count == 0u)
            return 0;

        const int lo = static_cast<int>(spec.axis.raw_lo);
        const int hi = static_cast<int>(spec.axis.raw_hi);
        const int value = clamp_int_(raw, lo, hi);

        if (sw.raw_count >= 2u)
        {
            std::uint8_t nearest = 0u;
            std::int32_t best = 0x7FFFFFFF;
            for (std::uint8_t i = 0u; i < sw.raw_count; ++i)
            {
                const std::int32_t delta =
                    static_cast<std::int32_t>(value) - static_cast<std::int32_t>(sw.raw_levels[i]);
                const std::int32_t magnitude = (delta < 0) ? -delta : delta;
                if (magnitude < best)
                {
                    best = magnitude;
                    nearest = i;
                }
            }
            return round_int16_(sw.vals[nearest]);
        }

        if (sw.auto_levels)
        {
            if (sw_centroid_count_[index] == 0u)
                seed_switch_centroids_(index, lo, hi, sw.count);

            const std::uint8_t nearest = nearest_centroid_(index, value);
            const int active = static_cast<int>(sw_active_idx_[index]);
            const int hysteresis_half = static_cast<int>(sw.hyst_us / 2u);

            if (active < 0)
            {
                sw_active_idx_[index] = static_cast<std::int8_t>(nearest);
            }
            else
            {
                const int current_center = static_cast<int>(sw_centroid_[index][static_cast<std::size_t>(active)]);
                const int current_delta = value - current_center;
                const int current_distance = (current_delta < 0) ? -current_delta : current_delta;
                if (current_distance > hysteresis_half)
                {
                    const int next_center = static_cast<int>(sw_centroid_[index][nearest]);
                    const int next_delta = value - next_center;
                    const int next_distance = (next_delta < 0) ? -next_delta : next_delta;
                    if (next_distance + 2 <= current_distance)
                        sw_active_idx_[index] = static_cast<std::int8_t>(nearest);
                }
            }

            const std::uint8_t output_index =
                (sw_active_idx_[index] >= 0) ? static_cast<std::uint8_t>(sw_active_idx_[index]) : nearest;
            return round_int16_(sw.vals[output_index]);
        }

        // Without explicit raw levels, distribute switch positions evenly
        // across the calibrated raw range. Application output values must not
        // influence where the physical switch changes position.
        std::int32_t best_distance = 0x7FFFFFFF;
        std::uint8_t nearest = 0u;
        for (std::uint8_t i = 0u; i < sw.count; ++i)
        {
            const float fraction = (sw.count > 1u) ? static_cast<float>(i) / static_cast<float>(sw.count - 1u) : 0.0f;
            const int target =
                static_cast<int>(round_int16_(static_cast<float>(lo) + fraction * static_cast<float>(hi - lo)));
            const std::int32_t delta = static_cast<std::int32_t>(value) - static_cast<std::int32_t>(target);
            const std::int32_t distance = (delta < 0) ? -delta : delta;
            if (distance < best_distance)
            {
                best_distance = distance;
                nearest = i;
            }
        }
        return round_int16_(sw.vals[nearest]);
    }

    /// @brief Resolve the configured failsafe output for one role.
    std::int16_t apply_failsafe_(std::size_t index) const noexcept
    {
        const RcChannelSpec &spec = cfg_.specs[index];
        const Failsafe &failsafe = cfg_.fs[index];
        switch (failsafe.mode)
        {
        case Failsafe::Mode::Value:
            return failsafe.value;
        case Failsafe::Mode::HoldLast:
            return last_good_.vals[index];
        case Failsafe::Mode::ClampToOutLo:
            return (spec.kind == ChannelKind::Axis) ? round_int16_(spec.axis.out_lo) : filtered_.vals[index];
        case Failsafe::Mode::ClampToOutHi:
            return (spec.kind == ChannelKind::Axis) ? round_int16_(spec.axis.out_hi) : filtered_.vals[index];
        default:
            return 0;
        }
    }

    /// @brief Advance receiver-signature confirmation using one accepted frame.
    void advance_signature_on_frame_(std::uint32_t now_ms) noexcept
    {
        if (!fsig_set_ || !config_valid_ || !status_.required_roles_valid || status_.proto_failsafe ||
            status_.frame_lost)
        {
            reset_signature_tracking_();
            return;
        }

        if (!signature_roles_valid_() || !close_to_failsafe_())
        {
            reset_signature_tracking_();
            return;
        }

        if (!fsig_matching_)
        {
            fsig_matching_ = true;
            fsig_first_ms_ = now_ms;
            status_.rx_failsafe_sig = (fsig_.hold_ms == 0u);
            return;
        }

        if (static_cast<std::uint32_t>(now_ms - fsig_first_ms_) >= static_cast<std::uint32_t>(fsig_.hold_ms))
            status_.rx_failsafe_sig = true;
    }

    /// @brief Check that every role selected by the signature has valid source data.
    bool signature_roles_valid_() const noexcept
    {
        for (std::size_t i = 0u; i < N; ++i)
        {
            if (fsig_.check[i] != 0u && !role_source_valid_[i])
                return false;
        }
        return true;
    }

    /// @brief Compare selected scaled values with the configured receiver signature.
    bool close_to_failsafe_() const noexcept
    {
        for (std::size_t i = 0u; i < N; ++i)
        {
            if (fsig_.check[i] == 0u)
                continue;
            const std::int32_t delta =
                static_cast<std::int32_t>(scaled_.vals[i]) - static_cast<std::int32_t>(fsig_.expected[i]);
            const std::uint32_t magnitude = static_cast<std::uint32_t>(delta < 0 ? -delta : delta);
            if (magnitude > static_cast<std::uint32_t>(fsig_.tol))
                return false;
        }
        return true;
    }

    /// @brief Clear receiver-signature candidate and confirmation state.
    void reset_signature_tracking_() noexcept
    {
        fsig_matching_ = false;
        fsig_first_ms_ = 0u;
        status_.rx_failsafe_sig = false;
    }

    /// @brief Record and adapt the selected centroid during an explicit learning session.
    void observe_and_learn_switch_(std::size_t index) noexcept
    {
        const RcChannelSpec &spec = cfg_.specs[index];
        if (!switch_learning_enabled_ || spec.kind != ChannelKind::Switch || !spec.sw.auto_levels)
            return;

        const int selected = static_cast<int>(sw_active_idx_[index]);
        if (selected < 0 || selected >= static_cast<int>(sw_centroid_count_[index]))
            return;

        const std::uint8_t channel = cfg_.role_to_channel[index];
        const int raw = clamp_int_(tx_.readRaw(static_cast<int>(channel)), static_cast<int>(spec.axis.raw_lo),
                                   static_cast<int>(spec.axis.raw_hi));
        const std::uint8_t centroid = static_cast<std::uint8_t>(selected);
        sw_observed_mask_[index] =
            static_cast<std::uint16_t>(sw_observed_mask_[index] | static_cast<std::uint16_t>(1u << centroid));
        learn_centroid_(index, centroid, raw, spec.sw.learn_alpha, spec.sw.min_sep_us);
    }

    /// @brief Seed evenly spaced switch centroids across a calibrated raw range.
    void seed_switch_centroids_(std::size_t index, int lo, int hi, std::uint8_t count) noexcept
    {
        if (count == 0u)
        {
            sw_centroid_count_[index] = 0u;
            sw_active_idx_[index] = -1;
            return;
        }

        if (hi < lo)
        {
            const int swap = lo;
            lo = hi;
            hi = swap;
        }

        const float step = (count > 1u) ? static_cast<float>(hi - lo) / static_cast<float>(count - 1u) : 0.0f;
        for (std::uint8_t k = 0u; k < count; ++k)
            sw_centroid_[index][k] = round_int16_(static_cast<float>(lo) + step * static_cast<float>(k));
        sw_centroid_count_[index] = count;
        sw_active_idx_[index] = -1;
    }

    /// @brief Find the closest configured centroid to a raw switch value.
    std::uint8_t nearest_centroid_(std::size_t index, int raw) const noexcept
    {
        std::uint8_t nearest = 0u;
        std::int32_t best = 0x7FFFFFFF;
        const std::uint8_t count = sw_centroid_count_[index];
        for (std::uint8_t k = 0u; k < count; ++k)
        {
            const std::int32_t delta =
                static_cast<std::int32_t>(raw) - static_cast<std::int32_t>(sw_centroid_[index][k]);
            const std::int32_t magnitude = (delta < 0) ? -delta : delta;
            if (magnitude < best)
            {
                best = magnitude;
                nearest = k;
            }
        }
        return nearest;
    }

    /// @brief Adapt one centroid while preserving configured neighbour separation.
    void learn_centroid_(std::size_t index, std::uint8_t centroid, int raw, float alpha,
                         std::uint16_t min_sep_us) noexcept
    {
        if (alpha <= 0.0f || centroid >= sw_centroid_count_[index])
            return;
        if (alpha > 1.0f)
            alpha = 1.0f;

        const float current = static_cast<float>(sw_centroid_[index][centroid]);
        int candidate = static_cast<int>(round_int16_(current + alpha * (static_cast<float>(raw) - current)));

        if (centroid > 0u)
        {
            const int minimum = static_cast<int>(sw_centroid_[index][centroid - 1u]) + static_cast<int>(min_sep_us);
            if (candidate < minimum)
                candidate = minimum;
        }
        if (static_cast<std::size_t>(centroid + 1u) < static_cast<std::size_t>(sw_centroid_count_[index]))
        {
            const int maximum = static_cast<int>(sw_centroid_[index][centroid + 1u]) - static_cast<int>(min_sep_us);
            if (candidate > maximum)
                candidate = maximum;
        }

        sw_centroid_[index][centroid] = static_cast<std::int16_t>(candidate);
    }

    /// @brief Validate monotonic spacing for one role's learned centroids.
    bool switch_centroids_valid_(std::size_t index, std::uint16_t min_sep_us) const noexcept
    {
        const std::uint8_t count = sw_centroid_count_[index];
        if (count < 2u)
            return false;
        for (std::uint8_t k = 1u; k < count; ++k)
        {
            const std::int32_t delta = static_cast<std::int32_t>(sw_centroid_[index][k]) -
                                       static_cast<std::int32_t>(sw_centroid_[index][k - 1u]);
            if (delta < static_cast<std::int32_t>(min_sep_us))
                return false;
        }
        return true;
    }

    /// @brief Reset frame-derived state after applying a configuration.
    void reset_runtime_for_config_() noexcept
    {
        switch_learning_enabled_ = false;
        reset_signature_tracking_();
        for (std::size_t i = 0u; i < N; ++i)
        {
            role_source_valid_[i] = false;
            sw_centroid_count_[i] = 0u;
            sw_active_idx_[i] = -1;
            sw_observed_mask_[i] = 0u;
            axis_center_out_[i] = 0.0f;
        }
        status_.has_frame = false;
        status_.link_ok = false;
        status_.required_roles_valid = false;
        status_.frame_valid = false;
        status_.healthy = false;
        status_.proto_failsafe = false;
        status_.frame_lost = false;
        status_.channel_count = 0u;
        status_.channel_valid_mask = 0u;
        status_.last_frame_ms = 0u;
        status_.last_frame_age = kInvalidFrameAgeMs;
        status_.fps = 0u;
        pull_transport_diagnostics_();
        fps_last_count_ = status_.frame_sequence;
        fps_last_t_ = 0u;
    }

    /// @brief Clear every working and exposed application frame.
    void zero_outputs_() noexcept
    {
        for (std::size_t i = 0u; i < N; ++i)
        {
            scaled_.vals[i] = 0;
            candidate_.vals[i] = 0;
            filtered_.vals[i] = 0;
            last_good_.vals[i] = 0;
            out_.vals[i] = 0;
            last_.vals[i] = 0;
        }
    }

    /// @brief Copy optional parser and transport counters into public diagnostics.
    void pull_transport_diagnostics_() noexcept
    {
        const std::uint32_t frames = detail::transport_frames(tx_, 0);
        if (frames != 0u || status_.has_frame)
            status_.frames = frames;
        status_.crc_errors = detail::transport_crc_errors(tx_, 0);
        status_.parse_errors = detail::transport_parse_errors(tx_, 0);
        status_.parser_timeouts = detail::transport_parser_timeouts(tx_, 0);
        status_.discarded_bytes = detail::transport_discarded_bytes(tx_, 0);
    }

    /// @brief Update the rolling accepted-frame rate at a bounded cadence.
    void update_fps_(std::uint32_t now_ms) noexcept
    {
        const std::uint32_t elapsed = static_cast<std::uint32_t>(now_ms - fps_last_t_);
        if (elapsed < 500u)
            return;

        const std::uint32_t delta_frames = detail::sequence_delta(status_.frame_sequence, fps_last_count_);
        const std::uint32_t rate = (elapsed != 0u) ? (delta_frames * 1000u) / elapsed : 0u;
        status_.fps = static_cast<std::uint16_t>((rate > 65535u) ? 65535u : rate);
        fps_last_count_ = status_.frame_sequence;
        fps_last_t_ = now_ms;
    }

    Transport &tx_;            ///< Receiver transport (not owned).
    RcTransportCaps caps_{};   ///< Transport capability flags.
    RcConfig<E> cfg_{};        ///< Active logical-role configuration.
    bool config_valid_{false}; ///< Cached configuration validity.

    RcFrame<N> scaled_{};         ///< Latest valid role values before filtering.
    RcFrame<N> candidate_{};      ///< Latest filtered candidate values, including failsafe candidates.
    RcFrame<N> filtered_{};       ///< Filter history advanced only by acceptable application frames.
    RcFrame<N> last_good_{};      ///< Last acceptable application values used by HoldLast.
    RcFrame<N> out_{};            ///< Safe/exposed role values.
    RcFrame<N> last_{};           ///< Previous exposed frame for changed().
    bool role_source_valid_[N]{}; ///< Per-role validity in the latest accepted transport frame.
    float axis_center_out_[N]{};  ///< Pre-resolved application-space centre outputs.

    RcLinkStatus status_{};            ///< Unified link and decoder status.
    RcFailsafeRule<N> fsig_{};         ///< Receiver-failsafe signature.
    bool fsig_set_{false};             ///< Signature configured flag.
    bool fsig_matching_{false};        ///< Explicit signature candidate state; timestamp zero is valid.
    std::uint32_t fsig_first_ms_{0u};  ///< First matching-frame timestamp.
    bool rxfs_apply_outputs_{false};   ///< Whether signature confirmation applies output failsafe.
    std::uint32_t fps_last_count_{0u}; ///< Previous frame counter for FPS estimation.
    std::uint32_t fps_last_t_{0u};     ///< Previous FPS sampling timestamp.

    std::int16_t sw_centroid_[N][kRcMaxSwitchVals]{}; ///< Learned/frozen switch centroids.
    std::uint8_t sw_centroid_count_[N]{};             ///< Centroid count per logical role.
    std::int8_t sw_active_idx_[N]{};                  ///< Current snapped centroid index.
    std::uint16_t sw_observed_mask_[N]{};             ///< Positions observed during the active learning session.
    bool switch_learning_enabled_{false};             ///< Explicit commissioning-only learning gate.
};
} // namespace rc
