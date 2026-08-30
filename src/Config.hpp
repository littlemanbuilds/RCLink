/**
 * MIT License
 *
 * @brief Fluent, bounds-checked configuration builders for RCLink.
 *
 * @file Config.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "Constants.hpp"
#include "Types.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace rc
{
namespace detail
{
static constexpr int kInt16MinValue = -32768; ///< Lowest output representable by RCLink.
static constexpr int kInt16MaxValue = 32767;  ///< Highest output representable by RCLink.

/// @brief Return whether a configuration value is finite.
inline bool finite_float(float value) noexcept
{
    return std::isfinite(value) != 0;
}

/// @brief Return whether a finite value fits RCLink's exposed int16 output.
inline bool fits_int16(float value) noexcept
{
    return finite_float(value) && value >= static_cast<float>(kInt16MinValue) &&
           value <= static_cast<float>(kInt16MaxValue);
}

/// @brief Resolve an axis centre output while preserving endpoint-centred compatibility.
inline float inferred_axis_center_output(const RcAxisSpec &axis) noexcept
{
    if (axis.has_out_center)
        return axis.out_center;
    if (axis.raw_center <= axis.raw_lo)
        return axis.out_lo;
    if (axis.raw_center >= axis.raw_hi)
        return axis.out_hi;
    return 0.5f * (axis.out_lo + axis.out_hi);
}
} // namespace detail

/**
     * @brief Configuration for an RCLink role enum ending in `Count`.
     *
     * All roles are required by default. Call @ref optional() for a role that may
     * legitimately be absent from a receiver frame. Unmapped required roles fail
     * validation instead of silently reading receiver channel zero.
     *
     * @tparam E Logical role enum.
     */
template <typename E> struct RcConfig
{
    static constexpr std::size_t N = static_cast<std::size_t>(E::Count); ///< Logical role count.

    std::uint8_t role_to_channel[N];    ///< Logical role to receiver channel; defaults to @ref kInvalidChannel.
    std::uint8_t role_mapped[N]{};      ///< Non-zero only after an explicit mapping.
    std::uint8_t role_required[N];      ///< Non-zero when role validity contributes to provider health.
    RcChannelSpec specs[N]{};           ///< Per-role channel specification.
    Failsafe fs[N]{};                   ///< Per-role failsafe policy.
    std::int16_t epsilon[N]{};          ///< Minimum exposed delta in scaled output units.
    float axis_ema_alpha[N]{};          ///< Per-axis EMA factor; zero disables filtering.
    std::uint16_t link_timeout_ms{200}; ///< Maximum accepted frame age.

    RcConfigError build_error{RcConfigError::None};    ///< First builder misuse detected before activation.
    std::size_t build_error_role{0};                   ///< Role index associated with @ref build_error.
    std::uint8_t build_error_channel{kInvalidChannel}; ///< Channel associated with @ref build_error.

    /**
         * @brief Create a fail-closed configuration with every role initially unmapped and required.
         */
    RcConfig()
    {
        for (std::size_t i = 0; i < N; ++i)
        {
            role_to_channel[i] = kInvalidChannel;
            role_mapped[i] = 0u;
            role_required[i] = 1u;
        }
    }

    /**
         * @brief Map a logical role to a receiver channel.
         *
         * @param role Logical role to assign.
         * @param rx Zero-based receiver channel index.
         * @return This configuration for fluent chaining.
         */
    RcConfig &map(E role, std::uint8_t rx)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, rx);
            return *this;
        }
        if (rx == kInvalidChannel || static_cast<std::size_t>(rx) >= kRcMaxReceiverChannels)
        {
            note_error_(RcConfigError::InvalidChannel, i, rx);
            return *this;
        }

        role_to_channel[i] = rx;
        role_mapped[i] = 1u;
        return *this;
    }

    /**
         * @brief Mark whether a role is required for a healthy frame.
         *
         * @param role Logical role to update.
         * @param required True when missing/invalid data must fail provider health.
         * @return This configuration for fluent chaining.
         */
    RcConfig &require(E role, bool required = true)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, kInvalidChannel);
            return *this;
        }
        role_required[i] = required ? 1u : 0u;
        return *this;
    }

    /**
         * @brief Mark a role as optional.
         *
         * @param role Logical role that may be absent without invalidating required-role health.
         * @return This configuration for fluent chaining.
         */
    RcConfig &optional(E role)
    {
        return require(role, false);
    }

    /**
         * @brief Fluent axis builder.
         */
    struct AxisB
    {
        RcChannelSpec *s; ///< Selected role specification, or nullptr for an invalid role.
        RcConfig *cfg;    ///< Parent configuration.
        std::size_t idx;  ///< Selected logical role index.

        /// @brief Construct a bounds-aware axis builder for one role.
        AxisB(RcChannelSpec *spec, RcConfig *parent, std::size_t role_index) noexcept
            : s(spec), cfg(parent), idx(role_index)
        {
        }

        /**
             * @brief Configure calibrated raw endpoints and physical neutral.
             *
             * @param lo Lower receiver value in microseconds.
             * @param hi Upper receiver value in microseconds.
             * @param center Neutral receiver value in microseconds.
             * @return This builder.
             */
        AxisB &raw(std::int16_t lo, std::int16_t hi, std::int16_t center)
        {
            if (!s)
                return *this;
            s->kind = ChannelKind::Axis;
            s->axis.raw_lo = lo;
            s->axis.raw_hi = hi;
            s->axis.raw_center = center;
            return *this;
        }

        /**
             * @brief Configure a deadband around the calibrated centre.
             *
             * @param d Deadband half-width in microseconds; negative values become zero.
             * @return This builder.
             */
        AxisB &deadband_us(std::int16_t d)
        {
            if (s)
                s->axis.deadband_us = (d < 0) ? 0 : d;
            return *this;
        }

        /**
             * @brief Configure the low and high application-space outputs.
             *
             * Centred axes require ascending outputs and use @ref invert() to
             * reverse direction. Endpoint-centred one-sided axes may reverse
             * these endpoints directly instead.
             *
             * @param lo Output at raw_lo.
             * @param hi Output at raw_hi.
             * @return This builder.
             */
        AxisB &out(float lo, float hi)
        {
            if (!s)
                return *this;
            if (!detail::finite_float(lo) || !detail::finite_float(hi))
            {
                cfg->note_error_(RcConfigError::NonFiniteValue, idx, kInvalidChannel);
                return *this;
            }
            s->axis.out_lo = lo;
            s->axis.out_hi = hi;
            return *this;
        }

        /**
             * @brief Explicitly configure the output corresponding to raw_center.
             *
             * When omitted, RCLink uses the output midpoint for an internal centre,
             * out_lo for raw_center == raw_lo, and out_hi for raw_center == raw_hi.
             *
             * @param value Application-space neutral value.
             * @return This builder.
             */
        AxisB &center_out(float value)
        {
            if (!s)
                return *this;
            if (!detail::finite_float(value))
            {
                cfg->note_error_(RcConfigError::NonFiniteValue, idx, kInvalidChannel);
                return *this;
            }
            s->axis.out_center = value;
            s->axis.has_out_center = true;
            return *this;
        }

        /**
             * @brief Configure the cubic-mix expo factor.
             *
             * @param e Expo factor in [0,1].
             * @return This builder.
             */
        AxisB &expo(float e)
        {
            if (!s)
                return *this;
            if (!detail::finite_float(e))
            {
                cfg->note_error_(RcConfigError::InvalidExpo, idx, kInvalidChannel);
                return *this;
            }
            if (e < 0.0f)
                e = 0.0f;
            if (e > 1.0f)
                e = 1.0f;
            s->axis.expo = e;
            return *this;
        }

        /**
             * @brief Reverse the normalized axis direction.
             *
             * @param inv True to invert the axis.
             * @return This builder.
             */
        AxisB &invert(bool inv = true)
        {
            if (s)
                s->axis.invert = inv;
            return *this;
        }

        /**
             * @brief Configure the role failsafe policy.
             *
             * @param m Failsafe mode.
             * @param v Value used by Failsafe::Mode::Value.
             * @return This builder.
             */
        AxisB &failsafe(Failsafe::Mode m, std::int16_t v = 0)
        {
            if (cfg && s)
            {
                cfg->fs[idx].mode = m;
                cfg->fs[idx].value = v;
            }
            return *this;
        }

        /**
             * @brief Finish the role builder.
             *
             * @return Parent configuration.
             */
        RcConfig &done()
        {
            return *cfg;
        }
    };

    /**
         * @brief Configure a logical role as an axis.
         *
         * @param role Logical role.
         * @return Bounds-checked axis builder.
         */
    AxisB axis(E role)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, kInvalidChannel);
            return AxisB{nullptr, this, i};
        }
        specs[i].kind = ChannelKind::Axis;
        return AxisB{&specs[i], this, i};
    }

    /**
         * @brief Fluent switch builder.
         */
    struct SwitchB
    {
        RcChannelSpec *s; ///< Selected role specification, or nullptr for an invalid role.
        RcConfig *cfg;    ///< Parent configuration.
        std::size_t idx;  ///< Selected logical role index.

        /// @brief Construct a bounds-aware switch builder for one role.
        SwitchB(RcChannelSpec *spec, RcConfig *parent, std::size_t role_index) noexcept
            : s(spec), cfg(parent), idx(role_index)
        {
        }

        /**
             * @brief Configure logical outputs for each switch position.
             *
             * @param v Ordered output values.
             * @return This builder.
             */
        SwitchB &values(std::initializer_list<float> v)
        {
            if (!s)
                return *this;
            if (v.size() > kRcMaxSwitchVals)
                cfg->note_error_(RcConfigError::InvalidSwitchCount, idx, kInvalidChannel);
            s->kind = ChannelKind::Switch;
            s->sw.count = static_cast<std::uint8_t>((v.size() > kRcMaxSwitchVals) ? kRcMaxSwitchVals : v.size());
            std::uint8_t i = 0u;
            for (float value : v)
            {
                if (i >= s->sw.count)
                    break;
                if (!detail::finite_float(value))
                {
                    cfg->note_error_(RcConfigError::NonFiniteValue, idx, kInvalidChannel);
                    value = 0.0f;
                }
                s->sw.vals[i++] = value;
            }
            return *this;
        }

        /**
             * @brief Configure deterministic raw snap levels and disable learning.
             *
             * @param lvls Raw receiver values in the same order as @ref values().
             * @return This builder.
             */
        SwitchB &raw_levels(std::initializer_list<std::int16_t> lvls)
        {
            if (!s)
                return *this;
            if (lvls.size() > kRcMaxSwitchVals)
                cfg->note_error_(RcConfigError::InvalidSwitchLevels, idx, kInvalidChannel);
            s->kind = ChannelKind::Switch;
            s->sw.raw_count =
                static_cast<std::uint8_t>((lvls.size() > kRcMaxSwitchVals) ? kRcMaxSwitchVals : lvls.size());
            std::uint8_t i = 0u;
            for (std::int16_t value : lvls)
            {
                if (i >= s->sw.raw_count)
                    break;
                s->sw.raw_levels[i++] = value;
            }
            s->sw.auto_levels = false;
            return *this;
        }

        /**
             * @brief Mark a switch as eligible for explicit commissioning-time learning.
             *
             * Learning does not run merely because this flag is true. The RcLink
             * instance must also be placed into switch-learning mode.
             *
             * @param enable True to permit learning.
             * @return This builder.
             */
        SwitchB &auto_levels(bool enable = true)
        {
            if (s)
            {
                s->kind = ChannelKind::Switch;
                s->sw.auto_levels = enable;
            }
            return *this;
        }

        /**
             * @brief Configure switch snap hysteresis.
             *
             * @param us Total hysteresis band in microseconds.
             * @return This builder.
             */
        SwitchB &hysteresis_us(std::uint16_t us)
        {
            if (s)
            {
                s->kind = ChannelKind::Switch;
                s->sw.hyst_us = us;
            }
            return *this;
        }

        /**
             * @brief Configure commissioning-time centroid learning rate.
             *
             * @param a EMA alpha in (0,1].
             * @return This builder.
             */
        SwitchB &learn_alpha(float a)
        {
            if (!s)
                return *this;
            if (!detail::finite_float(a))
            {
                cfg->note_error_(RcConfigError::InvalidSwitchLearning, idx, kInvalidChannel);
                return *this;
            }
            if (a < 0.0f)
                a = 0.0f;
            if (a > 1.0f)
                a = 1.0f;
            s->kind = ChannelKind::Switch;
            s->sw.learn_alpha = a;
            return *this;
        }

        /**
             * @brief Configure minimum spacing between learned switch centroids.
             *
             * @param us Minimum separation in microseconds.
             * @return This builder.
             */
        SwitchB &min_sep_us(std::uint16_t us)
        {
            if (s)
            {
                s->kind = ChannelKind::Switch;
                s->sw.min_sep_us = us;
            }
            return *this;
        }

        /**
             * @brief Configure the role failsafe policy.
             *
             * @param m Failsafe mode.
             * @param v Value used by Failsafe::Mode::Value.
             * @return This builder.
             */
        SwitchB &failsafe(Failsafe::Mode m, std::int16_t v = 0)
        {
            if (cfg && s)
            {
                cfg->fs[idx].mode = m;
                cfg->fs[idx].value = v;
            }
            return *this;
        }

        /**
             * @brief Finish the role builder.
             *
             * @return Parent configuration.
             */
        RcConfig &done()
        {
            return *cfg;
        }
    };

    /**
         * @brief Configure a logical role as a switch.
         *
         * @param role Logical role.
         * @return Bounds-checked switch builder.
         */
    SwitchB sw(E role)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, kInvalidChannel);
            return SwitchB{nullptr, this, i};
        }
        specs[i].kind = ChannelKind::Switch;
        return SwitchB{&specs[i], this, i};
    }

    /**
         * @brief Set the receiver freshness timeout.
         *
         * @param ms Maximum frame age in milliseconds; zero is invalid.
         * @return This configuration for fluent chaining.
         */
    RcConfig &setLinkTimeout(std::uint16_t ms)
    {
        if (ms == 0u)
            note_error_(RcConfigError::InvalidLinkTimeout, 0u, kInvalidChannel);
        else
            link_timeout_ms = ms;
        return *this;
    }

    /**
         * @brief Set output epsilon for a logical role.
         *
         * @param role Logical role.
         * @param e Suppression threshold; negative values become zero.
         * @return This configuration for fluent chaining.
         */
    RcConfig &setEpsilon(E role, std::int16_t e)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, kInvalidChannel);
            return *this;
        }
        epsilon[i] = (e < 0) ? 0 : e;
        return *this;
    }

    /**
         * @brief Set per-axis EMA filtering.
         *
         * @param role Logical role.
         * @param alpha Smoothing factor in [0,1].
         * @return This configuration for fluent chaining.
         */
    RcConfig &setAxisFilter(E role, float alpha)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, kInvalidChannel);
            return *this;
        }
        if (!detail::finite_float(alpha))
        {
            note_error_(RcConfigError::InvalidFilter, i, kInvalidChannel);
            return *this;
        }
        if (alpha < 0.0f)
            alpha = 0.0f;
        if (alpha > 1.0f)
            alpha = 1.0f;
        axis_ema_alpha[i] = alpha;
        return *this;
    }

    /**
         * @brief Set a per-role failsafe policy.
         *
         * @param role Logical role.
         * @param mode Failsafe mode.
         * @param v Value used by Failsafe::Mode::Value.
         * @return This configuration for fluent chaining.
         */
    RcConfig &setFailsafePolicy(E role, Failsafe::Mode mode, std::int16_t v = 0)
    {
        const std::size_t i = static_cast<std::size_t>(role);
        if (i >= N)
        {
            note_error_(RcConfigError::InvalidRole, i, kInvalidChannel);
            return *this;
        }
        fs[i].mode = mode;
        fs[i].value = v;
        return *this;
    }

  private:
    /// @brief Preserve the first fluent-builder error for later activation validation.
    void note_error_(RcConfigError error, std::size_t role, std::uint8_t channel)
    {
        if (build_error == RcConfigError::None)
        {
            build_error = error;
            build_error_role = role;
            build_error_channel = channel;
        }
    }
};

/**
     * @brief Resolve the application-space output at an axis physical centre.
     *
     * @param axis Axis specification.
     * @return Explicit or compatibility-inferred centre output.
     */
inline float axis_center_output(const RcAxisSpec &axis) noexcept
{
    return detail::inferred_axis_center_output(axis);
}

/**
     * @brief Validate a complete logical-role configuration before it becomes active.
     *
     * @tparam E Logical role enum.
     * @param cfg Configuration to validate.
     * @return Detailed activation result.
     */
template <typename E> RcConfigResult validate_config(const RcConfig<E> &cfg)
{
    typedef RcConfig<E> Config;
    const std::size_t n = Config::N;

    if (cfg.build_error != RcConfigError::None)
        return RcConfigResult(false, cfg.build_error, cfg.build_error_role, cfg.build_error_channel);
    if (cfg.link_timeout_ms == 0u)
        return RcConfigResult(false, RcConfigError::InvalidLinkTimeout, 0u, kInvalidChannel);

    for (std::size_t i = 0; i < n; ++i)
    {
        if (cfg.role_required[i] != 0u && cfg.role_mapped[i] == 0u)
            return RcConfigResult(false, RcConfigError::UnmappedRequiredRole, i, kInvalidChannel);

        if (cfg.role_mapped[i] != 0u)
        {
            const std::uint8_t channel = cfg.role_to_channel[i];
            if (channel == kInvalidChannel || static_cast<std::size_t>(channel) >= kRcMaxReceiverChannels)
                return RcConfigResult(false, RcConfigError::InvalidChannel, i, channel);
        }

        const RcChannelSpec &spec = cfg.specs[i];
        if (spec.kind == ChannelKind::Axis)
        {
            const RcAxisSpec &axis = spec.axis;
            if (axis.raw_lo >= axis.raw_hi)
                return RcConfigResult(false, RcConfigError::InvalidAxisRange, i, cfg.role_to_channel[i]);
            if (axis.raw_center < axis.raw_lo || axis.raw_center > axis.raw_hi)
                return RcConfigResult(false, RcConfigError::InvalidAxisCenter, i, cfg.role_to_channel[i]);
            if (axis.invert && (axis.raw_center == axis.raw_lo || axis.raw_center == axis.raw_hi))
                return RcConfigResult(false, RcConfigError::EndpointCenteredInversion, i, cfg.role_to_channel[i]);

            const int left_span = static_cast<int>(axis.raw_center) - static_cast<int>(axis.raw_lo);
            const int right_span = static_cast<int>(axis.raw_hi) - static_cast<int>(axis.raw_center);
            if ((left_span > 0 && left_span < kRcMinAxisSideSpanUs) ||
                (right_span > 0 && right_span < kRcMinAxisSideSpanUs) || (left_span == 0 && right_span == 0))
                return RcConfigResult(false, RcConfigError::AxisSpanTooSmall, i, cfg.role_to_channel[i]);

            const int deadband = static_cast<int>(axis.deadband_us);
            if (deadband < 0 || (left_span > 0 && deadband >= left_span) || (right_span > 0 && deadband >= right_span))
                return RcConfigResult(false, RcConfigError::InvalidDeadband, i, cfg.role_to_channel[i]);

            const float center_out = axis_center_output(axis);
            if (!detail::fits_int16(axis.out_lo) || !detail::fits_int16(axis.out_hi) || !detail::fits_int16(center_out))
                return RcConfigResult(false, RcConfigError::NonFiniteValue, i, cfg.role_to_channel[i]);
            const bool endpoint_centered = axis.raw_center == axis.raw_lo || axis.raw_center == axis.raw_hi;
            const float output_min = (axis.out_lo < axis.out_hi) ? axis.out_lo : axis.out_hi;
            const float output_max = (axis.out_lo < axis.out_hi) ? axis.out_hi : axis.out_lo;
            if (axis.out_lo == axis.out_hi || (!endpoint_centered && axis.out_lo > axis.out_hi) ||
                center_out < output_min || center_out > output_max)
                return RcConfigResult(false, RcConfigError::InvalidOutputRange, i, cfg.role_to_channel[i]);
            if (!detail::finite_float(axis.expo) || axis.expo < 0.0f || axis.expo > 1.0f)
                return RcConfigResult(false, RcConfigError::InvalidExpo, i, cfg.role_to_channel[i]);
            if (!detail::finite_float(cfg.axis_ema_alpha[i]) || cfg.axis_ema_alpha[i] < 0.0f ||
                cfg.axis_ema_alpha[i] > 1.0f)
                return RcConfigResult(false, RcConfigError::InvalidFilter, i, cfg.role_to_channel[i]);
        }
        else if (spec.kind == ChannelKind::Switch)
        {
            const RcSwitchSpec &sw = spec.sw;
            if (sw.count < 2u || sw.count > kRcMaxSwitchVals)
                return RcConfigResult(false, RcConfigError::InvalidSwitchCount, i, cfg.role_to_channel[i]);

            for (std::uint8_t k = 0u; k < sw.count; ++k)
            {
                if (!detail::fits_int16(sw.vals[k]))
                    return RcConfigResult(false, RcConfigError::NonFiniteValue, i, cfg.role_to_channel[i]);
            }

            if (sw.raw_count != 0u)
            {
                if (sw.raw_count != sw.count)
                    return RcConfigResult(false, RcConfigError::InvalidSwitchLevels, i, cfg.role_to_channel[i]);
                for (std::uint8_t a = 0u; a < sw.raw_count; ++a)
                {
                    if (sw.raw_levels[a] <= kNoSignalThresholdUs)
                        return RcConfigResult(false, RcConfigError::InvalidSwitchLevels, i, cfg.role_to_channel[i]);
                    for (std::uint8_t b = static_cast<std::uint8_t>(a + 1u); b < sw.raw_count; ++b)
                    {
                        const std::int32_t delta =
                            static_cast<std::int32_t>(sw.raw_levels[a]) - static_cast<std::int32_t>(sw.raw_levels[b]);
                        const std::uint32_t magnitude = static_cast<std::uint32_t>(delta < 0 ? -delta : delta);
                        if (magnitude < static_cast<std::uint32_t>(sw.min_sep_us))
                            return RcConfigResult(false, RcConfigError::InvalidSwitchSeparation, i,
                                                  cfg.role_to_channel[i]);
                    }
                }
            }

            if (sw.auto_levels)
            {
                if (sw.raw_count != 0u || !detail::finite_float(sw.learn_alpha) || sw.learn_alpha <= 0.0f ||
                    sw.learn_alpha > 1.0f || sw.min_sep_us == 0u)
                    return RcConfigResult(false, RcConfigError::InvalidSwitchLearning, i, cfg.role_to_channel[i]);

                const int switch_span = static_cast<int>(spec.axis.raw_hi) - static_cast<int>(spec.axis.raw_lo);
                const int required_span = static_cast<int>(sw.min_sep_us) * static_cast<int>(sw.count - 1u);
                if (switch_span <= 0 || switch_span < required_span)
                    return RcConfigResult(false, RcConfigError::InvalidSwitchSeparation, i, cfg.role_to_channel[i]);
            }
        }
        else
        {
            return RcConfigResult(false, RcConfigError::InvalidRole, i, cfg.role_to_channel[i]);
        }
    }

    return RcConfigResult(true, RcConfigError::None, 0u, kInvalidChannel);
}
} // namespace rc
