/**
 * MIT License
 *
 * @brief Fluent configuration builders.
 *
 * @file Config.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <initializer_list>
#include <Types.hpp>

namespace rc
{
    template <typename E>
    struct RcConfig
    {
        static constexpr std::size_t N = static_cast<std::size_t>(E::Count); ///< Number of logical roles in the enum.

        // Mapping & specs.
        std::uint8_t role_to_channel[N]{}; ///< Logical index → RX channel index.
        RcChannelSpec specs[N]{};          ///< Per logical index.

        // Failsafe & noise suppression.
        Failsafe fs[N]{};          ///< Per-channel failsafe policy.
        std::int16_t epsilon[N]{}; ///< Ignore |delta| <= epsilon on reporting.
        float axis_ema_alpha[N]{}; ///< 0=disabled, 1=very slow.

        // Link timeout.
        std::uint16_t link_timeout_ms{200}; ///< Link considered stale after this (ms).

        /**
         * @brief Map a logical role to a receiver channel index.
         * @param role Logical role to assign.
         * @param rx Receiver channel index to map to this role.
         * @return Reference to the current RcConfig instance (for chaining).
         */
        RcConfig &map(E role, std::uint8_t rx)
        {
            const auto i = static_cast<std::size_t>(role);
            if (i < N)
                role_to_channel[i] = rx;
            return *this;
        }

        /**
         * @brief Axis channel builder.
         *
         * Provides a fluent API to configure axis channels (input range,
         * output scaling, deadband, expo curve, inversion, and failsafe).
         */
        struct AxisB
        {
            RcChannelSpec &s; ///< Reference to the channel specification.
            RcConfig &cfg;    ///< Reference to the parent configuration.
            std::size_t idx;  ///< Index of this role.

            /**
             * @brief Configure raw input range for the axis.
             * @param lo Minimum raw value.
             * @param hi Maximum raw value.
             * @param center Center raw value.
             * @return Reference to the current AxisB builder (for chaining).
             */
            AxisB &raw(std::int16_t lo, std::int16_t hi, std::int16_t center)
            {
                s.kind = ChannelKind::Axis;
                s.axis.raw_lo = lo;
                s.axis.raw_hi = hi;
                s.axis.raw_center = center;
                return *this;
            }

            /**
             * @brief Set deadband for the axis in microseconds.
             * @param d Deadband in microseconds.
             * @return Reference to the current AxisB builder.
             */
            AxisB &deadband_us(std::int16_t d)
            {
                s.axis.deadband_us = (d < 0 ? 0 : d);
                return *this;
            }

            /**
             * @brief Set output range for the axis.
             * @param lo Minimum output value.
             * @param hi Maximum output value.
             * @return Reference to the current AxisB builder.
             */
            AxisB &out(float lo, float hi)
            {
                s.axis.out_lo = lo;
                s.axis.out_hi = hi;
                return *this;
            }

            /**
             * @brief Set exponential curve factor for the axis.
             * @param e Expo factor (0 = linear, 1 = maximum curve).
             * @return Reference to the current AxisB builder.
             */
            AxisB &expo(float e)
            {
                if (e < 0)
                    e = 0;
                if (e > 1)
                    e = 1;
                s.axis.expo = e;
                return *this;
            }

            /**
             * @brief Invert axis direction.
             * @param inv If true, inverts the axis (default = true).
             * @return Reference to the current AxisB builder.
             */
            AxisB &invert(bool inv = true)
            {
                s.axis.invert = inv;
                return *this;
            }

            /**
             * @brief Configure failsafe behavior for the axis.
             * @param m Failsafe mode.
             * @param def Default value to apply when failsafe is triggered (default = 0).
             * @return Reference to the current AxisB builder.
             */
            AxisB &failsafe(Failsafe::Mode m, std::int16_t v = 0)
            {
                cfg.fs[idx].mode = m;
                cfg.fs[idx].value = v;
                return *this;
            }

            /**
             * @brief Finalize axis configuration and return to parent config.
             * @return Reference to the RcConfig object.
             */
            RcConfig &done() { return cfg; }
        };

        /**
         * @brief Configure a channel role as an axis.
         * @param role Logical role to configure.
         * @return AxisB Builder object for configuring the axis channel.
         */
        AxisB axis(E role)
        {
            const auto i = static_cast<std::size_t>(role);
            if (i < N)
            {
                specs[i].kind = ChannelKind::Axis;
            }
            return AxisB{specs[static_cast<std::size_t>(role)], *this, static_cast<std::size_t>(role)};
        }

        /**
         * @brief Switch channel builder.
         *
         * Provides a fluent API to configure switch channels (logical values,
         * optional explicit raw input levels, auto-learning parameters, and failsafe behavior).
         */
        struct SwitchB
        {
            RcChannelSpec &s; ///< Reference to the channel specification.
            RcConfig &cfg;    ///< Reference to the parent configuration.
            std::size_t idx;  ///< Index of this role.

            /**
             * @brief Set logical values for switch positions.
             * @param v List of float values corresponding to switch positions.
             * @return Reference to the current SwitchB builder (for chaining).
             *
             * Values may be in either:
             *  - engineering units (e.g., -100..+100), or
             *  - normalized hints 0..2 (interpreted as -1..+1 positions).
             */
            SwitchB &values(std::initializer_list<float> v)
            {
                s.kind = ChannelKind::Switch;
                s.sw.count = static_cast<std::uint8_t>(
                    (v.size() > kRcMaxSwitchVals) ? kRcMaxSwitchVals : v.size());
                std::uint8_t i = 0;
                for (float x : v)
                    if (i < s.sw.count)
                        s.sw.vals[i++] = x;
                return *this;
            }

            /**
             * @brief Provide explicit raw µs levels for switch positions (disables auto-learning).
             * @param lvls List of raw input levels for switch positions (µs).
             * @return Reference to the current SwitchB builder.
             */
            SwitchB &raw_levels(std::initializer_list<std::int16_t> lvls)
            {
                s.kind = ChannelKind::Switch;
                s.sw.raw_count = static_cast<std::uint8_t>(
                    (lvls.size() > kRcMaxSwitchVals) ? kRcMaxSwitchVals : lvls.size());
                std::uint8_t i = 0;
                for (std::int16_t u : lvls)
                    if (i < s.sw.raw_count)
                        s.sw.raw_levels[i++] = u;

                // Explicit raw levels imply no auto-learning.
                s.sw.auto_levels = false;
                return *this;
            }

            /**
             * @brief Enable/disable auto-learning of raw switch levels (requires values()).
             * @param enable True to enable, false to disable.
             * @return Reference to the current SwitchB builder.
             */
            SwitchB &auto_levels(bool enable = true)
            {
                s.kind = ChannelKind::Switch;
                s.sw.auto_levels = enable;
                return *this;
            }

            /**
             * @brief Set hysteresis band for auto-learn snapping (µs).
             * @param us Half-width*2 band in microseconds; larger reduces chatter.
             * @return Reference to the current SwitchB builder.
             */
            SwitchB &hysteresis_us(std::uint16_t us)
            {
                s.kind = ChannelKind::Switch;
                s.sw.hyst_us = us;
                return *this;
            }

            /**
             * @brief Set learning rate (EMA alpha) for auto-learn centroids.
             * @param a Alpha in [0..1]; higher values move slower.
             * @return Reference to the current SwitchB builder.
             */
            SwitchB &learn_alpha(float a)
            {
                if (a < 0.f)
                    a = 0.f;
                if (a > 1.f)
                    a = 1.f;
                s.kind = ChannelKind::Switch;
                s.sw.learn_alpha = a;
                return *this;
            }

            /**
             * @brief Minimum separation between learned centroids (reserved for future enforcement).
             * @param us Separation in microseconds.
             * @return Reference to the current SwitchB builder.
             */
            SwitchB &min_sep_us(std::uint16_t us)
            {
                s.kind = ChannelKind::Switch;
                s.sw.min_sep_us = us;
                return *this;
            }

            /**
             * @brief Configure failsafe behavior for the switch.
             * @param m Failsafe mode.
             * @param def Default value to apply when failsafe is triggered (default = 0).
             * @return Reference to the current SwitchB builder.
             */
            SwitchB &failsafe(Failsafe::Mode m, std::int16_t v = 0)
            {
                cfg.fs[idx].mode = m;
                cfg.fs[idx].value = v;
                return *this;
            }

            /**
             * @brief Finalize switch configuration and return to parent config.
             * @return Reference to the RcConfig object.
             */
            RcConfig &done() { return cfg; }
        };

        /**
         * @brief Configure a channel role as a switch.
         * @param role Logical role to configure.
         * @return SwitchB Builder object for configuring the switch channel.
         */
        SwitchB sw(E role)
        {
            const auto i = static_cast<std::size_t>(role);
            if (i < N)
            {
                specs[i].kind = ChannelKind::Switch;
            }
            return SwitchB{specs[static_cast<std::size_t>(role)], *this, static_cast<std::size_t>(role)};
        }

        // ---- Misc ---- //

        /**
         * @brief Set the receiver link timeout.
         * @param ms Timeout in milliseconds before the link is considered lost.
         * @return Reference to the current RcConfig instance (for chaining).
         */
        RcConfig &setLinkTimeout(std::uint16_t ms)
        {
            link_timeout_ms = ms;
            return *this;
        }

        /**
         * @brief Set the epsilon (deadband) for a given role.
         * @param role Role to configure.
         * @param e Deadband in scaled output units (negative values are clamped to 0).
         * @return Reference to the current RcConfig instance (for chaining).
         */
        RcConfig &setEpsilon(E role, std::int16_t e)
        {
            epsilon[static_cast<std::size_t>(role)] = (e < 0 ? 0 : e);
            return *this;
        }

        /**
         * @brief Set the axis filter coefficient (EMA alpha) for a given role.
         * @param role Role to configure.
         * @param alpha Smoothing factor between 0.0 and 1.0 (values outside are clamped).
         * @return Reference to the current RcConfig instance (for chaining).
         */
        RcConfig &setAxisFilter(E role, float alpha)
        {
            if (alpha < 0)
                alpha = 0;
            if (alpha > 1)
                alpha = 1;
            axis_ema_alpha[static_cast<std::size_t>(role)] = alpha;
            return *this;
        }

        /**
         * @brief Set the failsafe policy for a given role (axis or switch) in one call.
         * @param role Role to configure.
         * @param mode Failsafe mode to apply when the link is stale.
         * @param def Output value used by Failsafe::Mode::Value (scaled units).
         * @return Reference to the current RcConfig instance (for chaining).
         */
        RcConfig &setFailsafePolicy(E role, Failsafe::Mode mode, std::int16_t v = 0)
        {
            const auto i = static_cast<std::size_t>(role);
            if (i < N)
            {
                fs[i].mode = mode;
                fs[i].value = v;
            }
            return *this;
        }
    };

} ///< namespace rc.