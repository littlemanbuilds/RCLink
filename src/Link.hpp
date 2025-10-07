/**
 * MIT License
 *
 * @brief RcLink: protocol-agnostic RC reader/mapper with shaping, filtering and failsafe.
 *
 * @file Link.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <type_traits>
#include <Types.hpp>
#include <Config.hpp>
#include <Constants.hpp>

namespace rc
{
    /**
     * @brief Protocol-agnostic RC link that maps raw transport frames to scaled outputs.
     *
     * Feed it a Transport that provides SBUS/iBUS-like methods and an enum E of roles
     * that ends with `Count`. Supports per-axis shaping, EMA filtering, link/failsafe logic,
     * and epsilon-based change suppression.
     *
     * @tparam Transport Concrete transport with: begin(), update(), readRaw(), protoFailsafe(), frameLost(), caps().
     * @tparam E Enum with role names and a trailing Count.
     */
    template <class Transport, typename E>
    class RcLink
    {
    public:
        static constexpr std::size_t N = static_cast<std::size_t>(E::Count); ///< Number of logical roles.

        // Compile-time sanity checks.
        static_assert(std::is_enum<E>::value, "RcLink<E>: E must be an enum with a trailing Count.");
        static_assert(static_cast<std::size_t>(E::Count) > 0, "RcLink<E>: E::Count must be > 0.");
        static_assert(kRcMaxSwitchVals >= 2 && kRcMaxSwitchVals <= 16,
                      "RcLink: kRcMaxSwitchVals must be in [2..16].");

        /**
         * @brief Construct with a transport instance.
         * @param t Reference to a transport object (not owned).
         */
        explicit RcLink(Transport &t) : tx_(t) {}

        /**
         * @brief Initialize the transport layer.
         * @param port Hardware serial reference.
         * @param baud Baud rate (transport may override).
         * @param rxPin RX pin.
         * @param txPin TX pin (ignored by RX-only transports).
         */
        void begin(::HardwareSerial &port, std::uint32_t baud, int rxPin, int txPin)
        {
            tx_.begin(port, baud, rxPin, txPin);
            caps_ = tx_.caps();
        }

        /**
         * @brief Apply configuration and pre-compute per-channel coefficients.
         * @param c Configuration with mapping, specs, filters, and epsilons.
         */
        void apply_config(const RcConfig<E> &c)
        {
            cfg_ = c;

            for (std::size_t i = 0; i < N; ++i)
            {
                const auto &sp = cfg_.specs[i];

                // Normalize ALL channels (Axis and Switch) to -1..+1 using raw lo/hi.
                const int lo = sp.axis.raw_lo;
                const int hi = sp.axis.raw_hi;
                const float span = static_cast<float>(hi - lo);

                k_scale_[i] = (span > 0.0f) ? (2.0f / span) : 0.0f;
                k_offset_[i] = -1.0f - lo * k_scale_[i];

                // Deadband re-expand terms for axes only.
                if (sp.kind == ChannelKind::Axis && sp.axis.deadband_us > 0 && span > 0.0f)
                {
                    db_norm_[i] = 2.0f * static_cast<float>(sp.axis.deadband_us) / span;
                    db_inv_span_[i] = (db_norm_[i] < 0.999f) ? (1.0f / (1.0f - db_norm_[i])) : 1.0f;
                }
                else
                {
                    db_norm_[i] = 0.0f;
                    db_inv_span_[i] = 1.0f;
                }

                // Seed auto-learn centroids for switches when enabled and no explicit raw levels are provided.
                if (sp.kind == ChannelKind::Switch && sp.sw.auto_levels && sp.sw.raw_count == 0 && sp.sw.count > 0)
                {
                    seed_switch_centroids_(i, lo, hi, sp.sw.count);
                }

                // Reset filter to exposed output to avoid a first-frame jump.
                filtered_.vals[i] = out_.vals[i];
            }
        }

        /**
         * @brief Configure a receiver-failsafe signature in scaled space.
         * @param r Failsafe signature rule (expected values, mask, tolerance, hold time).
         */
        void set_failsafe_signature(const RcFailsafeRule<N> &r)
        {
            fsig_ = r;
            fsig_set_ = true;
            fsig_first_ms_ = 0;
        }

        /**
         * @brief Build a frame containing the receiver-failsafe expected values.
         * @return RcFrame<N> with expected values (for checked roles) or current outputs.
         */
        RcFrame<N> failsafe_expected_frame() const noexcept
        {
            RcFrame<N> f{};
            for (std::size_t i = 0; i < N; ++i)
                f.vals[i] = fsig_.check[i] ? fsig_.expected[i] : out_.vals[i];
            return f;
        }

        /**
         * @brief NEW: Access the currently configured receiver-failsafe signature.
         * @return Const reference to the active signature rule.
         */
        const RcFailsafeRule<N> &failsafe_signature() const noexcept
        {
            return fsig_;
        }

        /**
         * @brief Enable/disable applying app failsafe outputs when the RX-failsafe
         *        signature is detected while frames are still arriving.
         * @param enable true to apply failsafe on RX signature (default), false to not.
         * @note Link-stale (@ref RcLinkStatus::link_ok == false) and protocol
         *       failsafe (@ref RcLinkStatus::proto_failsafe == true) will always
         *       apply failsafe outputs regardless of this setting.
         */
        void apply_rxfs_outputs(bool enable) { rxfs_apply_outputs_ = enable; }

        /**
         * @brief Query whether app failsafe outputs are applied on RX-failsafe signature.
         * @return true if RX-signature will trigger applying per-channel failsafe outputs.
         */
        bool is_apply_rxfs_outputs() const noexcept { return rxfs_apply_outputs_; }

        /**
         * @brief Set per-axis EMA filter coefficient (0..1).
         * @param role Logical role to configure.
         * @param alpha Smoothing factor; 0 disables; 1 is very slow.
         */
        void set_axis_filter(E role, float alpha)
        {
            cfg_.setAxisFilter(role, alpha);
        }

        /**
         * @brief Poll the transport and update scaled/filtered outputs.
         * @param now Millis timestamp. Defaults to millis() for convenience or FreeRTOS clocks.
         */
        /**
         * @brief Poll the transport and update scaled/filtered outputs.
         * @param now Millis timestamp. Defaults to millis() for convenience or FreeRTOS clocks.
         */
        void update(std::uint32_t now = millis())
        {
            const bool got = tx_.update(); ///< True if a full new frame parsed.

            if (got)
            {
                status_.frames++;
                status_.last_frame_ms = now;

                // Scale all channels.
                for (std::size_t i = 0; i < N; ++i)
                {
                    const std::uint8_t ch = cfg_.role_to_channel[i];
                    const int raw = tx_.readRaw(static_cast<int>(ch)); ///< µs or <= kNoSignalThresholdUs.
                    scaled_.vals[i] = scale_channel(i, raw);

                    // EMA filter (post-scale, axis only).
                    const float a = cfg_.axis_ema_alpha[i];
                    if (a > 0.0f && cfg_.specs[i].kind == ChannelKind::Axis)
                    {
                        const float prev = static_cast<float>(filtered_.vals[i]);
                        const float cur = static_cast<float>(scaled_.vals[i]);
                        const float out = prev + a * (cur - prev);
                        filtered_.vals[i] = static_cast<std::int16_t>(out >= 0 ? out + 0.5f : out - 0.5f);
                    }
                    else
                    {
                        filtered_.vals[i] = scaled_.vals[i];
                    }
                }

                // Protocol flags (if supported).
                status_.proto_failsafe = tx_.protoFailsafe();
                status_.frame_lost = tx_.frameLost();
            }

            // Link age & OK.
            status_.last_frame_age = (status_.last_frame_ms == 0u) ? 0u : (now - status_.last_frame_ms);
            status_.link_ok = (status_.last_frame_ms != 0u) && (now - status_.last_frame_ms <= cfg_.link_timeout_ms);

            // Pull transport counters into status (error/frames pipeline).
            // (Both provided by iBUS/sBUS transports; safe to call unconditionally.)
            status_.frames = tx_.frames();
            status_.crc_errors = tx_.crcErrors();

            // Receiver-failsafe signature. Using raw scaled values so detection is exact,
            // regardless of smoothing or suppression on outputs.
            status_.rx_failsafe_sig = false;
            if (status_.link_ok && fsig_set_)
            {
                if (close_to_failsafe(scaled_))
                {
                    if (fsig_first_ms_ == 0u)
                        fsig_first_ms_ = now;
                    if ((now - fsig_first_ms_) >= fsig_.hold_ms)
                        status_.rx_failsafe_sig = true;
                }
                else
                {
                    fsig_first_ms_ = 0u;
                }
            }
            else
            {
                fsig_first_ms_ = 0u;
            }

            // Expose safe outputs and apply per-channel failsafe policy when an RX signature
            // is detected and rxfs_apply_outputs_ is enabled (treat like link-lost for outputs).
            if (!status_.link_ok || (status_.rx_failsafe_sig && rxfs_apply_outputs_))
            {
                for (std::size_t i = 0; i < N; ++i)
                    out_.vals[i] = apply_failsafe(i);
            }
            else
            {
                // Epsilon suppression: only write when delta exceeds epsilon.
                for (std::size_t i = 0; i < N; ++i)
                {
                    const std::int16_t prev = out_.vals[i];
                    const std::int16_t cur = filtered_.vals[i];
                    std::int16_t de = static_cast<std::int16_t>(cur - prev);
                    if (de < 0)
                        de = static_cast<std::int16_t>(-de);
                    if (de > cfg_.epsilon[i])
                        out_.vals[i] = cur;
                }
            }

            // FPS estimator (~500 ms window).
            static std::uint32_t last_count = 0, last_t = 0;
            const std::uint32_t dt = now - last_t;
            if (dt >= 500u)
            {
                const std::uint32_t df = status_.frames - last_count;
                status_.fps = static_cast<std::uint16_t>((df * 1000u) / (dt ? dt : 1u));
                last_count = status_.frames;
                last_t = now;
            }
        }

        /**
         * @brief Get the current exposed (safe/epsilon) frame.
         * @return Copy of the current frame.
         */
        RcFrame<N> frame() const noexcept { return out_; }

        /**
         * @brief Read a channel by enum-like role.
         * @tparam EnumLike Any enum convertible to size_t index.
         * @param name Logical role name.
         * @return Scaled value for that role.
         */
        template <typename EnumLike>
        std::int16_t read(EnumLike name) const noexcept
        {
            return out_.vals[static_cast<std::size_t>(name)];
        }

        /**
         * @brief Read a channel by index.
         * @param i Logical index [0..N-1].
         * @return Scaled value for that index.
         */
        std::int16_t read_by_index(std::size_t i) const noexcept { return out_.vals[i]; }

        /**
         * @brief Obtain a snapshot of the link status.
         * @return Const reference to internal status.
         */
        const RcLinkStatus &status() const noexcept { return status_; }

        /**
         * @brief Whether the link is currently considered OK.
         * @return true if a fresh frame arrived within the timeout.
         */
        bool ok() const noexcept { return status_.link_ok; }

        /**
         * @brief Transport capability flags (as reported by the transport).
         * @return Const reference to capability struct.
         */
        const RcTransportCaps &caps() const noexcept { return caps_; }

        /**
         * @brief Check if the exposed frame changed since the last call.
         * @return true if any channel value differs from the previous snapshot.
         */
        bool changed() noexcept
        {
            bool diff = false;
            for (std::size_t i = 0; i < N; ++i)
            {
                if (out_.vals[i] != last_.vals[i])
                {
                    diff = true;
                    break;
                }
            }
            if (diff)
                last_ = out_;
            return diff;
        }

    private:
        /// @brief Clamp integer v to [lo, hi].
        static inline int clampi(int v, int lo, int hi)
        {
            return (v < lo) ? lo : ((v > hi) ? hi : v);
        }

        /// @brief Apply cubic-like expo mixing with factor e in [0,1].
        static inline float apply_expo(float x, float e)
        {
            if (e <= 0.0001f)
                return x;
            if (e >= 0.9999f)
                return x * x * x;
            return (1.0f - e) * x + e * (x * x * x);
        }

        /// @brief Scale one channel from raw µs to shaped output (axis/switch).
        std::int16_t scale_channel(std::size_t idx, int raw) const
        {
            const RcChannelSpec &sp = cfg_.specs[idx];

            // Missing signal → synthesize neutral per kind.
            if (raw < kNoSignalThresholdUs)
            {
                if (sp.kind == ChannelKind::Switch)
                {
                    if (sp.sw.count > 0)
                    {
                        const float v0 = sp.sw.vals[0];
                        return static_cast<std::int16_t>(v0 >= 0 ? v0 + 0.5f : v0 - 0.5f);
                    }
                }
                return 0;
            }

            // Clamp to calibration range.
            const int lo = sp.axis.raw_lo, hi = sp.axis.raw_hi;
            const int r = clampi(raw, lo, hi);

            // Normalize to -1..+1.
            float x = r * k_scale_[idx] + k_offset_[idx];

            if (sp.kind == ChannelKind::Axis)
            {
                if (sp.axis.invert)
                    x = -x;

                // Center deadband in normalized domain.
                const float db = db_norm_[idx];
                if (db > 0.0f)
                {
                    const float ax = (x >= 0 ? x : -x);
                    if (ax < db)
                    {
                        x = 0.0f;
                    }
                    else
                    {
                        const float s = (ax - db) * db_inv_span_[idx];
                        x = (x < 0.0f) ? -s : s;
                    }
                }

                // Expo and map to output range.
                x = apply_expo(x, sp.axis.expo);
                const float mid = 0.5f * (sp.axis.out_lo + sp.axis.out_hi);
                const float half = 0.5f * (sp.axis.out_hi - sp.axis.out_lo);
                const float y = mid + x * half;
                return static_cast<std::int16_t>(y >= 0 ? y + 0.5f : y - 0.5f);
            }
            else
            {
                // Switch snapping.
                const RcSwitchSpec &sw = sp.sw;
                if (sw.count == 0)
                    return 0;

                // 1) Prefer raw snap levels when provided (deterministic).
                if (sw.raw_count >= 2)
                {
                    int best = 0;
                    int bestd = 0x7FFFFFFF;
                    for (std::uint8_t i = 0; i < sw.raw_count; ++i)
                    {
                        int d = r - sw.raw_levels[i];
                        if (d < 0)
                            d = -d;
                        if (d < bestd)
                        {
                            bestd = d;
                            best = static_cast<int>(i);
                        }
                    }
                    const std::uint8_t snap = static_cast<std::uint8_t>((best < sw.count) ? best : (sw.count - 1));
                    const float v = sw.vals[snap];
                    return static_cast<std::int16_t>(v >= 0 ? v + 0.5f : v - 0.5f);
                }

                // 2) Auto-learned centroids when enabled and raw_levels are absent.
                if (sw.auto_levels)
                {
                    // Ensure seeded (in case apply_config was not called after spec changes).
                    if (sw_centroid_count_[idx] == 0 && sw.count > 0)
                        seed_switch_centroids_(idx, sp.axis.raw_lo, sp.axis.raw_hi, sw.count);

                    // Nearest centroid to current raw.
                    const std::uint8_t nearest = nearest_centroid_(idx, r);
                    const int active = sw_active_idx_[idx];

                    // Hysteresis: change only if we move away from current centroid beyond half-band.
                    const int hyst_half = static_cast<int>(sw.hyst_us / 2);
                    if (active < 0)
                    {
                        sw_active_idx_[idx] = static_cast<std::int8_t>(nearest);
                    }
                    else
                    {
                        const int cur_center = sw_centroid_[idx][active];
                        const int dcur = (r >= cur_center) ? (r - cur_center) : (cur_center - r);

                        if (dcur > hyst_half)
                        {
                            const int new_center = sw_centroid_[idx][nearest];
                            const int dnew = (r >= new_center) ? (r - new_center) : (new_center - r);
                            // Small bias to avoid ping-ponging right at a boundary.
                            if (dnew + 2 <= dcur)
                                sw_active_idx_[idx] = static_cast<std::int8_t>(nearest);
                        }
                    }

                    // Learn: pull the active centroid towards current raw (simple EMA).
                    const int act = sw_active_idx_[idx];
                    if (act >= 0 && act < static_cast<int>(sw_centroid_count_[idx]))
                        learn_centroid_(idx, static_cast<std::uint8_t>(act), r, sw.learn_alpha);

                    // Emit the value mapped to currently active index (or nearest if uninitialized).
                    const std::uint8_t emit_idx = (sw_active_idx_[idx] >= 0)
                                                      ? static_cast<std::uint8_t>(sw_active_idx_[idx])
                                                      : nearest;

                    const float v = sw.vals[(emit_idx < sw.count) ? emit_idx : (sw.count - 1)];
                    return static_cast<std::int16_t>(v >= 0 ? v + 0.5f : v - 0.5f);
                }

                // 3) Fallback: normalized-space snap when auto-learning is disabled and no raw_levels exist.
                float dist_best = 1e9f;
                std::uint8_t idx_best = 0;
                for (std::uint8_t i = 0; i < sw.count; ++i)
                {
                    // Accept values either in -100..100 or hints in 0..2 normalized.
                    const float t = (sw.vals[i] >= 0.f && sw.vals[i] <= 2.f)
                                        ? (sw.vals[i] - 1.f)
                                        : (sw.vals[i] / 100.f);
                    float d = x - t;
                    if (d < 0)
                        d = -d;
                    if (d < dist_best)
                    {
                        dist_best = d;
                        idx_best = i;
                    }
                }
                const float v = sw.vals[idx_best];
                return static_cast<std::int16_t>(v >= 0 ? v + 0.5f : v - 0.5f);
            }
        }

        /// @brief Compute failsafe output for channel idx.
        std::int16_t apply_failsafe(std::size_t idx) const
        {
            const auto &sp = cfg_.specs[idx];
            const auto &fs = cfg_.fs[idx];
            switch (fs.mode)
            {
            case Failsafe::Mode::Value:
                return fs.value;
            case Failsafe::Mode::HoldLast:
                return filtered_.vals[idx];
            case Failsafe::Mode::ClampToOutLo:
                return (sp.kind == ChannelKind::Axis)
                           ? static_cast<std::int16_t>(sp.axis.out_lo >= 0 ? sp.axis.out_lo + 0.5f : sp.axis.out_lo - 0.5f)
                           : filtered_.vals[idx];
            case Failsafe::Mode::ClampToOutHi:
                return (sp.kind == ChannelKind::Axis)
                           ? static_cast<std::int16_t>(sp.axis.out_hi >= 0 ? sp.axis.out_hi + 0.5f : sp.axis.out_hi - 0.5f)
                           : filtered_.vals[idx];
            default:
                return 0;
            }
        }

        /// @brief Check whether frame f matches the configured RX failsafe signature.
        bool close_to_failsafe(const RcFrame<N> &f) const
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                if (fsig_.check[i])
                {
                    int d = f.vals[i] - fsig_.expected[i];
                    if (d < 0)
                        d = -d;
                    if (d > static_cast<int>(fsig_.tol))
                        return false;
                }
            }
            return true;
        }

        // ---- Switch auto-learning helpers ---- //

        /**
         * @brief Seed equidistant raw-µs centroids across [lo..hi] for a switch.
         * @param idx Logical channel index.
         * @param lo Raw lower bound (µs).
         * @param hi Raw upper bound (µs).
         * @param count Logical positions for the switch.
         */
        void seed_switch_centroids_(std::size_t idx, int lo, int hi, std::uint8_t count) const noexcept
        {
            if (count == 0)
            {
                sw_centroid_count_[idx] = 0;
                sw_active_idx_[idx] = -1;
                return;
            }

            if (hi < lo)
            {
                const int t = lo;
                lo = hi;
                hi = t;
            }

            const float step = (count > 1) ? static_cast<float>(hi - lo) / static_cast<float>(count - 1) : 0.0f;
            for (std::uint8_t k = 0; k < count; ++k)
            {
                const float c = static_cast<float>(lo) + step * static_cast<float>(k);
                sw_centroid_[idx][k] = static_cast<std::int16_t>(c >= 0 ? c + 0.5f : c - 0.5f);
            }
            sw_centroid_count_[idx] = count;
            sw_active_idx_[idx] = -1;
        }

        /**
         * @brief Index of nearest centroid to raw.
         * @param idx Logical channel index.
         * @param raw Raw microseconds.
         */
        std::uint8_t nearest_centroid_(std::size_t idx, int raw) const noexcept
        {
            std::uint8_t best = 0;
            int bestd = 0x7FFFFFFF;
            const std::uint8_t n = sw_centroid_count_[idx];
            for (std::uint8_t k = 0; k < n; ++k)
            {
                int d = raw - sw_centroid_[idx][k];
                if (d < 0)
                    d = -d;
                if (d < bestd)
                {
                    bestd = d;
                    best = k;
                }
            }
            return best;
        }

        /**
         * @brief One-step EMA update for a centroid.
         * @param idx Logical channel index.
         * @param k Centroid index.
         * @param raw Raw microseconds.
         * @param alpha Learning rate (0..1].
         */
        void learn_centroid_(std::size_t idx, std::uint8_t k, int raw, float alpha) const noexcept
        {
            if (alpha <= 0.0f)
                return;
            if (alpha > 1.0f)
                alpha = 1.0f;

            const float c = static_cast<float>(sw_centroid_[idx][k]);
            const float u = c + alpha * (static_cast<float>(raw) - c);
            sw_centroid_[idx][k] = static_cast<std::int16_t>(u >= 0 ? u + 0.5f : u - 0.5f);
        }

    private:
        Transport &tx_;          ///< Transport reference (not owned).
        RcTransportCaps caps_{}; ///< Transport capability flags.
        RcConfig<E> cfg_{};      ///< Active configuration.

        RcFrame<N> scaled_{};   ///< Raw→scaled (pre-filter).
        RcFrame<N> filtered_{}; ///< Filtered post-scale values.
        RcFrame<N> out_{};      ///< Exposed (safe/epsilon) frame.
        RcFrame<N> last_{};     ///< Previous exposed frame (for changed()).

        // Status & receiver-failsafe signature tracking.
        RcLinkStatus status_{};           ///< Link status snapshot.
        RcFailsafeRule<N> fsig_{};        ///< RX failsafe signature rule.
        bool fsig_set_{false};            ///< Whether signature is configured.
        std::uint32_t fsig_first_ms_{0};  ///< First time signature matched (ms).
        bool rxfs_apply_outputs_ = false; ///< Apply app failsafe when RX signature is detected.

        // Precomputed per-channel coefficients.
        float k_scale_[N]{};     ///< Raw→[-1..+1] scale per channel.
        float k_offset_[N]{};    ///< Raw→[-1..+1] offset per channel.
        float db_norm_[N]{};     ///< Deadband width in normalized units (axes).
        float db_inv_span_[N]{}; ///< 1/(1 - deadband) for re-expansion (axes).

        // Auto-learn switch state (used only when raw_levels are absent).
        mutable std::int16_t sw_centroid_[N][kRcMaxSwitchVals]{}; ///< Learned raw µs per position.
        mutable std::uint8_t sw_centroid_count_[N]{};             ///< Active centroid count.
        mutable std::int8_t sw_active_idx_[N]{};                  ///< Current snapped index (-1 none).
    };

} ///< namespace rc.