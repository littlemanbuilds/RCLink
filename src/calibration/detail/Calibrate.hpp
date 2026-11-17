/**
 * MIT License
 *
 * @brief Shared bounded calibration statistics and guided presentation.
 *
 * @file Calibrate.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-08
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "Constants.hpp"
#include "Types.hpp"

#include <Arduino.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace rc
{
namespace calibrate
{
// ---- Tunables for classification ---- //

constexpr int kBinUs = 25;             ///< Histogram bin width (µs).
constexpr int kClusterTolUs = 120;     ///< Cluster merge tolerance (µs).
constexpr int kAxisMinSane = 800;      ///< Sane floor (µs).
constexpr int kAxisMaxSane = 2200;     ///< Sane ceil (µs).
constexpr int kMinLevelDwell = 8;      ///< Keep cluster only if ≥ this many hits.
constexpr int kMinBinDwell = 3;        ///< Bin considered “occupied” if ≥ hits.
constexpr int kAxisMinBins = 10;       ///< ≥ occupied bins → likely axis.
constexpr int kAxisCoveragePct = 25;   ///< Coverage % across span bins.
constexpr int kSwitchMaxLevels = 6;    ///< Cap for multi-position detection.
constexpr int kSpanNoneUs = 40;        ///< Below: no input.
constexpr int kSpanSwitchMaxUs = 1200; ///< Soft guard for 4..6 position spans.

// One private schema preserves the original flat public Stats aggregate (including
// C++14 initialization/forward declarations) and scales internal S.BUS storage.
// Inheritance changes aggregate semantics; duplicate field definitions can drift.
#define RCLINK_CALIBRATION_STATS_MEMBERS(Capacity) \
    static constexpr int K = Capacity; /**< Protocol-specific analogue channel capacity. */ \
    static constexpr int L = 8;                /**< Maximum clustered switch levels. */ \
    /* Per-channel running mins/max, last value, jitter. */ \
    int minv[static_cast<std::size_t>(K)]{};        /**< Minimum observed raw value per channel. */ \
    int maxv[static_cast<std::size_t>(K)]{};        /**< Maximum observed raw value per channel. */ \
    int last[static_cast<std::size_t>(K)]{};        /**< Last accepted raw value per channel. */ \
    int jitter_peak[static_cast<std::size_t>(K)]{}; /**< Peak small delta (jitter) per channel. */ \
    /* Clusters (for discrete switch levels). */ \
    int levels[static_cast<std::size_t>(K)][L]{};        /**< Cluster centers for switch levels. */ \
    std::uint16_t occ[static_cast<std::size_t>(K)][L]{}; /**< Dwell count per cluster. */ \
    std::uint8_t lcnt[static_cast<std::size_t>(K)]{};    /**< Number of clusters used per channel. */ \
    /* Histogram (coarse density to detect axes vs switches). */ \
    static constexpr int kHistMin = kAxisMinSane;                        /**< Histogram minimum (µs). */ \
    static constexpr int kHistMax = kAxisMaxSane;                        /**< Histogram maximum (µs). */ \
    static constexpr int kHistBins = (kHistMax - kHistMin) / kBinUs + 1; /**< Number of bins. */ \
    std::uint16_t hist[static_cast<std::size_t>(K)][kHistBins]{};                                  /**< Histogram counts. */ \
    /* Sample counts. */ \
    std::uint32_t samples[static_cast<std::size_t>(K)]{}; /**< Total accepted samples per channel. */

/**
 * @brief Public iBUS statistics, retaining the original flat aggregate and field layout.
 */
struct Stats
{
    RCLINK_CALIBRATION_STATS_MEMBERS(kIbusMaxChannels)
};

namespace detail
{
/**
 * @brief Fixed-capacity session statistics using the same fields as public Stats.
 *
 * @tparam Capacity Maximum analogue channels for this protocol.
 */
template <int Capacity> struct CalibrationStats
{
    RCLINK_CALIBRATION_STATS_MEMBERS(Capacity)
};
} // namespace detail
#undef RCLINK_CALIBRATION_STATS_MEMBERS

// ---- Helpers ---- //

/**
 * @brief Absolute value helper that avoids C++ math headers on small AVR cores.
 *
 * @param v Input value; must not be the minimum representable int.
 * @return Non-negative absolute value.
 */
static inline int abs_i(int v)
{
    return (v < 0) ? -v : v;
}

/**
 * @brief Clamp an integer to a specified range.
 *
 * @param v Input value.
 * @param lo Minimum allowed value.
 * @param hi Maximum allowed value.
 * @return Clamped integer value.
 */
static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * @brief Map a microsecond value into a histogram bin index.
 *
 * @param us Input value in microseconds.
 * @return Histogram bin index (0..kHistBins-1).
 */
static inline int bin_index(int us)
{
    if (us < Stats::kHistMin)
        us = Stats::kHistMin;
    if (us > Stats::kHistMax)
        us = Stats::kHistMax;
    return (us - Stats::kHistMin) / kBinUs;
}

namespace detail
{
/**
 * @brief Estimate an axis centre from coarse dwell density, falling back to midpoint.
 *
 * @param s Captured calibration statistics.
 * @param ch Zero-based channel below the selected capacity; invalid indices fail closed.
 * @param rawMin Observed lower endpoint.
 * @param rawMax Observed upper endpoint.
 * @tparam Statistics Shared-schema statistics type with a fixed channel capacity.
 * @return Provisional centre in converted raw units, not a measured mechanical neutral.
 *
 * Endpoint bins are excluded where possible so time spent against a stick stop
 * does not pull neutral away from the actual spring-return position. The
 * midpoint remains the deterministic fallback when the histogram is sparse.
 */
template <typename Statistics> static int estimate_axis_center(const Statistics &s, int ch, int rawMin, int rawMax)
{
    const int midpoint = rawMin + (rawMax - rawMin) / 2;
    const int span = rawMax - rawMin;
    if (ch < 0 || ch >= Statistics::K || span < kSpanNoneUs)
        return midpoint;

    const int margin = span / 5;
    const int searchMin = rawMin + margin;
    const int searchMax = rawMax - margin;
    const int bmin = bin_index(searchMin);
    const int bmax = bin_index(searchMax);

    std::uint16_t bestHits = 0u;
    int bestCenter = midpoint;
    int bestDistance = 0x7FFFFFFF;
    for (int b = bmin; b <= bmax; ++b)
    {
        const std::uint16_t hits = s.hist[ch][b];
        const int center = Stats::kHistMin + b * kBinUs + kBinUs / 2;
        const int distance = abs_i(center - midpoint);
        if (hits > bestHits || (hits == bestHits && hits != 0u && distance < bestDistance))
        {
            bestHits = hits;
            bestCenter = center;
            bestDistance = distance;
        }
    }
    // A coarse histogram bin can approach an endpoint on a narrow axis. Keep
    // provisional suggestions inside the core's minimum active-side span.
    const int estimate = (bestHits == 0u) ? midpoint : clampi(bestCenter, rawMin, rawMax);
    return clampi(estimate, rawMin + kRcMinAxisSideSpanUs, rawMax - kRcMinAxisSideSpanUs);
}
} // namespace detail

/**
 * @brief Retain the existing iBUS calibration helper signature.
 *
 * @param s iBUS calibration statistics.
 * @param ch Zero-based channel index; invalid indices fail closed.
 * @param rawMin Observed lower endpoint.
 * @param rawMax Observed upper endpoint.
 * @return Provisional centre estimate in converted raw units.
 */
static inline int estimate_axis_center(const Stats &s, int ch, int rawMin, int rawMax)
{
    return detail::estimate_axis_center(s, ch, rawMin, rawMax);
}


// ---- Channel classification ---- //

enum class ChClass : std::uint8_t
{
    NoInput, ///< No significant motion observed.
    Axis,    ///< Continuous axis.
    Switch2, ///< 2-position switch.
    Switch3, ///< 3-position switch.
    SwitchN  ///< 4–6 position switch (detected by clustering).
};

/**
 * @brief In-place insertion sort for switch level clusters.
 *
 * @param lv Array of level values to be sorted (modified in place).
 * @param oc Array of occupancy counts aligned with lv (modified in place).
 * @param n Number of elements in both arrays.
 */
static void sort_levels(int *lv, std::uint16_t *oc, std::uint8_t n)
{
    for (int a = 1; a < n; ++a)
    {
        const int key = lv[a];
        const std::uint16_t ok = oc[a];
        int j = a - 1;
        while (j >= 0 && lv[j] > key)
        {
            lv[j + 1] = lv[j];
            oc[j + 1] = oc[j];
            --j;
        }
        lv[j + 1] = key;
        oc[j + 1] = ok;
    }
}

namespace detail
{
/**
 * @brief Heuristic check to classify a channel as an axis using histogram data.
 *
 * @param s Statistics container holding histograms and min/max values.
 * @param ch Zero-based channel below the selected capacity; invalid indices fail closed.
 * @param spanUs Observed span in microseconds for this channel.
 * @return true If the channel is likely an axis.
 * @return false Otherwise (too narrow, too few bins occupied, or low coverage).
 * @tparam Statistics Shared-schema statistics type with a fixed channel capacity.
 */
template <typename Statistics> static bool is_axis_by_hist(const Statistics &s, int ch, int spanUs)
{
    if (ch < 0 || ch >= Statistics::K || spanUs < kSpanNoneUs)
        return false;
    const int bmin = bin_index(clampi(s.minv[ch], Stats::kHistMin, Stats::kHistMax));
    const int bmax = bin_index(clampi(s.maxv[ch], Stats::kHistMin, Stats::kHistMax));
    const int totalBins = (bmax >= bmin) ? (bmax - bmin + 1) : 0;
    if (totalBins <= 0)
        return false;

    int occupied = 0;
    for (int b = bmin; b <= bmax; ++b)
        if (s.hist[ch][b] >= kMinBinDwell)
            occupied++;
    const int coveragePct = (occupied * 100) / totalBins;

    return (occupied >= kAxisMinBins && coveragePct >= kAxisCoveragePct);
}
} // namespace detail

/**
 * @brief Retain the existing iBUS calibration helper signature.
 *
 * @param s iBUS calibration statistics.
 * @param ch Zero-based channel index; invalid indices fail closed.
 * @param spanUs Observed span.
 * @return True when dwell coverage supports an axis classification.
 */
static inline bool is_axis_by_hist(const Stats &s, int ch, int spanUs)
{
    return detail::is_axis_by_hist(s, ch, spanUs);
}


/**
 * @brief Heuristic snap for common 2- or 3-position switch endpoints.
 *
 * @param lv Array of detected level values (µs).
 * @param n Number of levels in lv (must be 2 or 3 to be considered).
 * @return ChClass::Switch2 if values look like a common 2-position switch.
 * @return ChClass::Switch3 if values look like a common 3-position switch.
 * @return ChClass::Axis otherwise.
 */
static inline ChClass snap_2_or_3pos_if_common(const int *lv, std::uint8_t n)
{
    if (n < 2 || n > 3)
        return ChClass::Axis;
    /**
     * @brief Compare bounded calibration values with an inclusive tolerance.
     *
     * @param v Raw calibration value in microseconds.
     * @param target Nominal receiver endpoint in microseconds.
     * @param tol Nonnegative tolerance in microseconds.
     * @return True when the absolute difference is within tolerance.
     */
    const auto near = [](int v, int target, int tol) { return abs_i(v - target) <= tol; };
    constexpr int kTol = 140;

    if (n == 2)
    {
        const bool looks2 = (near(lv[0], 1000, kTol) && near(lv[1], 2000, kTol)) ||
                            (near(lv[1], 1000, kTol) && near(lv[0], 2000, kTol));
        return looks2 ? ChClass::Switch2 : ChClass::Axis;
    }

    bool at1 = false, at2 = false, at3 = false;
    for (int i = 0; i < 3; i++)
    {
        at1 |= near(lv[i], 1000, kTol);
        at2 |= near(lv[i], 1500, kTol);
        at3 |= near(lv[i], 2000, kTol);
    }
    return (at1 && at2 && at3) ? ChClass::Switch3 : ChClass::Axis;
}

namespace detail
{
/**
 * @brief Classify a channel as Axis/Switch based on span, histogram, and clusters.
 *
 * @param s Statistics container (histograms, min/max, etc.).
 * @param ch Zero-based channel below the selected capacity; invalid indices fail closed.
 * @param spanUs Observed span in microseconds for this channel.
 * @param levels Pointer to clustered level centers (µs).
 * @param levelCount Number of clustered levels in levels.
 * @return ChClass::NoInput if span is too small.
 * @return ChClass::Axis if histogram density indicates a continuous axis or no switch match.
 * @return ChClass::Switch2 / ChClass::Switch3 / ChClass::SwitchN for discrete switches.
 * @tparam Statistics Shared-schema statistics type with a fixed channel capacity.
 */
template <typename Statistics> static ChClass classify_channel(const Statistics &s, int ch, int spanUs, const int *levels, std::uint8_t levelCount)
{
    (void)levels; // Retained helper signature; cluster count already decides discrete classification.
    if (ch < 0 || ch >= Statistics::K || spanUs < kSpanNoneUs)
        return ChClass::NoInput;
    if (is_axis_by_hist(s, ch, spanUs))
        return ChClass::Axis;

    if (levelCount == 2)
        return ChClass::Switch2;
    if (levelCount == 3)
        return ChClass::Switch3;
    if (levelCount >= 4 && levelCount <= kSwitchMaxLevels && spanUs <= kSpanSwitchMaxUs)
        return ChClass::SwitchN;

    return ChClass::Axis;
}
} // namespace detail

/**
 * @brief Retain the existing iBUS calibration helper signature.
 *
 * @param s iBUS calibration statistics.
 * @param ch Zero-based channel index; invalid indices fail closed.
 * @param spanUs Observed span.
 * @param levels Clustered switch levels.
 * @param levelCount Number of clustered levels.
 * @return Provisional channel classification.
 */
static inline ChClass classify_channel(const Stats &s, int ch, int spanUs, const int *levels, std::uint8_t levelCount)
{
    return detail::classify_channel(s, ch, spanUs, levels, levelCount);
}


namespace detail
{
/**
 * @brief Bound a provisional deadband so it cannot consume either axis side.
 *
 * @param jitter Observed peak small delta.
 * @param fallback Caller-selected fallback deadband.
 * @param useJitter Derive a candidate from jitter when true.
 * @param lo Observed minimum.
 * @param hi Observed maximum.
 * @param center Provisional centre respecting minimum side spans.
 * @return Nonnegative deadband smaller than both active sides, capped at 40 for jitter mode.
 */
static inline int suggested_deadband(int jitter, int fallback, bool useJitter, int lo, int hi, int center)
{
    const int left = center - lo, right = hi - center;
    const int side = left < right ? left : right;
    const int candidate = useJitter ? clampi(jitter > 0 ? jitter + 5 : fallback, 5, 40) : fallback;
    return clampi(candidate, 0, side > 0 ? side - 1 : 0);
}

} // namespace detail

namespace detail
{
/**
 * @brief Accumulate one sane sample with saturating statistics and checked bounds.
 *
 * @tparam Capacity Maximum analogue channels.
 * @param s Session-owned statistics.
 * @param chIdx Zero-based channel; out-of-range indices are ignored.
 * @param raw Converted raw observation; out-of-range values are ignored.
 * @param motionThreshUs Threshold for updating the last motion value.
 */
template <int Capacity>
inline void add_sample(CalibrationStats<Capacity> &s, int chIdx, int raw, int motionThreshUs)
{
    if (chIdx < 0 || chIdx >= Capacity) return;
    if (raw < kAxisMinSane || raw > kAxisMaxSane)
        return;

    if (s.samples[chIdx] != UINT32_MAX)
        s.samples[chIdx]++;
    if (raw < s.minv[chIdx])
        s.minv[chIdx] = raw;
    if (raw > s.maxv[chIdx])
        s.maxv[chIdx] = raw;

    // Jitter + last.
    if (s.last[chIdx] == -99999)
    {
        s.last[chIdx] = raw;
    }
    else
    {
        int d = raw - s.last[chIdx];
        if (d < 0)
            d = -d;
        if (d >= motionThreshUs)
            s.last[chIdx] = raw;
        if (d < 20 && d > s.jitter_peak[chIdx])
            s.jitter_peak[chIdx] = d;
    }

    // Histogram.
    const int bi = bin_index(raw);
    if (s.hist[chIdx][bi] != UINT16_MAX)
        s.hist[chIdx][bi]++;

    // Cluster into discrete levels (for switches).
    std::uint8_t &cnt = s.lcnt[chIdx];
    for (std::uint8_t i = 0; i < cnt; ++i)
    {
        if (abs_i(raw - s.levels[chIdx][i]) <= kClusterTolUs)
        {
            // Gentle EMA center.
            s.levels[chIdx][i] = (s.levels[chIdx][i] * 7 + raw) / 8;
            if (s.occ[chIdx][i] != UINT16_MAX)
                s.occ[chIdx][i]++;
            return;
        }
    }
    if (cnt < CalibrationStats<Capacity>::L)
    {
        s.levels[chIdx][cnt] = raw;
        s.occ[chIdx][cnt] = 1;
        cnt++;
    }
}

/**
 * @brief Classify only recommendations compatible with the default switch contract.
 *
 * The public classifier remains unchanged. EMA cluster drift can leave sorted
 * levels closer than RcSwitchSpec permits; omit that uncertain recommendation in
 * every text/JSON/map pass rather than silently changing separation defaults.
 *
 * @tparam Statistics Shared-schema statistics with fixed channel capacity.
 * @param s Session statistics.
 * @param ch Zero-based channel index.
 * @param span Observed raw span.
 * @param levels Sorted surviving cluster levels.
 * @param count Number of levels.
 * @return Provisional class, or NoInput when spacing is ambiguous/invalid.
 */
template <typename Statistics>
static ChClass recommendation_class(const Statistics &s, int ch, int span, const int *levels, std::uint8_t count)
{
    const ChClass cc = classify_channel(s, ch, span, levels, count);
    if (cc != ChClass::NoInput && cc != ChClass::Axis)
    {
        const int minSpacing = RcSwitchSpec{}.min_sep_us;
        for (std::uint8_t i = 1u; i < count; ++i)
            if (levels[i] - levels[i - 1u] < minSpacing)
                return ChClass::NoInput;
    }
    return cc;
}

/**
 * @brief Run shared sampling and suggestions with protocol-owned acceptance.
 *
 * @tparam Capacity Maximum analogue channels.
 * @tparam Transport Existing RCLink receiver transport.
 * @tparam Poll Callable that updates the transport and accepts only usable new batches.
 * @param bus Initialized transport supplying converted raw values.
 * @param poll Protocol-specific acquisition/acceptance callable.
 * @param protocol Banner protocol name.
 * @param guidance Short protocol wiring and validity instructions.
 * @param maxChannels Requested channel count, clamped to Capacity.
 * @param motionThreshUs Jitter motion threshold in converted raw units.
 * @param defaultDeadbandUs Fallback suggested deadband.
 * @param useJitterDeadband Derive suggested deadband from jitter when true.
 */
template <int Capacity, typename Transport, typename Poll>
inline void run_calibration(Transport &bus, Poll poll, const char *protocol, const char *guidance,
                            std::uint8_t maxChannels, int motionThreshUs, int defaultDeadbandUs,
                            bool useJitterDeadband)
{
    using SessionStats = CalibrationStats<Capacity>;
    // Init stats.
    SessionStats s{};
    for (int i = 0; i < SessionStats::K; ++i)
    {
        s.minv[i] = 32767;
        s.maxv[i] = -32768;
        s.last[i] = -99999;
        s.lcnt[i] = 0;
        s.samples[i] = 0;
        s.jitter_peak[i] = 0;
        for (int k = 0; k < SessionStats::L; ++k)
        {
            s.levels[i][k] = 0;
            s.occ[i][k] = 0;
        }
        for (int b = 0; b < SessionStats::kHistBins; ++b)
            s.hist[i][b] = 0;
    }

    delay(5000); ///< Allow time for serial monitor to start.

    // Banners (press key to begin).
    Serial.printf("\n--------------------------------- RC Calibrate (%s) -------------------------------\n", protocol);
    Serial.println(guidance);
    Serial.println(F("• Ensure your RC is on."));
    Serial.println(F("• Move sticks slowly to all extremes and let them return to the center."));
    Serial.println(F("• Turn all knobs and flip all switches—order is not important."));
    Serial.println(F("• Cover all channels; multiple slow movements improve accuracy."));
    Serial.println(F("• Press ANY key to BEGIN sampling. Press ANY key again to STOP.\n"));
    Serial.println(F("-------------------------------------------------------------------------------------\n"));
    Serial.print(F("Waiting for key to begin... "));
    while (!Serial.available())
    {
        (void)poll(static_cast<std::uint32_t>(millis()));
        delay(10);
    }
    while (Serial.available())
        (void)Serial.read();
    Serial.println(F("Go!"));
    Serial.println(F("\nLive converted raw readings (CH labels are one-based; mappings are zero-based)."));
    Serial.println(F("Only newly accepted healthy batches enter statistics. Cached readings may be stale."));

    // Main loop (press any key to stop).
    std::uint32_t last_print_ms = 0;
    char live[256]{};
    std::size_t live_size = 0u, live_sent = 0u;
    while (true)
    {
        if (Serial.available())
        {
            while (Serial.available())
                (void)Serial.read();
            break;
        }

        const bool new_frame = poll(static_cast<std::uint32_t>(millis()));

        const int chMax = (maxChannels <= SessionStats::K) ? maxChannels : SessionStats::K;
        for (int ch = 0; ch < chMax; ++ch)
        {
            const int v = bus.readRaw(ch); // ≤ kNoSignalThresholdUs => “no signal”.
            // update() reports a new valid decoded batch; readRaw() alone can be stale.
            if (new_frame && v > kNoSignalThresholdUs)
                add_sample(s, ch, v, motionThreshUs);
        }

        // One pending live line; receive first, then transmit only available TX
        // space in small chunks. Skipping a busy display never skips sampling.
        const std::uint32_t now = millis();
        if (now - last_print_ms >= 250u && live_sent == live_size && chMax > 0)
        {
            last_print_ms = now;
            live_size = live_sent = 0u;
            const int prefix = snprintf(live, sizeof(live), "Cached raw: ");
            if (prefix > 0) live_size = static_cast<std::size_t>(prefix);
            for (int i = 0; i < chMax; ++i)
            {
                const int raw = (s.last[i] == -99999) ? 0 : s.last[i];
                const int n = snprintf(live + live_size, sizeof(live) - live_size,
                                       "CH%d=%d%s", i + 1, raw, i + 1 == chMax ? "\n" : " ");
                if (n < 0 || static_cast<std::size_t>(n) >= sizeof(live) - live_size)
                {
                    live_size = 0u;
                    break;
                }
                live_size += static_cast<std::size_t>(n);
            }
        }
        const int space = Serial.availableForWrite();
        if (space > 0 && live_sent < live_size)
        {
            std::size_t n = live_size - live_sent;
            if (n > 32u) n = 32u;
            if (n > static_cast<std::size_t>(space)) n = static_cast<std::size_t>(space);
            live_sent += Serial.write(reinterpret_cast<const std::uint8_t *>(live + live_sent), n);
        }
        delay(2);
    }

    bool any_samples = false;
    for (int ch = 0; ch < SessionStats::K; ++ch)
        any_samples = any_samples || s.samples[ch] != 0u;
    if (!any_samples)
        Serial.println(F("No valid calibration samples received; check the receiver and repeat the session."));
    // Summary (raw).
    Serial.println(F("\n----------------------------------- Summary (raw) -----------------------------------\n"));
    Serial.println(F("CH | Min(raw) | Max(raw) | Span(raw) | Clusters | Samples"));
    Serial.println(F("---|----------|----------|--------"));
    for (int i = 0; i < SessionStats::K; ++i)
    {
        const int rawMin = (s.minv[i] == 32767) ? 0 : s.minv[i];
        const int rawMax = (s.maxv[i] == -32768) ? 0 : s.maxv[i];
        const int span = (rawMin && rawMax) ? (rawMax - rawMin) : 0;
        // Printing cast only: avoids using unsigned long as a storage type.
        Serial.printf("CH%d | %8d | %8d | %8d | %8u | %7lu\n", i + 1, rawMin, rawMax, span, static_cast<unsigned>(s.lcnt[i]),
                      static_cast<unsigned long>(s.samples[i]));
    }

    Serial.println(F("Centres are provisional dwell-bin estimates or inferred midpoints, not measured mechanical centres."));
    Serial.println(F("Verify every endpoint/neutral and complete travel; sparse sampling can misclassify controls. Fewer than 8 samples are omitted."));
    // Build suggestions.
    Serial.println(F("\n------------------------------------ Suggestions ------------------------------------\n"));
    for (int i = 0; i < SessionStats::K; ++i)
    {
        if (s.samples[i] < static_cast<std::uint32_t>(kMinLevelDwell))
            continue;

        // Prune weak clusters (low dwell).
        int lv[SessionStats::L];
        std::uint16_t oc[SessionStats::L];
        std::uint8_t n = 0;
        for (std::uint8_t k = 0; k < s.lcnt[i]; ++k)
        {
            if (s.occ[i][k] >= kMinLevelDwell)
            {
                lv[n] = s.levels[i][k];
                oc[n] = s.occ[i][k];
                ++n;
            }
        }
        if (n == 0 && s.lcnt[i] > 0)
        {
            lv[0] = s.levels[i][0];
            oc[0] = s.occ[i][0];
            n = 1;
        }
        sort_levels(lv, oc, n);

        const int rawMin = clampi(s.minv[i], kAxisMinSane, kAxisMaxSane);
        const int rawMax = clampi(s.maxv[i], kAxisMinSane, kAxisMaxSane);
        const int span = rawMax - rawMin;
        const int rawCenter = estimate_axis_center(s, i, rawMin, rawMax);

        const ChClass cc = recommendation_class(s, i, span, lv, n);

        if (cc == ChClass::NoInput)
        {
            Serial.printf("// CH%-2d : Insufficient movement/dwell or ambiguous switch spacing (span ≈ %d raw) -> Skipping; repeat traversal.\n\n", i + 1, span);
            continue;
        }

        if (cc == ChClass::Axis)
        {
            const int suggestedDeadband = suggested_deadband(s.jitter_peak[i], defaultDeadbandUs,
                useJitterDeadband, rawMin, rawMax, rawCenter);
            Serial.printf("// CH%d (Axis) : Range ≈ %d..%d raw (centre estimate ≈ %d; verify physical neutral)\n", i + 1, rawMin, rawMax, rawCenter);
            Serial.printf("cfg.axis(/*Role*/).raw(%d,%d,%d).out(-100,100).deadband_us(%d).expo(0.00f).done();\n\n",
                          rawMin, rawMax, rawCenter, suggestedDeadband);
        }
        else
        {
            const std::uint8_t positions = (cc == ChClass::Switch2) ? 2 : (cc == ChClass::Switch3) ? 3 : n;
            const std::uint8_t useN = positions ? positions : (n ? n : 2);

            Serial.printf("// CH%d (Switch - %u pos): Levels ≈ { ", i + 1, useN);
            for (std::uint8_t k = 0; k < useN && k < n; ++k)
            {
                Serial.print(lv[k]);
                if (k + 1 != useN)
                    Serial.print(F(", "));
            }
            Serial.println(F(" } raw"));

            Serial.printf("cfg.sw(/*Role*/).raw_levels({");
            for (std::uint8_t k = 0; k < useN && k < n; ++k)
            {
                Serial.print(lv[k]);
                if (k + 1 != useN)
                    Serial.print(F(", "));
            }
            Serial.print(F("}).values({"));
            for (std::uint8_t k = 0; k < useN && k < n; ++k)
            {
                Serial.print(k);
                if (k + 1 != useN)
                    Serial.print(F(","));
            }
            Serial.println(F("}).done();\n"));
        }
    }

    // JSON suggestion.
    Serial.println(F("---------------------------- JSON suggestion (optional) -----------------------------\n"));
    Serial.println(F("{"));

    // ---- Map ---- //

    Serial.println(F("  \"map\": {"));
    {
        bool firstEntry = true;
        for (int i = 0; i < SessionStats::K; ++i)
        {
            if (s.samples[i] < static_cast<std::uint32_t>(kMinLevelDwell))
                continue;

            // Recompute/prune clusters and decide if channel is in use.
            int lv[SessionStats::L];
            std::uint16_t oc[SessionStats::L];
            std::uint8_t n = 0;
            for (std::uint8_t k = 0; k < s.lcnt[i]; ++k)
                if (s.occ[i][k] >= kMinLevelDwell)
                {
                    lv[n] = s.levels[i][k];
                    oc[n] = s.occ[i][k];
                    ++n;
                }
            if (n == 0 && s.lcnt[i] > 0)
            {
                lv[0] = s.levels[i][0];
                oc[0] = s.occ[i][0];
                n = 1;
            }
            sort_levels(lv, oc, n);

            const int mn = clampi(s.minv[i], kAxisMinSane, kAxisMaxSane);
            const int mx = clampi(s.maxv[i], kAxisMinSane, kAxisMaxSane);
            const int sp = mx - mn;

            const ChClass cc = recommendation_class(s, i, sp, lv, n);
            if (cc == ChClass::NoInput)
                continue;

            if (!firstEntry)
                Serial.println(F(","));
            firstEntry = false;
            // Keys are "Ch1".."ChN" by convention, values are the receiver channel index.
            Serial.printf("    \"Ch%d\": %d", i + 1, i);
        }
        Serial.println();
    }
    Serial.println(F("  },"));

    // ---- Axes ---- //

    Serial.println(F("  \"axes\": {"));
    {
        bool firstEntry = true;
        for (int i = 0; i < SessionStats::K; ++i)
        {
            if (s.samples[i] < static_cast<std::uint32_t>(kMinLevelDwell))
                continue;

            // Prune/sort clusters.
            int lv[SessionStats::L];
            std::uint16_t oc[SessionStats::L];
            std::uint8_t n = 0;
            for (std::uint8_t k = 0; k < s.lcnt[i]; ++k)
                if (s.occ[i][k] >= kMinLevelDwell)
                {
                    lv[n] = s.levels[i][k];
                    oc[n] = s.occ[i][k];
                    ++n;
                }
            if (n == 0 && s.lcnt[i] > 0)
            {
                lv[0] = s.levels[i][0];
                oc[0] = s.occ[i][0];
                n = 1;
            }
            sort_levels(lv, oc, n);

            const int rawMin = clampi(s.minv[i], kAxisMinSane, kAxisMaxSane);
            const int rawMax = clampi(s.maxv[i], kAxisMinSane, kAxisMaxSane);
            const int rawCenter = estimate_axis_center(s, i, rawMin, rawMax);
            const int span = rawMax - rawMin;

            const ChClass cc = recommendation_class(s, i, span, lv, n);
            if (cc != ChClass::Axis)
                continue;

            const int db = suggested_deadband(s.jitter_peak[i], defaultDeadbandUs,
                useJitterDeadband, rawMin, rawMax, rawCenter);

            if (!firstEntry)
                Serial.println(F(","));
            firstEntry = false;

            // Default output range for axes: [-100,100].
            Serial.printf(
                "    \"Ch%d\": { \"raw\": [%d,%d,%d], \"deadband_us\": %d, \"out\": [-100,100], \"expo\": 0.00 }",
                i + 1, rawMin, rawMax, rawCenter, db);
        }
        Serial.println();
    }
    Serial.println(F("  },"));

    // ---- Switches ---- //

    Serial.println(F("  \"switches\": {"));
    {
        bool firstEntry = true;
        for (int i = 0; i < SessionStats::K; ++i)
        {
            if (s.samples[i] < static_cast<std::uint32_t>(kMinLevelDwell))
                continue;

            // Prune/sort clusters.
            int lv[SessionStats::L];
            std::uint16_t oc[SessionStats::L];
            std::uint8_t n = 0;
            for (std::uint8_t k = 0; k < s.lcnt[i]; ++k)
                if (s.occ[i][k] >= kMinLevelDwell)
                {
                    lv[n] = s.levels[i][k];
                    oc[n] = s.occ[i][k];
                    ++n;
                }
            if (n == 0 && s.lcnt[i] > 0)
            {
                lv[0] = s.levels[i][0];
                oc[0] = s.occ[i][0];
                n = 1;
            }
            sort_levels(lv, oc, n);

            const int rawMin = clampi(s.minv[i], kAxisMinSane, kAxisMaxSane);
            const int rawMax = clampi(s.maxv[i], kAxisMinSane, kAxisMaxSane);
            const int span = rawMax - rawMin;
            const ChClass cc = recommendation_class(s, i, span, lv, n);

            if (cc == ChClass::NoInput || cc == ChClass::Axis)
                continue;

            const std::uint8_t useN = n ? n : 2;

            if (!firstEntry)
                Serial.println(F(","));
            firstEntry = false;

            Serial.printf("    \"Ch%d\": { \"raw_levels\": [", i + 1);
            for (std::uint8_t k = 0; k < useN && k < n; ++k)
            {
                Serial.print(lv[k]);
                if (k + 1 != useN)
                    Serial.print(F(","));
            }
            Serial.print(F("], \"values\": ["));
            for (std::uint8_t k = 0; k < useN && k < n; ++k)
            {
                Serial.print(k);
                if (k + 1 != useN)
                    Serial.print(F(","));
            }
            Serial.print(F("] }"));
        }
        Serial.println();
    }
    Serial.println(F("  }"));
    Serial.println(F("}"));
    Serial.println(F("\n-------------------------------------------------------------------------------------\n"));
    Serial.println(F("Calibration finished. Before use, verify every control's true zero/neutral and endpoints; a non-spring throttle may use its low endpoint, not the estimated centre.\n\n"));
}

} // namespace detail
} // namespace calibrate
} // namespace rc
