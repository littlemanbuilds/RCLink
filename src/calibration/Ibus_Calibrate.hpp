/**
 * MIT License
 *
 * @brief Interactive iBUS calibrator (histogram + clustering -> config suggestions).
 *
 * @file Ibus_Calibrate.hpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-03
 * @copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <Constants.hpp>
#include <transport/Ibus.hpp>

namespace rc::calibrate
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

    /// @brief Small stats container.
    struct Stats
    {
        static constexpr int K = kIbusMaxChannels; ///< Maximum channels supported by iBUS.
        static constexpr int L = 8;                ///< Maximum clustered switch levels.

        // Per-channel running mins/max, last value, jitter.
        int minv[K]{};        ///< Minimum observed raw value per channel.
        int maxv[K]{};        ///< Maximum observed raw value per channel.
        int last[K]{};        ///< Last accepted raw value per channel.
        int jitter_peak[K]{}; ///< Peak small delta (jitter) per channel.

        // Clusters (for discrete switch levels).
        int levels[K][L]{};        ///< Cluster centers for switch levels.
        std::uint16_t occ[K][L]{}; ///< Dwell count per cluster.
        std::uint8_t lcnt[K]{};    ///< Number of clusters used per channel.

        // Histogram (coarse density to detect axes vs switches).
        static constexpr int kHistMin = kAxisMinSane;                        ///< Histogram minimum (µs).
        static constexpr int kHistMax = kAxisMaxSane;                        ///< Histogram maximum (µs).
        static constexpr int kHistBins = (kHistMax - kHistMin) / kBinUs + 1; ///< Number of bins.
        std::uint16_t hist[K][kHistBins]{};                                  ///< Histogram counts.

        // Sample counts.
        std::uint32_t samples[K]{}; ///< Total accepted samples per channel.
    };

    // ---- Helpers ---- //

    /**
     * @brief Clamp an integer to a specified range.
     * @param v Input value.
     * @param lo Minimum allowed value.
     * @param hi Maximum allowed value.
     * @return Clamped integer value.
     */
    static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

    /**
     * @brief Map a microsecond value into a histogram bin index.
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

    /**
     * @brief Heuristic check to classify a channel as an axis using histogram data.
     * @param s Statistics container holding histograms and min/max values.
     * @param ch Channel index to evaluate.
     * @param spanUs Observed span in microseconds for this channel.
     * @return true If the channel is likely an axis.
     * @return false Otherwise (too narrow, too few bins occupied, or low coverage).
     */
    static bool is_axis_by_hist(const Stats &s, int ch, int spanUs)
    {
        if (spanUs < kSpanNoneUs)
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

    /**
     * @brief Heuristic snap for canonical 2- or 3-position switches.
     * @param lv Array of detected level values (µs).
     * @param n Number of levels in lv (must be 2 or 3 to be considered).
     * @return ChClass::Switch2 if values look like a canonical 2-pos switch.
     * @return ChClass::Switch3 if values look like a canonical 3-pos switch.
     * @return ChClass::Axis otherwise.
     */
    static ChClass snap_2_or_3pos_if_canonical(const int *lv, std::uint8_t n)
    {
        if (n < 2 || n > 3)
            return ChClass::Axis;
        const auto near = [](int v, int target, int tol)
        { return std::abs(v - target) <= tol; };
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

    /**
     * @brief Classify a channel as Axis/Switch based on span, histogram, and clusters.
     * @param s Statistics container (histograms, min/max, etc.).
     * @param ch Channel index to evaluate.
     * @param spanUs Observed span in microseconds for this channel.
     * @param levels Pointer to clustered level centers (µs).
     * @param levelCount Number of clustered levels in levels.
     * @return ChClass::NoInput if span is too small.
     * @return ChClass::Axis if histogram density indicates a continuous axis or no switch match.
     * @return ChClass::Switch2 / ChClass::Switch3 / ChClass::SwitchN for discrete switches.
     */
    static ChClass classify_channel(const Stats &s,
                                    int ch,
                                    int spanUs,
                                    const int *levels,
                                    std::uint8_t levelCount)
    {
        if (spanUs < kSpanNoneUs)
            return ChClass::NoInput;
        if (is_axis_by_hist(s, ch, spanUs))
            return ChClass::Axis;

        if (levelCount == 2)
            return ChClass::Switch2;
        if (levelCount == 3)
            return ChClass::Switch3;
        if (levelCount >= 4 && levelCount <= kSwitchMaxLevels && spanUs <= kSpanSwitchMaxUs)
            return ChClass::SwitchN;

        if (levelCount == 2 || levelCount == 3)
        {
            const ChClass snap = snap_2_or_3pos_if_canonical(levels, levelCount);
            if (snap != ChClass::Axis)
                return snap;
        }
        return ChClass::Axis;
    }

    /**
     * @brief Interactive iBUS calibrator: prints live data and suggested config/JSON.
     * @param uart Hardware serial used for iBUS.
     * @param rxPin RX pin.
     * @param txPin TX pin (ignored).
     * @param baud UART baud rate (e.g., 115200).
     * @param maxChannels Maximum channels to sample (≤ kIbusMaxChannels).
     * @param motionThreshUs Delta in µs to treat as motion (for jitter).
     * @param defaultDeadbandUs Default deadband if jitter-based disabled.
     * @param useJitterDeadband Whether to derive deadband from observed jitter.
     */
    inline void run_ibus(::HardwareSerial &uart,
                         int rxPin, int txPin, std::uint32_t baud,
                         std::uint8_t maxChannels = kIbusMaxChannels,
                         int motionThreshUs = 8,
                         int defaultDeadbandUs = 20,
                         bool useJitterDeadband = true)
    {
        RcIbusTransport bus;

        // Init stats.
        Stats s{};
        for (int i = 0; i < Stats::K; ++i)
        {
            s.minv[i] = 32767;
            s.maxv[i] = -32768;
            s.last[i] = -99999;
            s.lcnt[i] = 0;
            s.samples[i] = 0;
            s.jitter_peak[i] = 0;
            for (int k = 0; k < Stats::L; ++k)
            {
                s.levels[i][k] = 0;
                s.occ[i][k] = 0;
            }
            for (int b = 0; b < Stats::kHistBins; ++b)
                s.hist[i][b] = 0;
        }

        // UART & transport.
        pinMode(rxPin, INPUT); // Pull configuration left to the user.
        bus.begin(uart, baud, rxPin, txPin);
        delay(5000); ///< Allow time for serial monitor to start.

        // Banners (press key to begin).
        Serial.println(F("\n--------------------------------- RC Calibrate (iBUS) -------------------------------\n"));
        Serial.println(F("• Ensure your RC is on."));
        Serial.println(F("• Move sticks slowly to all extremes and let them return to the center."));
        Serial.println(F("• Turn all knobs and flip all switches—order is not important."));
        Serial.println(F("• Cover all channels; multiple slow movements improve accuracy."));
        Serial.println(F("• Press ANY key to BEGIN sampling. Press ANY key again to STOP.\n"));
        Serial.println(F("-------------------------------------------------------------------------------------\n"));
        Serial.print(F("Waiting for key to begin... "));
        while (!Serial.available())
        {
            (void)bus.update();
            delay(10);
        }
        while (Serial.available())
            (void)Serial.read();
        Serial.println(F("Go!"));
        Serial.println(F("\nLive readings (µs): \n"));

        auto addSample = [&](std::uint8_t chIdx, int raw)
        {
            if (raw < kAxisMinSane || raw > kAxisMaxSane)
                return;

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
            s.hist[chIdx][bi]++;

            // Cluster into discrete levels (for switches).
            std::uint8_t &cnt = s.lcnt[chIdx];
            for (std::uint8_t i = 0; i < cnt; ++i)
            {
                if (std::abs(raw - s.levels[chIdx][i]) <= kClusterTolUs)
                {
                    // Gentle EMA center.
                    s.levels[chIdx][i] = (s.levels[chIdx][i] * 7 + raw) / 8;
                    s.occ[chIdx][i]++;
                    return;
                }
            }
            if (cnt < Stats::L)
            {
                s.levels[chIdx][cnt] = raw;
                s.occ[chIdx][cnt] = 1;
                cnt++;
            }
        };

        // Main loop (press any key to stop).
        std::uint32_t last_print_ms = 0;
        while (true)
        {
            if (Serial.available())
            {
                while (Serial.available())
                    (void)Serial.read();
                break;
            }

            (void)bus.update();

            const int chMax = (maxChannels <= Stats::K) ? maxChannels : Stats::K;
            for (int ch = 0; ch < chMax; ++ch)
            {
                const int v = bus.readRaw(ch); // ≤ kNoSignalThresholdUs => “no signal”.
                if (v > kNoSignalThresholdUs)
                    addSample(static_cast<std::uint8_t>(ch), v);
            }

            // Live print ~4 Hz.
            const std::uint32_t now = millis();
            if (now - last_print_ms >= 250u)
            {
                last_print_ms = now;
                Serial.print(F("µs: "));
                for (int i = 0; i < chMax; ++i)
                {
                    const int raw = (s.last[i] == -99999) ? 0 : s.last[i];
                    Serial.print(raw);
                    if (i + 1 != chMax)
                        Serial.print(F(", "));
                }
                Serial.println();
            }
            delay(2);
        }

        // Summary (raw).
        Serial.println(F("\n----------------------------------- Summary (raw) -----------------------------------\n"));
        Serial.println(F("Ch | Span(µs) | Clusters | Samples"));
        Serial.println(F("---|----------|----------|--------"));
        for (int i = 0; i < Stats::K; ++i)
        {
            const int rawMin = (s.minv[i] == 32767) ? 0 : s.minv[i];
            const int rawMax = (s.maxv[i] == -32768) ? 0 : s.maxv[i];
            const int span = (rawMin && rawMax) ? (rawMax - rawMin) : 0;
            // Printing cast only: avoids using unsigned long as a storage type.
            Serial.printf("%2d | %8d | %8u | %7lu\n",
                          i, span,
                          static_cast<unsigned>(s.lcnt[i]),
                          static_cast<unsigned long>(s.samples[i]));
        }

        // Build suggestions.
        Serial.println(F("\n------------------------------------ Suggestions ------------------------------------\n"));
        for (int i = 0; i < Stats::K; ++i)
        {
            if (s.samples[i] == 0)
                continue;

            // Prune weak clusters (low dwell).
            int lv[Stats::L];
            std::uint16_t oc[Stats::L];
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
            const int rawCenter = (rawMin + rawMax) / 2;

            const ChClass cc = classify_channel(s, i, span, lv, n);

            if (cc == ChClass::NoInput)
            {
                Serial.printf("// Ch_%-2d : No input detected (span ≈ %d µs) -> Skipping.\n\n", i, span);
                continue;
            }

            if (cc == ChClass::Axis)
            {
                int suggestedDeadband = defaultDeadbandUs;
                if (useJitterDeadband)
                {
                    suggestedDeadband = s.jitter_peak[i] > 0 ? (s.jitter_peak[i] + 5) : defaultDeadbandUs;
                    if (suggestedDeadband < 5)
                        suggestedDeadband = 5;
                    if (suggestedDeadband > 40)
                        suggestedDeadband = 40;
                }
                Serial.printf("// Ch_%d (Axis) : Range ≈ %d..%d µs (center ≈ %d)\n",
                              i, rawMin, rawMax, rawCenter);
                Serial.printf("cfg.axis(/*Role*/).raw(%d,%d,%d).out(-100,100).deadband_us(%d).expo(0.00f).done();\n\n",
                              rawMin, rawMax, rawCenter, suggestedDeadband);
            }
            else
            {
                const std::uint8_t positions = (cc == ChClass::Switch2)   ? 2
                                               : (cc == ChClass::Switch3) ? 3
                                                                          : n;
                const std::uint8_t useN = positions ? positions : (n ? n : 2);

                Serial.printf("// Ch_%d (Switch - %u pos): Levels ≈ { ", i, useN);
                for (std::uint8_t k = 0; k < useN && k < n; ++k)
                {
                    Serial.print(lv[k]);
                    if (k + 1 != useN)
                        Serial.print(F(", "));
                }
                Serial.println(F(" } µs"));

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
            for (int i = 0; i < Stats::K; ++i)
            {
                if (s.samples[i] == 0)
                    continue;

                // Recompute/prune clusters and decide if channel is in use.
                int lv[Stats::L];
                std::uint16_t oc[Stats::L];
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

                const ChClass cc = classify_channel(s, i, sp, lv, n);
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
            for (int i = 0; i < Stats::K; ++i)
            {
                if (s.samples[i] == 0)
                    continue;

                // Prune/sort clusters.
                int lv[Stats::L];
                std::uint16_t oc[Stats::L];
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
                const int rawCenter = (rawMin + rawMax) / 2;
                const int span = rawMax - rawMin;

                const ChClass cc = classify_channel(s, i, span, lv, n);
                if (cc != ChClass::Axis)
                    continue;

                int db = defaultDeadbandUs;
                if (useJitterDeadband)
                {
                    db = s.jitter_peak[i] > 0 ? (s.jitter_peak[i] + 5) : defaultDeadbandUs;
                    if (db < 5)
                        db = 5;
                    if (db > 40)
                        db = 40;
                }

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
            for (int i = 0; i < Stats::K; ++i)
            {
                if (s.samples[i] == 0)
                    continue;

                // Prune/sort clusters.
                int lv[Stats::L];
                std::uint16_t oc[Stats::L];
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
                const ChClass cc = classify_channel(s, i, span, lv, n);

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
        Serial.println(F("Calibration finished. Copy the suggestion above into your config.\n\n"));
    }

} ///< namespace rc::calibrate.