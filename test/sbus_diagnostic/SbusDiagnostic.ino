/**
 * MIT License
 *
 * @brief Observe native S.BUS codes, conversion, extrema and protocol evidence.
 *
 * Isolated ESP32-S3 N16R8 bench: receiver S.BUS -> GPIO18, common GND.
 * No receiver TX, actuators or storage. Serial monitor/capture: 115200 baud.
 * See test/sbus_diagnostic/README.md for capture instructions and limitations.
 *
 * @file SbusDiagnostic.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-11-17
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */
#define RCLINK_NO_LEGACY_GLOBAL_ALIASES // Use RCLink's namespace API, as in the other examples.
#include <RCLink.h>

#include <cstdarg>
#include <cstdio>

namespace {
using namespace rc;

// Read the transport directly: this utility measures receiver channels without
// assigning application roles or changing calibration. Public channel indices are
// zero-based (0–15); the display labels them CH1–CH16 for the operator.
RcSbusEsp32Transport receiver;
const int kRxPin = 18;
const std::uint32_t kTimeoutMs = 200;
portMUX_TYPE uartCounterMux = portMUX_INITIALIZER_UNLOCKED;
std::uint32_t uartOverflows = 0, uartErrors = 0;

// Historical extrema retain exact native codes and their converted raw values.
// Native is the original 11-bit code, not a pulse measurement. Converted values
// use RCLink's existing pulse-width-style convention, not universal endpoints.
// Configure role calibration later from measured converted endpoints/centre.
struct Extrema {
    bool seen{false};
    std::uint16_t low{0}, high{0};
    int rawLow{0}, rawHigh{0};
};
Extrema extrema[6];
bool hasFrame = false;
std::uint32_t lastCsv = 0, lastDisplay = 0, dropped = 0;
bool helpPending = true, summaryPending = false;
// One pending output record; receiver processing always precedes small TX chunks.
char output[2048];
std::size_t outputSize = 0, outputSent = 0;
bool formattingFailed = false;

void append(const char *format, ...) {
    if (formattingFailed) return;
    va_list args;
    va_start(args, format);
    const int count = vsnprintf(output + outputSize, sizeof(output) - outputSize, format, args);
    va_end(args);
    if (count < 0 || static_cast<std::size_t>(count) >= sizeof(output) - outputSize) {
        formattingFailed = true;
        return;
    }
    outputSize += static_cast<std::size_t>(count);
}

// A successful native read means a frame was accepted sometime, even if it is
// now stale or faulted. Freshness is independent of protocol/RF health. Unsigned
// differences handle millis() rollover within one timer cycle.
std::uint32_t age(std::uint32_t now) {
    return hasFrame ? now - receiver.lastGoodMs() : kInvalidFrameAgeMs;
}

// A diagnostic reference scale, deliberately unclamped and independent of roles.
// Retain floating precision until formatting; this is not a calibrated command.
double percent(std::uint16_t native) {
    return (static_cast<double>(native) - 992.0) * 200.0 / 1639.0;
}

void observe() {
    // Called only after a newly accepted update, so this sample is fresh. Exclude
    // lost/failsafe frames: receiver fallback positions would pollute calibration.
    // Current diagnostics still show those values. Earlier frames in a batch may
    // be superseded; these extrema do not claim every wire-frame observation.
    if (receiver.frameLost() || receiver.protoFailsafe()) return;
    for (int ch = 0; ch < 6; ++ch) {
        std::uint16_t native;
        if (!receiver.tryReadNative(ch, native)) continue;
        const int raw = receiver.readRaw(ch);
        auto &e = extrema[ch];
        if (!e.seen || native < e.low) { e.low = native; e.rawLow = raw; }
        if (!e.seen || native > e.high) { e.high = native; e.rawHigh = raw; }
        e.seen = true;
    }
}

void csv(std::uint32_t now) {
    // The framework error callback runs in another task. Copy its counters under
    // a short ESP32 critical section; formatting and TX run outside the lock.
    portENTER_CRITICAL(&uartCounterMux);
    const auto overflows = uartOverflows, errors = uartErrors;
    portEXIT_CRITICAL(&uartCounterMux);
    append("D,%lu,%lu", static_cast<unsigned long>(now), static_cast<unsigned long>(receiver.frames()));
    for (int ch = 0; ch < 6; ++ch) {
        std::uint16_t native = 0;
        if (receiver.tryReadNative(ch, native)) append(",%u,%d,%.4f", native, receiver.readRaw(ch), percent(native));
        else append(",,,");
    }
    // Current flags describe the latest frame. Cumulative counters count accepted
    // frames bearing each bit, not RF-loss incidents, and survive later healthy
    // frames in the same poll. They wrap unsigned; no application latch is added.
    append(",%lu,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
           static_cast<unsigned long>(age(now)), hasFrame,
           hasFrame && age(now) <= kTimeoutMs, receiver.frameLost(), receiver.protoFailsafe(),
           static_cast<unsigned long>(receiver.lostFrames()), static_cast<unsigned long>(receiver.failsafeFrames()),
           static_cast<unsigned long>(dropped), static_cast<unsigned long>(overflows),
           static_cast<unsigned long>(errors), static_cast<unsigned long>(receiver.crcErrors()),
           static_cast<unsigned long>(receiver.parserTimeouts()), static_cast<unsigned long>(receiver.discardedBytes()));
}

void display(std::uint32_t now, bool summary) {
    append("# %s t=%lu age=%lu fresh=%d latest_lost=%d latest_failsafe=%d\n",
           summary ? "SUMMARY" : "LIVE", static_cast<unsigned long>(now),
           static_cast<unsigned long>(age(now)), hasFrame && age(now) <= kTimeoutMs,
           receiver.frameLost(), receiver.protoFailsafe());
    append("# CH current_native converted signed%% | historical_native_min/max converted_min/max\n");
    for (int ch = 0; ch < 6; ++ch) {
        std::uint16_t native;
        if (receiver.tryReadNative(ch, native)) {
            append("# %d %u %d %.4f | ", ch + 1, native, receiver.readRaw(ch), percent(native));
            const auto &e = extrema[ch];
            if (e.seen) append("%u/%u %d/%d\n", e.low, e.high, e.rawLow, e.rawHigh);
            else append("unobserved\n");
        } else append("# %d unobserved\n", ch + 1);
    }
    if (summary) {
        append("# CH7-10 native/converted (current, not extrema):");
        for (int ch = 6; ch < 10; ++ch) {
            std::uint16_t native;
            if (receiver.tryReadNative(ch, native)) append(" CH%d=%u/%d", ch + 1, native, receiver.readRaw(ch));
            else append(" CH%d=unobserved", ch + 1);
        }
        append("\n");
    }
}

void help() {
    append("# RCLink S.BUS commissioning; r=reset extrema, s=summary incl CH7-10, h=help\n");
    append("# Native=11-bit code; converted=pulse-width-style units, not measured pulse width.\n");
    append("# Signed%%=(native-992)*200/1639; diagnostic only, unclamped, not an application command.\n");
    append("# Extrema: healthy latest-frame samples; CSV: latest accepted frame incl stale/fault data.\n");
    append("# Counts cover parsed accepted frames; 10Hz CSV is not continuous wire-frame capture.\n");
    append("# D,ms,frames,n1,c1,p1,n2,c2,p2,n3,c3,p3,n4,c4,p4,n5,c5,p5,n6,c6,p6,age_ms,has_frame,link_ok,lost,failsafe,lost_frames,failsafe_frames,log_drops,uart_overflows,uart_errors,footer_errors,parser_timeouts,discarded_bytes\n");
}
} // namespace

void setup() {
    Serial.begin(115200);
    // UART framework allocates its fixed buffers at startup, never in RCLink parsing.
    Serial2.setRxBufferSize(1024);
    receiver.begin(Serial2, 100000, kRxPin, -1);
    Serial2.onReceiveError([](hardwareSerial_error_t error) {
        portENTER_CRITICAL(&uartCounterMux);
        if (error == UART_BUFFER_FULL_ERROR || error == UART_FIFO_OVF_ERROR) ++uartOverflows;
        else ++uartErrors;
        portEXIT_CRITICAL(&uartCounterMux);
    });
}

void loop() {
    const auto now = static_cast<std::uint32_t>(millis());
    if (receiver.update(now)) { hasFrame = true; observe(); }
    // Bound command work too. One host process owns the serial port.
    for (int n = 0; n < 8 && Serial.available() > 0; ++n) {
        const int command = Serial.read();
        if (command == 'r') {
            for (auto &e : extrema) e = Extrema{};
            summaryPending = true;
        } else if (command == 's') summaryPending = true;
        else if (command == 'h') helpPending = true;
    }
    // A single pending batch avoids an unbounded queue. Keep unsent bytes intact;
    // omit scheduled batches when busy and report them in log_drops. r clears only
    // extrema; s/h remain pending until a batch can be formatted. Old pending text
    // keeps its original timestamp and must not be read as a new measurement.
    const bool csvDue = static_cast<std::uint32_t>(now - lastCsv) >= 100;
    const bool displayDue = static_cast<std::uint32_t>(now - lastDisplay) >= 200;
    if (csvDue) lastCsv = now;
    if (displayDue) lastDisplay = now;
    if (outputSent == outputSize) {
        outputSize = outputSent = 0;
        formattingFailed = false;
        if (helpPending) { help(); helpPending = false; }
        if (csvDue) csv(now);
        if (displayDue || summaryPending) { display(now, summaryPending); summaryPending = false; }
        if (formattingFailed) { ++dropped; outputSize = 0; }
    } else if (csvDue || displayDue) ++dropped;
    // Poll the receiver first, then write only a small chunk that already fits in
    // the UART TX buffer. No print/flush waits are added to receiver servicing.
    // The signed diagnostic percentage is not a calibrated application command.
    const int space = Serial.availableForWrite();
    if (space > 0 && outputSent < outputSize) {
        std::size_t count = outputSize - outputSent;
        if (count > 32) count = 32;
        if (count > static_cast<std::size_t>(space)) count = static_cast<std::size_t>(space);
        outputSent += Serial.write(reinterpret_cast<const std::uint8_t *>(output + outputSent), count);
    }
}
