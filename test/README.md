# RCLink tests

RCLink separates hardware-neutral mapping/parser tests from target compile tests.

## Local native tests

Run `./test/run_host_checks.sh` for the complete local host suite. It runs strict native tests with available host compilers, sanitizer coverage, ordinary development-tree contracts, and a clean staged-distribution check with protected-file rejection probes. Run an individual script when you only need one part.

The native suite covers asymmetric axis centres, one-sided inversion rejection/reversed output, acceptable-value HoldLast history, invalid/missing roles, timestamp rollover, frame-sequence ownership, failsafe signatures, switch commissioning evidence, parser recovery, diagnostics, and tagged-v1.1 compatibility with its global-alias opt-out.

## Target compile matrix

GitHub Actions uses PlatformIO to compile `test/portable_compile/portable_compile.ino` across ESP32-S3, ESP8266, RP2040, SAMD, SAM/Due, Teensy 4.1, and STM32. AVR and MegaAVR are not supported targets. The public ESP32-S3 examples are compiled separately because they intentionally use `Serial2` and ESP32-oriented wiring.

Native tests validate logic. Target compiles validate toolchain/API compatibility. Physical receiver tests are still required before a release is considered hardware-validated.


## Corrective regressions

`test_corrective.cpp` accepts F01, F02, F03, F06, F07 or F08 to run one group;
no argument runs all groups. `test_json.cpp` and `test_calibration.cpp` cover the
optional helpers. Run them with `bash test/run_helper_tests.sh`; set
`ARDUINOJSON_INCLUDE` to an already installed ArduinoJson 7.x `src` directory if
it is not in the default local PlatformIO dependency directory. Missing headers
produce NOT_RUN with exit 2, never an implicit dependency installation.
`BUILD` can direct generated binaries outside the repository. On macOS, use
`SDKROOT=$(xcrun --show-sdk-path)` if Clang cannot locate its standard headers.
Sanitized native and helper executables enable leak detection on Linux by default;
macOS disables leak detection because its runtime does not support it. `ASAN_OPTIONS`
can explicitly override this default. ASan/UBSan and leak detection are separate
capabilities; a macOS sanitizer pass does not verify Linux LeakSanitizer.
For a real GCC run on macOS, set `GCC_CXX` to the installed versioned executable;
Apple's `g++` command may be Clang.

| Finding | Root cause and permanent regression |
| --- | --- |
| F01 | Switch selection mutated before frame acceptance. `test_corrective.cpp` F01 covers protocol failsafe, lost frames, missing/invalid required roles, shared-channel validity, signature candidates/confirmation, learning, HoldLast, EMA and recovery. |
| F02 | Hysteresis compared distances from a centroid. F02 tests midpoint bands, both directions, noise, large transitions and unequal learned spacing. |
| F03 | Explicit levels matched an endpoint-clamped sample. F03 tests reachable levels below/inside/above defaults, nearest-only boundaries and rejected invalid/plausibility configurations. |
| F04 | ArduinoJson converted values before validating types. `test_json.cpp` checks malformed channels, arrays, fields, integer bounds, valid zero and 31, ignored unknown roles and atomic failure. |
| F05 / W05 | Calibrator reread cached values each poll; counters could wrap. `test_calibration.cpp` exercises actual mock-stream sampling, no input/disconnect, repeat packets, corruption/recovery, 65,548 packets, dwell saturation and classification precedence. The redundant classification branch is removed; the existing snap helper is retained for source compatibility. |
| F06 | Transport begin left receiver authorization stale. F06 checks initial/repeated begin, no-data periods, histories and recovery. |
| F07 | Transport update exposes a latest-value batch. F07 preserves one sequence increment per batch independently of wire-frame diagnostics, and tests single frames, idle polling and FPS. No behavioral correction or red assertion is appropriate. |
| F08 | Zero diagnostics were conditionally suppressed without a frame. F08 checks reapplication before/after frames and transport reset; sequence stays link-lifetime while diagnostics mirror transport lifetime. |

The new defect assertions fail against the original 1.3.0 source; F07 passes as a
contract-preservation check. The independent audit fixtures also exercise 2,400
decoder round trips, 100,000 noise bytes per decoder and 95,500 axis observations.
Those external programs are additional evidence, not a substitute for this suite.

The unchanged `test_rclink.cpp` retains HoldLast rejection history, counter-independent
sequencing and wrap, endpoint-inversion restrictions, asymmetric scaling, bounds-safe
mappings, missing channels, receiver signatures, parser corruption/recovery, coherent
shared-channel capture, opt-in plausibility, and switch-learning validity. Both
namespace compatibility fixtures remain active. Distribution probes reject private
controls and generated artifacts in a disposable public stage.

## Isolated target consumers

`build_platformio_consumer.py` runs an explicitly selected consumer build under the
protected runner with already provisioned prerequisites. The pioarduino ESP32-S3
post-link restore hook gets disposable copies of its two mutable build scripts;
shared SDK binaries stay read-only. Pico takes the exact separately provisioned
`pioasm-dummy` package. No dependency installation occurs during verification.


## Native S.BUS observability regressions

`test_sbus.cpp` exercises the actual ESP32 transport against a fixed-storage serial
fixture and an independent bit-by-bit frame packer. It checks every code 0–2047 at
all 16 positions, exact converted references, accepted footer variants, invalid reads,
pre-frame/reset state, differing malformed/partial payloads, stale reads, batched
lost/failsafe evidence, recovery and unchanged calibrated outputs. Global allocation
instrumentation confirms the fixture's receive/read/reset paths allocate nothing;
ASan/UBSan checks their lifetimes and bounds. Real UART framework startup allocations
are outside that host assertion. No tests or safety-negative expectations are removed.

Two confirmed diagnostic limitations motivated additive APIs: native codes were
previously discarded after conversion, and latest-frame flags could hide earlier
faults in a polling batch. Original behavior is retained; native copies and cumulative
fault counters provide the missing evidence. Existing consumers keep their raw,
role and latest-status semantics. A test requiring these new APIs cannot compile
against 1.3.1; this is an API addition, not a demonstrated old decoder defect. The
mock batch explicitly proves fault→healthy flag overwrite and retained counters.
Physical measurements independently agree with the unchanged conversion. No
production scaling/decoder correction, baseline red bug assertion or full platform
campaign is warranted by that evidence. Run native/sanitizer scripts as above.


Independent review identified a test gap: flags/counters alone did not prove native
readability while fault frames were current. A disposable mutant that copies native
codes only for healthy frames passed the earlier fixture. Distinct lost-only and
failsafe-only payloads now check every native/converted channel together with frame
identity/status, before the combined fault-to-healthy batch. The same mutant must
fail those assertions; the actual implementation must pass. No production correction
was needed, and the mutation never modifies repository source.


## Host S.BUS capture and release identity

`python3 -B test/test_capture_sbus.py` uses synthetic serial chunks, no hardware or
pyserial dependency. It checks quiet/full echo, exact complete CSV preservation,
commands/annotations across split records, EOF/partial/oversized/malformed input,
serial failures, missing dependencies, exclusive file creation and clean shutdown.
The previous capture echoed and flushed every line; tests explicitly prove that
quiet mode preserves the same records without telemetry on the console. Use
`python3 -B test/capture_sbus.py [--echo] PORT OUTPUT.txt` with `test/sbus_diagnostic/SbusDiagnostic.ino`, the opt-in advanced producer.
The native host checks and CI include the capture tests.

Release contracts enforce exact 1.4.0 strings and numeric macros, README identity,
changelog/checklist identity, API keywords and the six-example inventory. Disposable
public-stage negatives reject version drift and leaked private finalization reports,
while preserving all previous privacy and generated-file probes. Local matrix builds
use the seven environments in platformio.ini and six independent ESP32-S3 sketches.
Remote CI and publication are separate, not implied by successful local checks.


## Shared guided calibration

Examples 02/03 use the same private statistics/presentation engine with 14/16 fixed
channels. Public run_ibus and Stats remain available; run_sbus adds only protocol
initialization and batch fault filtering. Advanced CSV capture lives separately in
`test/sbus_diagnostic` and remains paired with capture_sbus.py and its seven tests.

The real-helper `test_sbus_calibration.cpp` checks both public helpers, human labels
versus zero-based JSON maps, all 16 channels, iBUS 14 bounds, no frames, incomplete and
bad frames, recovery, rejected fault-bearing batches (including superseded faults),
stale polls, repeated sessions, blocked display TX, timer rollover, saturating
statistics and semantically equal cross-protocol JSON. Generated switch/axis JSON
is parsed and validated using the real RCLink configuration API. The original iBUS
sample-count, parser, classification and saturation assertions remain active.

A reproduced narrow-axis recommendation defect came from coarse histogram centre
selection near an endpoint plus an overlarge deadband. Provisional centres now obey
the existing minimum side span, and suggested deadband stays below either side.
The focused 1500–1540 case validates after correction; its original suggestion is
rejected by the existing configuration validator. Valid wider-axis estimates retain
their established arithmetic. Centres are never claimed as measured mechanical
neutrals; insufficient (<8) observations are omitted from recommendations.


Independent review caught two additional cases before release qualification.
The original flat Stats accepted C++14 aggregate construction; an extraction using
inheritance rejected it. One private, immediately undefined field-schema macro now
preserves the original public struct/initialization and unique helper signatures
without duplicate schemas or runtime storage. calibration_compat.cpp protects it.
A real-helper 10/10/20 sequence at raw 1000/1210/1120 reproduces EMA cluster drift:
Switch2 levels become closer than the existing default minimum separation. One
shared emission predicate now omits that uncertain recommendation from text, map
and JSON, with repeat-traversal guidance; the public classifier is unchanged.
Both iBUS and S.BUS regression sessions verify the omission. No transport scaling
or receiver-specific defaults were changed.

The paired helper test also invokes the actual Example 02/03 setup() functions with
scheduled serial fixtures, checking console baud, accepted samples and generated
maps. This protects beginner workflow parity beyond filenames or sketch length.
