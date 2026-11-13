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
