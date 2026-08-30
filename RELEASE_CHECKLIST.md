# RCLink release checklist

## Software gates
- [x] `./test/run_native_tests.sh` passes: 38 tests / 1252 assertions, plus v1.1 compatibility and opt-out compile fixtures.
- [x] `./test/run_sanitizers.sh` passes with ASan/UBSan. Leak detection is disabled on Darwin because this Apple Clang runtime does not support it.
- [x] `./test/run_host_checks.sh` passes.
- [x] `./test/check_release_contracts.sh` passes.
- [x] Local PlatformIO portable matrix passes for ESP32-S3, classic ESP32, ESP8266, RP2040, SAMD/MKR Zero, SAM/Due, Teensy 4.1 and STM32 Blue Pill.
- [ ] GitHub Actions portable target matrix passes remotely.
- [x] All five ESP32-S3 examples compile locally.
- [x] ArduinoJson dependency resolves through PlatformIO metadata as ArduinoJson 7.4.3.

## Hardware gates
- [ ] iBUS receiver tested on ESP32-S3 using the documented wiring
- [ ] Asymmetric stick calibration verified around physical neutral
- [ ] Receiver disconnect produces stale/failsafe output inside configured timeout
- [ ] Missing/truncated/noisy iBUS traffic recovers without application restart
- [ ] Receiver failsafe signature verified with the intended transmitter/receiver pair
- [ ] Switch positions verified from explicit raw levels for safety-relevant selectors
- [ ] SBUS tested on ESP32 when that transport is part of the release claim

## Package gates
- [x] `library.properties`, `library.json`, `RCLink.h` versions match.
- [x] Root `.clang-format`, LMB presentation checks, and representative style-rejection probes pass locally.
- [x] README beginner path and migration notes match the implementation.
- [x] CHANGELOG describes behavioural changes made for v1.2.0.
- [x] Examples progress from a complete normal-use iBUS link through calibration and advanced features.
- [x] Staged distribution contains no private control state, build/editor/OS debris or nested archives.
- [x] Protected/private-file rejection probes pass.
