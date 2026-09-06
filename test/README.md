# RCLink tests

RCLink separates hardware-neutral mapping/parser tests from target compile tests.

## Local native tests

Run `./test/run_host_checks.sh` for the complete local host suite. It runs strict native tests with available host compilers, sanitizer coverage, ordinary development-tree contracts, and a clean staged-distribution check with protected-file rejection probes. Run an individual script when you only need one part.

The native suite covers asymmetric axis centres, one-sided inversion rejection/reversed output, acceptable-value HoldLast history, invalid/missing roles, timestamp rollover, frame-sequence ownership, failsafe signatures, switch commissioning evidence, parser recovery, diagnostics, and tagged-v1.1 compatibility with its global-alias opt-out.

## Target compile matrix

GitHub Actions uses PlatformIO to compile `test/portable_compile/portable_compile.ino` across ESP32-S3, ESP8266, RP2040, SAMD, SAM/Due, Teensy 4.1, and STM32. AVR and MegaAVR are not supported targets. The public ESP32-S3 examples are compiled separately because they intentionally use `Serial2` and ESP32-oriented wiring.

Native tests validate logic. Target compiles validate toolchain/API compatibility. Physical receiver tests are still required before a release is considered hardware-validated.

