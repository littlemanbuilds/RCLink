# Changelog

All notable RCLink changes are documented here.

## [1.2.0] - 2026-08-30

### Fixed

- Axis scaling now honours `raw_center` with piecewise low/centre/high mapping, including asymmetric transmitter calibration and endpoint-centred one-sided controls.
- Re-applying a configuration now clears stale frame/validity state until a new frame is evaluated under the new mapping.
- Deadband is applied around the calibrated physical centre instead of an inferred midpoint.
- Unmapped roles use `kInvalidChannel` rather than silently reading receiver channel 0.
- Missing/out-of-range receiver channels no longer become plausible application values.
- Public role/index reads and configuration builders are bounds checked and fail closed.
- Oversized switch value/raw-level arrays are rejected instead of silently truncating to the internal maximum.
- Epsilon comparisons use 32-bit arithmetic, removing signed 16-bit overflow on large output changes.
- Receiver-failsafe signatures reject empty masks and use explicit matching state instead of timestamp zero as a sentinel.
- Frame age is explicitly invalid before the first accepted frame.
- Switch position selection is based on raw receiver geometry rather than application output labels.
- `HoldLast` now preserves distinct last acceptable application values across protocol failsafe, frame-lost, invalid-required-role, stale-link, and applied receiver-signature transitions.
- RCLink now owns `frame_sequence` and FPS evidence independently of optional transport frame diagnostics.
- Endpoint-centred inversion is rejected; reversed one-sided output endpoints remain supported without `.invert()`.
- Switch-learning finalization now requires valid observations of every configured learnable position.

### Added

- `RcConfigResult` / `RcConfigError` configuration validation contract.
- Per-frame channel count, channel-valid bitmap, frame sequence and required-role validity in `RcLinkStatus`.
- `healthy()`, `role_valid()` and `try_read()` provider/role health helpers.
- Required/optional role configuration.
- Commissioning-only adaptive switch learning gate, centroid validation and export/finalization helpers.
- Hardware-neutral iBUS and SBUS decoders with inter-byte timeout, corruption recovery and diagnostics.
- Native deterministic tests, sanitizer tests, release-contract checks and multi-platform PlatformIO CI.
- A single `test/run_host_checks.sh` host validation entry point.
- Beginner-oriented README path and v1.1 migration guidance.
- Compile-only tagged-v1.1 compatibility and namespace-only opt-out fixtures.
- Clean staged-distribution and protected/private-file rejection checks.

### Changed

- Adaptive switch learning is off by default; ordinary switches without explicit raw levels use deterministic evenly distributed raw positions.
- Protocol/parser diagnostics are exposed through the unified status object.
- The umbrella header no longer performs `using namespace rc;`; documented v1.1 types, role helpers, `FsRule<E>`, and signature builders use curated compatibility declarations unless `RCLINK_NO_LEGACY_GLOBAL_ALIASES` is defined.
- AVR and MegaAVR were removed from the v1.2 support claim and matrix; public headers use ordinary C++ standard headers and no longer define facilities inside `namespace std`.
- Example 01 is now a complete minimal iBUS link; calibration and advanced examples follow it in teaching order.

## [1.1.0]

- Previous public release represented by the supplied project archive.
