# Changelog

All notable RCLink changes are documented here.

## [1.3.1] - 2026-11-13

### Fixed

- Learned switch selection is provisional until application-frame acceptance, preserving accepted selection, learning, filtering and HoldLast state across rejected frames.
- Learned switch hysteresis uses adjacent midpoint boundaries with half the configured total band in each direction; explicit and ordinary deterministic switches retain nearest-only selection.
- Explicit switch raw levels match valid unclamped receiver samples, making calibrated levels beyond default endpoints reachable without broadening raw validity.
- JSON configuration loading checks known-field types and integer bounds before conversion and leaves the caller configuration unchanged on failure. Unknown roles and absent fields remain ignored.
- iBUS calibration samples only new decoded batches, saturates sample/histogram/dwell counters, and removes an unreachable classification branch while preserving the helper API.
- Transport reinitialization clears old frame authorization, histories and runtime learning state; transport diagnostics now refresh zero counts after reset/configuration reapplication.

### Clarified

- Receiver-programmed failsafe frames after RF loss are expected behavior, not an RCLink defect. Generic setup guidance distinguishes signature reporting from output enforcement, confirmation-window exposure and application-owned motion authorization/rearming. Example 05 retains its diagnostic behavior with clarified comments.
- `frame_sequence` and FPS count decoded update batches, independently of optional transport counters. `frames` and parser counters mirror the transport lifetime; sequence spans configuration/reinitialization.
- Provider health and role validity are checked together for motion-related examples; HoldLast does not authorize motion.
- Source-compatible v1.1/v1.2 APIs remain. Malformed JSON rejection and corrected learned hysteresis are intentional corrective behavior changes.

### Validation

- Verified the configured FS-i6X/FS-iA6B four-channel receiver-failsafe signature with output enforcement: a 250 ms hold confirmed after 264 ms in the diagnostic fixture, then applied zero outputs. Full-scale commands remained exposed before confirmation; recovery cleared the signature. Documented safe receiver-programmed positions, auxiliary disarm and application-owned deliberate rearming; no vehicle-safety guarantee is implied.
- A safer programmed receiver pattern produced neutral/zero motion and auxiliary DISARMED before signature confirmation at 255 ms. The temporary diagnostic arm latch required a separate rearm after recovery. Signature tolerance must distinguish the selector states; tolerance 2 cannot distinguish values 0 and 1, so the trial used 0/100.
- Added focused switch, JSON, calibration, restart and diagnostic regressions, including original-source failure comparisons and distribution rejection probes. See RELEASE_CHECKLIST.md for executed qualification and outstanding physical gates.
- Completed all declared local portable builds and all five ESP32-S3 example builds using isolated build prerequisites. A focused ESP32-S3 iBUS smoke test with FS-i6X/FS-iA6B hardware verified acquisition, stick mapping, stable neutral, explicit switch transitions, missing-frame failsafe and recovery. Transmitter-off deliberately produced programmed receiver failsafe frames; general RF-loss detection and physical SBUS remain unqualified.
- The reviewed pre-amendment candidate passed all 13 GitHub Actions jobs. The final amended release commit must pass CI before tagging; the focused iBUS smoke test does not establish RF or vehicle-level safety qualification.
- The public 1.3.1 release includes the previously unpublished 1.3.0 improvements below; the preceding published GitHub Release was 1.2.0.

## [1.3.0] - 2026-09-07

### Fixed

- Accepted frames now cache each mapped, transport-valid receiver channel once, sharing its raw value across aliased roles, validity checks, mapping, signature evaluation and switch learning.
- The advanced example uses `healthy()` for provider usability, retaining `link_ok` as a freshness diagnostic.

### Added

- Opt-in axis/switch `.valid_raw(lo_us, hi_us)` inclusive plausibility bounds, independent of calibration. Rejected roles fail closed before mapping and cannot update filtering, HoldLast history or learning; configurations without bounds retain existing clamping behaviour.
- `RcConfigError::InvalidRawValidity` rejects malformed bounds, no-signal overlap and excluded calibration endpoints or explicit switch levels at activation.
- Native regression coverage for changing transports, shared physical channels, plausibility boundaries, failsafe history, filtering, learning and recovery.

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
