# RCLink v1.4.1 release checklist

## Current release scope

Version 1.4.1 corrects distribution packaging and release documentation. Its
functional baseline is the published v1.4.0 release, which added native S.BUS
diagnostics, cumulative fault-frame counters and guided calibration.

## Local technical qualification

Use installed prerequisites and the existing pinned PlatformIO environments.

- GCC and Clang native/protocol and corrective regressions, compatibility fixtures,
  JSON/calibration helpers, native S.BUS/counter coverage, shared calibration parity/JSON/invalid-evidence cases and ASan/UBSan.
- Synthetic host-capture tests: quiet/full echo, complete records, commands,
  annotations, partial/oversized/malformed input, failure diagnostics and shutdown.
- Seven portable consumers: `esp32-s3-devkitc-1`, `esp8266_nodemcuv2`, `pico`,
  `mkrzero`, `due`, `teensy41`, `bluepill_f103c8`.
- All six examples separately for ESP32-S3 in learning order: basic iBUS,
  iBUS calibration, S.BUS calibration, Flysky mapping, JSON, advanced features;
  also compile the separate test/sbus_diagnostic/SbusDiagnostic.ino producer.
  never place multiple sketch entry points in a consumer project.
- Exact 1.4.1 version surfaces, public API documentation, example catalogue,
  curated distribution and private/generated-file rejection probes.
- Bind the release to the independent review and CI evidence for the final commit.
  Retain the earlier functional and physical observations within their original scope.

## Verified physical S.BUS scope — observed 2026-11-17

The isolated ESP32-S3 N16R8 / FS-i6X / FS-iA6B session used inverted 100000-baud
8E2 on GPIO18, no receiver TX and no actuators. CH1–CH6 travel and CH7–CH10 switch
positions were observed. All 28,950 available native/converted pairs matched the
unchanged conversion. Typical travel 240–1807 produced 1041–1997; resting 1024
produced 1519. CH4's high 1796–1797 produced 1990–1991. Throttle's normal low rest
was 243–245 (1043–1044), distinct from its sweep minimum 240. These are receiver
observations, not universal calibration defaults or physical pulse-width measurements.

Transmitter-off produced frame-lost then failsafe flags while fresh receiver frames
continued; recovery cleared current flags and retained cumulative fault counts.
Signal interruption stopped accepted frames, produced stale native evidence and
crossed the 200 ms freshness threshold between sampled ages 166 ms and 266 ms. Recovery
restored fresh data. Same-poll fault-to-healthy preservation is deterministic
software-fixture evidence, not a forced physical wire sequence.

The 10 Hz capture had a disclosed reader gap, an excluded partial final record and
scheduled logging omissions. Guaranteed RF-loss latency, continuous wire-level
capture, universal receiver compatibility and vehicle/actuator safety remain
**unqualified**. RF onset and electrical transitions were not instrumented. Physical
asymmetric calibration, plausibility-limit rejection and learned-switch hysteresis
remain unperformed cases. No additional physical test is implied by source acceptance.

## Delivery sequence

1. Review the source candidate and its complete public file projection. The owner
   stages, commits and pushes the approved files.
2. Verify the pushed commit and all required GitHub Actions jobs for that SHA.
   Confirm the new version is unused. The owner creates the local `v1.4.1` tag
   at the verified commit.
3. In protected scratch, use the Python environment for PlatformIO Core 6.1.19
   to run `python -B test/pack_release.py commit REPOSITORY COMMIT OUTPUT CHECKER`
   with the full commit SHA, a new output directory and the path to the LMB
   Harness `publication.py`. Verify the generated plan and all four archives with
   the Harness release procedure before the owner publishes the tag and GitHub
   release.
4. Obtain exact artifact-bound owner authorization before uploading the normalized
   PlatformIO archive. Retrieve the actual GitHub ZIP and TAR, PlatformIO archive
   and Arduino Library Manager ZIP independently. Close the release only after
   the separate public distribution review.

## Historical 1.3.1 qualification — context only

The predecessor source and focused iBUS/signature behavior were qualified separately.
Its [historical GitHub Actions run](https://github.com/littlemanbuilds/RCLink/actions/runs/43492344319)
covered the earlier candidate. It does not qualify 1.4.0. The observations below are
retained history, not current delivery instructions or additional safety guarantees.

## Physical iBUS smoke test — observed on 2026-11-13

ESP32-S3-DevKitC-1 N16R8, FlySky FS-i6X transmitter and FS-iA6B receiver;
receiver supplied at 5 V, owner-confirmed iBUS level conversion to 3.3 V on GPIO18
with common ground. Motors and steering actuation remained disconnected.

- [x] Healthy acquisition and required-role validity before diagnostic control permission.
- [x] Channel 1 steering: 1000–1998 µs mapped to -100–100; neutral about 1500 µs mapped to zero. Channel 3 throttle: 1003–2000 µs mapped to 0–100.
- [x] Explicit two-position channel 7: 1000/2000 µs mapped to 0/1; repeated selection and sampled resting output showed no unexpected changes.
- [x] Receiver power loss stopped frames; the 200 ms timeout denied control and invalidated roles at frame age 201 ms on both attempts. Configured zero failsafe outputs were observed, including switch output 1→0.
- [x] Transmitter-off tested separately: the receiver deliberately transmitted its programmed failsafe positions, with channels 1–3 at 2000 µs. Without a configured signature, health and diagnostic permission remained true. This expected receiver behavior is not an RCLink defect.
- [x] Receiver restoration recovered without application restart. Power-off signal disconnection followed by startup kept roles invalid and outputs zero; power-off reconnection recovered on fresh frames.
- [x] Repeated centre/switch checks and receiver recovery; reception reinitialization cleared previous authorization before new data.

- [x] FS-i6X/FS-iA6B configured CH1–CH4 signature verified in a separate focused session: raw 2000/2000/2000/1000 µs scaled to 100/100/100/-100, tolerance 2, hold 250 ms. Confirmation occurred after 264 ms from first matching update (including polling/serial overhead); `rx_failsafe_sig` true, health false and enforced Value0 outputs followed. Full-scale commands and healthy status were exposed before confirmation. First nonmatching update cleared the signature and recovered normal values. This validates the configured pattern, not RF presence or vehicle safety.

This bounded session sampled diagnostics, not every frame or an injected invalid-frame
campaign. A midstream UART reinitialization produced 10 framing/resynchronization
errors, then stable reception; normal initial and final sessions had zero CRC/parser
errors. No material library defect was observed.

## Safer receiver-pattern trial — observed on 2026-11-13

- [x] After the owner saved enabled CH1/2/4=0%, CH3=-100%, CH7=-100% presets, transmitter-off changed active stick commands to raw 1500/1500/1000/1500 µs, CH7=1000 µs. Motion outputs were zero and the diagnostic application disarmed on the first matching update, before signature confirmation at 255 ms for a 250 ms hold.
- [x] The temporary auxiliary mapping used 0/100 with tolerance 2. An earlier 0/1 mapping matched both states within that inclusive tolerance; this was corrected in fixture configuration, without a library change.
- [x] Recovery with the auxiliary armed restored health but left the diagnostic latch disarmed until an explicit rearm command. This tests the fixture's application logic, not a vehicle controller.

The safer settings were temporary; the owner chose to restore the original pattern
and confirmed restoring it. No further RF-off check followed that restoration.
The first attempted safer receiver setup had not been saved and still emitted the
original pattern; the owner corrected the settings before the successful retry.
Ordinary stick movement did not activate the original signature during the observed
sequence, but its simultaneous corner pattern remains reproducible by live controls.
Timing is measured from the first matching decoded update, includes fixture serial
overhead, and excludes the receiver's unmeasured RF-loss detection latency.
The single-channel failsafe-OFF comparison was not performed; OFF/hold behavior is
documented from the transmitter manual. Native signature/history regressions were
inspected and reused without rerunning completed qualification.
