# RCLink v1.3.1 release checklist

## Accepted release scope

The owner accepted a **software-qualified source release**, followed by one focused
physical iBUS and receiver-signature tests. The observed ESP32-S3 behavior below supplements that scope;
other hardware qualification remains deferred. No compiled binaries are required.

## Completed software and package requirements

- [x] Native GCC and Clang regressions, including historical protections and v1.1 compatibility.
- [x] Corrective switch/failsafe, JSON type-validation and calibration regressions, including negative cases.
- [x] ASan/UBSan checks across native, corrective, JSON and calibration tests. Local Darwin leak detection is unavailable and disabled.
- [x] Independent decoder round trips/noise and axis-property checks.
- [x] Portable architecture builds: ESP32-S3, ESP8266, RP2040, SAMD/MKR Zero, SAM/Due, Teensy 4.1 and STM32/Blue Pill.
- [x] All five ESP32-S3 examples compile as separate library consumers.
- [x] Consistent 1.3.1 metadata, MIT licence, manifests and declared ArduinoJson dependency.
- [x] Curated source-distribution checks and private/generated-file rejection probes.
- [x] Independent Maintainer and coordinator software acceptance.
- [x] GitHub Actions: all 13 jobs PASS on the reviewed pre-amendment candidate ([successful run](https://github.com/littlemanbuilds/RCLink/actions/runs/43492344319)).

Reuse the completed source qualification. Documentation amendment still requires
CI on the new exact commit. Record that final commit and successful run in the
published release notes/evidence, rather than hardcoding a final SHA here and
creating another amendment cycle. Workflow configuration alone is not CI evidence.

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

## Additional hardware qualification — owner-deferred, NOT_RUN

These unperformed checks remain outstanding and limit the release claim. Software
regressions and the focused smoke test do not substitute for them.

- [ ] Asymmetric stick calibration on the physical installation.
- [ ] Plausibility limits measured with margin, including rejection and recovery.
- [ ] Deliberately truncated/noisy physical traffic and restoration.
- [ ] Learned switch hysteresis on physical hardware.
- [ ] Physical SBUS operation on ESP32 before any physically validated SBUS claim.

Only the observed iBUS integration, missing-frame failsafe and configured signature
recognition/enforcement are hardware verified; general RF-loss detection, SBUS and
vehicle safety are not. This receiver deliberately continued fresh frames at its
programmed full-scale positions with its transmitter off, so applications must validate their
receiver failsafe configuration and any RF-loss signature before enabling actuation.
RCLink is not an independently certified safety system. Further physical work requires
owner authorization with actuators disconnected and compatible signal wiring.

## Outstanding owner-operated release requirements

- [ ] Review and stage README.md, CHANGELOG.md, RELEASE_CHECKLIST.md and the comment-only Example 05 correction, then amend the existing corrective commit without changing its message; preserve unrelated work.
- [ ] Fetch and confirm remote main has not advanced unexpectedly; push the amendment using an explicit safe force-with-lease operation. Resolve unexpected changes rather than overwriting them.
- [ ] Verify all 13 GitHub Actions jobs PASS on the exact amended release commit.
- [ ] Create and push the new v1.3.1 tag pointing to that verified commit. Never move or overwrite a historical tag.
- [ ] Publish the GitHub source release with accumulated changes since published v1.2.0, compatibility guidance and the focused iBUS scope and remaining physical limitations.
- [ ] Confirm the published tag/release and source inventory contain only public source, examples, manifests, licence and documentation/test material; no private controls, historical administrative evidence, Git internals, credentials or generated artifacts.
- [ ] Recheck Arduino Library Manager indexing and PlatformIO Registry publication for 1.3.1, completing any separate registry update before claiming availability.

RCLink is registered in both registries. As checked on 2026-11-13, their latest
listed version is 1.2.0; 1.3.1 is not yet available there. GitHub publication and
registry availability are separate states. Use the curated source ZIP or the new
Git tag only after publication. Do not upload the development-directory ZIP.

This checklist authorizes no Git, publication, flashing or connection changes.
