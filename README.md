# RCLink

RCLink is a small Arduino RC receiver library that turns raw receiver channels into **named, calibrated application controls**.

It provides iBUS and ESP32 SBUS transports, role mapping, axis shaping, deterministic switch decoding, filtering, link/failsafe status, receiver-failsafe signature detection, an optional ArduinoJson configuration loader, and an iBUS calibration helper.

RCLink is intentionally **not** a motor controller, safety supervisor, authority router, or vehicle-state library. Its job is to answer a narrower question well:

> **What is the receiver currently asking for, and is that receiver data valid enough to use?**

- **Version:** 1.3.1
- **Author:** Little Man Builds
- **License:** MIT
- **Qualification:** software-qualified with focused ESP32-S3 iBUS and configured receiver-signature tests. General RF-loss detection, SBUS hardware and vehicle safety remain unqualified. See [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).

---

## Contents

- [Why RCLink exists](#why-rclink-exists)
- [Design boundaries](#design-boundaries)
- [Installation](#installation)
- [Supported targets](#supported-targets)
- [Beginner path](#beginner-path)
- [Core concepts](#core-concepts)
- [Role mapping](#role-mapping)
- [Axis configuration](#axis-configuration)
- [Switch configuration](#switch-configuration)
- [Filtering and epsilon](#filtering-and-epsilon)
- [Validity and health model](#validity-and-health-model)
- [Failsafe behaviour](#failsafe-behaviour)
- [Receiver-failsafe signatures](#receiver-failsafe-signatures)
- [Physical Receiver Setup and Failsafe Configuration](#physical-receiver-setup-and-failsafe-configuration)
- [Commissioning switch learning](#commissioning-switch-learning)
- [Transports and parser resilience](#transports-and-parser-resilience)
- [JSON configuration](#json-configuration)
- [iBUS calibration helper](#ibus-calibration-helper)
- [Examples](#examples)
- [Testing and validation](#testing-and-validation)
- [API reference](#api-reference)
- [Migration from v1.1](#migration-from-v11)
- [Project integration guidance](#project-integration-guidance)
- [Repository structure](#repository-structure)
- [Deliberate limitations](#deliberate-limitations)
- [License](#license)

---

## Why RCLink exists

An RC receiver normally gives your program numbered channels such as CH1, CH2, CH3 and CH7. The application usually wants something more meaningful:

```text
receiver channel 0  -> steering
receiver channel 1  -> direction
receiver channel 2  -> speed
receiver channel 8  -> drive mode
```

RCLink creates that boundary. It keeps protocol decoding and application meaning separate, then adds the calibration and validity checks needed to avoid turning missing or malformed input into a believable command.

### What v1.2 specifically improves

RCLink v1.2 closes several important edge cases found during integration review:

- an asymmetric physical centre is now actually used during scaling;
- unmapped roles no longer silently read channel 0;
- missing channels are represented as invalid instead of becoming plausible values;
- role/index APIs fail closed instead of allowing out-of-bounds access;
- epsilon arithmetic cannot overflow a signed 16-bit intermediate;
- adaptive switch learning is commissioning-only rather than always active;
- empty receiver-failsafe signatures are rejected;
- timestamp zero is a valid timestamp rather than an internal sentinel;
- frame age is explicitly invalid before the first frame;
- iBUS/SBUS decoders recover from truncation, corruption and stream noise.

---

## Design boundaries

RCLink owns:

- receiver byte-stream parsing;
- decoded channel availability;
- logical-role mapping;
- raw calibration;
- axis deadband/expo/inversion;
- switch position decoding;
- optional EMA filtering;
- epsilon suppression;
- receiver freshness;
- protocol and configured receiver-failsafe state;
- per-role validity and configured failsafe outputs.

RCLink deliberately does **not** own:

- motor actuation;
- steering actuation;
- emergency-stop policy;
- user/driver authority arbitration;
- vehicle motion state;
- SafetyCore logic;
- persistence of commissioning data;
- inter-library message transport.

That boundary keeps RCLink reusable. A car, robot, boat, camera rig, accessibility controller, or test fixture can all consume it without RCLink depending on those projects.

---

## Installation

### Arduino IDE

Include the public umbrella header after installing the library:

```cpp
#include <RCLink.h>
```

For a manual 1.3.1 install, use the curated source ZIP from the
[GitHub release](https://github.com/littlemanbuilds/RCLink/releases) once published:

**Sketch → Include Library → Add .ZIP Library…**

For a ZIP installation, install ArduinoJson 7.x separately through Library Manager;
7.4.3 is the tested dependency. Include `RCLink.h` from your sketch.

### PlatformIO

```ini
lib_deps =
    https://github.com/littlemanbuilds/RCLink.git#v1.3.1
```

The prospective `v1.3.1` tag must be published before this pinned Git URL resolves.
RCLink is registered in [Arduino Library Manager](https://www.arduino.cc/reference/en/libraries/rclink/)
and [PlatformIO Registry](https://registry.platformio.org/libraries/littlemanbuilds/RCLink).
As checked on 2026-11-13, both list 1.2.0 as their latest version. Do not assume
1.3.1 is available from either registry until its listing is confirmed after release.

`ArduinoJson` is declared as a dependency; PlatformIO specifies `^7.4.2`, and 7.4.3
was used for qualification. Upgrade to the published 1.3.1 source/tag, rebuild your
application, and recheck its mapping, calibration and failsafe policy on the intended
receiver before enabling actuation.

---

## Supported targets

ESP32-S3 is the primary reference target. ESP32-family builds receive the full
supplied feature set: the Stream-based iBUS transport, hardware-neutral iBUS
and SBUS decoders, and the ESP32 inverted-UART SBUS transport.

The hardware-neutral core, decoders, and Stream-based iBUS path are also
supported on this selected portable set; see RELEASE_CHECKLIST.md for current candidate evidence:

- ESP8266;
- RP2040;
- SAMD (MKR Zero);
- SAM (Arduino Due);
- Teensy 4.1;
- STM32.

AVR and MegaAVR are not v1.2 support targets. RCLink does not emulate missing
C++ standard-library facilities to advertise those toolchains.

The supplied **SBUS transport is ESP32-only** because it uses the ESP32
inverted UART API. Portable compilation demonstrates source/toolchain
compatibility for the claimed surface; it does not demonstrate receiver UART,
electrical, RF, or failsafe behavior on physical boards.

The public learning examples target **ESP32-S3**, matching the normal LMB/MLMB development platform. They use GPIO18 for the iBUS RX example connection unless the sketch states otherwise.

---

## Beginner path

Start with `01_BasicIbusLink`. It shows the complete normal lifecycle without
JSON, receiver-failsafe signatures, or adaptive learning.

### 1. Connect the receiver

For the supplied iBUS examples:

```text
Receiver iBUS signal -> ESP32-S3 GPIO18
Receiver GND         -> ESP32-S3 GND
```

Power the receiver at the voltage required by the receiver hardware. **Do not assume its supply voltage is safe for an ESP32 GPIO.** Verify the receiver's signal level before connection.

### 2. Include RCLink and declare application roles

```cpp
#include <RCLink.h>

#define MY_ROLES(X) \
    X(Steering)     \
    X(Throttle)     \
    X(Mode)
RC_DECLARE_ROLES(MyRole, MY_ROLES)
```

### 3. Create the transport and link

The transport owns iBUS byte reception. `RcLink` owns mapping, validation,
shaping, freshness, and safe application outputs.

```cpp
rc::RcIbusTransport transport;
rc::RcLink<rc::RcIbusTransport, MyRole> rclink(transport);
```

### 4. Start iBUS and build a small configuration

Call `begin()` with the board's serial port and receiver RX pin, then map each
role explicitly.

```cpp
rclink.begin(Serial2, 115200, 18, -1);

RC_CONFIG(MyRole, cfg);
cfg.map(MyRole::Steering, 0);
cfg.map(MyRole::Throttle, 2);
cfg.map(MyRole::Mode, 6);

cfg.axis(MyRole::Steering)
   .raw(1000, 2000, 1500)
   .deadband_us(8)
   .out(-100.0f, 100.0f)
   .done();
cfg.axis(MyRole::Throttle)
   .raw(1000, 2000, 1000)
   .out(0.0f, 100.0f)
   .done();
cfg.sw(MyRole::Mode)
   .raw_levels({1000, 2000})
   .values({0.0f, 1.0f})
   .done();
```

A required role that is not mapped makes the configuration invalid. RCLink
does not quietly substitute channel 0.

### 5. Apply and check the configuration

```cpp
const rc::RcConfigResult result = rclink.apply_config(cfg);

if (!result)
{
    Serial.println("RCLink configuration rejected");
}
```

Stop setup or otherwise fail closed if the result is false.

### 6. Poll and read safe values

```cpp
rclink.update();

if (rclink.healthy())
{
    const int16_t steering = rclink.read(MyRole::Steering);
    const int16_t throttle = rclink.read(MyRole::Throttle);
}
```

Call `update()` frequently enough to drain receiver bytes. `healthy()` requires
fresh, valid required-role evidence with no receiver/protocol failsafe state.

`status().link_ok` and `ok()` report accepted-frame freshness only; a fresh frame
can still contain unusable roles. Provider health is not permission for the whole
machine to move.

### Raw calibration and plausibility

Calibration endpoints control scaling, including clamping outside those endpoints.
Opt-in `valid_raw(lo_us, hi_us)` limits instead decide whether a captured sample is
credible enough to use, before scaling, filtering or switch learning:

```cpp
cfg.axis(MyRole::Steering).raw(1000, 2000, 1500).valid_raw(900, 2100).done();
cfg.sw(MyRole::Mode).raw_levels({1000, 2000}).values({0, 1}).valid_raw(900, 2100).done();
```

These numbers illustrate the API. Safety/control applications should choose limits
from measured receiver behaviour with suitable margin, rather than guessing tight
limits. Bounds are inclusive, must exceed the no-signal threshold, and must contain
the calibrated axis endpoints or explicit switch levels. Switches without explicit
levels use the axis endpoints. Invalid configuration returns `InvalidRawValidity`.
Limits use signed 16-bit microseconds, matching calibration storage.

An out-of-range required role makes `healthy()` false and applies global failsafes;
an optional role applies its own failsafe without invalidating otherwise healthy
required roles. Rejected samples cannot update filter history, HoldLast history or
learning. With no enabled envelope, v1.2 no-signal checks and clamping remain intact.
Each mapped, transport-valid physical channel is read once per accepted frame;
aliased roles, scaling, signatures and learning all use that captured value.
Custom transports must keep the accepted frame stable across different channels;
this cache does not provide an atomic hardware snapshot of a changing receiver.
The optional JSON loader does not configure plausibility limits; apply these fluent
settings to its resulting configuration before activation.

For an individual role:

```cpp
if (rclink.healthy() && rclink.role_valid(MyRole::Steering))
{
    const int16_t value = rclink.read(MyRole::Steering);
    // value is fresh and valid for this role
}
```

### 7. Calibrate the actual transmitter and receiver

Next open `02_IbusCalibration`, move every stick, knob, and switch through its
full travel, then replace the illustrative raw values above with measured
values. A physical centre such as `1000 / 1588 / 2000` is valid; RCLink uses
the explicit centre rather than assuming the midpoint.

---

## Core concepts

### Role

A named application control such as `Steering`, `Throttle`, `Mode`, or `CameraTilt`.

### Receiver channel

The numbered physical channel decoded from iBUS/SBUS.

### Mapping

The relationship from an application role to a receiver channel.

### Required role

A role that must be present and valid for the provider to be considered healthy. Roles are required by default.

### Optional role

A role that may be missing without invalidating all required controls. An optional missing role independently receives its configured failsafe value.

### Freshness

Whether the most recently accepted receiver frame is within `link_timeout_ms`.

### Validity

Whether the channels required for a logical role actually existed and were valid in the accepted frame.

### Health

Whether RCLink is configured, fresh, role-valid, and not reporting a receiver/protocol failsafe condition.

These concepts are deliberately separate.

---

## Role mapping

The fastest mapping is declaration order:

```cpp
RC_CONFIG(MyRole, cfg);
RC_CFG_MAP_DEFAULT(MyRole, cfg);
```

That maps role 0 → channel 0, role 1 → channel 1, and so on.

For an explicit mapping:

```cpp
cfg.map(MyRole::Steering, 0);
cfg.map(MyRole::Throttle, 2);
```

RCLink initializes every mapping to `kInvalidChannel`. A role therefore cannot accidentally inherit receiver channel 0 simply because the configuration object was zero-initialized.

### Optional roles

```cpp
cfg.optional(MyRole::Auxiliary);
```

To make it required again:

```cpp
cfg.require(MyRole::Auxiliary, true);
```

Use optional roles sparingly for controls the application can genuinely live without.

---

## Axis configuration

### Asymmetric centre mapping

Axis mapping is piecewise:

```text
raw_lo -------- raw_center -------- raw_hi
  |                  |                 |
out_lo          centre output        out_hi
```

This means an axis calibrated as:

```cpp
.raw(1000, 2000, 1600)
.out(-100.0f, 100.0f)
```

produces neutral at raw `1600`, not at `1500`.

### Centre output

For the common bipolar axis, the inferred centre is zero:

```cpp
.out(-100.0f, 100.0f)
```

For a unipolar axis with an internal centre:

```cpp
.out(0.0f, 100.0f)
```

RCLink infers the midpoint output unless a centre output is explicitly configured through the builder API.

An endpoint-centred input such as throttle preserves the endpoint:

```cpp
.raw(1000, 2000, 1000)
.out(0.0f, 100.0f)
```

### Deadband

```cpp
.deadband_us(8)
```

Deadband is measured around `raw_center`. Configuration validation rejects a deadband that consumes an entire usable side of the axis.

### Expo

```cpp
.expo(0.35f)
```

`0.0` is linear. Values up to `1.0` progressively soften response around centre. Non-finite values invalidate configuration; finite builder inputs outside `[0,1]` are clamped into range.

### Inversion

Use explicit inversion:

```cpp
.invert(true)
```

Do not reverse the raw bounds to invert a channel. Raw bounds must stay ordered.

Inversion is valid for centred/bidirectional axes. Configuration rejects
`.invert()` when `raw_center` equals `raw_lo` or `raw_hi`, because negating that
one-sided normalized range collapses one side. To reverse a one-sided mapping,
reverse its application output endpoints without `.invert()`:

```cpp
.raw(1000, 2000, 1000)
.out(100.0f, 0.0f)
```

---

## Switch configuration

### Recommended: explicit raw levels

```cpp
cfg.sw(MyRole::Mode)
   .values({0.0f, 1.0f, 2.0f})
   .raw_levels({1000, 1500, 2000})
   .done();
```

The raw positions are validated for count and minimum separation.

### Simple deterministic fallback

If you configure values without raw levels:

```cpp
cfg.sw(MyRole::Mode).values({0.0f, 1.0f, 2.0f}).done();
```

RCLink distributes physical positions evenly across the calibrated raw range. The switch decision is based on **raw receiver geometry**, never on the numeric spacing of the output labels.

For a control that affects authority or safety, explicit measured raw levels are preferred.

---

## Filtering and epsilon

### EMA filtering

```cpp
cfg.setAxisFilter(MyRole::Steering, 0.20f);
```

- `0.0` disables filtering;
- lower non-zero values smooth more;
- higher values follow the source faster;
- values outside `[0,1]` are clamped by the builder;
- non-finite values invalidate configuration.

### Epsilon suppression

```cpp
cfg.setEpsilon(MyRole::Steering, 1);
```

The exposed value only changes when the filtered value differs by more than the configured epsilon.

Internally the difference is computed in 32-bit arithmetic. This matters when application ranges approach `int16_t` limits; `-32768 → +32767` must not overflow and appear to be a tiny change.

---

## Validity and health model

RCLink v1.2 intentionally distinguishes several states.

### `has_frame`

At least one complete transport frame has been accepted.

Before that happens:

```cpp
status().last_frame_age == rc::kInvalidFrameAgeMs
```

It is **not** reported as age zero.

### `link_ok`

The last accepted frame is younger than the configured link timeout.

A fresh frame can still be unusable if a mapped channel is missing.

### `channel_count`

Number of channels in the most recent accepted frame.

### `channel_valid_mask`

A bit mask identifying which decoded channels are valid.

### `required_roles_valid`

All roles marked required were mapped and their receiver channels were present/valid in the latest accepted frame.

### `frame_valid`

The latest accepted frame satisfied the current required-role mapping.

### `frame_sequence`

RCLink increments this application-facing sequence once whenever it accepts
new receiver-frame evidence from one successful transport `update()` batch.
One poll may drain multiple wire frames and exposes only its latest decoded values;
that poll increments the sequence once. `fps` counts these decoded update batches,
including batches later rejected by application validity/failsafe rules. Exact
wire-frame counts, where available, remain in `status().frames`.
The sequence spans configuration reapplication and `begin()`, is independent of optional
transport diagnostic counters and wraps naturally from `UINT32_MAX` to zero. Consumers
should compare sequence deltas with unsigned arithmetic.

### `healthy`

Equivalent to a provider-level check that requires:

- valid configuration;
- fresh link;
- valid required roles;
- no protocol failsafe;
- no frame-lost state;
- no confirmed receiver-failsafe signature.

Use:

```cpp
if (rclink.healthy())
{
    // receiver provider is usable
}
```

### Per-role validity

```cpp
if (rclink.healthy() && rclink.role_valid(MyRole::Steering))
{
    ...
}
```

or:

```cpp
if (rclink.healthy() && rclink.role_valid(MyRole::Steering))
{
    const int16_t steering = rclink.read(MyRole::Steering);
    ...
}
```

### Bounds-safe reads

The normal `read()` and `read_by_index()` paths are bounds checked. An invalid role/index fails closed rather than indexing arbitrary memory.

`role_valid()` can remain true during protocol failsafe: it reports source presence
and freshness, while `healthy()` reports provider usability. For motion or authority
selection, check both provider health and the particular role's validity, then apply
the application's own safety/authority rules. HoldLast retains a value; it does not
authorize motion.

`begin()` clears frame validity, accepted output/filter history, and runtime switch
learning state while preserving caller configuration and signature settings. A new
frame is required for health after restart. `status().frames` and parser counters
mirror the transport's current lifetime since its last reset, including zero.
`apply_config()` does not restart the transport; it refreshes those diagnostics.

Use `role_valid()` when you need source freshness/validity. `try_read()` only distinguishes a valid role index from an invalid one while copying the current safe output.

---

## Failsafe behaviour

Each role can define its safe output policy:

```cpp
cfg.setFailsafePolicy(MyRole::Throttle, rc::Failsafe::Mode::Value, 0);
```

Available modes are:

- `Value`;
- `HoldLast`;
- `ClampToOutLo`;
- `ClampToOutHi`.

For motion commands, an explicit safe `Value` is usually clearer than `HoldLast`.

`HoldLast` preserves the last filtered value produced by receiver evidence that
was acceptable for normal application use. Protocol failsafe, frame-lost,
invalid required-role, invalid-configuration, and applied receiver-signature
frames cannot replace that history. Signature candidate frames are also kept
out of the history that would be exposed if the signature becomes confirmed.

RCLink applies failsafe outputs when:

- configuration is invalid;
- the link is stale;
- a required role is unavailable;
- protocol failsafe is asserted;
- a receiver-failsafe signature is confirmed **and** `apply_rxfs_outputs(true)` is selected.

An optional missing role receives only its own failsafe output while valid required roles can remain usable.

---

## Receiver-failsafe signatures

Some receivers continue transmitting valid protocol frames during receiver failsafe but force selected channels to known values. RCLink can detect that pattern in scaled application space.

For measured safe values of zero (which can also occur during normal operation):

```cpp
RC_SET_FS_SIGNATURE_SELECTED(MyRole, rclink,
    /* tolerance */ 2,
    /* hold ms   */ 50,
    {{MyRole::Steering,   0},
     {MyRole::Throttle,   0}});
```

Then choose whether that signature only reports status or also applies configured failsafe outputs:

```cpp
rclink.apply_rxfs_outputs(true);
```

### Important v1.2 rules

- an empty signature mask is rejected;
- matching roles must themselves be valid;
- the hold interval advances across accepted matching frames;
- matching candidates cannot overwrite applied `HoldLast` history;
- timestamp zero is handled normally using explicit matching state;
- stale transport state clears the signature candidate.

You can configure without a macro:

```cpp
rc::RcFailsafeRule<static_cast<size_t>(MyRole::Count)> rule;
...
const rc::RcSignatureResult result = rclink.set_failsafe_signature(rule);
```

---

## Physical Receiver Setup and Failsafe Configuration

Supply the receiver within its specified range, use a common receiver/MCU ground,
and connect its data output to the correct MCU UART RX. The ESP32-S3 examples use
GPIO18 at 115200 baud, RX only. Check pin conflicts and **signal voltage separately
from receiver supply voltage**; use suitable level conversion for a 3.3 V MCU if
needed. Verify the interface before connecting it, power off before rewiring, and
commission with motion-producing equipment disconnected.

RF loss between transmitter and receiver differs from receiver-to-MCU UART loss.
RCLink detects stale communication, but a receiver may keep sending fresh iBUS
frames containing held or deliberately programmed failsafe positions after RF loss.
iBUS has no RF-loss flag. In the [FS-i6X manual](https://www.flysky-eu.com/en/transmitter/fs-i6x/manual),
failsafe **OFF** retains the channel's last position; an enabled setting selects a
programmed position. OFF does not mean zero or disabled output. Verify the actual
receiver arrangement, saved settings and received values.

Prefer receiver-programmed safe motion values to held or extreme commands. Measure
neutral and zero-drive positions in the consuming application's calibration and
mapping; transmitter percentage labels are not RCLink scaled values. A dedicated
auxiliary disarm control can provide an additional gate. Its selected values must
be distinguishable within the signature's inclusive, global tolerance: tolerance 2
matches both 0 and 1, whereas values 0/100 remain distinct.

For example, reuse the earlier `MyRole` mapping, with measured safe steering and
throttle values of zero. Here `Mode` serves as a dedicated DISARMED/ARMED input,
using the example's measured raw switch levels; adapt all calibration to the actual
installation. Configure explicit safe outputs and check both setup results:

```cpp
cfg.sw(MyRole::Mode).raw_levels({1000, 2000}).values({0, 100}).done();
cfg.setFailsafePolicy(MyRole::Steering, rc::Failsafe::Mode::Value, 0);
cfg.setFailsafePolicy(MyRole::Throttle, rc::Failsafe::Mode::Value, 0);
cfg.setFailsafePolicy(MyRole::Mode, rc::Failsafe::Mode::Value, 0);
const auto signature = rc::make_signature_selected(
    MyRole{}, /* tolerance */ 2, /* hold ms */ 250,
    {{MyRole::Steering, 0}, {MyRole::Throttle, 0}, {MyRole::Mode, 0}});
const bool receiverConfigured =
    static_cast<bool>(rclink.apply_config(cfg)) &&
    static_cast<bool>(rclink.set_failsafe_signature(signature));
rclink.apply_rxfs_outputs(true);
```

Keep application actuation inhibited if `receiverConfigured` is false. Signature
matching uses selected **unfiltered scaled application values**. New valid matching
updates must span the configured hold; confirmation sets `status().rx_failsafe_sig`
and makes `healthy()` false. The output policy is separate:

- `apply_rxfs_outputs(false)` is intentionally status-only: it reports confirmation
  without substituting the configured failsafe outputs for those received values.
- `apply_rxfs_outputs(true)` applies configured failsafe outputs after confirmation.

Neither mode guarantees suppression during confirmation: ordinary receiver values
can remain exposed and health can remain true. When enforcement is enabled, matching
candidates are excluded from accepted HoldLast/filter history. A new nonmatching
valid update clears signature detection and permits normal receiver processing to
resume. Neither matching nor recovery proves RF presence: legitimate live controls
can produce the same pattern, including the example's neutral/disarmed combination.

RCLink owns decoding, validation, calibration/shaping, freshness, signature matching,
configured fallback and diagnostics. The application owns authority, fault latching,
deliberate rearming, actuator enable and emergency behavior. **Health alone is not
motion authorization.** Gate hazardous actions on required-role validity, disarm and
application authority as well as health; apply safe behavior on stale or confirmed
faults. Recovery must not automatically rearm. Test actual receiver behavior and
confirmation latency: software signatures alone cannot guarantee immediate RF-loss
response or end-to-end stopping. See the measured integration scope below.

---

## Commissioning switch learning

Adaptive switch learning can be useful while characterising an unfamiliar transmitter. It should not silently continue moving centroids in normal service.

In v1.2 learning therefore requires **both**:

1. a switch configured with `.auto_levels(true)`; and
2. an explicit commissioning gate:

```cpp
rclink.set_switch_learning_enabled(true);
```

After exercising the switch positions:

```cpp
if (rclink.finalize_switch_learning())
{
    int16_t levels[3];
    rclink.copy_switch_levels(MyRole::Mode, levels, 3);
}
```

Enabling learning resets the observation evidence for each learnable switch.
`finalize_switch_learning()` succeeds only after valid receiver evidence has
selected every configured position of every learnable switch and centroid
separation remains valid; it always disables learning. Seeded defaults alone
are not commissioning evidence. Missing/invalid role samples do not count.

Persist the resulting levels in the application if required, then configure them later as explicit `.raw_levels(...)` values.

For authority-mode and safety-relevant selectors, this commissioning/freeze workflow is preferable to runtime adaptation.

---

## Transports and parser resilience

### iBUS

`RcIbusTransport` provides:

- normal iBUS checksum validation;
- variable channel-count frames within protocol limits;
- channel-valid bitmap;
- inter-byte timeout recovery;
- re-synchronization after corruption/noise;
- parser/error counters.

Typical ESP32-S3 startup:

```cpp
rc::RcIbusTransport transport;
rc::RcLink<rc::RcIbusTransport, MyRole> rclink(transport);

rclink.begin(Serial2, 115200, 18, -1);
```

### SBUS

`RcSbusEsp32Transport` is ESP32-only and uses inverted 100000-baud 8E2 UART reception.

The decoder exposes:

- 16 analogue channels;
- protocol failsafe;
- frame-lost flag;
- timeout/recovery diagnostics.

### Parser diagnostics

`RcLinkStatus` exposes:

```cpp
st.frames
st.crc_errors
st.parse_errors
st.parser_timeouts
st.discarded_bytes
```

These are useful for commissioning and diagnostics. The application should normally make control decisions from validity/freshness/failsafe state rather than a single parser counter.

---

## JSON configuration

JSON support is optional and lives in:

```cpp
#include <Json.hpp>
```

Because the loader resolves role names generated by `RC_DECLARE_ROLES`, include it **after** the role declaration:

```cpp
#define MY_ROLES(X) X(Steering) X(Throttle)
RC_DECLARE_ROLES(MyRole, MY_ROLES)

#include <Json.hpp>
```

The supplied JSON example demonstrates channel mapping, raw calibration, deadband, output range, switch levels and shaping.

A JSON parse succeeding does not replace normal configuration validation. Apply the resulting config through `rclink.apply_config(cfg)` and check the result.

---

## iBUS calibration helper

The calibration helper is intentionally separate from the runtime mapper:

```cpp
#include <calibration/Ibus_Calibrate.hpp>
```

Run `02_IbusCalibration` when commissioning a new transmitter/receiver pair,
after confirming the normal object lifecycle with Example 01.

Calibration is commissioning information. The values you measure should eventually become deterministic runtime configuration rather than continuously changing calibration state.

---

## API reference

### Main types

```cpp
rc::RcConfig<Role>
rc::RcLink<Transport, Role>
rc::RcLinkStatus
rc::RcConfigResult
rc::RcFailsafeRule<N>
rc::RcIbusTransport
rc::RcSbusEsp32Transport // ESP32 only
```

### Configuration

```cpp
cfg.map(role, channel)
cfg.require(role, true)
cfg.optional(role)
cfg.axis(role)...done()
cfg.sw(role)...done()
cfg.setLinkTimeout(ms)
cfg.setEpsilon(role, value)
cfg.setAxisFilter(role, alpha)
cfg.setFailsafePolicy(role, mode, value)
```

### Runtime

```cpp
rclink.update()
rclink.frame()
rclink.read(role)
rclink.read_by_index(index)
rclink.try_read(role, out)
rclink.role_valid(role)
rclink.status()
rclink.ok()
rclink.healthy()
rclink.changed()
```

### Receiver-failsafe signature

```cpp
rclink.set_failsafe_signature(rule)
rclink.clear_failsafe_signature()
rclink.apply_rxfs_outputs(true)
```

### Commissioning learning

```cpp
rclink.set_switch_learning_enabled(true)
rclink.finalize_switch_learning()
rclink.copy_switch_levels(role, out, capacity)
```

### Namespace compatibility

Older RCLink releases imported the whole `rc` namespace globally. v1.2 removes that broad namespace pollution.

For source compatibility, `RCLink.h` supplies curated global compatibility for
the documented v1.1 types plus `FsRule<E>`, `make_signature_all(...)`,
`make_signature_with_overrides(...)`, and `make_signature_selected(...)`.
`RC_DECLARE_ROLES` also retains global `to_string(...)`, `role_count(...)`, and
`role_by_index(...)`. It does not restore `using namespace rc;`.

New code should prefer:

```cpp
rc::RcLink<...>
rc::RcConfig<...>
```

To test a clean namespace-only project:

```cpp
#define RCLINK_NO_LEGACY_GLOBAL_ALIASES
#include <RCLink.h>
```

---

## Examples

RCLink intentionally keeps the public example set small.

### `01_BasicIbusLink`

The shortest complete normal-use path: transport, link, mapping, centred and
one-sided axes, switch, configuration activation, update, health, and reads.

### `02_IbusCalibration`

A deliberate commissioning utility for identifying channel ranges and switch positions.

### `03_FlyskyFSi6XDemo`

A straightforward ten-channel Flysky example using named roles, axes and switches.

### `04_JsonConfigDemo`

Loads equivalent configuration from JSON. Use after you understand the direct builder approach.

### `05_AdvancedFeatureDemo`

Adds filtering, epsilon suppression, per-role failsafe policies, receiver-failsafe signature detection and status diagnostics.

---

## Testing and validation

Testing is in `test/`, not a legacy `platformio.ci.ini` flow.

### Native deterministic suite

```bash
./test/run_native_tests.sh
```

The suite compiles the hardware-neutral core as C++11 with strict warnings and tests:

- asymmetric centre boundaries and full monotonic sweeps;
- endpoint-centred axes;
- centred inversion and rejected endpoint-centred inversion;
- acceptable-value `HoldLast` history across failsafe/recovery;
- deadband around the calibrated centre;
- invalid/non-finite configuration;
- required/optional mapping;
- invalid role/index access;
- missing channels;
- first-frame state;
- timestamp wraparound;
- 16-bit epsilon edge cases;
- receiver-failsafe signature configuration/hold behaviour;
- deterministic switch geometry;
- switch-learning observation/finalization evidence;
- RCLink-owned frame sequence and FPS accounting;
- tagged-v1.1 source compatibility and namespace-only opt-out;
- iBUS decode, checksum errors, truncation, timeout, noise and recovery;
- SBUS decode, flags, bad footer, truncation, timeout, noise and recovery;
- diagnostic propagation.

### Sanitizers

```bash
./test/run_sanitizers.sh
```

Runs AddressSanitizer and UndefinedBehaviorSanitizer over the deterministic suite.

### Cross-platform compile matrix

The selected portable API and all five ESP32-S3 examples passed the
[13-job GitHub Actions run](https://github.com/littlemanbuilds/RCLink/actions/runs/43492344319)
on the reviewed pre-amendment candidate, including both SAMD and SAM boards.
The final amended release commit still requires its own successful CI run; see
[RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).

### Hardware tests still matter

Host tests cannot prove:

- the receiver's real electrical signal level;
- UART inversion behaviour on physical ESP32 hardware;
- transmitter-specific failsafe configuration;
- EMI/noise behaviour in the final installation;
- end-to-end application stopping behaviour.

A focused ESP32-S3 test with an FS-i6X transmitter and FS-iA6B receiver verified
live iBUS acquisition, stick mapping, stable neutral, explicit switch transitions,
missing-frame failsafe and recovery. With a 200 ms timeout, receiver power loss
invalidated controls at frame age 201 ms. Signal-disconnected startup denied controls
until fresh frames arrived after reconnection.

The receiver was deliberately programmed to transmit its failsafe positions after
transmitter-off, including full-scale steering and throttle values. This is expected
receiver behavior, not an RCLink defect. Without a configured signature, RCLink remained healthy;
communication timeout alone does not detect this receiver's RF loss. A subsequent
four-channel signature test with enforcement enabled confirmed the programmed
pattern after 264 ms for a 250 ms hold and then applied zero outputs; full-scale
commands remained exposed before confirmation. Recovery cleared the signature on
the first nonmatching update. A temporary safer-preset trial confirmed after 255 ms
with the same 250 ms hold; motion values were already zero and its diagnostic
application disarmed before confirmation. Both signature timings include fixture
overhead and exclude the receiver's internal RF-loss reaction time. Recovery succeeded;
the diagnostic latch required a separate rearm. Physical SBUS, general RF-loss detection and
installation safety remain unqualified. See
[RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md) for the observed scope and deferred checks.

---

## Migration from v1.1

RCLink v1.2 is intended to be source-compatible for normal v1.1 sketches, but several unsafe edge behaviours intentionally change.

### 1. Asymmetric centres now work correctly

Old behaviour effectively used the midpoint of `raw_lo` and `raw_hi` for scaling even when `raw_center` was different.

Now:

```cpp
.raw(1000, 2000, 1600)
```

means raw `1600` is the axis centre.

If a project accidentally compensated for the old bug elsewhere, remove that compensation.

### 2. Required roles must be mapped

A fresh `RcConfig` no longer maps every role to channel zero through zero initialization.

Use:

```cpp
RC_CFG_MAP_DEFAULT(Role, cfg);
```

or explicit `.map(...)` calls.

If a role is genuinely optional:

```cpp
cfg.optional(Role::Aux);
```

### 3. Missing channels fail closed

A short frame cannot silently manufacture a legitimate command for a role whose channel was absent.

Check `healthy()` for the whole provider, `role_valid()` for individual source validity, and `try_read()` only when you need bounds-checked role copying.

### 4. Adaptive switch learning is no longer always active

`auto_levels(true)` only makes a switch *eligible* for learning. Enable learning explicitly during commissioning.

Normal switches with `.values(...)` and no explicit levels remain easy to use: RCLink deterministically distributes their positions across the raw range.

### 5. Empty failsafe signatures are rejected

Check the returned `RcSignatureResult` in new code.

### 6. Frame age before first frame changed

Before a valid frame:

```cpp
status().last_frame_age == rc::kInvalidFrameAgeMs
```

rather than zero.

### 7. Broad `using namespace rc` was removed

Traditional names remain available through compatibility aliases. New code should use `rc::` explicitly.

Endpoint-centred inversion is now rejected. Reverse a one-sided application
range through `.out(high, low)` without `.invert()`.

---

## Project integration guidance

For a larger RTOS/control project, a clean boundary is:

```text
receiver/UART
     |
     v
  RCLink
     |
     | value + validity + freshness + failsafe status
     v
application adapter / snapshot publisher
     |
     v
authority / safety logic
     |
     v
actuator command
```

Do not make RCLink depend on the application's bus or SafetyCore. Instead, copy the relevant RCLink state into the application's own provider snapshot.

A useful provider snapshot normally includes:

- mapped values;
- `healthy`;
- frame sequence;
- frame timestamp/age;
- required-role validity;
- protocol failsafe;
- receiver-signature failsafe.

The application can then decide what those conditions mean for authority and motion.

---

## Deliberate limitations

- RCLink is polling-based; it does not own a task or interrupt.
- It currently provides iBUS and ESP32 SBUS, not every RC protocol.
- `RcLinkStatus::healthy` is receiver-provider health, **not** proof that the whole machine is safe to move.
- RSSI/LQ remain unknown unless a future transport actually provides them.
- Adaptive switch learning is a commissioning convenience, not persistent storage.
- The library does not know whether the physical transmitter operator is authorized to control the application.
- Physical UART and failsafe behaviour must be verified on the actual receiver/transmitter pair.

---

## Repository structure

```text
RCLink/
├── src/
│   ├── RCLink.h
│   ├── Constants.hpp
│   ├── Types.hpp
│   ├── Config.hpp
│   ├── Link.hpp
│   ├── RcMacros.hpp
│   ├── Json.hpp
│   ├── calibration/
│   │   └── Ibus_Calibrate.hpp
│   └── transport/
│       ├── Ibus.hpp
│       ├── Sbus_Esp32.hpp
│       └── detail/
│           ├── IbusDecoder.hpp
│           └── SbusDecoder.hpp
├── examples/
├── test/
├── .github/workflows/
├── README.md
├── CHANGELOG.md
├── RELEASE_CHECKLIST.md
├── library.properties
├── library.json
├── keywords.txt
└── LICENSE
```

---

### Switch matching and hysteresis

Explicit `raw_levels()` match the valid receiver sample before endpoint clamping.
Levels outside default 1000–2000 µs endpoints remain reachable; `valid_raw()` still
rejects samples outside its configured inclusive bounds, and no-signal rules still
apply. Explicit levels and ordinary evenly spaced switches retain nearest-level
selection without hysteresis.

For `auto_levels()` switches, `hysteresis_us(h)` is the total band centred on each
adjacent centroid midpoint. Moving upward requires passing midpoint + h/2; moving
downward requires passing midpoint − h/2. Equality retains the accepted position;
the initial sample selects the nearest centroid. Only acceptable frames commit
selection and commissioning observations. Unequal learned spacing uses each actual
adjacent midpoint. Persisted explicit levels retain the nearest-only contract.

JSON loading rejects strings, null, booleans and fractional channel indices rather
than coercing them. Known fields require their documented types; unknown roles and
absent fields are ignored. Failure leaves the supplied config unchanged. Always
validate with `apply_config()` before use.

Calibration records one observation per newly decoded update batch, never repeated
polls of cached values. Batched wire packets contribute only the latest values.
Histogram and dwell counters saturate instead of wrapping.

## Version history

Current version:

```text
1.3.1
```

See [CHANGELOG.md](CHANGELOG.md) for detailed release history.

---

## License

RCLink is released under the **MIT License**. See [LICENSE](LICENSE).

Copyright © 2026 Little Man Builds (Darren Osborne).
