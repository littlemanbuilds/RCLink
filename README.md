# RCLink

Header‑only RC link for **ESP32 Arduino** projects with **iBUS** and **SBUS** transports, flexible role mapping, per‑channel shaping and filtering, an optional JSON config loader, and **receiver‑failsafe signature detection** with a selectable **apply policy**.

The high-level link is transport-agnostic, but this package is built and tested for ESP32 Arduino. The SBUS transport is intentionally ESP32-focused because it uses the ESP32 inverted UART path.

> **Author:** Little Man Builds  
> **License:** MIT

---

## Highlights

- **Transports:** iBUS (Flysky family) and **SBUS** (100 000 baud, 8E2, inverted; ESP32‑ready).
- **Role mapping:** `RC_DECLARE_ROLES`, `RC_CONFIG`, `RC_CFG_MAP_DEFAULT`.
- **Per‑channel shaping:** raw calibration (µs), deadband, output range, expo, invert.
- **Filtering:** per‑axis EMA (`setAxisFilter`) + epsilon suppression (`setEpsilon`).
- **Failsafe:** per‑channel policies + **receiver‑failsafe signatures** (select which roles & expected values).
- **Failsafe apply policy:** link loss and protocol failsafe always apply safe outputs; `apply_rxfs_outputs(true/false)` controls only receiver-signature matches.
- **Status:** unified `RcLinkStatus` (fps, link age, protocol flags, CRC errors).
- **Header‑only** core; optional JSON loader (ArduinoJson) and an iBUS calibrator.
- **iBUS Calibrator:** interactive histogram/clustering helper to generate channel specs.

---

## Contents

- [RCLink](#rclink)
  - [Highlights](#highlights)
  - [Contents](#contents)
  - [Installation](#installation)
  - [Supported Targets](#supported-targets)
  - [Concepts](#concepts)
  - [Transports](#transports)
  - [Role Declaration \& Mapping](#role-declaration--mapping)
  - [Quick Start](#quick-start)
  - [Per‑Channel Configuration](#perchannel-configuration)
    - [Axis builder](#axis-builder)
    - [Switch builder](#switch-builder)
    - [Filtering \& Epsilon](#filtering--epsilon)
    - [Per‑channel failsafe policy](#perchannel-failsafe-policy)
  - [Receiver‑Failsafe Signatures](#receiverfailsafe-signatures)
  - [Status \& Logging](#status--logging)
  - [JSON Config Loader](#json-config-loader)
  - [iBUS Calibrator](#ibus-calibrator)
  - [Examples](#examples)
    - [1) Basic iBUS (print values when they change)](#1-basic-ibus-print-values-when-they-change)
    - [2) SBUS on ESP32 (Flysky roles)](#2-sbus-on-esp32-flysky-roles)
    - [3) Shaping + filters + epsilon](#3-shaping--filters--epsilon)
    - [4) Failsafe policies + signature apply](#4-failsafe-policies--signature-apply)
  - [Performance Notes](#performance-notes)
  - [FAQ](#faq)
  - [License](#license)

---

## Installation

**Arduino IDE:** (once published) search **RCLink** in Library Manager and install.  
**Manual:** download the release ZIP → _Sketch → Include Library → Add .ZIP Library…_  
**PlatformIO:** add to `lib_deps`:

```ini
lib_deps =
  LittleManBuilds/RCLink
```

Optional dependency: **ArduinoJson** (only if you use the JSON loader).

---

## Supported Targets

- **Tested target:** ESP32-S3 DevKitC-1 with PlatformIO `espressif32` + Arduino (`platformio.ini`).
- **Packaged target:** ESP32 Arduino (`architectures=esp32`, `platforms=espressif32`).
- **iBUS:** uses Arduino-style `HardwareSerial` at 115200 8N1. It may work on other Arduino cores with the same UART API, but release validation here is ESP32.
- **SBUS:** `RcSbusEsp32Transport` is ESP32-only. It uses 100000 baud, 8E2, inverted UART via the ESP32 `HardwareSerial::begin(..., true)` overload.
- **JSON loader and calibrator:** written for Arduino sketches and validated with the ESP32 package target above.

---

## Concepts

- **Role** – a logical control in your app (`Throttle`, `Steering`, etc.) declared via `RC_DECLARE_ROLES`.
- **Channel** – receiver channel index (iBUS: 0..13, SBUS: 0..15).
- **Mapping** – binds a role to a RX channel index (`cfg.map()` or macro helpers).
- **Axis** – continuous input (sticks, knobs) with shaping (deadband, expo, invert, scaling).
- **Switch** – discrete positions (2..8) with either fixed raw levels or **auto‑learn** snapping.
- **Epsilon** – suppress small post‑filter changes to reduce log spam and jitter.
- **Failsafe** – per‑channel output policy during link loss; independent **signature detection** can flag when RX enters its own failsafe pattern and, if enabled, apply your policy immediately.
- **Status** – `RcLinkStatus` reports link freshness, fps, last frame age, protocol flags, and counters.

---

## Transports

- **iBUS** (`RcIbusTransport`) — standard 115 200 8N1; RX‑only UART; tested on ESP32 Arduino.
- **SBUS (ESP32)** (`RcSbusEsp32Transport`) — **100 000 baud, 8E2, inverted**; exposes protocol failsafe and frame‑lost flags; ESP32-only.

Both transports implement the same minimal interface used by `RcLink`, so you can switch between them on ESP32 without changing higher‑level code.

---

## Role Declaration & Mapping

```cpp
#include <RCLink.h>

#define FLYSKY_ROLES(X) \
  X(Ch1_RH) /* roll  */ \
  X(Ch2_RV) /* pitch */ \
  X(Ch3_LV) /* thr   */ \
  X(Ch4_LH) /* yaw   */

RC_DECLARE_ROLES(Flysky, FLYSKY_ROLES)

RcIbusTransport transport;
RcLink<RcIbusTransport, Flysky> rclink(transport);

void setup() {
  Serial.begin(115200);
  rclink.begin(Serial2, 115200, /*rx*/18, /*tx*/-1);

  RC_CONFIG(Flysky, cfg);
  RC_CFG_MAP_DEFAULT(Flysky, cfg); // role0->ch0, role1->ch1, ...
  rclink.apply_config(cfg);
}
```

Helpers generated by `RC_DECLARE_ROLES`:

- `enum class Flysky : uint8_t { ..., Count };`
- `to_string(Flysky::Ch1_RH)` → `"Ch1_RH"`
- `role_count(Flysky{})` → number of roles

> The umbrella header `RCLink.h` brings the public types and helpers into scope for sketches.

---

## Quick Start

```cpp
#include <RCLink.h>

#define ROLES(X) X(Ch1) X(Ch2) X(Ch3) X(Ch4)
RC_DECLARE_ROLES(MyRC, ROLES)

RcIbusTransport transport;
RcLink<RcIbusTransport, MyRC> rclink(transport);

void setup() {
  Serial.begin(115200);
  rclink.begin(Serial2, 115200, 18, -1);

  RC_CONFIG(MyRC, cfg);
  RC_CFG_MAP_DEFAULT(MyRC, cfg);
  rclink.apply_config(cfg);
}

void loop() {
  rclink.update();
  if (rclink.changed()) {
    RC_PRINT_ALL(rclink, MyRC);
  }
  delay(10);
}
```

---

## Per‑Channel Configuration

### Axis builder

```cpp
RC_CONFIG(Flysky, cfg);
RC_CFG_MAP_DEFAULT(Flysky, cfg);

// CH1: 1000..2000 µs centered at 1500 µs, deadband 8 µs, output -100..+100, expo 0.35
cfg.axis(Flysky::Ch1_RH)
   .raw(1000, 2000, 1500)
   .deadband_us(8)
   .out(-100.f, 100.f)
   .expo(0.35f)
   .invert(false)
   .failsafe(Failsafe::Mode::Value, 0)  // optional per‑channel policy
   .done();
```

Notes:

- `raw(lo, hi, center)` defines input bounds for scaling.
- `deadband_us()` is in **microseconds** around the center.
- `expo(0..1)` applies a cubic‑mix curve (`0` = linear).
- `out(lo, hi)` sets the scaled range returned by `read(role)`.
- `invert()` flips the axis direction.
- `.failsafe(mode, def)` writes into the config’s failsafe table for this role.

### Switch builder

```cpp
// CH7: 2‑pos switch → values {0,1}
cfg.sw(Flysky::Ch7_SwA).values({0.f, 1.f}).done();

// CH9: 3‑pos switch → values {-1,0,+1}, fixed raw µs levels
cfg.sw(Flysky::Ch9_SwC)
   .values({-1.f, 0.f, +1.f})
   .raw_levels({1000,1500,2000})    // explicit snap points
   .failsafe(Failsafe::Mode::Value, 0)
   .done();

// CH10: 2‑pos with auto‑learned levels + hysteresis
cfg.sw(Flysky::Ch10_SwD)
   .values({0.f, 1.f})
   .auto_levels(true)
   .hysteresis_us(60)
   .learn_alpha(0.20f)   // higher values adapt faster
   .done();
```

### Filtering & Epsilon

```cpp
// EMA filter (0..1). 0 disables; higher values follow input faster. Axis only.
cfg.setAxisFilter(Flysky::Ch1_RH, 0.20f);
cfg.setAxisFilter(Flysky::Ch2_RV, 0.20f);
cfg.setAxisFilter(Flysky::Ch3_LV, 0.10f);

// Epsilon (scaled units) – only publish when |delta| > epsilon
cfg.setEpsilon(Flysky::Ch1_RH, 1);
cfg.setEpsilon(Flysky::Ch2_RV, 1);
cfg.setEpsilon(Flysky::Ch3_LV, 1);
```

### Per‑channel failsafe policy

Applied when the link is **not ok** (stale/lost; see [Status](#status--logging)).

```cpp
// Mode::Value -> emits the provided default value
cfg.setFailsafePolicy(Flysky::Ch1_RH, Failsafe::Mode::Value, 0);

// Mode::HoldLast -> freezes at last good filtered value
cfg.setFailsafePolicy(Flysky::Ch2_RV, Failsafe::Mode::HoldLast);

// Clamp to axis output ends (axes only)
cfg.setFailsafePolicy(Flysky::Ch3_LV, Failsafe::Mode::ClampToOutLo);
cfg.setFailsafePolicy(Flysky::Ch4_LH, Failsafe::Mode::ClampToOutHi);
```

---

## Receiver‑Failsafe Signatures

A **receiver‑failsafe signature** detects when the RX is still sending frames but has entered a specific pattern (e.g., the transmitter’s built‑in failsafe posture). You control what happens next via **your per‑channel policy** and the **apply policy**:

- `status().rx_failsafe_sig` — set when the incoming _scaled_ values match the configured signature for at least `hold_ms`.
- `apply_rxfs_outputs(true/false)` — if **true**, the link immediately **applies your failsafe policy outputs** (as if the link were lost). If **false** (default), it only reports `rx_failsafe_sig` and continues to pass through the live frames.
- `status().proto_failsafe` — protocol-level failsafe is safety-critical and always applies your failsafe policy outputs, regardless of `apply_rxfs_outputs()`.

**Signature macros** (scaled space):

```cpp
// 1) All roles == expected (±tol), must persist hold_ms
RC_SET_FS_SIGNATURE_ALL(Flysky, rclink, /*expected=*/0, /*tol=*/3, /*hold_ms=*/250);

// 2) Default expected with per‑role overrides
RC_SET_FS_SIGNATURE_OVERRIDES(Flysky, rclink, /*default=*/0, /*tol=*/3, /*hold_ms=*/250,
  { {Flysky::Ch3_LV, 10} } // throttle expected 10; others use default
);

// 3) Only check selected roles (others ignored)
RC_SET_FS_SIGNATURE_SELECTED(Flysky, rclink, /*tol=*/3, /*hold_ms=*/250,
  { {Flysky::Ch1_RH, 0}, {Flysky::Ch2_RV, 0}, {Flysky::Ch3_LV, 0}, {Flysky::Ch4_LH, 0} }
);

// Choose how a detected signature affects outputs:
rclink.apply_rxfs_outputs(true);  // apply your policy when signature matches
// rclink.apply_rxfs_outputs(false); // (default) report only; keep following frames
```

> Implementation detail: signature matching uses the **scaled values before epsilon suppression**, so detection is not delayed by logging/epsilon settings.

---

## Status & Logging

`RcLinkStatus` provides a live snapshot:

```cpp
const RcLinkStatus& st = rclink.status();
if (!st.link_ok)        Serial.println("Link STALE (no frame within timeout)");
if (st.proto_failsafe)  Serial.println("Protocol failsafe flag asserted");
if (st.frame_lost)      Serial.println("Transport reports a lost frame");
if (st.rx_failsafe_sig) Serial.println("Receiver failsafe signature matched");
```

**Sticky print once** example showing `RC_PRINT_ALL` versatility:

```cpp
static bool printedFailsafe = false;

void loop() {
  rclink.update();
  const auto& st = rclink.status();

  if (!st.link_ok || st.proto_failsafe || st.rx_failsafe_sig) {
    if (!printedFailsafe) {
      Serial.println(st.link_ok ? "Failsafe: signature or protoFS" : "Failsafe: link lost");
      RC_PRINT_ALL(rclink, Flysky); // safe outputs for link/proto FS; RX signature depends on apply policy
      printedFailsafe = true;
    }
  } else {
    if (rclink.changed()) {
      RC_PRINT_ALL(rclink, Flysky); // print only when values change
    }
    printedFailsafe = false; // reset for next time
  }
}
```

---

## JSON Config Loader

The optional loader applies mapping + per‑channel specs from a JSON string. This JSON is typically produced by the **iBUS Calibration tool** (see `examples/01_Ibus_Calibration.ino`) and can also be read from SPIFFS, SD, or Serial. To use it:

1. Include **after** your `RC_DECLARE_ROLES(...)` so the generated helpers exist:

```cpp
#include <RCLink.h>
RC_DECLARE_ROLES(MyRC, /* ... */)
#include <Json.hpp>
```

2. Call `load_json(MyRC{}, cfg, jsonText)` and then `rclink.apply_config(cfg)`.

**Schema (high‑level):**

- `map`: object of `"RoleName" -> channelIndex`.
- `axes.RoleName`: fields `raw [lo,hi,center]`, `deadband_us`, `out [lo,hi]`, `expo`, `invert`.
- `switches.RoleName`: fields `values [..]`, optional `raw_levels [..]`, `auto_levels`, `hyst_us`, `learn_alpha`.

```
{
  "map": { "RoleName": 0, ... },
  "axes": {
    "RoleName": {
      "raw": [lo, hi, center],
      "deadband_us": 0,
      "out": [lo, hi],
      "expo": 0.0,
      "invert": true
    }
  },
  "switches": {
    "RoleName": {
      "values": [0,1,2],
      "raw_levels": [1000,1500,2000],
      "auto_levels": true,
      "hyst_us": 60,
      "learn_alpha": 0.2
    }
  }
}
```

---

## iBUS Calibrator

A helper to interactively explore raw ranges, switch levels, and produce suggested config (both fluent code and JSON). Typical sketch outline:

```cpp
#include <RCLink.h>
#include <calibration/Ibus_Calibrate.hpp>

void setup() {
  Serial.begin(115200);
  calibrate::run_ibus(Serial2, /*rxPin=*/18, /*txPin=*/-1, /*baud=*/115200);
}
void loop() {}
```

Follow the on‑screen instructions to move sticks/switches and copy the suggested JSON settings into your app.

---

## Examples

### 1) Basic iBUS (print values when they change)

```cpp
#include <RCLink.h>

#define ROLES(X) X(Ch1) X(Ch2) X(Ch3) X(Ch4)
RC_DECLARE_ROLES(MyRC, ROLES)

RcIbusTransport transport;
RcLink<RcIbusTransport, MyRC> rclink(transport);

void setup() {
  Serial.begin(115200);
  rclink.begin(Serial2, 115200, 18, -1);
  RC_CONFIG(MyRC, cfg); RC_CFG_MAP_DEFAULT(MyRC, cfg);
  rclink.apply_config(cfg);
}
void loop() {
  rclink.update();
  if (rclink.changed()) RC_PRINT_ALL(rclink, MyRC);
  delay(10);
}
```

### 2) SBUS on ESP32 (Flysky roles)

```cpp
#include <RCLink.h>

#define FLYSKY_ROLES(X) X(Ch1_RH) X(Ch2_RV)
RC_DECLARE_ROLES(Flysky, FLYSKY_ROLES)

RcSbusEsp32Transport transport;
RcLink<RcSbusEsp32Transport, Flysky> rclink(transport);

void setup() {
  Serial.begin(115200);
  // SBUS: 100000 baud, 8E2, inverted
  rclink.begin(Serial2, 100000, /*rx=*/18, /*tx=*/-1);
  RC_CONFIG(Flysky, cfg); RC_CFG_MAP_DEFAULT(Flysky, cfg);
  rclink.apply_config(cfg);
}
void loop() {
  rclink.update();
  Serial.printf("RH=%d, RV=%d\n",
    rclink.read(Flysky::Ch1_RH),
    rclink.read(Flysky::Ch2_RV));
  delay(20);
}
```

### 3) Shaping + filters + epsilon

```cpp
RC_CONFIG(Flysky, cfg); RC_CFG_MAP_DEFAULT(Flysky, cfg);

cfg.axis(Flysky::Ch1_RH)
   .raw(1000,2000,1500).deadband_us(8).out(-100,100).expo(0.35f).done();
cfg.axis(Flysky::Ch2_RV)
   .raw(1000,2000,1500).deadband_us(8).out(-100,100).expo(0.35f).done();
cfg.axis(Flysky::Ch3_LV) // throttle
   .raw(1000,2000,1000).deadband_us(4).out(0,100).expo(0.15f).done();

cfg.setAxisFilter(Flysky::Ch1_RH, 0.20f);
cfg.setAxisFilter(Flysky::Ch2_RV, 0.20f);
cfg.setAxisFilter(Flysky::Ch3_LV, 0.10f);

cfg.setEpsilon(Flysky::Ch1_RH, 1);
cfg.setEpsilon(Flysky::Ch2_RV, 1);
cfg.setEpsilon(Flysky::Ch3_LV, 1);
```

### 4) Failsafe policies + signature apply

```cpp
// Per‑role failsafe outputs when link is stale
cfg.setFailsafePolicy(Flysky::Ch1_RH, Failsafe::Mode::Value, 0);
cfg.setFailsafePolicy(Flysky::Ch3_LV, Failsafe::Mode::ClampToOutLo);

// Signature: RX sends all zeros for ≥250 ms (example)
RC_SET_FS_SIGNATURE_ALL(Flysky, rclink, 0, 3, 250);

// Apply policy when signature is detected (or just report if false)
rclink.apply_rxfs_outputs(true);
```

---

## Performance Notes

- iBUS/SBUS parsing uses fixed buffers; no heap allocations or IRQs required by the core.
- Call `update()` steadily (e.g., every 5–10 ms). Use `fps` in `status()` to observe cadence.
- Epsilon suppression (`setEpsilon`) avoids spurious `changed()` triggers and reduces serial spam.
- Signature matching cost is O(N) over your roles with small constants.

---

## FAQ

**How do I detect the RC is switched off?**  
If no new frame within `cfg.link_timeout_ms`, `status().link_ok == false`. You can also watch `last_frame_age` and `fps`.

**Does the signature detection respect filters or epsilon?**  
Detection uses **scaled** values (pre‑epsilon), so it isn’t blocked by epsilon settings. EMA filters still shape the scaled values; use a reasonable `tol`.

**Do I need ArduinoJson?**  
Only for the JSON loader. Core `RcLink` + transports don’t require it.

**Can switches auto‑learn raw levels?**  
Yes. Leave `raw_levels` unset and enable `auto_levels(true)`. Adjust `hysteresis_us` and `learn_alpha` to taste.

**What units does `read(role)` return?**  
Your configured **output range** (e.g., `-100..+100` or `0..100`).

**Can I read defensively when a role/index may be unchecked?**
Yes. Use `read_or(role, fallback)` or `read_by_index_or(index, fallback)` when the value might come from external data.

---

## License

MIT © Little Man Builds
