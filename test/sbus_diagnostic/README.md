# Advanced S.BUS developer diagnostic

This opt-in ESP32-S3 fixture preserves the detailed engineering monitor separately
from the six public learning examples. Beginner commissioning uses
`examples/03_SbusCalibration/03_SbusCalibration.ino` and the shared guided helper.
Build **only** `test/sbus_diagnostic/SbusDiagnostic.ino` as the sketch entry point,
using the repository ESP32-S3 environment and current RCLink source. It does not
control actuators or change receiver configuration.

The isolated bench uses GPIO18 RX, common GND and compatible signal levels,
inverted 100000-baud 8E2, no receiver TX, monitor 115200. The fixture displays CH1–CH6
native codes, converted raw values and reference signed percentages, current versus
historical extrema at 5 Hz, and D CSV at 10 Hz. r resets extrema, s prints a summary
including current CH7–CH10, h prints help. Native 11-bit codes are not pulse widths;
converted raw and the unclamped signed reference are not calibrated role commands.
Fault samples stay visible but do not enter extrema. Current flags and cumulative
accepted flag-bearing frame counts have different meanings; neither is a safety latch.
The producer's existing bounded TX queue, UART callback counters and CSV schema are
unchanged. Serial printing follows receiver polling; scheduled output omissions and
UART/parser errors are observable rather than hidden.

For capture, use **one** serial reader (close Serial Monitor first). For example,
with the already installed PlatformIO Python/pyserial, run the following locally,
substituting the verified port. It records exact lines; type `r`, `s`, or `h` followed
by Enter, or `q` to finish. Capture stationary centre, each endpoint and full sweeps,
return to rest, then request summaries for CH7–CH10 switch positions. Annotate each
physical action and fault test in the host log; do not change receiver settings.

```sh
python3 -B test/capture_sbus.py /dev/cu.VERIFIED_PORT /tmp/sbus-session.txt
```

Analyze `D,` lines with a CSV reader; `#` lines are human display/help/annotations.
The header lists every field. By default the host prints only startup, commands,
annotations, integrity warnings and a completion summary. Add `--echo` before the
port to echo full telemetry when needed; terminal backpressure can slow capture.
The file always retains complete serial records, including malformed bytes. A
partial final record is preserved in an explicit comment and is not a complete CSV
sample. Oversized records are preserved with a warning, using bounded serial buffering.
Blank channel fields mean no accepted frame; age is
`kInvalidFrameAgeMs` then. `link_ok` means accepted-frame age <= 200 ms, independently
of protocol flags. `log_drops` counts scheduled output batches omitted while the
single fixed TX buffer is occupied (not dropped receiver frames). UART overflow/error
counters record framework error callbacks; they cannot guarantee detection of every
lost byte. Footer, timeout and discarded-byte counters expose parser recovery.
Host line loss/truncation must be checked separately. TX sends at most 32 available
bytes per loop after polling reception; formatting and storage are bounded. UART
framework buffers allocate during startup, outside RCLink's receive parser.

Use stationary intervals to report native/converted min, median and max and verify
each row against the conversion above. Unsigned counter deltas reveal superseded
fault evidence. During transmitter-off and signal interruption, record flags,
counters, age and recovery separately. Host annotations and 10 Hz sampling leave
unknown RF onset, receiver-internal delay and electrical transition timing; these
logs do not qualify guaranteed RF-loss latency. S.BUS framing has no checksum, so
an accepted structural frame is not proof against arbitrary wire corruption.

