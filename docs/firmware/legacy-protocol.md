# Historical vehicle protocol

The original integrated host used short UTF-8/ASCII commands terminated by line
feed. This page reconstructs that dialect so it can be tested against the
existing car or implemented as an explicit compatibility adapter.

## Integrated prototype dialect

| Host operation | Candidate wire command | Historical host behavior |
| --- | --- | --- |
| Automatic/controller mode | `A\n` | Sent when enabling controller-driven operation |
| Manual mode | `M\n` | Sent when returning control to the GUI |
| Forward direction | `D F\n` | Direction selected independently of speed |
| Reverse direction | `D R\n` | Direction selected independently of speed |
| Propulsion request | `V <0..100>\n` | Integer, percentage-like range |
| Steering request | `W <raw>\n` | Candidate calibrated range 200..2900, center 1750 |
| Engage brake | `S 1\n` | Used by manual brake and stop-assist paths |
| Release brake | `Q 1\n` | Used by manual release paths |
| Emergency-stop sequence | `V 0\n`, then `S 1\n` | Two commands sent in this order |

Transport behavior reconstructed from the final integrated script:

- 115200 baud;
- serial timeout 0.5 seconds;
- one-second wait after opening the port;
- line-feed command terminator;
- minimum 50 ms between queued writes; and
- newline-delimited responses accepted for display, without a defined response
  grammar or command-to-response correlation.

These values are the strongest software evidence available, but the missing
firmware and serial transcript prevent them from being called a verified board
contract.

## Dialect variations in repository history

Earlier experiments used incompatible spellings:

| Source family | Variants observed |
| --- | --- |
| Early ESP32 wrapper | `V50`, `D`, `R`, `F`, `Z` |
| Standalone Arduino experiment | `S`, `D`, `A` |
| Integrated controller/Lidar application | `A`, `M`, `D F`, `D R`, `V n`, `W n`, `S 1`, `Q 1` |

This variation is why the integrated dialect must be named and selected
explicitly rather than called simply “the Arduino protocol.” A captured session
from the existing car should settle which firmware build and spelling remain
installed.

## Maintained protocol profiles

The maintained host now supports both protocols through explicit,
configuration-selected strategies in
[`protocol_profiles.py`](../../src/car_interface/domain/protocol_profiles.py).
It never probes a serial device to guess which dialect is attached.

The default [protocol v1](../protocol.md) profile sends frames such as:

```text
!CAR,1,CMD,0,SPD,25*E24B
```

Protocol v1 adds normalized signed requests, sequence numbers, CRC-16, ACK/NACK,
arming, heartbeat, and reset semantics. The `school_car_legacy_v0` profile maps
the same domain commands into the reconstructed lines above:

```text
domain command
  ├─ car_v1                -> checksummed request + correlated ACK/NACK
  └─ school_car_legacy_v0  -> exact legacy line(s) + write-only receipt
```

Signed speed requests become direction plus magnitude (`D F`/`D R`, then
`V n`), normalized steering is mapped piecewise through the configured
200/1750/2900 calibration, brake uses `S 1`/`Q 1`, and stop/reset operations
emit `V 0` followed by `S 1`. The dispatcher enforces the configured 50 ms
spacing between legacy frames.

The legacy implementation does not generate a synthetic ACK merely because a
line was written. Dispatch receipts expose the selected profile, number of
frames written, and `acknowledged = false`. Physical compatibility with the
installed firmware remains pending until the transcript below is captured.

Select it only through an explicit hardware configuration:

```toml
[car_interface]
mode = "hardware"
protocol = "school_car_legacy_v0"
require_ack = false
controller_steering_invert = true
legacy_steering_minimum = 200
legacy_steering_center = 1750
legacy_steering_maximum = 2900
legacy_minimum_command_interval_ms = 50
```

See the complete [configuration profile](../hardware/configuration-profile.md)
for ports, Lidar, timing, and operating limits.

## Compatibility verification record

Capture at least this transcript from the physical car with driven outputs
disconnected or restrained:

| Step | Command | Expected observation to record |
| ---: | --- | --- |
| 1 | open at 115200, wait 1 s | Boot text, board reset timing, idle outputs |
| 2 | `M` | Response text and mode state |
| 3 | `W 1750` | Center output waveform/position |
| 4 | `D F`, `V 0` | Forward state with zero propulsion |
| 5 | `V 25`, then `V 0` | Output magnitude and response timing |
| 6 | `D R`, `V 0` | Reverse state with zero propulsion |
| 7 | `S 1` | Brake output behavior and response |
| 8 | `Q 1` | Brake-release output behavior and response |
| 9 | unknown/malformed line | Parser behavior and retained output state |
| 10 | disconnect while nonzero request was last sent | Firmware timeout/watchdog behavior |

Archive raw bytes with timestamps, the board USB identity, firmware hash if
recoverable, and the exact host commit. That transcript becomes the fixture for
legacy-adapter tests and the authoritative replacement for reconstructed guesses.
