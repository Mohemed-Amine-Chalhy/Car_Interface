# Vehicle command protocol

The `school_car_legacy_v0` profile uses short UTF-8/ASCII commands terminated by
line feed. This page defines the candidate vehicle dialect used for physical-car
validation and by the explicit compatibility adapter.

## Vehicle command dialect

| Host operation | Candidate wire command | Host behavior |
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

The vehicle host configuration defines this transport behavior:

- 115200 baud;
- serial timeout 0.5 seconds;
- one-second wait after opening the port;
- line-feed command terminator;
- minimum 50 ms between queued writes; and
- newline-delimited responses accepted for display, without a defined response
  grammar or command-to-response correlation.

These values are candidate settings. The missing firmware and serial transcript
prevent them from being called a verified board contract.

## Protocol selection

Other board-firmware variants accept compact commands such as `V50`, `D`, `R`,
`F`, `Z`, `S`, or `A`. Those spellings are not interchangeable with this
profile. The host therefore selects a named dialect through configuration and
never labels a command set only by its controller-board family. Confirm the
installed dialect with a captured session from the physical car.

## Host protocol profiles

The host supports both protocols through explicit, configuration-selected
strategies in
[`protocol_profiles.py`](../../src/car_interface/domain/protocol_profiles.py).
It never probes a serial device to guess which dialect is attached.

The default [protocol v1](../protocol.md) profile sends frames such as:

```text
!CAR,1,CMD,0,SPD,25*E24B
```

Protocol v1 adds normalized signed requests, sequence numbers, CRC-16, ACK/NACK,
arming, heartbeat, and reset semantics. The `school_car_legacy_v0` profile maps
the same domain commands into the vehicle lines above:

```text
domain command
  ├─ car_v1                -> checksummed request + correlated ACK/NACK
  └─ school_car_legacy_v0  -> vehicle line(s) + write-only receipt
```

Signed speed requests become direction plus magnitude (`D F`/`D R`, then
`V n`), normalized steering is mapped piecewise through the configured
200/1750/2900 calibration, brake uses `S 1`/`Q 1`, and stop/reset operations
emit `V 0` followed by `S 1`. The dispatcher enforces the configured 50 ms
spacing between school-car frames.

The school-car profile does not generate a synthetic ACK merely because a line
was written. Dispatch receipts expose the selected profile, number of frames
written, and `acknowledged = false`. Validate the commands against the installed
firmware with the transcript below before a physical run.

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

Archive raw bytes with timestamps, the board USB identity, firmware hash when
available, and the exact host commit. That transcript becomes the fixture for
adapter tests and the validation evidence for the candidate settings above.
