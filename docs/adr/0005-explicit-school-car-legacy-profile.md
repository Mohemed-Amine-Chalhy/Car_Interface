# ADR-0005: Add an explicit school-car legacy protocol profile

- Status: Accepted
- Date: 2026-08-22
- Decision owners: Car Interface maintainers
- Supersedes: the blanket legacy exclusion in ADR-0003
- Superseded by: none

## Context

The demonstrated school car used newline-delimited commands understood by its
ESP32/Arduino-era firmware. The current `car_v1` protocol is deliberately
incompatible because it adds sequence numbers, CRC protection, ACK/NACK, and a
watchdog contract. Replacing the vehicle firmware is not a prerequisite for
using the current host architecture with the existing car.

The historical firmware did not provide command-correlated acknowledgements.
The host must not report an operating-system serial write as firmware
acknowledgement.

## Decision

Keep `car_v1` as the default and add `school_car_legacy_v0` as an explicit
configuration choice. Do not probe the port or auto-detect a protocol.

The compatibility profile emits the historical ASCII lines:

- `A` and `M` for automatic/remote and manual modes;
- `D F` or `D R`, immediately followed by non-negative `V <percent>`, for a
  signed speed request;
- `W <raw>` using configured minimum/center/maximum piecewise calibration;
- `S 1` and `Q 1` for brake application and release.

Arming selects `A`, matching the demonstrated controller-driving path.
Disarming asserts the brake before returning to `M`. Emergency stop and reset
write zero speed and apply the brake. The dispatcher keeps all frames from one
domain command contiguous and enforces a configurable inter-frame interval,
which defaults to 50 ms.

The compatibility profile does not synthesize ACKs, sequences, checksums, or a
firmware heartbeat. Receipts explicitly distinguish all-frames-written from
firmware-acknowledged. Hardware mode and protocol selection remain explicit.

## Consequences

- The current host can emit the recovered school-car dialect without weakening
  or changing `car_v1`; physical compatibility still requires bench validation.
- Legacy delivery confirms serial writes only; it cannot prove firmware
  parsing, actuator application, or mechanical response.
- Steering calibration and controller-axis inversion are vehicle-profile
  settings and must be measured on the physical car.
- The ESP32-WROOM-32 development board and Arduino Uno R3 are confirmed
  components, but their roles and GPIO assignments remain undocumented until
  the original firmware or wiring is recovered.

## Verification

- Golden tests cover every historical command mapping.
- Tests cover asymmetric piecewise steering calibration and inversion of the
  physical controller steering axis.
- Dispatcher tests cover contiguous signed-speed writes, pacing, partial-write
  reporting, and truthful write-only receipts.
- Existing protocol-v1 codec, dispatcher, simulation, and integration tests
  remain mandatory.

## Rollback or supersession

Remove the compatibility profile only after the physical car has been migrated
to a validated `car_v1` firmware. Do not replace explicit selection with
auto-detection.
