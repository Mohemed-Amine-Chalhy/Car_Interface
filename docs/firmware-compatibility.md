# Firmware-host compatibility

Host code and vehicle firmware share a versioned command contract. Compatibility
is established through protocol behavior, timing, configuration, and regression
tests rather than a matching baud rate alone.

## Status

The host implements acknowledged `car_v1` and the demonstrated vehicle's
`school_car_legacy_v0` command profile. The installed firmware binary is not
stored in this repository, so its exact build remains TBD until it is captured
from the car during a vehicle-side validation session.

## Compatibility policy

- The protocol major version is the integer in every frame (`CAR,1`).
- Host and firmware MUST reject unsupported major versions.
- Additive behavior within a major version must not change existing opcode,
  range, ACK, stop, or watchdog semantics.
- Any incompatible framing, command meaning, range, safety-state, or timing
  change requires a new protocol major version and an ADR.
- A vehicle profile records the exact firmware artifact it was tested with;
  “latest firmware” is not a reproducible dependency.
- A downgrade restores a host/firmware/configuration set that was tested
  together.

Protocol v1 does not yet provide a command that reports a semantic firmware
build identifier. Until a reviewed identity/manifest extension exists, the
operator must verify firmware by controlled flashing and artifact SHA-256, while
the wire parser verifies protocol version on every frame. Adding an identity
command must be backward-compatible or advance the protocol version.

## Firmware build record

For every candidate firmware build, archive:

- repository URL and immutable source commit;
- toolchain, board package, board/partition settings, and library lock/versions;
- reproducible build instructions;
- binary name, size, and SHA-256;
- supported ESP32 and motor-driver hardware revisions;
- protocol version and configured baud rate;
- watchdog interval and safe-output implementation;
- boot/reset/brownout behavior;
- conformance and hardware validation report; and
- known limitations and rollback instructions.

## Compatibility matrix

Maintain this table in each release branch:

| Host profile | Protocol | Firmware artifact | Hardware revision | Status | Evidence |
| --- | --- | --- | --- | --- | --- |
| `car_v1` | `CAR,1` | Conforming firmware build | Configured vehicle | Host-tested | Simulator and protocol regression suite |
| `school_car_legacy_v0` | Newline commands | 2025 vehicle build | School car | Adapter-tested | Encoding, pacing, partial-write, configuration, and composition tests |

Add the captured firmware hash, board revision, serial transcript, and vehicle
session results to this matrix after hardware validation.

## Flash and verify

The exact flashing tool depends on the firmware project and board. The firmware
release must provide its own commands. In all cases:

1. isolate propulsion power;
2. record the board identity and previous firmware hash/version;
3. verify the downloaded artifact's SHA-256;
4. flash with reviewed board settings;
5. power-cycle and verify safe output before connecting propulsion;
6. run protocol conformance with no propulsion power; and
7. repeat wheels-clear watchdog and E-stop validation.

Treat a new firmware artifact as a new compatibility test target.

## Host behavior on incompatibility

The host must remain or return zero-speed and braked, display a stable protocol
fault, discard pending motion, and refuse to arm. Operators must not be offered
a “continue anyway” control.
