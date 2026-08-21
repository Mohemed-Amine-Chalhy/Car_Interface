# Firmware compatibility

Host code and ESP32 firmware form one safety-relevant release unit. A matching
baud rate or an `ACK` string is not enough to establish compatibility.

## Current status

The host production package defines protocol v1 in [protocol.md](protocol.md).
The repository has not yet identified a qualified ESP32 firmware source tree or
immutable firmware artifact. Therefore no physical firmware is currently listed
as production-qualified.

Do not recover commands from removed prototype files in Git history. Those
commits contain multiple incompatible formats and no enforceable
version/CRC/watchdog contract.

## Compatibility policy

- The protocol major version is the integer in every frame (`CAR,1`).
- Host and firmware MUST reject unsupported major versions.
- Additive behavior within a major version must not change existing opcode,
  range, ACK, stop, or watchdog semantics.
- Any incompatible framing, command meaning, range, safety-state, or timing
  change requires a new protocol major version and an ADR.
- A host release lists exact qualified firmware artifacts; “latest firmware” is
  not an acceptable production dependency.
- Downgrade is allowed only to a host/firmware/configuration set that was
  previously qualified together.

Protocol v1 does not yet provide a command that reports a semantic firmware
build identifier. Until a reviewed identity/manifest extension exists, the
operator must verify firmware by controlled flashing and artifact SHA-256, while
the wire parser verifies protocol version on every frame. Adding an identity
command must be backward-compatible or advance the protocol version.

## Required firmware release record

For every candidate firmware build, archive:

- repository URL and immutable source commit;
- toolchain, board package, board/partition settings, and library lock/versions;
- reproducible build instructions;
- binary name, size, and SHA-256;
- supported ESP32 and motor-driver hardware revisions;
- protocol version and configured baud rate;
- watchdog interval and safe-output implementation;
- boot/reset/brownout behavior;
- conformance and hardware qualification report; and
- known limitations and rollback instructions.

## Compatibility matrix

Maintain this table in each release branch:

| Host release | Protocol | Firmware artifact | Hardware revision | Status | Evidence |
| --- | --- | --- | --- | --- | --- |
| Unreleased | 1 | Not yet provided | Not yet qualified | **Blocked** | Complete protocol and hardware qualification |

Never change an existing row from blocked to qualified without linking immutable
artifacts and signed test evidence.

## Flash and verify

The exact flashing tool depends on the firmware project and board. The firmware
release must provide its own commands. In all cases:

1. isolate propulsion power;
2. record the board identity and previous firmware hash/version;
3. verify the downloaded artifact's SHA-256;
4. flash with reviewed board settings;
5. power-cycle and verify safe output before connecting propulsion;
6. run protocol conformance with no propulsion power; and
7. repeat wheels-clear watchdog and E-stop qualification.

Flashing firmware invalidates prior qualification unless the exact artifact was
already covered.

## Host behavior on incompatibility

The host must remain or return zero-speed and braked, display a stable protocol
fault, discard pending motion, and refuse to arm. Operators must not be offered
a “continue anyway” control.
