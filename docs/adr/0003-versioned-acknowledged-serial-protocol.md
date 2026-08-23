# ADR-0003: Use a versioned, acknowledged serial protocol and firmware watchdog

- Status: Accepted
- Date: 2026-08-21
- Decision owners: Car Interface maintainers
- Superseded in part by: [ADR-0005](0005-explicit-school-car-legacy-profile.md), for explicit compatibility with the demonstrated school car

## Context

The host and firmware need one unambiguous contract for command spelling,
ranges, timing, and steering semantics. Queue admission is not proof of firmware
receipt, stop commands must outrank motion, and communication loss needs an
independent firmware timeout.

## Decision drivers

- Detect corruption, incompatibility, loss, and stale responses.
- Distinguish enqueue, transmission, and application.
- Prioritize stop behavior and prevent replay of old motion.
- Provide an independently enforced communication-loss safe state.

## Considered options

1. Keep legacy text commands and add logging. This cannot establish integrity or
   unambiguous compatibility.
2. Use protocol-v1 ASCII frames with version, sequence, CRC, strict values,
   ACK/NACK, priority, heartbeat, and watchdog.
3. Adopt a binary/network protocol immediately. This adds tooling complexity
   without a current requirement that justifies it.

## Decision

Use option 2 as specified in [protocol.md](../protocol.md). Every command needs a
matching ACK/NACK. Emergency/safety commands outrank ordinary control and evict
pending motion when directed by the safety policy. Invalid traffic never
refreshes the firmware watchdog. Watchdog expiry independently zeros propulsion,
asserts brake, and inhibits motion.

## Consequences

- Host and firmware must be tested and recorded as a compatible pair.
- CRC detects accidental corruption but does not authenticate a malicious
  endpoint; physical/local access remains part of the threat model.
- Sequence and retry semantics add state that needs conformance testing.
- Legacy firmware remains incompatible with `car_v1`. ADR-0005 permits it only
  through a separately selected, write-only compatibility profile.

## Verification

- Golden frame and parser property tests.
- Dispatcher ordering, ACK/NACK, timeout, and queue-saturation tests.
- Firmware conformance corpus for malformed/overlength input.
- Wheels-clear host crash, cable removal, and watchdog measurements.

## Rollback or supersession

Rollback must restore the complete previously tested host and firmware pair.
Never add an automatic runtime legacy fallback. Breaking changes to `car_v1`
require a new protocol version and ADR. The explicit profile governed by
ADR-0005 does not change this protocol.
