# ADR-0002: Default to simulation and require explicit hardware opt-in

- Status: Accepted
- Date: 2026-08-21
- Decision owners: Car Interface maintainers
- Supersedes: hard-coded/import-time hardware startup in prototypes
- Superseded by: none

## Context

Prototype scripts could open hard-coded COM ports or send commands when run or
imported. Developers, test collectors, and operators need a reliable way to
explore the application without any chance of actuating a vehicle.

## Decision drivers

- Prevent accidental device access and motion.
- Make onboarding and automated testing hardware-independent.
- Make operator intent visible at the process boundary.
- Preserve a straightforward physical workflow after safety checks.

## Considered options

1. Infer hardware from connected ports. Rejected because detection is ambiguous
   and makes accidental activation easy.
2. Let a configuration file select hardware silently. Rejected because copied
   or stale configuration can surprise an operator.
3. Default to simulation and require a deliberately long command-line
   acknowledgement for every hardware process start.

## Decision

Use option 3. `car-interface run` defaults to simulation. Hardware requires both
`--mode hardware` and the exact
`--i-understand-this-controls-real-hardware` flag. The developer wrapper
`scripts/dev.py run-hardware` requires the same flag. Configuration and device
presence cannot substitute for acknowledgement.

Tests exclude hardware by default and require the separate `hardware` marker and
explicit test command.

## Consequences

- Typical runs, tests, and CI cannot open physical devices accidentally.
- Hardware commands are intentionally verbose and unsuitable for auto-start.
- The flag confirms intent but does not validate wiring, firmware, or the test
  area; operator checklists remain mandatory.
- Packaged builds must preserve the same default and gate.

## Verification

- CLI tests cover missing, misspelled, and present acknowledgement flags.
- Simulation tests assert only simulated adapter descriptions and no serial
  imports/connections.
- Hardware tests verify configuration errors fail before opening devices.
- Packaged-app smoke tests confirm simulation is the default.

## Rollback or supersession

If the gate fails, remove hardware distribution or disable hardware mode until a
fix is released. Weakening the acknowledgement requires a superseding ADR and
equivalent prevention of accidental activation.
