# ADR-0001: Use a layered production architecture with injected adapters

- Status: Accepted
- Date: 2026-08-21
- Decision owners: Car Interface maintainers
- Superseded by: none

## Context

The application coordinates device I/O, safety decisions, controller mapping,
Lidar processing, serial queues, and Tkinter updates. These concerns need clear
ownership so failure behavior remains testable and the UI stays responsive.

## Decision drivers

- Deterministic safety verification without physical hardware.
- Exclusive ownership of each device and clear concurrency boundaries.
- One production entry point and configuration model.
- Optional adapters that do not infect core domain dependencies.
- Clear boundaries between domain, service, adapter, and presentation code.

## Considered options

1. Keep all device and control behavior in the Tkinter application. This reduces
   the number of modules but couples UI and hardware behavior.
2. Use a layered package with immutable domain state, application services,
   injected adapters, and a passive UI.
3. Rewrite as a distributed or web system. This expands the security and
   operational surface without solving the local safety model first.

## Decision

Use option 2. Production code lives under `src/car_interface`. Domain modules
are pure and unaware of UI/device libraries. Services own lifecycle and
dispatch. Adapters normalize real or simulated devices. Tkinter consumes bounded
events only on its main thread. Configuration and CLI code form the composition
root and perform no import-time I/O.

## Consequences

- Safety transitions and protocol behavior can be unit-tested.
- Simulation and fault injection become first-class workflows.
- More interfaces and small modules require deliberate composition.
- Third-party libraries and untyped values remain constrained to adapters.

## Verification

- Import tests assert no ports/devices open.
- Strict types cover the production package.
- Integration tests replace every real adapter with simulation/fakes.
- GUI tests assert worker events are marshalled through the main thread.
- Architecture boundaries are reviewed with dependency/import checks.

## Rollback or supersession

Any replacement architecture requires a new ADR with equivalent or stronger
safety isolation and a tested migration path.
