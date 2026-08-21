# ADR-0001: Use a layered production architecture with injected adapters

- Status: Accepted
- Date: 2026-08-21
- Decision owners: Car Interface maintainers
- Supersedes: prototype monolithic structure
- Superseded by: none

## Context

The repository began as several large and near-duplicate Tkinter scripts. Device
I/O, safety decisions, controller mapping, Lidar processing, serial queues, and
widget updates were interleaved. Some modules connected to devices at import
time, some callbacks blocked the UI thread, and worker threads could update
Tkinter directly. This made failure behavior hard to reason about or test.

## Decision drivers

- Deterministic safety verification without physical hardware.
- Exclusive ownership of each device and clear concurrency boundaries.
- One production entry point and configuration model.
- Optional adapters that do not infect core domain dependencies.
- Incremental migration from historical behavior.

## Considered options

1. Continue improving the largest combined Tkinter file. This preserves short-
   term behavior but retains coupling and unsafe test boundaries.
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

Historical root and `Manette/` prototypes are removed from the maintained tree;
Git history retains them for archaeology only. They are excluded from supported
entry points and must not be restored for hardware use.

## Consequences

- Safety transitions and protocol behavior can be unit-tested.
- Simulation and fault injection become first-class workflows.
- More interfaces and small modules require deliberate composition.
- Any behavior recovered from history must be specified and migrated through the
  maintained architecture instead of copied blindly.
- Third-party libraries and untyped values remain constrained to adapters.

## Verification

- Import tests assert no ports/devices open.
- Strict types cover the production package.
- Integration tests replace every real adapter with simulation/fakes.
- GUI tests assert worker events are marshalled through the main thread.
- Architecture boundaries are reviewed with dependency/import checks.

## Rollback or supersession

Do not roll back to a monolithic hardware entry point. A replacement architecture
requires a new ADR demonstrating equivalent or stronger safety isolation and a
tested migration/rollback path.
