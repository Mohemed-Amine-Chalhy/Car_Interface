# Architecture

Car Interface uses a layered, dependency-injected architecture so control and
safety decisions can be tested without importing a GUI toolkit or opening a
device. The design replaces the historical large Tkinter scripts, which were
removed from the maintained tree and remain only in Git history.

## Dependency direction

```text
CLI / Tkinter UI
       |
       v
application services  --->  thread-safe presentation events
       |
       v
domain state + safety policy + protocol commands
       ^
       |
device interfaces  <---  real adapters or simulated adapters
```

Dependencies point toward the domain. Domain modules do not import Tkinter,
pygame, pyserial, RPLidar, filesystem configuration, or a wall clock.

## Repository layout

```text
src/car_interface/
  __main__.py             python -m car_interface
  cli.py                  argument parsing and composition root
  config.py               typed TOML/environment configuration
  diagnostics.py          non-invasive environment/support metadata
  factory.py              side-effect-free real/simulated adapter composition
  logging_config.py       console and bounded rotating logs
  domain/
    state.py              immutable state, device health, stable fault codes
    commands.py           typed commands and protocol-v1 codec
    safety.py             deterministic safety state machine
  services/
    control.py            lifecycle and device orchestration
    dispatcher.py         prioritized acknowledged command delivery
    events.py             bounded events for presentation
    lidar_analysis.py     pure scan normalization/obstacle calculations
    models.py             immutable presentation snapshots
  adapters/
    base.py               device Protocol interfaces and normalized values
    serial_transport.py   exclusive pyserial actuator transport
    pygame_controller.py  normalized game-controller input
    rplidar_source.py      threaded latest-scan source
    simulated.py          deterministic in-memory devices
  ui/
    ...                   Tkinter presentation only
tests/
  unit/                   pure behavior
  integration/            components with fakes/simulation
  hardware/               explicit opt-in only
scripts/
  bootstrap.ps1/.sh       reproducible setup
  dev.py                  one cross-platform developer command surface
```

The exact tree can evolve, but dependency direction and safety ownership are
stable architectural constraints.

## Domain

`ControlState` is an immutable snapshot with a monotonically increasing
revision. It contains the safety phase, mode, normalized command values, brake
and E-stop state, required-device health/freshness, and any active fault.

`SafetyStateMachine` applies a deterministic safety policy. Callers supply
monotonic timestamps and device observations. A
transition returns both the new state and an ordered decision describing
commands to dispatch. Stop-like decisions also state that pending motion must be
discarded.

This makes the important property testable: given the same state and event, the
same safe decision results without a serial port or GUI.

## Commands and protocol

`ControlCommand` validates opcode and value at construction. Speed and steering
are normalized percentages in `[-100, 100]`; hardware-specific conversion
belongs in compatible firmware, not the UI.

The dispatcher is the exclusive owner of `VehicleTransport`. It:

- assigns sequence numbers;
- encodes CRC-protected protocol frames;
- prioritizes emergency and safety commands;
- discards/coalesces pending motion when required;
- waits for the matching ACK/NACK within a bounded timeout;
- sends heartbeat traffic scheduled by the control monitor; and
- reports communication/protocol failures to the safety policy.

An enqueue result is not an acknowledgement. The application may show a command
as successful only after a matching protocol ACK.

See [protocol.md](protocol.md).

## Device adapters

The application depends on small structural interfaces:

- `VehicleTransport`: connect, transact one frame, disconnect;
- `ControllerSource`: connect, return a normalized snapshot, disconnect; and
- `LidarSource`: connect, return the newest immutable scan and monotonic
  timestamp, disconnect.

Real adapters translate library-specific data and errors at this boundary.
Simulated adapters implement the same interfaces and are selected by default.
Creating an adapter must not connect to a device; connection is an explicit
lifecycle operation.

The simulated actuator also models protocol sequence checking, ARM/BRK/EST
semantics, and a latched independent watchdog. Watchdog expiry enters its safe
state and rejects heartbeat/unsafe traffic with `WATCHDOG_TIMEOUT` until reset
or reconnect. It is a test oracle, not evidence about unprovided ESP32 firmware.

## Concurrency model

- Tkinter owns the main thread. All widget reads/writes occur there.
- One dispatcher context exclusively owns actuator serial transactions.
- The RPLidar adapter owns its scan thread and publishes only immutable latest
  snapshots.
- Controller polling and service work never call Tkinter directly.
- Workers publish bounded `ServiceEvent` values; the UI periodically drains
  them from the main thread.
- Scan events are coalesced because only the newest visualization matters.

The presentation event buffer never participates in safety decisions. A slow UI
must not delay heartbeat, E-stop, or fault handling.

Every worker has an idempotent stop operation and a bounded join. A daemon flag
is not the shutdown design.

## Lifecycle

1. CLI parses arguments without device side effects.
2. Configuration is loaded and validated.
3. Logging is configured.
4. The composition root selects simulated or real adapters.
5. Services start in a neutral, braked, disconnected state.
6. Explicit connect operations establish devices; arming remains separate.
7. Safety policy controls every state and motion transition.
8. Shutdown first requests safe output and disarm, stops producers, disconnects
   adapters, drains bounded cleanup, and closes the UI.

If the host cannot complete shutdown, the firmware watchdog and independent
physical cutoff remain essential.

## Configuration and observability

Configuration is immutable after load and follows the precedence documented in
[configuration.md](configuration.md). Unknown keys are errors.

The log format includes timestamp, level, logger, and thread. Operational
messages should also carry the relevant state, sequence, or stable fault code.
High-frequency scan points and controller axes must not flood logs. Credentials
are not required; device identifiers and local paths still require redaction
before sharing.

## Deferred vision scope

v0.1 contains no camera/vision adapter, OpenCV or Ultralytics dependency, model
asset, or vision-enabled packaging path. Vision is not installable or supported
in this release. A future proposal must use a new ADR and prove failure
isolation, bounded concurrency, asset provenance, and that vision cannot clear
or weaken a brake, E-stop, or fault.

## Historical code

The earlier root-level prototypes and `Manette/` snapshots were removed from the
maintained tree. Git history preserves them for archaeology only; they are not
supported entry points and must not be restored for hardware use. The small root
`main.py` is only a compatibility shim to the maintained package and contains no
application behavior.

See [ADR-0001](adr/0001-layered-production-architecture.md) for the decision.
