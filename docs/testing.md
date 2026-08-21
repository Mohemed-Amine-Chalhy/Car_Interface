# Testing strategy

Automated tests default to zero hardware access. No ordinary test may open a
serial port, initialize a physical controller or Lidar, or send a motion
command. Hardware tests require a separate marker and explicit command.

## Standard test run

```powershell
.\.venv\Scripts\python.exe scripts\dev.py test
```

Pass additional pytest arguments after `--`:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py test -- -k emergency_stop -vv
```

The default pytest configuration enables strict configuration/markers, a
30-second per-test timeout, branch coverage, terminal missing-line output, and
XML coverage output at `.reports/coverage.xml`. The repository-wide minimum is
80% branch coverage. Safety/domain modules should remain at or above 90% branch
coverage even where the aggregate gate is lower.

## Test layers

### Unit

The pure, deterministic unit suite must cover:

- state invariants and every valid/invalid transition;
- command value/range validation and priority;
- CRC golden vectors, frame parsing, malformed input, and response matching;
- controller normalization/dead zones;
- Lidar geometry, units, field of view, path width, threshold, and freshness;
- configuration precedence, coercion, unknown fields, and unsafe ranges; and
- bounded event/queue behavior.

Use injected monotonic timestamps. Do not sleep to test timeouts.

### Integration and simulation

The `integration` suite must cover multiple production components with simulated
or fake adapters:

- startup/connect/arm/drive/brake/disarm/shutdown;
- simulated firmware ARM/BRK/EST enforcement and latched watchdog expiry;
- priority inversion and motion-queue eviction;
- ACK, NACK, late/mismatched response, timeout, corruption, and disconnect;
- controller and Lidar loss/staleness;
- worker failure and event-buffer saturation;
- repeated connect/disconnect and idempotent cleanup; and
- UI smoke behavior with events marshalled to Tkinter's thread.

Simulation tests assert observable vehicle state, not only log text.

### Property tests

Hypothesis is appropriate for protocol bytes/fields, normalized range clamping,
arbitrary event sequences, queue sizes, and Lidar point clouds. Important
properties include:

- invalid input never creates an out-of-range domain command;
- speed is zero in every phase other than `DRIVING`;
- E-stop/fault remains latched without a valid reset;
- reset never restores previous motion; and
- any required-device failure sequence eventually reaches a safe phase.

### GUI

GUI tests use the `gui` marker and simulated services. They must verify construction,
state rendering, control enablement, event draining, E-stop availability, and
close behavior. A headless CI host may skip tests that require a real display;
the Windows package smoke test remains required.

### Hardware-in-the-loop

Only files under `tests/hardware` with the `hardware` marker may access devices.
The command is intentionally explicit:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py test --hardware
```

The runner sets `CAR_INTERFACE_HARDWARE_TESTS=1` only for that subprocess. A
hardware test should also require explicit port/rig configuration and fail
closed when it is absent. Never set the variable globally or remove the marker.

Before running, satisfy the controls in [CONTRIBUTING.md](../CONTRIBUTING.md) and
record results in [hardware-qualification.md](hardware-qualification.md).

## Safety regression matrix

| Trigger | Expected domain result | Required output behavior |
| --- | --- | --- |
| Controller disconnect | Latched `FAULT` | Evict motion, speed 0, brake on |
| Lidar stale/lost when required | Latched `FAULT` | Evict motion, speed 0, brake on |
| Serial disconnect | `DISCONNECTED` or latched fault per lifecycle | Stop best effort; firmware watchdog independently stops |
| Missing/mismatched ACK | Latched `FAULT` | Evict motion; no command considered applied |
| Software E-stop | Latched `EMERGENCY_STOP` | Emergency priority, speed 0, brake on |
| Brake/disarm | `BRAKING`/`SAFE_CONNECTED` | Evict pending motion, speed 0, brake on |
| Arm/resume with retained throttle | Reject transition | Remain braked until fresh neutral and explicit re-arm |
| Brake release without Arm | Remain `BRAKING` | Brake stays asserted; no queued motion resumes |
| Reset with unhealthy device | Remain latched | No motion command |
| Valid reset | `SAFE_CONNECTED` | Zero speed/brake; explicit re-arm required |
| Application close/crash | No host motion continuation | Best-effort stop plus firmware watchdog |
| Simulated firmware watchdog expiry | Latched safe firmware state | Speed 0, brake on, disarmed; heartbeat/unsafe traffic NACKed until reset/reconnect |

## Quality gate

```powershell
.\.venv\Scripts\python.exe scripts\dev.py check
```

CI uses `check --ci --skip-audit` for the platform matrix and runs the complete
audit in a separate security job. After the documented licensing gate is
resolved, a published release uses `release-check` without skips.

## Writing reliable tests

- Assert state, commands, ordering, and deadlines rather than implementation
  details.
- Prefer fakes to mocks for stateful devices.
- Never rely on test order or global device state.
- Give each worker a bounded cleanup assertion.
- Treat flaky timing as a defect, especially around stop behavior.
- Add a regression test with every defect fix.
- Keep logs as supporting evidence; they are not the sole assertion.
