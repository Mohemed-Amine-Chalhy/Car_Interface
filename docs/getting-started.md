# Getting started

This guide prepares a development environment and runs Car Interface without
physical devices. Simulation is the only appropriate first run.

## 1. Prerequisites

- Git;
- [uv](https://docs.astral.sh/uv/) available on `PATH` (bootstrap uses it to
  install Python 3.13 from `.python-version`); and
- Windows 10/11 for full desktop and packaging work.

Linux and macOS may be used for pure domain, protocol, and test work. Device
names, controller behavior, Tk support, and packaged artifacts can differ and
are not release substitutes for Windows testing.

Do not install dependencies globally. Historical prototype scripts were removed
from the maintained tree because some performed hardware I/O directly; do not
restore or run them from Git history.

## 2. Bootstrap

Clone the canonical repository, change into its root, and run one bootstrap
script.

PowerShell:

```powershell
.\scripts\bootstrap.ps1
```

POSIX shell:

```bash
./scripts/bootstrap.sh
```

The script creates `.venv`, synchronizes the committed lockfile, and installs
pre-commit hooks. Camera/vision dependencies and model assets are not shipped or
installable in v0.1.

If dependency declarations changed intentionally, follow
[Development](development.md) to regenerate the lockfile. Do not work around
`--locked` failures by installing ad hoc packages.

## 3. Diagnose the environment

PowerShell:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py doctor
```

POSIX shell:

```bash
.venv/bin/python scripts/dev.py doctor
```

`doctor` reports tool availability, Python compatibility, Tk support,
configuration validity, and optional device dependencies. A missing physical
device is not an error for simulation.

## 4. Run simulation

```powershell
.\.venv\Scripts\python.exe scripts\dev.py run-sim
```

To let the application traverse the complete portfolio walkthrough
automatically, add the simulation-only showcase flag:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py run-sim --showcase
```

Simulation uses in-memory ESP32, controller, and Lidar adapters. It starts
neutral and braked, creates no serial connection, and is suitable for UI and
safety-state testing. Closing the application should leave the simulated
vehicle at zero speed with the brake asserted.

The simulated ESP32 enforces arm/brake/E-stop rules and runs an independent,
latched firmware-watchdog model. When valid traffic stops beyond its deadline,
it zeros speed, asserts brake, disarms, and NACKs heartbeat or unsafe traffic
with `WATCHDOG_TIMEOUT` until reset or reconnect. This verifies host behavior;
it does not prove any physical firmware implements the watchdog.

## 5. Run the quality gate

```powershell
.\.venv\Scripts\python.exe scripts\dev.py check
```

This runs formatting checks, linting, strict type checking, non-hardware tests,
coverage checks, and source/dependency checks. Use `check --skip-audit` when the
dependency vulnerability service is unavailable; CI runs that
network-backed check separately.

## Next steps

- Operators: [Operator guide](operator-guide.md)
- Contributors: [Development](development.md)
- Reviewers: [Software showcase](showcase.md)
- Hardware owners: [Safety](safety.md), then
  [Hardware setup](hardware-setup.md)
- Problems: [Troubleshooting](troubleshooting.md)
