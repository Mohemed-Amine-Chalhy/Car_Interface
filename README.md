# Car Interface

Car Interface is a Windows-first desktop control application for an ESP32-based
vehicle. Its v0.1 application scope is manual driving with a game controller,
RPLidar visualization and obstacle-assist inputs, and a safe-by-default
simulator for development without hardware.

## Project background and demonstration

Car Interface grew out of an autonomous-driving car built by a
multidisciplinary team of engineers. Mohamed Amine Chalhy was part of that team
and was responsible for the vehicle software and AI models. The complete
engineering team is recognized in [Credits](docs/credits.md).

The maintained v0.1 package focuses on manual control, Lidar-assisted safety,
and simulation. The AI and computer-vision work from the original engineering
project is historical context and is not included in the current release.

### Original project video

![Original autonomous-driving car and control-interface demonstration](https://media4.giphy.com/media/v1.Y2lkPTc5MGI3NjExaHA0dHo1YW5wbjdua25ob2ZmN2Zya3Y1cnFocGFhb2E0c3g3Nm5kZiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/IpNjWXPWk869qG8ss6/giphy.gif)

[Watch the project showcase on Instagram](https://www.instagram.com/p/DJD9AVDM7V6/)

> [!WARNING]
> This software is not a certified safety system and has not been validated on
> every physical vehicle. A moving vehicle can injure people and damage
> property. Use an independent, physical emergency-stop/power cutoff, keep the
> test area controlled, and complete the hardware qualification checklist before
> allowing the wheels to contact the ground. Never depend on the GUI, Lidar, a
> wireless controller, or a host-computer process as the only stop mechanism.

## Status and scope

The maintained tree now contains one production package and its supporting
tests/tooling. Historical prototypes were removed from the working tree and
remain available only through Git history. The v0.1 scope is:

- simulation is the default execution mode;
- physical hardware requires explicit operator opt-in;
- ESP32, controller, and Lidar failures must transition to a safe state;
- the host/firmware protocol is versioned and requires acknowledgements and a
  firmware-side watchdog;
- camera, computer-vision, and YOLO support is deferred: v0.1 ships no vision
  adapter, dependency extra, model asset, or vision-enabled build path; and
- simulation and software qualification do not establish physical safety.

No release should be described as hardware-qualified until the checklist in
[Hardware qualification](docs/hardware-qualification.md) has been completed for
the exact vehicle, firmware, host build, and controller model.

The repository does not include an identified, qualified ESP32 firmware
artifact. Physical operation and any hardware-qualified release are therefore
currently blocked. Public or commercial distribution is also blocked pending a
copyright-holder license decision; see [LICENSE](LICENSE).

## Quick start: simulator

Requirements:

- Windows 10/11 for the full desktop application;
- [uv](https://docs.astral.sh/uv/), which installs the Python version declared in
  `.python-version`; and
- PowerShell 7+ recommended on Windows. A POSIX bootstrap script is also
  provided for logic-only development on Linux/macOS.

From PowerShell:

```powershell
.\scripts\bootstrap.ps1
.\.venv\Scripts\python.exe scripts\dev.py doctor
.\.venv\Scripts\python.exe scripts\dev.py run-sim
```

From a POSIX shell:

```bash
./scripts/bootstrap.sh
.venv/bin/python scripts/dev.py doctor
.venv/bin/python scripts/dev.py run-sim
```

Bootstrap creates a local virtual environment, installs the locked development
dependencies, and installs the repository's Git hooks. It does not connect to
hardware.

If the bootstrap scripts or lockfile are not present in your checkout, that
checkout predates the maintained environment and must not be used for hardware
operation.

## Hardware operation

**Current status: BLOCKED.** No qualified firmware artifact or completed
physical-hardware qualification is present. The command below exists for
controlled commissioning only after those blockers are resolved; it is not an
authorization to operate a vehicle.

Hardware mode is deliberately not a quick-start path. Before commissioning it:

1. Read [Safety](docs/safety.md) and prepare an independent physical cutoff.
2. Verify wiring and power domains using [Hardware setup](docs/hardware-setup.md).
3. Install the externally verified firmware artifact and complete the protocol
   validation checks in
   [Firmware compatibility](docs/firmware-compatibility.md).
4. Configure ports and limits as described in
   [Configuration](docs/configuration.md).
5. Run the wheels-off-ground procedure in the
   [Operator guide](docs/operator-guide.md).
6. Start hardware mode only with the exact explicit acknowledgement shown below.

Do not bypass the acknowledgement in scripts, shortcuts, or application code.

```powershell
.\.venv\Scripts\python.exe scripts\dev.py run-hardware `
  --config C:\path\to\vehicle.toml `
  --i-understand-this-controls-real-hardware
```

## Quality checks

The local check command is the same quality gate used by CI:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py check
```

Individual commands include `format`, `lint`, `typecheck`, `test`, `security`,
`build`, `sbom`, `checksums`, and `release-check`. Run
`python scripts/dev.py --help` for the exact options in the current checkout.
Hardware-in-the-loop tests are always opt-in and are never part of ordinary test
collection.

## Documentation

- [Documentation map](docs/README.md)
- [Getting started](docs/getting-started.md)
- [Operator guide](docs/operator-guide.md)
- [Safety model](docs/safety.md)
- [Configuration reference](docs/configuration.md)
- [Hardware setup](docs/hardware-setup.md)
- [Architecture](docs/architecture.md)
- [Serial protocol](docs/protocol.md)
- [Development](docs/development.md)
- [Testing](docs/testing.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Release and rollback](docs/release.md)
- [Threat model](docs/threat-model.md)
- [Support](docs/support.md)
- [Credits](docs/credits.md)

## Contributing and security

See [CONTRIBUTING.md](CONTRIBUTING.md) before opening a change. Report a
potential vulnerability or unsafe behavior privately according to
[SECURITY.md](SECURITY.md), not in a public issue. Community participation is
covered by [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## Copyright, licensing, and third-party material

No open-source license is currently granted. Copyright remains with the
respective holders, and public or commercial distribution is blocked until they
authorize a common license. Read [LICENSE](LICENSE) and complete the provenance
review in [Third-party assets](docs/third-party-assets.md) before distributing
any artifact.
