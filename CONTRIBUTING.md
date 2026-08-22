# Contributing

Car Interface welcomes focused improvements to the desktop application,
simulation, device adapters, firmware compatibility, tests, and documentation.

## Set up the project

On Windows:

```powershell
.\scripts\bootstrap.ps1
.\.venv\Scripts\python.exe scripts\dev.py doctor
```

On Linux or macOS for logic-only development:

```bash
./scripts/bootstrap.sh
.venv/bin/python scripts/dev.py doctor
```

The bootstrap scripts synchronize the committed `uv.lock` and install the Git
hooks. Update `pyproject.toml` and regenerate `uv.lock` together when changing a
dependency.

## Development workflow

1. Create a focused branch from `main`.
2. Add or update tests for the behavior being changed.
3. Keep refactoring separate from behavioral changes where practical.
4. Run formatting and the full local quality gate:

   ```powershell
   .\.venv\Scripts\python.exe scripts\dev.py format
   .\.venv\Scripts\python.exe scripts\dev.py check
   ```

5. Update documentation and `CHANGELOG.md` for user-visible changes.
6. Open a pull request that explains the implementation and verification.

Pre-commit hooks provide fast feedback, while `scripts/dev.py check` is the
complete gate used before review.

## Code expectations

Production changes should:

- preserve typed interfaces and pass strict mypy;
- include deterministic tests that run without attached hardware;
- cover relevant I/O, error, timeout, and concurrency paths;
- keep device connections and long-running work out of module imports;
- keep Tkinter access on the UI thread;
- use bounded queues and explicit cleanup for background workers; and
- avoid committing machine-specific ports, local paths, credentials, generated
  output, or large binary assets.

Changes to public configuration or a serial protocol should update the matching
example, tests, and technical documentation in the same pull request. Use an
ADR when the architecture or compatibility contract changes materially.

## Hardware-related changes

State the hardware impact clearly in the pull request. Include:

- affected vehicle profile, board, firmware, protocol, and calibration values;
- simulator or fake-adapter regression coverage;
- bench-test setup and exact host/firmware commits when physical testing was
  performed;
- observed command/response behavior and relevant logs; and
- any compatibility or rollback considerations.

Physical testing is optional for ordinary software contributions. Do not imply
that a software-only test validates an untested board or vehicle configuration.

## Commits and pull requests

Prefer small commits with imperative subjects, for example:

```text
Add legacy steering calibration profile
Test fragmented serial responses
Document ESP32 board configuration
```

A pull request should contain one coherent change, pass the quality gate, and
identify any remaining work. Add screenshots for visible UI changes and concise
logs or measurements for device behavior.

Report security vulnerabilities privately according to [SECURITY.md](SECURITY.md).
