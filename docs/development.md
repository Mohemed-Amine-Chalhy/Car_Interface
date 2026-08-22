# Development workflow

This project uses one reproducible Python 3.13 environment and one task runner.
The production package is under `src/car_interface`; historical prototype
scripts have been removed from the maintained tree.

## Environment and dependency model

`uv` manages Python, synchronization, and the committed `uv.lock`.

```powershell
.\scripts\bootstrap.ps1              # base app plus dev/build groups
.\scripts\bootstrap.ps1 -Check       # bootstrap, diagnose, and run all checks
```

POSIX equivalents are `./scripts/bootstrap.sh` and `--check`.

Declared dependency sets:

- runtime: pygame, pyserial, and rplidar-roboticia;
- `dev`: Ruff, mypy, pytest/coverage/timeout, Hypothesis, Bandit, pip-audit,
  pre-commit, and type stubs; and
- `build`: build, the locked Hatchling backend, and PyInstaller.

Camera/vision work is deferred. v0.1 has no vision adapter, dependency extra,
model asset, bootstrap option, or build flag. A future implementation requires a
clear architecture, isolated execution, tests, and a packaging plan.

To change dependencies:

1. edit `pyproject.toml` with a justified compatible range;
2. run `uv lock`;
3. run `uv sync --locked --all-groups`;
4. run `scripts/dev.py check`; and
5. review declaration, lockfile, transitive dependency, and vulnerability
   changes together.

Do not revive `requirements.txt` as the source of truth.

## Developer commands

Run commands with the environment's Python, or invoke `python scripts/dev.py`;
the runner safely relaunches itself through the locked uv environment.

| Command | Purpose |
| --- | --- |
| `doctor` | Verify Python 3.13, Git, uv, required modules, Tkinter, and lock freshness without opening devices |
| `format [--check]` | Apply or verify Ruff formatting |
| `lint [--fix]` | Run Ruff's configured rules, optionally applying safe fixes |
| `typecheck` | Run strict mypy on production code and scripts |
| `test [--hardware] [-- PYTEST_ARGS]` | Run non-hardware tests by default, or the isolated hardware marker explicitly |
| `security [--skip-bandit] [--skip-audit]` | Run source analysis and locked runtime dependency vulnerability checks |
| `check [--ci] [--skip-audit]` | Lock, format, lint, type, test/coverage, and source/dependency checks; `--ci` writes reports |
| `run-sim [--config PATH]` | Start only simulated adapters |
| `run-hardware ... --i-understand-this-controls-real-hardware` | Explicit physical-device start; see the operator guide |
| `build [--clean]` | Build wheel/sdist and, on Windows, the desktop bundle/archive/checksum |
| `checksums` | Create a sorted `dist/SHA256SUMS.txt` manifest for release artifacts |
| `release-check [options]` | Require a clean, exactly tagged Windows checkout, run all checks, build, smoke-test, and checksum the release |

`--allow-dirty`, `--allow-untagged`, and `--skip-audit` support local
diagnostics. The default release check expects a clean tagged checkout and all
dependency checks.

## Style and type checking

Ruff is the formatter, import sorter, linter, and modernization tool. Its target
is Python 3.13 and line length is 100. Do not add file-wide
ignores to silence a design problem. A narrow rule exemption needs a comment and
review.

Mypy runs in strict mode for `src/car_interface` and `scripts`. Missing stubs are
allowed only for the configured third-party adapter libraries. Keep untyped
values at adapter boundaries; convert them immediately to validated domain
types.

General rules:

- keep imports side-effect free;
- inject time, adapters, and callbacks;
- use immutable dataclasses/enums for domain state;
- validate at boundaries and fail closed;
- catch only errors a layer can handle meaningfully;
- use monotonic time for deadlines/freshness and wall time only for records;
- bound queues, buffers, retries, logs, and shutdown waits;
- never log raw continuous controller/Lidar streams at normal levels; and
- never update Tkinter widgets from a worker thread.

## Git hooks

Bootstrap installs pre-commit and pre-push hooks.

Pre-commit checks include file endings/whitespace, syntax and configuration
formats, case conflicts, merge markers, debug statements, private keys, a 2 MB
new-file limit, lockfile consistency, Ruff lint fixes, and Ruff formatting.

Pre-push runs `scripts/dev.py check --skip-audit`; the separate CI dependency
job runs the network-backed vulnerability check and must pass before merge. Run
`scripts/dev.py check` without `--skip-audit` locally when network access is
available.

Run all hooks manually with:

```powershell
uv run --locked --group dev pre-commit run --all-files
uv run --locked --group dev pre-commit run --all-files --hook-stage pre-push
```

## CI

Pull requests and primary-branch pushes run:

- quality checks on Windows and Ubuntu;
- separate source and dependency vulnerability checks; and
- a Windows build after quality and dependency checks pass.

Actions are pinned to immutable commits. After the quality and dependency jobs
pass, CI builds, smoke-tests, checksums, and uploads the Windows package.

## Adding a device adapter

1. Add or reuse a small interface in `adapters/base.py`.
2. Keep third-party imports inside the adapter when practical.
3. Do not connect in `__init__` or at module import.
4. Normalize values and translate library errors to stable adapter errors.
5. Make disconnect idempotent and worker shutdown bounded.
6. Add a deterministic fake/simulator and contract tests.
7. Feed health/freshness into the safety policy; never let an adapter mutate UI
   or domain state directly.
8. Document configuration, supported behavior, and validation requirements.

## Architecture changes

Create an ADR from [adr/0000-template.md](adr/0000-template.md) for changes to
layering, safety state, protocol, concurrency, runtime modes, dependency policy,
or packaging. ADRs are append-only records: supersede an accepted decision with
a new ADR instead of rewriting its history.
