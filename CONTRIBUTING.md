# Contributing

Thank you for helping improve Car Interface. This project controls machinery,
so a change that appears cosmetic can still affect timing, operator attention,
or failure behavior. Treat safety behavior as part of the public API.

The repository currently grants no open-source license. Do not submit material
unless you are authorized to do so and can identify its provenance. Acceptance
does not by itself grant permission to use or redistribute the project; public
distribution remains blocked until the copyright holders select a license.

## Before starting

1. Read [docs/safety.md](docs/safety.md) and
   [docs/architecture.md](docs/architecture.md).
2. Search existing issues and decisions under [docs/adr](docs/adr).
3. For a protocol, safety-state, dependency, or user-visible behavior change,
   agree on the design before implementation and add or update an ADR.
4. Never use physical hardware merely to discover what an unclear requirement
   means. Start with the simulator and fakes.

## Environment

On Windows:

```powershell
.\scripts\bootstrap.ps1
.\.venv\Scripts\python.exe scripts\dev.py doctor
```

On a POSIX development host:

```bash
./scripts/bootstrap.sh
.venv/bin/python scripts/dev.py doctor
```

The bootstrap process must use the committed lockfile. Do not hand-edit a lock
file. Use the project dependency workflow documented by `scripts/dev.py --help`
and include both the dependency declaration and regenerated lockfile in the same
change.

## Development workflow

1. Create a focused branch.
2. Add or update tests before changing safety behavior.
3. Make the smallest coherent change.
4. Run formatting and the complete local quality gate:

   ```powershell
   .\.venv\Scripts\python.exe scripts\dev.py format
   .\.venv\Scripts\python.exe scripts\dev.py check
   ```

5. Update documentation and `CHANGELOG.md` for user-visible changes.
6. Commit generated files only when the repository intentionally tracks them.

Pre-commit hooks are a fast feedback mechanism, not a replacement for
`scripts/dev.py check`. Do not skip hooks to merge a change. If a hook is wrong,
fix its configuration in a separate, reviewable commit.

## Change requirements

Every production-code change should have:

- typed interfaces and no unexplained `Any` in domain or safety code;
- deterministic unit tests without attached hardware;
- failure-path coverage where I/O or concurrency is involved;
- structured, non-sensitive logging;
- no device connection, movement, or long-running work at import time; and
- no Tkinter calls from worker threads.

Changes to stop behavior, command priority, limits, timeouts, controller loss,
Lidar staleness, startup, or shutdown also require:

- a safety requirement or invariant stated in the change description;
- tests that fail before and pass after the change;
- simulator or fault-injection evidence;
- an ADR if the safety model or protocol contract changes; and
- an explicit note that software tests do not constitute physical validation.

## Commit and review guidance

Prefer small commits with imperative subjects, for example:

```text
Add protocol acknowledgement timeout
Test controller-loss transition to safe state
Document firmware compatibility handshake
```

Keep refactoring separate from behavioral changes when practical. A pull
request should explain the problem, approach, safety impact, test evidence,
documentation changes, and rollback plan. Include screenshots only for UI work
and redact port names, usernames, tokens, and machine-specific paths.

At least one reviewer should understand the affected subsystem. Safety- and
protocol-related changes require a second reviewer and must not be self-merged.

## Tests involving hardware

Hardware-in-the-loop tests are isolated from normal pytest discovery. Run them
only when all of the following are true:

- the test rig is identified and reserved;
- the driven wheels are mechanically clear of the ground unless the test plan
  specifically requires controlled motion;
- an independent physical E-stop/power cutoff has been tested;
- the exact firmware and host revisions are recorded; and
- a second person is present for tests that can create motion.

Record the results using [docs/hardware-qualification.md](docs/hardware-qualification.md).

## Reporting unsafe behavior

Stop testing immediately, place the system in a mechanically safe state, and
follow [SECURITY.md](SECURITY.md). Do not publish exploitable or hazardous
details before maintainers can assess them.
