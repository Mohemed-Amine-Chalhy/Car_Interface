# Project results and evidence

This page separates what the team demonstrated on the physical 2025 car from
what the maintained codebase verifies automatically today. That distinction
makes the project easier to evaluate without overstating measurements that were
not retained.

## Physical prototype outcomes

| Outcome | Available evidence |
| --- | --- |
| A custom car was physically built | Recorded project media and the twelve-person engineering-team record |
| Host-to-vehicle control worked on the prototype | Demonstration of the car with the Python operator workflow; historical source remains in Git history |
| Game-controller operation was integrated | Historical application implementation and physical demonstration |
| RPLidar sensing was integrated | Historical 2D visualization, path-corridor processing, and obstacle-assistance implementation |
| YOLO perception was integrated | Historical camera inference code with detections, class labels, and confidence display |
| ESP32 and Arduino Uno R3 electronics were used | Confirmed hardware from the original build |

[Watch the project showcase](https://www.instagram.com/p/DJD9AVDM7V6/)
or return to the [Engineering case study](case-study.md). The
[perception dossier](perception/README.md) traces the historical implementation
and a measurable restoration path.

## Maintained software evidence

The current branch provides a repeatable engineering baseline:

| Quality area | Evidence in the repository |
| --- | --- |
| Automated behavior | More than 100 tests span unit, integration, and regression suites; physical tests are separately opt-in |
| Coverage | CI enforces an aggregate coverage floor of 80% with branch coverage enabled |
| Type quality | Strict mypy checks cover the application package, tooling script, and entry point |
| Code quality | Ruff formatting and linting run locally, in pre-commit, and in CI |
| Security hygiene | Bandit source scanning and locked-dependency vulnerability auditing |
| Cross-platform verification | Quality jobs execute on current Windows and Ubuntu GitHub runners |
| Reproducibility | Python 3.13, a committed `uv.lock`, bootstrap scripts, and locked CI synchronization |
| Packaging | Wheel, source archive, Windows application bundle, SHA-256 checksums, and CycloneDX SBOM workflow |
| Maintainability | Typed layered architecture with isolated domain, services, adapters, UI, and configuration |
| 2025-car compatibility | Explicit `school_car_legacy_v0` translation with tests for command mapping, steering calibration, pacing, partial writes, configuration, and service composition |

The CI workflow is visible in
[GitHub Actions](https://github.com/Mohemed-Amine-Chalhy/Car_Interface/actions/workflows/ci.yml).

## Reproduce the software checks

After following [Getting started](getting-started.md), run:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py check
```

The equivalent locked command used by CI is:

```powershell
uv run --locked --group dev python scripts/dev.py check --ci --skip-audit
```

Dependency auditing runs in the separate security job and can be reproduced
with:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py security
```

See [Testing](testing.md) for suite boundaries and hardware-test opt-in rules.

## Results that were not retained

The original project media proves an integrated physical demonstration, but it
does not provide a controlled benchmark dataset. The repository therefore does
not publish invented values for:

- maximum vehicle speed or braking distance;
- host-to-actuator command latency;
- Lidar scan rate under the demonstrated configuration;
- camera frame rate or YOLO inference latency;
- detection precision, recall, or mAP;
- battery endurance; or
- long-duration reliability.

Likewise, the repository demonstrates YOLO inference integration but does not
contain training datasets, experiment logs, or model artifacts that would
support a claim of custom model training.

## Next physical validation report

The existing car provides a clear route to turn the qualitative demonstration
into a measured engineering report. A future test session should capture:

1. exact ESP32 and Arduino firmware revisions and board responsibilities;
2. wiring, pin assignments, actuator ranges, and steering center calibration;
3. serial command/response transcripts and end-to-end command latency;
4. repeatable stopping distances at several commanded speeds;
5. Lidar scan frequency and distance error at known target positions;
6. camera resolution, inference device, frame rate, latency, and model identity;
7. a fixed evaluation set with detection precision, recall, and mAP if a custom
   model is recovered;
8. battery configuration and measured runtime; and
9. a new demonstration linked to exact host and firmware commit hashes.

Until that session is complete, the precise public claim is:

> The team built and demonstrated the physical car in 2025. The current
> repository is a maintained, production-style rearchitecture of its host
> control software. It includes an automated-test-backed compatibility profile
> for the recovered 2025 command dialect, which has not yet been exercised
> against the original vehicle firmware.

Configuration inputs for that work are documented in
[Configuration](configuration.md), with device preparation in
[Hardware setup](hardware-setup.md) and known specifications in the
[vehicle hardware dossier](hardware/README.md).
