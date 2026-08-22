# Project results and reproducibility

The project combines a demonstrated physical vehicle with a software baseline
that can be evaluated independently through source, simulation, tests, CI, and
packaged builds.

## Physical prototype outcomes

| Delivered outcome | Engineering record |
| --- | --- |
| A custom autonomous-driving car was assembled and demonstrated | Recorded project media and the twelve-person multidisciplinary team record |
| Desktop-to-vehicle control worked | Python operator application, embedded serial command workflow, and physical demonstration |
| Game-controller driving was integrated | Analog steering, throttle, brake, and direction processing in the original application |
| RPLidar sensing was integrated | Live 2D rendering, projected-path geometry, and obstacle-assistance behavior |
| YOLO perception was integrated | Camera inference with class labels, confidence values, bounding boxes, and approximate-distance display |
| ESP32 and Arduino electronics were installed | Confirmed ESP32-WROOM-32/CP2102 and Arduino Uno R3-style boards from the original build |

[Watch the physical-car showcase](https://www.instagram.com/p/DJD9AVDM7V6/)
or read the [engineering case study](case-study.md).

## Verified software baseline

The maintained host has a reproducible, hardware-free verification path:

| Engineering signal | Verified result |
| --- | --- |
| Automated test suite | **113 tests passed** |
| Aggregate coverage | **82.94%** |
| Static typing | Strict mypy checks across the application package and development tooling |
| Formatting and linting | Ruff runs locally, through pre-commit, and in CI |
| Cross-platform CI | Quality checks execute on current Windows and Ubuntu GitHub-hosted runners |
| Packaging | Wheel, source distribution, and a smoke-tested PyInstaller Windows application bundle |
| Environment reproducibility | Python 3.13, committed uv lockfile, and custom PowerShell/Bash bootstrap scripts |
| Architecture | Typed domain, application services, protocol profiles, device adapters, simulator, configuration, and UI layers |
| Original-car protocol mapping | Regression coverage for command translation, steering calibration, pacing, partial writes, configuration, and service composition |

The workflow history is visible in
[GitHub Actions](https://github.com/Mohemed-Amine-Chalhy/Car_Interface/actions/workflows/ci.yml).

## What the automated suite exercises

- control-state transitions, latching, neutral interlocks, brake, and reset;
- bounded priority dispatch and motion-command eviction;
- protocol framing, checksums, sequence matching, ACK/NACK, and timeouts;
- original-car command translation, steering mapping, and 50 ms pacing;
- serial partial writes, disconnects, stale inputs, and worker failures;
- deterministic simulated vehicle, firmware, controller, and Lidar behavior;
- RPLidar geometry, path-corridor filtering, and obstacle assessment;
- configuration precedence and validation;
- UI callbacks and thread-safe event delivery; and
- integration and regression scenarios across services and adapters.

Representative tests are available in:

- [state-machine tests](../tests/unit/test_domain_safety.py)
- [dispatcher regression tests](../tests/unit/test_dispatcher_regressions.py)
- [protocol-profile tests](../tests/unit/test_protocol_profiles.py)
- [Lidar-analysis tests](../tests/unit/test_lidar_analysis.py)
- [integrated control-service tests](../tests/integration/test_control_service.py)

## Reproduce the result

After following [Getting started](getting-started.md), run:

~~~powershell
.\.venv\Scripts\python.exe scripts\dev.py check
~~~

The locked CI command is:

~~~powershell
uv run --locked --group dev python scripts/dev.py check --ci --skip-audit
~~~

Build the Python distributions and Windows desktop bundle with:

~~~powershell
.\.venv\Scripts\python.exe scripts\dev.py build
~~~

See [Development workflow](development.md) and
[Testing strategy](testing.md) for the command surface and suite structure.

## Measurement roadmap

The original media demonstrates the integrated car qualitatively. A focused
vehicle session can add a quantitative system profile:

1. recover exact ESP32 and Arduino firmware revisions and board
   responsibilities;
2. record wiring, GPIO assignments, actuator ranges, and steering center;
3. capture serial request/response traces and end-to-end command latency;
4. measure stopping distance across several commanded speeds;
5. measure RPLidar scan frequency and known-target distance error;
6. record camera mode, inference hardware, frame rate, and end-to-end latency;
7. evaluate precision, recall, and mAP if the original model and evaluation set
   are recovered;
8. capture battery configuration and measured runtime; and
9. link a new demonstration to exact host, firmware, and configuration
   revisions.

This roadmap turns the working prototype record into a repeatable benchmark
without substituting estimates for measurements. Known settings and capture
steps are already organized in the
[vehicle specification](hardware/vehicle-specification.md),
[board configuration](firmware/board-configuration.md), and
[perception benchmark plan](perception/restoration-plan.md).
