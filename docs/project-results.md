# Project results

The project combines a demonstrated physical vehicle with a software baseline
that can be evaluated independently through source, simulation, tests, CI, and
packaged builds.

## Physical vehicle outcomes

| Delivered outcome | Engineering record |
| --- | --- |
| A custom autonomous-driving car was assembled and demonstrated | Recorded project media and the twelve-person multidisciplinary team record |
| Desktop-to-vehicle control worked | Python operator application, embedded serial command workflow, and physical demonstration |
| Game-controller driving was integrated | Analog steering, throttle, brake, and direction processing in the operator application |
| RPLidar sensing was integrated | Live 2D rendering, projected-path geometry, and obstacle-assistance behavior |
| YOLO perception was integrated | Camera inference with class labels, confidence values, bounding boxes, and approximate-distance display |
| ESP32 and Arduino electronics were installed | ESP32-WROOM-32/CP2102 and Arduino Uno R3-style boards used in the vehicle |

[Watch the physical-car showcase](https://www.instagram.com/p/DJD9AVDM7V6/)
or read the [engineering case study](case-study.md).

## Software quality

The software stack can be checked through simulation, automated tests, CI, and
packaged builds:

| Engineering signal | Result |
| --- | --- |
| Automated test suite | **121 tests passed** |
| Aggregate coverage | **83.59%** |
| Static typing | Strict mypy checks across the application package and development tooling |
| Formatting and linting | Ruff runs locally, through pre-commit, and in CI |
| Cross-platform CI | Quality checks execute on current Windows and Ubuntu GitHub-hosted runners |
| Packaging | Wheel, source distribution, and a smoke-tested PyInstaller Windows application bundle |
| Environment reproducibility | Python 3.13, committed uv lockfile, and custom PowerShell/Bash bootstrap scripts |
| Architecture | Typed domain, application services, protocol profiles, device adapters, simulator, configuration, and UI layers |
| Vehicle protocol profile | Regression coverage for command translation, steering calibration, pacing, partial writes, configuration, and service composition |

CI results are available in
[GitHub Actions](https://github.com/Mohemed-Amine-Chalhy/Car_Interface/actions/workflows/ci.yml).

## What the automated suite exercises

- control-state transitions, latching, neutral interlocks, brake, and reset;
- bounded priority dispatch and motion-command eviction;
- protocol framing, checksums, sequence matching, ACK/NACK, and timeouts;
- vehicle command translation, steering mapping, and 50 ms pacing;
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
