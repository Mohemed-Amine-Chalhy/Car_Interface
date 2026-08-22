# Engineering case study

## Autonomous Vehicle Control & AI Perception Platform

In 2025, a multidisciplinary team of twelve engineers built and demonstrated a
physical autonomous-driving car. The platform combined a custom chassis,
ESP32 and Arduino electronics, a Windows operator application, game-controller
input, RPLidar sensing, and camera-based YOLO perception.

**I owned the vehicle-software and AI-integration workstream.** My scope
connected the operator, sensors, inference pipeline, and embedded vehicle
controllers into one usable system.

[Watch the recorded physical-car showcase](https://www.instagram.com/p/DJD9AVDM7V6/)

## Engineering challenge

The project needed to coordinate systems with very different timing and failure
characteristics:

- analog game-controller input;
- a desktop GUI and operator commands;
- serial communication with embedded electronics;
- high-frequency polar Lidar scans;
- camera capture and YOLO inference; and
- propulsion, steering, and brake actuation.

The main software problem was not an isolated algorithm. It was building a
coherent real-time control workflow across devices, worker threads, UI events,
sensor updates, and physical actuation.

## My engineering contribution

| Workstream | Engineering delivered |
| --- | --- |
| Operator application | Python/Tkinter interface for device connections, driving state, manual controls, controller operation, logs, and live feedback |
| Vehicle control | Translation of steering, throttle, brake, and direction intent into embedded-controller commands |
| Game controller | Analog normalization, dead zones, direction selection, and bounded control values through pygame |
| Embedded communication | Host serial integration with the ESP32/Arduino vehicle electronics |
| RPLidar | Polar-to-vehicle coordinate conversion, live 2D rendering, path-corridor filtering, and closest-obstacle assessment |
| Obstacle assistance | Configurable in-path stop threshold connected to the operator control workflow |
| YOLO perception | Camera capture, inference, class/confidence extraction, detection overlays, and approximate-distance presentation |
| System integration | Coordination of operator input, sensors, inference, embedded commands, and vehicle response |

### Vehicle-control software

The original Python application brought the working vehicle workflow into one
desktop interface. Controller input was normalized into steering, propulsion,
direction, and brake intent, then converted into the command ranges expected by
the embedded electronics.

The maintained code expresses the same concerns through a typed state machine,
validated command objects, application services, explicit protocol profiles,
and isolated device adapters.

### Lidar perception

The RPLidar path converted polar scan samples from millimetres into a live,
vehicle-relative 2D view. It projected a corridor in front of the car, selected
points inside that corridor, tracked the closest obstacle, and connected the
result to an assisted-stop threshold.

This work joined geometry, streaming sensor data, operator visualization, and a
vehicle-control response rather than treating the Lidar as a standalone demo.

### YOLO integration

The physical prototype used a camera-to-YOLO pipeline that displayed bounding
boxes, class labels, confidence values, and an approximate-distance overlay.
My contribution was the inference and application integration. The retained
engineering record supports that integration scope, while the maintained
package currently focuses on control and RPLidar.

The original pipeline is reconstructed in the
[historical YOLO11n engineering record](perception/historical-yolo-pipeline.md).

## Demonstrated system

![Physical vehicle hardware topology](assets/hardware-topology.svg)

~~~mermaid
flowchart LR
    Operator --> Controller[Game controller]
    Operator --> GUI[Python/Tkinter application]
    Controller --> GUI
    Lidar[RPLidar] --> GUI
    Camera --> YOLO[YOLO inference]
    YOLO --> GUI
    GUI -->|USB serial| Boards[ESP32 + Arduino Uno R3]
    Boards --> Vehicle[Propulsion, steering, and braking]
~~~

The recorded media shows the assembled vehicle and operator software working
together. Hardware components, reconstructed settings, and the firmware recovery
plan are documented in the [physical-platform dossier](hardware/README.md).

## From integrated prototype to maintainable system

The original implementation optimized for delivering a working
multidisciplinary hardware project. The current repository turns those lessons
into a codebase that can be reviewed, tested, and extended without requiring the
vehicle for every change.

| Prototype concern | Maintained engineering response |
| --- | --- |
| GUI callbacks coordinating device libraries | Small device interfaces and dependency-injected adapters |
| Hardware required for most testing | Deterministic simulated vehicle, controller, firmware, and Lidar |
| Command strings assembled around the UI | Typed commands and explicit protocol profiles |
| Concurrent UI and device work | Worker ownership plus bounded presentation events |
| Timing-sensitive command traffic | Prioritized, bounded dispatcher with pacing and acknowledgement tracking |
| Machine-specific settings | Validated TOML, environment, and CLI configuration |
| Manual confidence checks | Unit, integration, regression, and opt-in hardware test layers |
| Ad hoc setup | Locked dependencies, custom bootstrap scripts, pre-commit hooks, and CI |

The maintained dependency direction is intentionally simple:

~~~text
Tkinter UI / CLI
        |
        v
application services  --->  presentation events
        |
        v
typed domain state, commands, and protocol profiles
        ^
        |
real or simulated device adapters
~~~

See [Architecture](architecture.md) for the full design.

## Engineering decisions worth highlighting

### Deterministic simulation

In-memory vehicle, controller, firmware, and Lidar adapters implement the same
interfaces as physical devices. Tests can drive connection sequences, state
transitions, command ordering, timeouts, sensor staleness, and reconnection
without special equipment.

### Domain logic independent of the GUI

Control phases and transitions do not depend on Tkinter, pygame, serial ports,
or wall-clock time. The state machine can be tested as ordinary Python, while
the UI renders events rather than owning control policy.

### Bounded real-time work

The dispatcher owns serial transactions, prioritizes stop-related commands,
bounds queue growth, tracks acknowledgements, and rejects stale work. Device
workers publish bounded events and never update Tkinter widgets directly.

### Explicit protocol compatibility

The host provides two selected-at-configuration protocol profiles: checksummed,
sequenced protocol v1 and the original car's newline command mapping. The legacy
profile preserves command pacing and steering calibration while reporting
serial writes without inventing firmware acknowledgements.

### Reproducible developer experience

The project pins Python and dependencies, provides PowerShell and Bash bootstrap
scripts, runs Ruff and strict mypy, executes tests on Windows and Ubuntu, and
builds a smoke-tested Windows application bundle.

## Verified software result

| Measure | Current verified baseline |
| --- | --- |
| Automated suite | **121 tests passed** |
| Aggregate coverage | **83.59%** |
| Static typing | Strict mypy across application and tooling |
| Continuous integration | Windows and Ubuntu quality jobs plus Windows packaging |
| Delivery formats | Wheel, source distribution, and PyInstaller desktop bundle |
| Test scope | State transitions, command dispatch, protocol parsing, legacy translation, simulation, Lidar geometry, configuration, UI callbacks, and regressions |

The checks are reproducible with one custom command:

~~~powershell
.\.venv\Scripts\python.exe scripts\dev.py check
~~~

## Outcome

The project demonstrates two complementary results:

1. a real team-built autonomous-vehicle prototype integrating embedded control,
   operator software, RPLidar sensing, and YOLO perception; and
2. a typed, layered, simulation-backed host application that makes the software
   architecture and behavior independently reviewable.

Continue with [Project results](project-results.md), inspect the
[source tour](../README.md#technical-tour), or explore the
[vehicle specification](hardware/vehicle-specification.md).
