# Autonomous Vehicle Control & AI Perception Platform

*Engineering case study*

In 2025, a multidisciplinary team of twelve engineers built and demonstrated a
physical autonomous-vehicle platform. It combined a custom chassis,
ESP32 and Arduino electronics, a Windows operator application, game-controller
input, RPLidar sensing, and camera-based YOLO perception.

I was responsible for the vehicle software and AI integration. My work
connected the operator, sensors, inference pipeline, and embedded vehicle
controllers in one system.

[Watch the physical drive test](assets/showcase/physical-drive-demo.mp4) ·
[Watch the full project showcase](https://www.instagram.com/p/DJD9AVDM7V6/)

## Engineering challenge

The project needed to coordinate systems with very different timing and failure
characteristics:

- analog game-controller input;
- a desktop GUI and operator commands;
- serial communication with embedded electronics;
- high-frequency polar Lidar scans;
- camera capture and YOLO inference; and
- propulsion, steering, and brake actuation.

The main software challenge was coordinating a real-time control workflow across
devices, worker threads, UI events, sensor updates, and physical actuation.

## My engineering contribution

| Workstream | My work |
| --- | --- |
| Operator application | Python/Tkinter interface for device connections, driving state, manual controls, controller operation, logs, and live feedback |
| Vehicle control | Translation of steering, throttle, brake, and direction intent into embedded-controller commands |
| Game controller | Analog normalization, dead zones, direction selection, and bounded control values through pygame |
| Embedded communication | Host serial integration with the vehicle electronics |
| RPLidar | Polar-to-vehicle coordinate conversion, live 2D rendering, path-corridor filtering, and closest-obstacle assessment |
| Obstacle assistance | Configurable in-path stop threshold connected to the operator control workflow |
| YOLO perception | Camera capture, inference, class/confidence extraction, detection overlays, and approximate-distance presentation |
| System integration | Coordination of operator input, sensors, inference, embedded commands, and vehicle response |

### Vehicle-control software

The Python application brings the vehicle workflow into one desktop interface.
Controller input is normalized into steering, propulsion, direction, and brake
intent, then converted into the command ranges expected by the embedded
electronics. A typed state machine, validated command objects, application
services, protocol profiles, and device adapters keep each responsibility clear.

### Lidar perception

The RPLidar path converted polar scan samples from millimetres into a live,
vehicle-relative 2D view. It projected a corridor in front of the car, selected
points inside that corridor, tracked the closest obstacle, and connected the
result to an assisted-stop threshold.

The Lidar result feeds both the live operator display and the assisted-stop
control path.

### YOLO integration

The camera-to-YOLO pipeline displayed bounding boxes, class labels, confidence
values, and approximate distance estimates. I handled the inference and
application integration. The [YOLO11n integration notes](perception/yolo-pipeline.md)
cover the camera pipeline in detail.

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
    GUI -->|USB serial| VehicleController[Host-facing vehicle controller]
    VehicleController --> Vehicle[Propulsion, steering, and braking]
~~~

The recorded media shows the assembled vehicle and operator software working
together. Hardware components, board settings, and setup details are documented
in the [physical-platform guide](hardware/README.md).

## Software design

The host application uses clear boundaries between control policy, device
access, command delivery, perception, configuration, and presentation. This
keeps the software easy to test and lets development continue with or without
the car connected.

| Design concern | Implementation |
| --- | --- |
| Device integration | Small device interfaces and dependency-injected adapters |
| Development without the car | Simulated vehicle, controller, firmware, and Lidar |
| Command handling | Typed commands and selectable protocol profiles |
| UI and worker concurrency | Worker ownership plus bounded presentation events |
| Timing-sensitive traffic | Prioritized dispatcher with pacing and acknowledgement tracking |
| Machine-specific settings | Validated TOML, environment, and CLI configuration |
| Verification | Unit, integration, regression, and opt-in hardware tests |
| Developer setup | Locked dependencies, bootstrap scripts, pre-commit hooks, and CI |

The dependency direction is simple:

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

## Design decisions

### Simulation

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

### Protocol compatibility

The host provides two selectable protocol profiles: checksummed, sequenced
protocol v1 and the vehicle's newline command mapping. The vehicle profile
preserves command pacing and steering calibration and reports completed serial
writes.

### Development workflow

The project pins Python and dependencies, provides PowerShell and Bash bootstrap
scripts, runs Ruff and strict mypy, executes tests on Windows and Ubuntu, and
builds a smoke-tested Windows application bundle.

## Software quality

| Measure | Result |
| --- | --- |
| Automated suite | **121 tests passed** |
| Aggregate coverage | **83.59%** |
| Static typing | Strict mypy across application and tooling |
| Continuous integration | Windows and Ubuntu quality jobs plus Windows packaging |
| Delivery formats | Wheel, source distribution, and PyInstaller desktop bundle |
| Test scope | State transitions, command dispatch, protocol parsing, vehicle command translation, simulation, Lidar geometry, configuration, UI callbacks, and regressions |

The checks are reproducible with one custom command:

~~~powershell
.\.venv\Scripts\python.exe scripts\dev.py check
~~~

## Outcome

The project brought together embedded control, operator software, RPLidar
sensing, and YOLO perception. This repository packages the typed control and
Lidar application, simulator, automated tests, and Windows build alongside the
engineering notes for the camera integration.

Continue with [Project results](project-results.md), inspect the
[code map](../README.md#code-map), or explore the
[vehicle specification](hardware/vehicle-specification.md).
