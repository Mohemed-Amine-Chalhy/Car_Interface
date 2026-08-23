# Vehicle specification

This specification describes the project car. Values are grouped by evidence
quality so they can be used to configure and verify the vehicle.

## System capability

The vehicle combines:

- analog steering and throttle from a game controller;
- explicit forward/reverse selection and brake actuation;
- USB serial communication with embedded vehicle electronics;
- live 2D RPLidar visualization and projected-path obstacle detection;
- an automatic stop-assist threshold;
- camera capture and YOLO11n object-detection overlays; and
- a Windows desktop interface for connections, control, visualization, and
  logs.

The driving scope covers operator control, perception-assisted stopping, and
camera inference. Route planning and end-to-end navigation are outside the
system boundary.

## Operating configuration

| Parameter | Recorded value | Evidence state | Interpretation |
| --- | ---: | --- | --- |
| Vehicle path width | 42 cm | **Project configuration** | Corridor width used by Lidar geometry; physical body width still needs measurement |
| Host-to-controller serial rate | 115200 baud | **Project and host default** | Confirm against both firmware images |
| Serial framing | UTF-8/ASCII command plus LF | **Project configuration** | Vehicle application queued newline-terminated commands |
| Board startup wait | 1 s vehicle profile; 2 s host default | **Project value / host default** | Accommodates USB-triggered reset; measure actual boot-ready time |
| Minimum compatibility command interval | 50 ms | **Project configuration** | Host-side queue spacing in the vehicle application |
| Steering minimum | 200 | **Candidate calibration** | Raw command unit was not documented |
| Steering center | 1750 | **Candidate calibration** | Explicit value in the vehicle application |
| Steering maximum | 2900 | **Candidate calibration** | Other recorded limits exist; validate on the car |
| Compatibility propulsion request | 0–100 | **Project command range** | Percentage-like request, not a measured speed |
| Host speed cap | 50% | **Host default** | Software limit, not a top-speed claim |
| Controller steering axis | pygame axis 0 | **Project and host mapping** | Verify against the exact controller |
| Controller throttle axis | pygame axis 5 | **Project and host mapping** | Expected released value: -1 |
| Controller brake axis | pygame axis 4 | **Project and host mapping** | Expected released value: -1 |
| Direction selector | pygame hat 0 vertical | **Project and host mapping** | Up/down selects forward/reverse in the host application |
| Stick dead zone | 0.15 | **Project configuration** | Steering returns to the 1750 center command within the dead zone |
| Trigger dead zone | 0.10 | **Project configuration** | Trigger input is normalized from -1..1 to 0..100 |

## RPLidar configuration

| Parameter | Recorded value | Notes |
| --- | ---: | --- |
| Sensor family | RPLidar | Exact model remains TBD |
| Distance input | millimetres | Converted to centimetres by dividing by 10 |
| Vehicle-forward reference | sensor angle 90° | Converted to vehicle angle with `vehicle_angle = sensor_angle - 90°` |
| Visual/analysis field | 180° front sector | Vehicle-relative -90° through +90° |
| Obstacle analysis horizon | 500 cm | Vehicle-application value; other experiment configurations use different horizons |
| Display horizon | 2000 cm | Visualization range, not a verified sensor performance claim |
| Stop-assist threshold | 50 cm | Configurable default in the project UI and host application |
| Temporal history | 5 scans | Used to stabilize closest-obstacle presentation |
| Path test | `abs(distance × sin(angle)) <= 21 cm` | Half of the configured 42 cm corridor |

The 42 cm corridor does not encode a documented clearance margin. A verified
physical profile should store both measured body width and an independently
chosen navigation margin.

## Vision configuration

| Parameter | Recorded value | Evidence state |
| --- | --- | --- |
| Camera selector | index 0 | **Project configuration** |
| Windows capture backend | DirectShow first, default backend fallback | **Project configuration** |
| Capture buffering | bounded queue of 10 frames, oldest dropped when full | **Project configuration** |
| Model | `yolo11n.pt` loaded through Ultralytics | **Project integration** |
| Display size | 640 × 360 | **Project UI setting** |
| Distance heuristic focal length | 720 px | **Uncalibrated candidate** |
| Close-object overlay | estimated distance below 150 cm | **Project UI threshold** |

The next benchmark captures camera FPS, inference latency, detection precision,
distance accuracy, and the exact model identifier. See the
[perception dossier](../perception/README.md) for the validation plan.

## Mechanical and electrical specification gaps

| Specification | Required value |
| --- | --- |
| Overall length × measured width × height | TBD from physical measurement |
| Wheelbase and track width | TBD from physical measurement |
| Ready-to-run mass and payload | TBD from scale measurement |
| Wheel diameter and drive ratio | TBD from physical inspection |
| Motor model, count, rated voltage/current | TBD from labels/datasheets |
| Motor-driver model and control mode | TBD from board label and wiring |
| Steering actuator and mechanical range | TBD from label and endpoint test |
| Brake actuator and default state | TBD from label, wiring, and firmware |
| Battery chemistry, series count, capacity | TBD from pack label |
| Regulators, fuses, connectors, wire gauge | TBD from electrical inspection |
| RPLidar/camera mounting position and orientation | TBD from measurement |

Record these values in the vehicle profile after measurement.
