# Vehicle specification

This specification describes the car that motivated the repository. Values are
grouped by evidence quality so that the document can serve both as a portfolio
case study and as a practical reconstruction checklist.

## System capability

The demonstrated prototype combined:

- analog steering and throttle from a game controller;
- explicit forward/reverse selection and brake actuation;
- USB serial communication with embedded vehicle electronics;
- live 2D RPLidar visualization and projected-path obstacle detection;
- an automatic stop-assist threshold;
- camera capture and YOLO11n object-detection overlays; and
- a Windows desktop interface for connections, control, visualization, and
  logs.

These capabilities are evidenced in the original integrated application. They
do not imply that the car performed full path planning or end-to-end autonomous
navigation.

## Reconstructed operating configuration

| Parameter | Historical value | Evidence state | Interpretation |
| --- | ---: | --- | --- |
| Vehicle path width | 42 cm | **Historical configuration** | Corridor width used by Lidar geometry; physical body width still needs measurement |
| Host-to-controller serial rate | 115200 baud | **Historical and maintained default** | Confirm against both firmware images |
| Serial framing | UTF-8/ASCII command plus LF | **Historical** | Integrated prototype queued newline-terminated commands |
| Board startup wait | 1 s historical; 2 s maintained default | **Historical / maintained** | Accommodated USB-triggered reset; measure actual boot-ready time |
| Minimum legacy command interval | 50 ms | **Historical** | Host-side queue spacing in the integrated prototype |
| Steering minimum | 200 | **Candidate calibration** | Raw command unit was not documented |
| Steering center | 1750 | **Candidate calibration** | Explicit value in the final integrated prototype |
| Steering maximum | 2900 | **Candidate calibration** | Earlier branches contained other limits; validate on the car |
| Legacy propulsion request | 0–100 | **Historical command range** | Percentage-like request, not a measured speed |
| Maintained speed cap | 50% | **Maintained default** | Software limit, not a top-speed claim |
| Controller steering axis | pygame axis 0 | **Historical and maintained mapping** | Verify against the exact controller |
| Controller throttle axis | pygame axis 5 | **Historical and maintained mapping** | Expected released value: -1 |
| Controller brake axis | pygame axis 4 | **Historical and maintained mapping** | Expected released value: -1 |
| Direction selector | pygame hat 0 vertical | **Historical and maintained mapping** | Up/down selected forward/reverse in the integrated host |
| Stick dead zone | 0.15 | **Historical** | Steering returned to the 1750 center command within dead zone |
| Trigger dead zone | 0.10 | **Historical** | Trigger normalized from -1..1 to 0..100 |

## RPLidar configuration

| Parameter | Historical value | Notes |
| --- | ---: | --- |
| Sensor family | RPLidar | Exact model remains TBD |
| Distance input | millimetres | Converted to centimetres by dividing by 10 |
| Vehicle-forward reference | sensor angle 90° | Converted to vehicle angle with `vehicle_angle = sensor_angle - 90°` |
| Visual/analysis field | 180° front sector | Vehicle-relative -90° through +90° |
| Obstacle analysis horizon | 500 cm | Final integrated prototype value; earlier experiments used other horizons |
| Display horizon | 2000 cm | Visualization range, not a verified sensor performance claim |
| Stop-assist threshold | 50 cm | Configurable default in the integrated UI and maintained host |
| Temporal history | 5 scans | Used to stabilize closest-obstacle presentation |
| Path test | `abs(distance × sin(angle)) <= 21 cm` | Half of the configured 42 cm corridor |

The 42 cm corridor did not encode a documented clearance margin. A future
physical profile should store both measured body width and an independently
chosen navigation margin.

## Historical vision configuration

| Parameter | Historical value | Evidence state |
| --- | --- | --- |
| Camera selector | index 0 | **Historical** |
| Windows capture backend | DirectShow first, default backend fallback | **Historical** |
| Capture buffering | bounded queue of 10 frames, oldest dropped when full | **Historical** |
| Model | `yolo11n.pt` loaded through Ultralytics | **Historical integration** |
| Display size | 640 × 360 | **Historical UI setting** |
| Distance heuristic focal length | 720 px | **Uncalibrated candidate** |
| Close-object overlay | estimated distance below 150 cm | **Historical UI threshold** |

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

Completing those rows turns this reconstructed specification into a reproducible
vehicle profile without changing any of the confirmed project history.
