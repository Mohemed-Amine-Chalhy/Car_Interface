# Car Interface engineering documentation

I'm Mohamed Amine Chalhy. This documentation presents my vehicle-software and
AI-integration work within the twelve-engineer team that built and demonstrated
the physical autonomous-driving car in 2025. It covers the software
architecture, physical platform, and engineering results in this repository.

![Vehicle system overview](assets/vehicle-system-overview.svg)

## Start here

For a focused technical evaluation:

1. Read the [engineering case study](case-study.md) for the problem, my
   contribution, and the system-level outcome.
2. Review [project results](project-results.md) for the quality results:
   121 tests, 83.59% aggregate coverage, strict typing, CI, and Windows
   packaging.
3. Follow the [architecture guide](architecture.md) from UI and services into
   the typed domain and device adapters.
4. Inspect the [control state machine](../src/car_interface/domain/safety.py),
   [command dispatcher](../src/car_interface/services/dispatcher.py),
   [protocol profiles](../src/car_interface/domain/protocol_profiles.py), and
   [simulated devices](../src/car_interface/adapters/simulated.py).

## Project and engineering story

- [Engineering case study](case-study.md)
- [Project results and reproducibility](project-results.md)
- [Architecture](architecture.md)
- [Engineering team](credits.md)
- [Architecture decisions](adr/README.md)

## Physical platform

- [Vehicle platform overview](hardware/README.md)
- [Vehicle specification](hardware/vehicle-specification.md)
- [Component inventory](hardware/component-inventory.md)
- [Host and vehicle configuration profiles](hardware/configuration-profile.md)
- [ESP32 and Arduino firmware guide](firmware/README.md)
- [Board configuration](firmware/board-configuration.md)
- [Vehicle serial protocol](firmware/legacy-protocol.md)

## Perception

- [Perception engineering](perception/README.md)
- [YOLO11n pipeline](perception/yolo-pipeline.md)
- [Perception benchmark plan](perception/validation-plan.md)

The vehicle combines RPLidar geometry with a camera/YOLO pipeline. The control
package covers the vehicle and RPLidar runtime, while the perception guide
documents the camera-to-detection pipeline.

## Build, run, and inspect

- [Getting started](getting-started.md)
- [Configuration reference](configuration.md)
- [Development workflow](development.md)
- [Testing strategy](testing.md)
- [Troubleshooting](troubleshooting.md)
- [Operator guide](operator-guide.md)
- [Hardware setup](hardware-setup.md)
- [Serial protocol v1](protocol.md)
- [Firmware-host compatibility](firmware-compatibility.md)

## Terminology

- **physical vehicle**: the car built and demonstrated by the engineering team
  in 2025;
- **host application**: the typed Python application under
  [src/car_interface](../src/car_interface);
- **vehicle protocol**: the newline-delimited command dialect used by the car;
- **protocol v1**: the versioned, checksummed protocol implemented by the
  host application; and
- **vehicle profile**: one recorded set of board, firmware, calibration,
  controller, and perception settings.
