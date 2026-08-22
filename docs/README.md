# Car Interface engineering documentation

I'm Mohamed Amine Chalhy. This documentation presents my vehicle-software and
AI-integration work within the twelve-engineer team that built and demonstrated
the physical autonomous-driving car in 2025. It connects our working prototype
to the typed, testable host architecture maintained in this repository.

![Vehicle system overview](assets/vehicle-system-overview.svg)

## Start here

For a focused technical evaluation:

1. Read the [engineering case study](case-study.md) for the problem, my
   contribution, and the system-level outcome.
2. Review [project results](project-results.md) for the verified baseline of
   113 tests, 82.94% aggregate coverage, strict typing, CI, and Windows
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
- [Architecture decision records](adr/README.md)

## Physical platform

- [Vehicle platform overview](hardware/README.md)
- [Vehicle specification](hardware/vehicle-specification.md)
- [Component inventory](hardware/component-inventory.md)
- [Host and vehicle configuration profiles](hardware/configuration-profile.md)
- [ESP32 and Arduino firmware dossier](firmware/README.md)
- [Board configuration and recovery plan](firmware/board-configuration.md)
- [Historical vehicle protocol](firmware/legacy-protocol.md)

## Perception

- [Perception engineering](perception/README.md)
- [Historical YOLO11n pipeline](perception/historical-yolo-pipeline.md)
- [Perception restoration and benchmark plan](perception/restoration-plan.md)

The original vehicle combined RPLidar geometry with a camera/YOLO pipeline. The
maintained package currently implements the control and RPLidar path; the
camera-to-detection work remains documented as demonstrated historical
engineering.

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

## Documentation conventions

Hardware and historical details use four simple status labels:

| Label | Meaning |
| --- | --- |
| **Confirmed** | Directly identified from the vehicle record, maintained source, supplied hardware evidence, or an official component reference |
| **Historical** | Implemented in the original working prototype |
| **Reconstructed** | Derived from surviving code and ready for comparison with the existing car |
| **TBD** | A value to capture from the car, firmware, wiring, or a measured test |

This keeps specifications useful without mixing measured values with
reconstructed settings.

## Terminology

- **physical prototype**: the car built and demonstrated by the engineering team
  in 2025;
- **maintained host**: the typed Python application under
  [src/car_interface](../src/car_interface);
- **legacy protocol**: the newline-delimited command dialect used by the
  original integrated host;
- **protocol v1**: the versioned, checksummed protocol implemented by the
  maintained host; and
- **vehicle profile**: one recorded set of board, firmware, calibration,
  controller, and perception settings.
