# Car Interface engineering documentation

This documentation describes a real academic autonomous-vehicle prototype and
the maintained, production-style host application derived from its original
control stack. Start with the system overview below, then follow the path for
the part of the project you want to understand.

![Vehicle system overview](assets/vehicle-system-overview.svg)

## Project dossier

- [Physical vehicle platform](hardware/README.md)
- [Component inventory and evidence](hardware/component-inventory.md)
- [Vehicle specification](hardware/vehicle-specification.md)
- [Host and vehicle configuration profile](hardware/configuration-profile.md)
- [Controller-board and firmware dossier](firmware/README.md)
- [ESP32 and Arduino board configuration](firmware/board-configuration.md)
- [Historical vehicle protocol](firmware/legacy-protocol.md)
- [Perception engineering](perception/README.md)
- [Historical YOLO11n pipeline](perception/historical-yolo-pipeline.md)
- [Perception restoration and validation plan](perception/restoration-plan.md)
- [Engineering team](credits.md)

## Run and operate

1. [Getting started](getting-started.md)
2. [Configuration](configuration.md)
3. [Operator guide](operator-guide.md)
4. [Hardware setup](hardware-setup.md)
5. [Troubleshooting](troubleshooting.md)
6. [Hardware qualification](hardware-qualification.md)
7. [Safety model](safety.md)

## Build and maintain

- [Architecture](architecture.md)
- [Development workflow](development.md)
- [Testing strategy](testing.md)
- [Serial protocol v1](protocol.md)
- [Firmware compatibility](firmware-compatibility.md)
- [Release and rollback](release.md)
- [Threat model](threat-model.md)
- [Support](support.md)
- [Third-party assets](third-party-assets.md)
- [Architecture decision records](adr/README.md)

Project-wide policies are in [CONTRIBUTING.md](../CONTRIBUTING.md),
[SECURITY.md](../SECURITY.md), [CODE_OF_CONDUCT.md](../CODE_OF_CONDUCT.md), and
[LICENSE](../LICENSE).

## Evidence labels

The hardware dossier uses four labels so that a polished project description
does not blur observation and inference:

| Label | Meaning |
| --- | --- |
| **Confirmed** | Identified from owner-supplied evidence, a maintained source file, or an official component reference |
| **Historical** | Implemented in the original working prototype code retained in Git history |
| **Candidate** | A strong reconstruction that still needs confirmation on the physical car |
| **TBD** | Not recoverable from the present repository or supplied photographs |

Historical values are useful compatibility targets, but they are not presented
as measured electrical, mechanical, or AI-performance results.

## Terminology

- **prototype**: the physical car built and demonstrated by the engineering
  team during the academic project;
- **maintained host**: the typed application under `src/car_interface`;
- **legacy protocol**: the newline-delimited command dialect used by the
  original integrated host;
- **protocol v1**: the versioned, checksummed protocol implemented by the
  maintained host;
- **vehicle profile**: the recorded hardware, calibration, controller, and
  perception configuration for one car; and
- **qualification**: recorded testing of one exact hardware/software
  configuration rather than a claim about every compatible component.
