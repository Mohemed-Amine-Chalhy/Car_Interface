# Car Interface documentation

Use this page as the entry point for maintained documentation. A document that
describes an unimplemented requirement says so explicitly; absence of that note
must not be interpreted as physical-hardware validation.

## Operators

1. [Safety](safety.md)
2. [Getting started](getting-started.md)
3. [Hardware setup](hardware-setup.md)
4. [Configuration](configuration.md)
5. [Operator guide](operator-guide.md)
6. [Troubleshooting](troubleshooting.md)
7. [Hardware qualification](hardware-qualification.md)

## Developers and maintainers

- [Architecture](architecture.md)
- [Development workflow](development.md)
- [Testing strategy](testing.md)
- [Serial protocol](protocol.md)
- [Firmware compatibility](firmware-compatibility.md)
- [Release and rollback](release.md)
- [Threat model](threat-model.md)
- [Support](support.md)
- [Third-party assets](third-party-assets.md)
- [Credits](credits.md)
- [Architecture decision records](adr/README.md)

Project-wide policies are in [CONTRIBUTING.md](../CONTRIBUTING.md),
[SECURITY.md](../SECURITY.md), [CODE_OF_CONDUCT.md](../CODE_OF_CONDUCT.md), and
[LICENSE](../LICENSE).

## Terminology

- **safe state**: propulsion command is zero, the brake command is asserted,
  motion commands are inhibited, and the condition is visible to the operator;
- **armed**: all required checks have passed and motion commands may be accepted;
- **E-stop**: a latched request to enter the safe state; software E-stop is not
  a substitute for an independent physical cutoff;
- **host**: the computer running Car Interface;
- **firmware**: code running on the ESP32;
- **device adapter**: the boundary between domain logic and a real or simulated
  controller, Lidar, or serial connection; and
- **qualification**: recorded testing of one exact hardware/software
  configuration, not a general claim about other configurations.
