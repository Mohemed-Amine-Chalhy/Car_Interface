# Architecture decision records

ADRs capture decisions that constrain future changes. They explain why a choice
was made; implemented behavior is defined by code, tests, and the operator and
protocol documentation.

| ADR | Decision | Status |
| --- | --- | --- |
| [0001](0001-layered-production-architecture.md) | Layered production architecture with injected adapters | Accepted |
| [0002](0002-simulation-default-and-hardware-opt-in.md) | Simulation default and explicit hardware acknowledgement | Accepted |
| [0003](0003-versioned-acknowledged-serial-protocol.md) | Versioned, CRC-protected, acknowledged protocol and firmware watchdog | Accepted |
| [0004](0004-perception-package-boundary.md) | Controller, Lidar, and perception package boundary | Accepted |
| [0005](0005-explicit-school-car-legacy-profile.md) | Explicit school-car legacy protocol compatibility | Accepted |

Use [0000-template.md](0000-template.md) for a new record. Allocate the next
number, discuss the safety and rollback consequences, and link implementation
and test evidence. Supersede an accepted decision with a new ADR.
