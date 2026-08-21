# Architecture decision records

ADRs capture decisions that constrain future changes. They explain why a choice
was made; maintained behavior is still defined by code, tests, and the current
operator/protocol documentation.

| ADR | Decision | Status |
| --- | --- | --- |
| [0001](0001-layered-production-architecture.md) | Layered production architecture with injected adapters | Accepted |
| [0002](0002-simulation-default-and-hardware-opt-in.md) | Simulation default and explicit hardware acknowledgement | Accepted |
| [0003](0003-versioned-acknowledged-serial-protocol.md) | Versioned, CRC-protected, acknowledged protocol and firmware watchdog | Accepted |
| [0004](0004-defer-vision-from-v0.1.md) | Controller/Lidar v0.1 scope; vision deferred and absent | Accepted |

Use [0000-template.md](0000-template.md) for a new record. Allocate the next
number, discuss the safety and rollback consequences, and link implementation
and test evidence. Do not delete or rewrite an accepted ADR to hide a change;
add a new ADR that supersedes it.
