# Threat model

This is a focused engineering analysis for a local Windows application
controlling a USB-connected vehicle.

## Assets and security goals

- prevent unauthorized or accidental motion;
- preserve integrity of speed, steering, brake, arm, and E-stop commands;
- detect incompatible, corrupt, missing, replayed, or stale device traffic;
- protect firmware and configuration integrity; and
- preserve availability of stop paths under bounded faults.

## Trust boundaries

- operator and CLI/configuration input;
- host process and local user account;
- USB/Bluetooth drivers and attached devices;
- serial transport between host and ESP32;
- ESP32 firmware and motor-control electronics;
- third-party Python packages/model weights;
- the locked Python environment and CI build path.

Protocol CRC detects accidental corruption; it provides neither endpoint
authentication nor confidentiality. An attacker with local account, USB, build,
or firmware access may exceed what protocol v1 can prevent.

## Principal threats and controls

| Threat | Primary controls | Residual risk/action |
| --- | --- | --- |
| Accidental hardware start | Simulation default, explicit mode plus long acknowledgement, no import-time I/O | Flag can be scripted; prohibit auto-start and retain physical cutoff |
| Wrong serial device/firmware | Protocol/version validation, strict parser, external firmware hash record | v1 lacks cryptographic device identity; controlled flashing and physical port control required |
| Corrupt/stale/reordered command | CRC, sequence, ACK timeout, bounded queue, motion eviction, heartbeat/watchdog | CRC is not malicious-tamper protection |
| Host/UI freeze or crash | Device workers, safety-independent dispatcher, firmware watchdog, physical cutoff | OS/USB/firmware common-cause failures remain |
| Controller spoof/loss | Required-device freshness, normalization, latched fault on loss | Bluetooth/local device impersonation is not cryptographically prevented |
| Lidar spoof/blindness/staleness | Freshness checks and faulting, physical exclusion zone | Lidar is supplemental and environmentally limited |
| Malicious config/local code | Strict schema/ranges, reviewed hashes, least-privilege account | Local administrator can bypass software controls |
| Dependency/build compromise | Locked dependencies, pinned CI actions, source/dependency checks, checksums | Package registries and CI infrastructure remain external dependencies |
| Denial of service via malformed serial input | 128-byte bound, strict ASCII/parser, timeout/fault | Firmware parser must implement the same bounds independently |

## Assumptions

- The physical cutoff is correctly engineered and cannot be overridden by host
  or ESP32 software.
- The operating host is dedicated or appropriately managed, patched, and not
  controlled by an untrusted user during operation.
- Physical access to USB, firmware programming, battery, and motor electronics
  is controlled.
- Firmware hashes and vehicle-profile values are recorded with the test run.

When these assumptions are false, do not operate the vehicle.

## Out of scope for protocol v1

- cryptographic device authentication;
- encrypted serial traffic;
- remote/network control;
- over-the-air firmware update;
- multi-user authorization.

Adding network or remote-control features requires a new threat model and ADR
before implementation.

## Review triggers

Review this document when adding a network, remote API, updater, new device
class, autonomous behavior, vision control input, secrets, or telemetry.
