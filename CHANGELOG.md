# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and released versions will follow [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- Production documentation for setup, operation, safety, architecture,
  configuration, protocol, testing, troubleshooting, and release management.
- Contributor, security-reporting, code-of-conduct, and conservative copyright
  policies pending a copyright-holder license decision.
- A safe-by-default development workflow centered on simulation and explicit
  hardware opt-in.

### Changed

- Defined controller plus Lidar as the v0.1 application scope.
- Deferred camera/YOLO work and removed its unused dependencies, model asset,
  and build flags from v0.1.
- Removed historical standalone prototypes from the maintained tree; Git history
  preserves them for reference.

### Security

- Documented the requirement for an independent physical emergency stop,
  firmware watchdog, protocol acknowledgement, safe startup, and fault-state
  behavior.

## Release history

No production release has been published. Historical repository commits are
prototype work and do not imply production readiness or hardware qualification.
Public distribution remains blocked until copyright holders authorize a license;
physical release remains blocked until compatible firmware and hardware
qualification are complete.

Add version-comparison links after the first release tag exists in the canonical
repository: <https://github.com/Mohemed-Amine-Chalhy/Car_Interface>.
