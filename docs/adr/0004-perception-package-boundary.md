# ADR-0004: Define the v0.1 perception boundary

- Status: Accepted
- Date: 2026-08-21
- Decision owners: Car Interface maintainers
- Release scope: v0.1

## Context

The vehicle project includes controller, Lidar, camera, and YOLO engineering
work. The v0.1 desktop package provides controller and Lidar integration but no
runtime vision adapter. Bundling unused inference packages or build switches
would increase installation size, performance variability, and maintenance
cost without providing a runnable packaged feature.

## Decision drivers

- Establish one achievable and testable v0.1 boundary.
- Keep safety timing independent of inference workloads.
- Do not ship unused code, dependencies, flags, or unverified binary assets.
- Require an explicit architecture and reproducibility decision for every
  packaged perception adapter.

## Considered options

1. Make controller, Lidar, camera, and YOLO mandatory for v0.1.
2. Bundle dormant vision dependencies and a model without a runtime adapter.
3. Keep the v0.1 package focused on controller, Lidar, protocol, and simulation.

## Decision

Use option 3. v0.1 supports controller input, RPLidar, the ESP32 protocol, and
simulation. It has no camera/vision adapter, OpenCV or Ultralytics dependency,
model weight, vision-specific dependency group, bootstrap option, or
vision-enabled build flag. Vision is not installable, packaged, or supported in
this release.

## Consequences

- The package boundary, lockfile, and bundle match implemented code.
- Unidentified model weights cannot enter a build accidentally.
- Camera features are specified separately rather than implied by dormant
  package dependencies.
- v0.1 documentation does not advertise a runnable camera subsystem.

## Verification

- `pyproject.toml` and `uv.lock` contain no runtime OpenCV, Ultralytics, or NumPy
  vision stack and no vision-specific dependency group.
- Bootstrap, build, and release commands expose no vision flag.
- The source package contains no model weight or camera adapter.
- The base simulator, tests, and package build pass without vision libraries.

## Perception package requirements

A packaged vision feature requires a new ADR, a runtime adapter, bounded
worker/queue design, failure-isolation tests, performance budgets, pinned
dependencies, model identity and checksum records, reproducible packaging, and
proof that a vision result cannot clear or weaken brake, E-stop, or fault state.
