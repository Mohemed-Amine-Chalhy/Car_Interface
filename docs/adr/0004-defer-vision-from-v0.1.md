# ADR-0004: Defer vision and keep v0.1 focused on controller and Lidar

- Status: Accepted
- Date: 2026-08-21
- Decision owners: Car Interface maintainers
- Supersedes: ambiguous prototype feature scope
- Superseded by: none

## Context

Historical prototypes mixed controller/Lidar behavior with incomplete camera and
YOLO experiments. No maintained vision adapter used the model or dependencies,
and the model's redistribution provenance had not been established. Carrying
dormant dependencies or build switches would increase the licensing, packaging,
performance, and support surface without providing a working feature.

## Decision drivers

- Establish one achievable and testable v0.1 boundary.
- Keep safety timing independent of inference workloads.
- Do not ship unused code, dependencies, flags, or unverified binary assets.
- Require an explicit architecture and provenance decision before expanding
  scope.

## Considered options

1. Make controller, Lidar, camera, and YOLO mandatory for v0.1.
2. Retain dormant vision dependencies and a model for possible future work.
3. Remove all vision dependencies, assets, and build paths and defer the feature.

## Decision

Use option 3. v0.1 supports controller input, RPLidar, the ESP32 protocol, and
simulation. It has no camera/vision adapter, OpenCV or Ultralytics dependency,
model weight, vision-specific dependency group, bootstrap option, or
vision-enabled build flag. Vision is not installable, packaged, or supported in
this release.

Historical prototypes were removed from the maintained tree and remain only in
Git history. Their behavior is not a supported entry point.

## Consequences

- The production boundary, lockfile, SBOM, and bundle match implemented code.
- Unverified model weights cannot enter a release accidentally.
- Camera features require a future scoped implementation rather than being
  implied by dormant dependencies.
- v0.1 documentation and support must not offer camera troubleshooting or
  vision build instructions.

## Verification

- `pyproject.toml` and `uv.lock` contain no project-declared OpenCV, Ultralytics,
  NumPy/Pillow vision stack, or vision-specific dependency group.
- Bootstrap, build, SBOM, and release commands expose no vision flag.
- The maintained tree contains no model weight or camera adapter.
- The base simulator, tests, and package build pass without vision libraries.

## Future proposal

Adding vision requires a new ADR, a maintained adapter, bounded worker/queue
design, failure-isolation tests, performance budgets, model and dependency
provenance, an SBOM/license review, packaging/update behavior, and proof that a
vision result cannot clear or weaken brake, E-stop, or fault state.
