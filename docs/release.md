# Release and rollback

Car Interface follows Semantic Versioning for the host package and keeps
protocol compatibility separate. A version tag or passing CI does not by itself
mean a physical vehicle configuration is qualified.

## Release types

- **Development build**: simulation/testing only; no stability or hardware claim.
- **Release candidate**: passed automated gates and clean-machine packaging;
  eligible for controlled hardware qualification.
- **Qualified release**: one exact host artifact, firmware artifact,
  configuration, and hardware revision passed the recorded qualification.

Only the final category may be used for normal operation, and only within its
recorded limits.

## Versioning

- Patch: compatible bug/documentation fixes with unchanged protocol semantics.
- Minor: backward-compatible host features or optional adapters.
- Major: incompatible public behavior or host configuration changes.
- Protocol breaking changes independently increment the integer wire version and
  require a firmware compatibility plan.

Update `pyproject.toml`, `CHANGELOG.md`, compatibility matrix, and application
version together. The Git tag is exactly `v<project-version>`.

## Release prerequisites

- clean Windows checkout of the intended commit;
- reviewed, current `uv.lock`;
- no unresolved critical/high safety or security findings;
- CI quality and security jobs passing;
- copyright holders have authorized a distribution license and dependency
  notice/provenance review is complete;
- immutable firmware artifact and compatibility record for hardware claims;
- clean-machine simulator smoke test; and
- completed physical qualification for hardware claims.

## Build and automated qualification

After the copyright holders authorize a distribution license, a local candidate
may be checked before tagging:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py release-check --allow-untagged
```

Until then, use `check`, `build`, `sbom`, and `checksums` independently;
`release-check` deliberately fails while the conservative distribution-blocking
notice remains in `LICENSE`.

After licensing is resolved, the untagged option is for candidate diagnosis
only. For a release:

```powershell
git status --short
git tag -s v0.1.0 -m "Car Interface 0.1.0"
.\.venv\Scripts\python.exe scripts\dev.py release-check
```

Use the actual version; do not copy `v0.1.0` after it changes. Published
releases must not use `--allow-dirty`, `--allow-untagged`, or `--skip-audit`.

`release-check` runs the complete checks, builds wheel and source distribution,
creates and smoke-tests the Windows one-folder desktop bundle, archives it,
generates SHA-256 records, and writes a CycloneDX runtime SBOM.

Expected Windows output includes:

```text
dist/CarInterface/
dist/CarInterface-windows-x86_64.zip
dist/CarInterface-windows-x86_64.zip.sha256
dist/car_interface-<version>-py3-none-any.whl
dist/car_interface-<version>.tar.gz
dist/car-interface.cdx.json
dist/SHA256SUMS.txt
```

Exact wheel/sdist normalization is defined by the build backend. Verify actual
filenames rather than scripting against the example blindly.

v0.1 has no camera/vision adapter, dependency extra, model asset, or
vision-enabled build. Do not add vision files manually to a release artifact.

## Verify artifacts

On a clean Windows machine without the source checkout or development Python:

1. verify archive SHA-256;
2. scan according to organizational security policy;
3. extract the entire archive to a new path;
4. start the packaged application and confirm simulation is the default;
5. run connect/arm/drive/brake/E-stop/reset/fault/close simulation smoke tests;
6. confirm logs are written to the per-user directory, not the install tree;
7. confirm missing hardware fails clearly; and
8. check uninstall/removal leaves user logs intentionally handled.

Authenticode signing is required before external Windows distribution. The
repository has no signing certificate or secret, so this remains an external
release blocker. Record signer identity, certificate chain, timestamp, and
signed hashes without storing private signing material in the repository.

## Publish

Publication is deliberately manual. Select the exact signed version tag as the
ref for the **Release** workflow, check its distribution-rights confirmation,
and dispatch it. The job rejects an untagged ref and `release-check` rejects the
current distribution-blocking `LICENSE`. When both gates are satisfied, the job
reruns qualification, signs GitHub build-provenance and CycloneDX SBOM
attestations, and creates the GitHub release. Before dispatching:

- compare the tag version and artifact metadata;
- verify checksums and SBOM exist;
- write release notes from `CHANGELOG.md` with safety-relevant changes first;
- link firmware compatibility and qualification evidence;
- state supported Windows/hardware/controller/Lidar versions and limits;
- state that camera/vision is deferred and absent; and
- document known issues and rollback triggers.

The current copyright notice grants no distribution rights, and no compatible
firmware artifact or physical qualification exists. Therefore public/commercial
publication and any hardware-qualified v0.1 release are currently blocked;
`release-check` enforces the copyright gate. The repository variable
`CAR_INTERFACE_DISTRIBUTION_APPROVED` must also remain unset so ordinary CI does
not upload release candidates.

Retain source, lockfile, build logs, reports, artifacts, checksums, SBOM,
firmware, configuration hash, and qualification evidence.

## Rollback triggers

Immediately withdraw/quarantine a release for unexpected motion, ineffective
stop, missed watchdog, incorrect controller mapping, protocol ambiguity,
unbounded queue/resource growth, a critical vulnerability, corrupted artifact,
or unverifiable provenance.

## Operator rollback

1. Stop with the physical cutoff and isolate propulsion power.
2. Preserve logs and identify host, firmware, and configuration hashes.
3. Mark the affected set as quarantined; do not repeatedly retry it.
4. Restore the complete last-known-qualified tuple: host artifact, firmware
   artifact, configuration, and supported hardware revision.
5. Verify hashes and repeat no-power and wheels-clear checks before motion.

Never roll back only the host when protocol or safety behavior changed.

## Maintainer rollback

Do not rewrite or replace published artifacts under an existing version. Mark
the release withdrawn, publish a security/safety advisory as appropriate, fix
forward on a review branch, add regression tests, increment the version, rebuild
from clean source, and requalify. Preserve the withdrawn artifacts for incident
analysis under restricted access.
