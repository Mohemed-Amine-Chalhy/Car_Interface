# Third-party dependencies and provenance

The repository currently grants no open-source license. Copyright remains with
the respective holders; see [LICENSE](../LICENSE). A project license decision
also would not automatically relicense third-party packages, firmware, images,
models, sounds, fonts, or copied code.

## Maintained-tree status

- Historical prototype scripts, notebook output, demonstration media links, and
  the unverified `yolo11n.pt` model were removed from the maintained tree.
- v0.1 has no camera/vision adapter, OpenCV or Ultralytics dependency,
  vision-specific dependency group, model asset, or vision-enabled build.
- The runtime dependency set is pygame, pyserial, and rplidar-roboticia as locked
  by `uv.lock`.
- No ESP32 firmware artifact is shipped or qualified.

Removal prevents those historical items from entering new artifacts; it does
not establish rights in earlier Git history. Do not restore, redistribute, or
copy an historical asset without a documented rights review.

## Distribution gate

Before any public or commercial distribution:

1. identify the copyright holders and obtain authorization for a common project
   license;
2. generate `scripts/dev.py sbom` from the exact tagged locked environment;
3. produce and review a dependency-license/notice report, including the pinned
   older RPLidar driver;
4. satisfy each dependency's attribution, source, or other obligations;
5. verify that the artifact contains only intended source/data files; and
6. archive the license authorization, notices, SBOM, hashes, and review evidence
   with the release.

The CycloneDX SBOM identifies versions but is not a copyright permission or
license opinion. This checklist is a release control, not legal advice.

## Adding an asset or firmware

Record alongside the change:

- exact filename and purpose;
- creator and copyright holder;
- canonical source URL or internal provenance record;
- version/date and SHA-256;
- license text or written permission;
- transformations performed;
- whether redistribution, modification, and commercial use are permitted;
- required attribution/notices; and
- packaging, update, size, and security implications.

Do not commit or package an asset while any required field is unknown. A future
vision/model addition also requires the new ADR and safety isolation work listed
in [ADR-0004](adr/0004-defer-vision-from-v0.1.md).
