# Release engineering

Car Interface uses Semantic Versioning and tags releases as `v<version>`. Build
Windows artifacts from a clean checkout of the exact tag so every download maps
to one source revision.

## 1. Prepare the version

1. Confirm CI passes on `main`.
2. Update the version in `pyproject.toml`.
3. Move the relevant `CHANGELOG.md` entries into the new version section.
4. Synchronize the lockfile and run the complete quality gate:

   ```powershell
   uv sync --locked --all-groups
   uv run --locked --all-groups python scripts/dev.py check --ci
   ```

5. Commit the version change and confirm the worktree is clean.

## 2. Create the tag

Use the version from `pyproject.toml`:

```powershell
git tag -a v0.1.0 -m "Car Interface 0.1.0"
git push origin main
git push origin v0.1.0
```

Replace `v0.1.0` with the actual release version. Never move or reuse a
published version tag.

## 3. Build on Windows

From a clean checkout of the tag on Windows:

```powershell
uv sync --locked --all-groups
uv run --locked --all-groups python scripts/dev.py release-check
```

The release check reruns formatting, linting, strict type checking, tests, and
security checks before building the Python distributions and Windows desktop
bundle. The build command also starts the frozen executable with `--version` as
an automated smoke test and creates the ZIP archive and checksum manifest.

Expected files include:

```text
dist/CarInterface/
dist/CarInterface-windows-x86_64.zip
dist/CarInterface-windows-x86_64.zip.sha256
dist/car_interface-<version>-py3-none-any.whl
dist/car_interface-<version>.tar.gz
dist/SHA256SUMS.txt
```

## 4. Smoke-test the artifact

Test the ZIP on a clean Windows machine rather than from the source checkout:

1. extract the entire archive to a new directory;
2. run `CarInterface.exe --version` and confirm it matches the tag;
3. start the application and confirm simulation is the default;
4. connect the simulated devices;
5. exercise steering, speed, brake, E-stop/reset, disconnect, and shutdown;
6. confirm the UI remains responsive and logs are written to the user data
   directory; and
7. record the Windows version and result in the release notes.

Physical-car testing, when performed, should identify the vehicle profile,
firmware version, controller, and serial protocol separately from the desktop
artifact smoke test.

## 5. Verify checksums

Review `dist/SHA256SUMS.txt` and independently hash the files before uploading:

```powershell
Get-FileHash dist\CarInterface-windows-x86_64.zip -Algorithm SHA256
Get-Content dist\SHA256SUMS.txt
```

The computed digest must match the manifest and the adjacent `.zip.sha256`
file.

## 6. Publish

Create a GitHub release from the exact version tag and attach:

- the Windows ZIP;
- wheel and source distribution;
- `.zip.sha256`; and
- `SHA256SUMS.txt`.

Build release notes from `CHANGELOG.md`. Include highlights, compatibility
changes, tested environment, known limitations, and upgrade steps. Download the
published files once and repeat the checksum comparison to catch upload errors.

## Rollback

If a release has a regression:

1. mark the affected GitHub release as pre-release or add a prominent warning;
2. direct users to the previous known-good version and its checksum;
3. preserve logs and the failing configuration needed to reproduce the issue;
4. fix the problem on a branch and add a regression test;
5. increment the version, rebuild from a clean tag, and republish; and
6. never replace files under an existing version tag.

For a physical vehicle, roll back the host, firmware, and configuration as a
tested set when the protocol or calibration changed.
