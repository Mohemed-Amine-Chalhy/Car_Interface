# Support

## Before requesting help

1. Put physical hardware in a safe state and isolate propulsion power.
2. Run `scripts/dev.py doctor`.
3. Search [Troubleshooting](troubleshooting.md) and existing issues.
4. Reproduce with simulation if possible.
5. Confirm the issue occurs in the newest supported release.

Safety vulnerabilities and unexpected-motion reports must use the private route
in [SECURITY.md](../SECURITY.md), not a public issue.

## Include in a support request

- concise expected and observed behavior;
- exact reproduction steps and whether simulation reproduces it;
- Car Interface version/commit and whether source or packaged app was used;
- Python version for source runs and Windows version/build;
- configuration with personal paths/device identifiers redacted;
- firmware version/artifact hash and protocol version for hardware issues;
- controller, RPLidar, ESP32, and vehicle hardware revisions;
- `doctor` output;
- the smallest relevant log excerpt; and
- screenshots only when they add state that text/logs do not capture.

Do not upload battery locations, facility layouts, usernames, Bluetooth
addresses, USB serial numbers, private repository URLs, signing material, or
credentials.

## Diagnostic bundle

Create a local bundle without opening devices:

```powershell
.\.venv\Scripts\python.exe -m car_interface diagnostics `
  --config C:\path\to\vehicle.toml `
  --support-bundle C:\path\to\car-interface-support.zip
```

The bundle contains application/platform/dependency metadata and bounded
rotating logs. Inspect it before sharing. If bundle creation fails, attach
redacted `doctor --json` output and only the relevant files from:

```text
%LOCALAPPDATA%\CarInterface\logs\
```

Diagnostics include configuration values and local paths; generation is not
automatic consent to share them.

## Support boundaries

Maintainers can help with the supported production package, simulator, declared
device adapters, protocol v1, and documented build workflow. The following need
their own engineering/qualification rather than ordinary software support:

- undocumented vehicle wiring or power systems;
- modified firmware or protocol variants;
- unsupported motor drivers, controllers, or Lidar models;
- prototype scripts recovered from Git history;
- autonomous navigation;
- camera/YOLO behavior, which is deferred and not shipped in v0.1; and
- configurations outside validated ranges.

No support response is authorization to operate an unsafe rig.
