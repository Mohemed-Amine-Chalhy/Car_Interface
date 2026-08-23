# Troubleshooting

Troubleshoot physical hardware only after the vehicle is mechanically safe and
propulsion power is isolated. Never repeatedly retry a failed connection while
the vehicle can move.

## Start with diagnostics

```powershell
.\.venv\Scripts\python.exe scripts\dev.py doctor
.\.venv\Scripts\python.exe -m car_interface doctor `
  --config C:\path\to\vehicle.toml
```

The developer doctor validates Python, tools, Tkinter, and lock consistency. The
application doctor reports dependency availability, effective mode, and serial
ports without opening a device. Add `--json` to the application doctor for a
machine-readable report.

Then inspect the latest rotating log:

```text
%LOCALAPPDATA%\CarInterface\logs\car-interface.log
```

When sharing logs, redact usernames, absolute paths, serial numbers, Bluetooth
identifiers, and any site-specific information.

## Bootstrap or lockfile failure

Symptoms include missing `uv`, an unsupported Python version, or `uv sync
--locked` reporting that the lock is stale.

- Install the Python version from `.python-version` and a current `uv`.
- Run the bootstrap from the repository root.
- Confirm the checkout contains `pyproject.toml` and `uv.lock` from the same
  commit.
- Do not fix a stale lock by using unlocked installation. If dependency changes
  are intentional, regenerate and review the lock under the development process.

## Tkinter unavailable

`doctor` should identify this. Install a Python distribution that includes Tcl/Tk.
Do not install an unrelated PyPI package named `tkinter`. Pure tests can still
run on a headless host, but the desktop UI cannot.

## ESP32 port missing or access denied

- Confirm the device appears in Windows Device Manager.
- Close Arduino Serial Monitor, terminal programs, and other Car Interface
  instances.
- Reconnect the USB cable and identify the port again; COM numbers can change.
- Check the USB cable supports data, not power only.
- Check the required USB-to-serial driver from the board manufacturer.

Do not grant broad administrator privileges as a routine fix.

## Protocol or compatibility rejected

Likely causes are wrong port, wrong firmware, baud mismatch, corrupted frames,
or an unsupported protocol version.

- Isolate propulsion power.
- Verify the selected port identifies as the ESP32 endpoint.
- Compare the host release, protocol version, firmware version, and artifact hash
  with [firmware compatibility](firmware-compatibility.md).
- Confirm baud rate and line framing.
- Preserve NACK/error codes and raw frame metadata without bypassing CRC or ACK.

Never disable `require_ack` to make hardware connect.

## ACK timeout or heartbeat fault

- Treat the vehicle as unsafe until the physical cutoff is engaged.
- Check USB power, cable integrity, connector strain, and firmware reset cause.
- Look for CPU starvation or blocking work in host/firmware logs.
- Reproduce with the simulator's timeout/failure injection.
- Do not increase timeouts until latency is measured and the safety impact is
  reviewed.

## Controller not detected or wrong mapping

- Confirm Windows recognizes the device before starting the app.
- Connect only one controller while commissioning.
- Verify `controller_id` and inspect every axis/button in the OS controller tool.
- Check Bluetooth battery and power-management behavior.
- Record the model and mapping; device-name guesses are not enough.

If a controller disconnects while armed, the expected response is a latched
fault with zero propulsion and brake assertion. Any other response is a
control defect that must be fixed before the next hardware run.

## Lidar missing, reversed, or stale

- Verify the Lidar port is distinct from the ESP32 port.
- Close other Lidar viewers.
- Check orientation, cable strain, motor/spin behavior, and manufacturer limits.
- Place a target at known angles and distances to validate orientation and units.
- Inspect scan timestamps rather than relying only on a changing visualization.

The pinned `rplidar-roboticia` driver has blocking internals. If unplug or a
partial response delays shutdown, use the physical cutoff, preserve logs and
timings, and stop using that hardware configuration; do not mask the
problem by extending application timeouts.

A stale required Lidar must inhibit motion. Do not increase
`lidar_stale_seconds` merely to mask intermittent hardware.

## Application remains in `SAFE_CONNECTED`

This is expected when any arming condition is incomplete. Check actuator ACK,
firmware compatibility, controller health/neutral, Lidar freshness, active
brake/E-stop/fault latches, and configuration validity. The app should state the
specific unmet condition. Do not modify the state machine to force arming.

## E-stop or fault will not reset

Reset is allowed only after the triggering condition is removed and required
devices are healthy and fresh. Correct the cause, restore neutral input, and use
the explicit reset. A reset returns to a safe, non-driving state.

## UI frozen or delayed

Use the physical cutoff if hardware is energized. Preserve thread dumps and
logs. Device I/O, inference, sleeps, and worker callbacks must not block the
Tkinter event thread; Tkinter widgets must not be touched from worker threads.
Reproduce in simulation before attempting another hardware run.

## Camera or vision startup

The v0.1 desktop package does not ship a Camera/YOLO extra, model asset, runtime
adapter, or build flag. Its packaged runtime covers controller and Lidar
integration. Any packaged vision adapter requires an isolated design,
reproducible model artifact, and validation plan.

## Pre-commit or quality-gate failure

Run the failing command directly through `scripts/dev.py`. Formatting changes
may need to be staged again. Do not use `--no-verify`; fix the file or the shared
configuration. For a temporarily unavailable vulnerability service, local work
may use `check --skip-audit`; CI runs the network-backed check separately.

## Packaged app fails but source works

- Run `scripts/dev.py release-check` first.
- Test the complete `dist/CarInterface/` directory, not only its executable.
- Check that configuration and optional assets were included intentionally.
- Build on Windows for Windows; PyInstaller artifacts are platform-specific.
- Review antivirus quarantine without disabling endpoint protection globally.

If the problem remains, open a
[GitHub issue](https://github.com/Mohemed-Amine-Chalhy/Car_Interface/issues)
with the version, reproduction steps, and a short redacted log excerpt.
