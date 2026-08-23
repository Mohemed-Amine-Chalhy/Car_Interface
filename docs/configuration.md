# Configuration reference

Car Interface configuration is intentionally strict: unknown keys, malformed
values, and unsafe ranges are errors. Simulation remains the default.

## Precedence

From lowest to highest priority:

1. built-in defaults;
2. a TOML file supplied with `--config`;
3. `CAR_INTERFACE_*` environment variables; and
4. explicit CLI overrides.

A configuration value is not an authorization boundary. Setting `mode =
"hardware"` or `CAR_INTERFACE_MODE=hardware` does not satisfy the hardware
safety acknowledgement.

## TOML example

```toml
[car_interface]
mode = "simulation"
esp32_port = "COM3"
lidar_port = "COM6"
baud_rate = 115200
serial_startup_delay_seconds = 2.0
controller_id = 0
controller_steering_invert = false
protocol = "car_v1"
require_ack = true
ack_timeout_seconds = 0.2
heartbeat_interval_seconds = 0.1
command_stale_seconds = 0.5
lidar_stale_seconds = 0.75
auto_stop_distance_cm = 50.0
vehicle_width_cm = 42.0
max_speed_percent = 50
legacy_steering_minimum = 200
legacy_steering_center = 1750
legacy_steering_maximum = 2900
legacy_minimum_command_interval_ms = 50
log_level = "INFO"
```

A copyable file is available at [examples/car-interface.toml](examples/car-interface.toml).
Keep machine-specific configuration outside Git.

## Keys

| TOML key | Environment variable | Default | Validation and meaning |
| --- | --- | --- | --- |
| `mode` | `CAR_INTERFACE_MODE` | `simulation` | `simulation` or `hardware`; hardware still requires explicit CLI acknowledgement |
| `esp32_port` | `CAR_INTERFACE_ESP32_PORT` | unset | Required in hardware mode; surrounding whitespace is trimmed |
| `lidar_port` | `CAR_INTERFACE_LIDAR_PORT` | unset | Required in hardware mode, trimmed, and must differ from `esp32_port` case-insensitively |
| `baud_rate` | `CAR_INTERFACE_BAUD_RATE` | `115200` | Integer from 1,200 through 4,000,000; must match firmware |
| `serial_startup_delay_seconds` | `CAR_INTERFACE_SERIAL_STARTUP_DELAY_SECONDS` | `2.0` | Finite number from 0 through 4 seconds; wait after opening ESP32 serial before clearing buffers and starting the safe handshake |
| `controller_id` | `CAR_INTERFACE_CONTROLLER_ID` | `0` | Non-negative pygame controller index |
| `controller_steering_invert` | `CAR_INTERFACE_CONTROLLER_STEERING_INVERT` | `false` | Inverts physical axis 0 after reading it; the vehicle controller profile uses `true` |
| `protocol` | `CAR_INTERFACE_PROTOCOL` | `car_v1` | `car_v1` or the explicit hardware-only `school_car_legacy_v0`; never auto-detected |
| `require_ack` | `CAR_INTERFACE_REQUIRE_ACK` | `true` | Required for `car_v1`; must be `false` for the write-only vehicle profile |
| `ack_timeout_seconds` | `CAR_INTERFACE_ACK_TIMEOUT_SECONDS` | `0.2` | `car_v1` response timeout from 0.05 through 10 seconds |
| `heartbeat_interval_seconds` | `CAR_INTERFACE_HEARTBEAT_INTERVAL_SECONDS` | `0.1` | `car_v1` heartbeat interval from 0.05 through 5 seconds |
| `command_stale_seconds` | `CAR_INTERFACE_COMMAND_STALE_SECONDS` | `0.5` | Motion freshness limit; for `car_v1`, it must exceed heartbeat interval plus ACK timeout and supplies the code-level watchdog contract |
| `lidar_stale_seconds` | `CAR_INTERFACE_LIDAR_STALE_SECONDS` | `0.75` | Positive maximum scan age before a required-device fault |
| `auto_stop_distance_cm` | `CAR_INTERFACE_AUTO_STOP_DISTANCE_CM` | `50.0` | 5 through 500 cm; must be justified by measured stopping distance |
| `vehicle_width_cm` | `CAR_INTERFACE_VEHICLE_WIDTH_CM` | `42.0` | 10 through 500 cm; include vehicle envelope and margin |
| `max_speed_percent` | `CAR_INTERFACE_MAX_SPEED_PERCENT` | `50` | Integer 1 through 100; clamps requested propulsion |
| `legacy_steering_minimum` | `CAR_INTERFACE_LEGACY_STEERING_MINIMUM` | `200` | Raw left endpoint for the legacy `W` command; must be below center |
| `legacy_steering_center` | `CAR_INTERFACE_LEGACY_STEERING_CENTER` | `1750` | Raw straight-ahead value for piecewise steering calibration |
| `legacy_steering_maximum` | `CAR_INTERFACE_LEGACY_STEERING_MAXIMUM` | `2900` | Raw right endpoint for the legacy `W` command; must be above center and at most 65,535 |
| `legacy_minimum_command_interval_ms` | `CAR_INTERFACE_LEGACY_MINIMUM_COMMAND_INTERVAL_MS` | `50` | 1 through 1,000 ms between legacy serial lines |
| `log_level` | `CAR_INTERFACE_LOG_LEVEL` | `INFO` | `DEBUG`, `INFO`, `WARNING`, `ERROR`, or `CRITICAL` |

Boolean environment values accept `1/0`, `true/false`, `yes/no`, or `on/off`,
case-insensitively.

Opening many ESP32 USB serial adapters resets the board. The startup delay gives
firmware time to boot into safe outputs before the host clears startup bytes and
sends the selected profile's initial commands. Reducing it can make startup
traffic race the reset; increasing it delays connection readiness. Validate the
value for the exact board/bootloader.

## Timing relationship

The `car_v1` protocol contract requires the following relationship:

```text
heartbeat interval + ACK timeout < firmware watchdog timeout
```

Host command/controller/Lidar freshness timeouts are independent safe-state
triggers and must also be justified. There must be enough margin for expected
host scheduling and serial latency. A longer timeout reduces nuisance faults
but allows a failed/stale condition to persist longer. Record the measured worst
case and rationale in the vehicle test record; do not tune by guesswork.

`command_stale_seconds` configures the host's expected watchdog contract; it
does not program or prove the ESP32 watchdog. Firmware must be built/configured
with a compatible independently verified value.

## Reproducible vehicle profiles

- Maintain one reviewed configuration per tested vehicle.
- Record its SHA-256 hash with the build and vehicle-test results.
- Never put credentials or personal data in configuration; none are required.
- Treat changes to speed, dimensions, stop distance, timeouts/startup delay,
  controller index, or device identity as vehicle-profile changes.
- Use `INFO` in normal operation. `DEBUG` can be noisy and should not be enabled
  during motion until its timing impact is evaluated.
- Keep `require_ack = true` for `car_v1`. The explicit legacy profile rejects
  that setting because its firmware has no command-correlated response.

## Logs

On Windows, rotating logs are stored by default under:

```text
%LOCALAPPDATA%\CarInterface\logs\car-interface.log
```

Up to five rotated 2 MB files are retained. On non-Windows hosts, the base is
`$XDG_STATE_HOME/CarInterface` or `~/.local/state/CarInterface`. Logs can contain
device names and operational timing; inspect and redact them before sharing.
