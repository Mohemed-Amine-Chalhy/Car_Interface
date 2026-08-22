# Host and vehicle configuration profile

The maintained host accepts a flat `[car_interface]` TOML table and provides two
explicit wire profiles. COM ports must be replaced with the identities observed
on the physical car; the application never guesses a protocol from a port.

## Maintained protocol-v1 profile

```toml
[car_interface]
mode = "hardware"
esp32_port = "COM5" # replace with the identified vehicle-controller port
lidar_port = "COM6" # replace with the identified RPLidar port
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
log_level = "INFO"
```

`car_v1` uses checksummed, sequenced commands and correlated ACK/NACK responses.

## Existing school-car compatibility profile

The maintained host includes the explicit `school_car_legacy_v0` adapter for
the original newline-delimited vehicle commands:

```toml
[car_interface]
mode = "hardware"
esp32_port = "COM5" # replace with the identified vehicle-controller port
lidar_port = "COM6" # replace with the identified RPLidar port
baud_rate = 115200
serial_startup_delay_seconds = 1.0
controller_id = 0
controller_steering_invert = true
protocol = "school_car_legacy_v0"
require_ack = false
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

The legacy firmware did not define command-correlated acknowledgements, so this
profile truthfully reports successful serial writes rather than fabricated
firmware ACKs. It is available only in explicit hardware mode and must be paired
with `require_ack = false`. Physical serial-transcript verification on the
existing car is still pending.

## Evidence-backed prototype profile

These fields describe the reconstructed physical configuration. They are a
documentation schema, not keys accepted by the current host parser.

```toml
# Reference profile only — do not pass this table to the current CLI.
[vehicle]
profile_id = "academic-car-2025"
platform = "custom-autonomous-vehicle-prototype"
configured_path_width_cm = 42.0
measured_width_cm = "TBD"

[vehicle.controllers.esp32]
module = "ESP-WROOM-32"
carrier = "38-pin DevKit-style, vendor/revision TBD"
usb_uart = "CP2102"
installed_role = "candidate-primary-vehicle-controller"
firmware = "TBD"

[vehicle.controllers.arduino]
board = "Uno R3 form factor"
mcu = "ATmega328P"
installed_role = "TBD; auxiliary actuator controller is a candidate"
firmware = "TBD"

[vehicle.legacy_serial]
baud_rate = 115200
line_ending = "LF"
startup_delay_seconds = 1.0
minimum_command_interval_ms = 50
protocol = "school_car_legacy_v0"

[vehicle.steering]
minimum_raw = 200
center_raw = 1750
maximum_raw = 2900
invert_input = true
calibration_state = "historical-candidate"

[vehicle.controller]
pygame_index = 0
steering_axis = 0
throttle_axis = 5
brake_axis = 4
direction_hat = 0
steering_deadzone = 0.15
trigger_deadzone = 0.10

[vehicle.lidar]
family = "RPLidar"
model = "TBD"
forward_sensor_angle_degrees = 90.0
front_view_degrees = 180.0
analysis_horizon_cm = 500.0
display_horizon_cm = 2000.0
stop_assist_distance_cm = 50.0
history_scans = 5

[vehicle.camera]
model = "TBD"
index = 0
display_width = 640
display_height = 360

[vehicle.vision]
historical_model = "YOLO11n"
model_sha256 = "TBD"
status = "historical integration; not shipped by maintained v0.1"
```

## Configuration promotion checklist

A historical or candidate value becomes a verified car profile only when its
evidence is recorded:

| Field group | Verification artifact |
| --- | --- |
| Board identity | Front/back photographs, USB identifiers, flash query |
| Firmware | Source commit, build environment, binary SHA-256 |
| Pins and wiring | Reviewed schematic and continuity check |
| Steering limits | Wheels-clear endpoint/center calibration record |
| Propulsion | Motor-driver truth table and measured command response |
| Controller | pygame event capture for every axis/hat/button |
| Lidar | Model label, mount angle, known-distance scan capture |
| Camera | Device label, selected mode, intrinsic calibration |
| YOLO | Model card, weights checksum, dataset/source, test metrics |

Store machine-specific COM ports outside the shared profile. Prefer stable USB
VID/PID/serial identities when the device exposes them, then map those
identities to the current COM numbers during startup.
