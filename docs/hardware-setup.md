# Hardware setup

This guide describes integration checks, not a wiring schematic. The repository
does not currently provide enough verified electrical design information to
specify pin numbers, voltage levels, current capacity, motor-driver wiring, or
battery protection. Obtain and review the exact component schematics before
energizing the rig.

The repository also provides no qualified ESP32 firmware artifact or completed
physical qualification. Physical operation remains blocked until both exist.

## Expected v0.1 components

- Windows host computer with two reliable USB connections;
- ESP32 running compatible Car Interface firmware;
- motor/steering drivers and a mechanically sound vehicle;
- RPLidar supported by the installed adapter and driver;
- USB or Bluetooth game controller with a validated mapping;
- correctly rated battery, fuse, wiring, connectors, and power distribution;
- independent physical emergency-stop or propulsion-power cutoff; and
- restraints or a stand that keeps driven wheels clear during commissioning.

Camera/vision hardware is deferred and unsupported in v0.1. No camera adapter or
vision dependency is shipped.

## Electrical prerequisites

Before connecting the host:

- confirm logic and motor voltage compatibility from component datasheets;
- confirm a common reference only where the electrical design requires it;
- keep motor current out of USB and logic wiring;
- use appropriate fusing close to the energy source;
- ensure the physical cutoff removes or inhibits propulsion even if the ESP32
  output is stuck;
- ensure the motor controller's unpowered, reset, and disconnected input states
  cannot command motion; and
- restrain cables so steering and wheels cannot pull them loose.

Do not infer wiring from historical COM-port names or command strings.

## Firmware

The ESP32 must implement protocol v1 in [protocol.md](protocol.md), including:

- safe outputs during boot and reset;
- protocol-name/version validation on every frame before arming;
- strict frame length, syntax, sequence, and CRC validation;
- ACK/NACK responses;
- an independent heartbeat watchdog that zeros propulsion and asserts braking;
- latched E-stop semantics; and
- no retention of a previous motion command across reconnect or reset.

Record the firmware source revision and artifact hash. See
[firmware compatibility](firmware-compatibility.md).

The host waits `serial_startup_delay_seconds` (default 2.0, allowed 0 through 4)
after opening the ESP32 port, then clears serial buffers and begins the safe
protocol handshake. This accommodates boards that reset when USB serial opens.
Measure boot readiness for the exact board/firmware before changing it; firmware
must hold safe outputs throughout boot regardless of the host delay.

## Identify Windows ports

1. Disconnect both ESP32 and Lidar.
2. Open Device Manager and note existing **Ports (COM & LPT)** entries.
3. Connect only the ESP32 and record its newly appearing port and USB device
   identity.
4. Disconnect it, connect only the Lidar, and record that port and identity.
5. Reconnect both and confirm identities remain distinguishable.

COM numbers can change between USB sockets or driver installations. A port name
is not proof of device identity. Protocol v1 rejects an endpoint that cannot
exchange valid versioned frames, but it does not report a cryptographic or
semantic firmware identity. Use controlled flashing and the recorded firmware
artifact SHA-256 to establish identity.

Do not use a port already open in Arduino Serial Monitor, another terminal, or
another application instance. ESP32 and Lidar ports must be different; matching
port names are rejected during hardware configuration validation.

## Controller commissioning

Use Windows **Set up USB game controllers** or the manufacturer's tool to check:

- the expected controller is index 0, or configure the correct index;
- all axes return consistently to neutral;
- steering direction matches the UI;
- throttle and brake triggers are independent and cover the expected range;
- no controller button is assumed to reset E-stop or fault state; reset is an
  explicit, confirmed GUI action by default; and
- disconnecting USB/Bluetooth is detected immediately.

Controller name heuristics are not qualification. Record the exact model,
connection type, driver, mapping, and results.

The built-in pygame mapping currently expects axis 0 for steering, axis 5 for
the right-trigger throttle, axis 4 for the left-trigger brake, and hat 0 vertical
for forward/reverse. There is no default controller-button E-stop reset. If a
device reports different released trigger values or axis numbers, it is not
compatible until a reviewed mapping/calibration change is made; do not drive by
trial and error.

## RPLidar commissioning

v0.1 uses `rplidar-roboticia`, an older driver with blocking serial internals.
Its scan iterator starts the Lidar motor; the host adapter must not start it a
second time. Until the exact device/driver combination passes unplug,
partial-response, scan-worker-exit, and bounded-shutdown latency tests, physical
qualification is blocked.

- mount the unit rigidly, upright as required by its manual, with an unobstructed
  view;
- route its cable away from steering and rotating components;
- verify the configured port and driver;
- confirm displayed angles match the vehicle's forward direction;
- measure a target at known distances to confirm millimetre-to-centimetre
  normalization; and
- test scan staleness by unplugging the sensor while armed on the stand.
- measure behavior for unplug during a partial response and verify application
  shutdown completes within the documented bound without leaving a live worker.

The vehicle width configuration must include protrusions and a justified margin.

## First powered run

1. Remove or mechanically isolate propulsion power.
2. Connect the host, ESP32 logic power, controller, and Lidar.
3. Run `scripts/dev.py doctor`, then run `.\.venv\Scripts\python.exe -m
   car_interface doctor --config C:\path\to\vehicle.toml`; resolve every
   hardware-relevant failure and confirm both expected serial ports are listed.
4. Start hardware mode using its explicit acknowledgement.
5. Confirm the application remains safe-connected and braked.
6. Confirm the controlled-flash firmware record, protocol-v1 ACKs, heartbeat,
   controller freshness, and Lidar freshness are healthy.
7. Exercise E-stop and device-disconnect paths without propulsion power.
8. Place the vehicle on a suitable stand and enable propulsion power.
9. Test physical cutoff first, then software E-stop, using the lowest speed.
10. Continue with [hardware qualification](hardware-qualification.md).

Never make the first powered run with wheels on the ground.
