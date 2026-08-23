# Control-safety architecture

Car Interface treats motion control as stateful, failure-prone I/O. Its software
design combines explicit state transitions, bounded command delivery, stale-data
detection, latching faults, and deterministic recovery. Physical test sessions
use an independent propulsion cutoff outside the host and embedded software.

## Software boundary

Inside the software boundary:

- domain state and transitions;
- validation and clamping of normalized speed and steering commands;
- priority/eviction of queued motion when a stop decision occurs;
- connection, acknowledgement, heartbeat, and freshness monitoring;
- neutral/braked startup and best-effort safe shutdown;
- fault reporting and structured logs; and
- simulated fault injection and automated verification.

Outside the software boundary:

- motor-driver electrical behavior;
- battery, contactor, fusing, and physical cutoff design;
- ESP32 firmware scheduling and watchdog implementation;
- steering and braking mechanics;
- Lidar field of view, reflections, occlusion, and environmental suitability;
- operating-site controls; and
- human training and supervision.

Outside-boundary controls must be engineered and tested independently.

## Invariants

The production application is designed around these invariants:

1. Simulation is the default; configuration alone cannot silently enter
   hardware mode.
2. Startup, disconnect, disarm, brake, E-stop, and fault decisions command zero
   propulsion and assert braking before accepting any new motion.
3. An E-stop or fault evicts pending ordinary motion commands.
4. E-stop and fault states are latched. They do not clear merely because a
   device reconnects or an input returns to normal.
5. Required devices are healthy and their data fresh before arming. By default,
   the actuator link, controller, and Lidar are required.
6. Arming and post-brake resume require a fresh neutral controller input;
   retained throttle never resumes motion automatically.
7. Missing acknowledgement, heartbeat failure, stale commands, stale Lidar,
   controller loss, serial loss, protocol error, or worker failure causes a
   safe transition.
8. Firmware independently stops and brakes when valid host heartbeat/control
   traffic is absent beyond its watchdog deadline.
9. Closing the UI never serves as the only means of stopping a moving vehicle.

These invariants are exercised by automated unit, integration, and regression
tests. The complete vehicle is validated separately with the hardware checklist.

## State machine

| State | Motion accepted | Meaning | Exit requirement |
| --- | --- | --- | --- |
| `DISCONNECTED` | No | No usable actuator session | Connect and complete protocol checks |
| `SAFE_CONNECTED` | No | Connected, zero propulsion, brake asserted | All required devices healthy/fresh, then explicit arm |
| `ARMED` | No propulsion until drive input | Ready to accept bounded operator input | Valid drive input, brake, disarm, E-stop, fault, or disconnect |
| `DRIVING` | Yes, within configured limits | Valid motion command active | Neutral/brake, disarm, E-stop, fault, timeout, or disconnect |
| `BRAKING` | No | Brake requested; pending motion is discarded | Fresh neutral input followed by explicit Arm; a release request alone has no state effect |
| `EMERGENCY_STOP` | No | Operator or assist E-stop, latched | Hazard removed, devices healthy, explicit reset; returns safe, not driving |
| `FAULT` | No | Required health/freshness/protocol invariant failed, latched | Cause corrected, devices healthy/fresh, explicit reset; returns safe |

Disconnect is valid from every state and returns to `DISCONNECTED`. Reset never
restores the previous speed.

## Stop layers

Use all applicable layers:

1. **Neutral input** requests zero speed during ordinary control.
2. **Software brake** requests zero speed and brake assertion, with motion queue
   eviction.
3. **Software E-stop** latches the software state and sends prioritized stop
   commands.
4. **Firmware watchdog** independently enters its safe output state when host
   traffic is invalid or absent.
5. **Physical E-stop/power cutoff** removes or inhibits propulsion independently
   of both processors.

An on-screen E-stop is useful but cannot satisfy layer 5.

## Lidar operating model

Obstacle assist is based on fresh points inside the configured vehicle corridor.
Its threshold is calibrated from sensor mounting, field of view, vehicle width,
command latency, and measured stopping distance. RPLidar reflections and
occlusions are handled as test conditions rather than assumptions.

## Hardware test preflight

- inspect the chassis, wheels, steering, wiring, connectors, fuses, and battery;
- verify the physical cutoff with propulsion power enabled and the wheels clear;
- identify the exact host build and firmware version;
- make sure ESP32 and Lidar ports are not swapped;
- validate the controller mapping and neutral/dead-zone behavior;
- confirm the configured speed limit and Lidar threshold;
- keep bystanders outside the controlled test area;
- assign one operator and, for motion testing, one cutoff observer; and
- start in the wheels-off-ground configuration.

## Failure capture

If motion is unexpected, do not troubleshoot through the GUI:

1. operate the independent physical cutoff;
2. isolate propulsion power when safe;
3. preserve logs, versions, configuration, and a timeline;
4. reproduce with the simulator or wheels-clear rig; and
5. add a regression test before closing the defect.

## Vehicle validation prerequisites

Before an application build is exercised on the vehicle, record:

- independent physical cutoff and documented electrical safe state;
- firmware source or an identified firmware artifact;
- controlled firmware identity records plus protocol-version and CRC validation;
- acknowledgement and timeout enforcement;
- firmware-side heartbeat watchdog tested by removing host communication;
- bounded RPLidar unplug/partial-response/worker-shutdown behavior with the
  pinned driver;
- automated safety transition and queue-priority tests;
- clean-machine packaged-build test; and
- completed [hardware validation](hardware-validation.md).
