# Safety model

## Safety notice

Car Interface can send commands to machinery. Software defects, USB failures,
controller disconnects, stale Lidar data, firmware defects, electrical faults,
or operator mistakes can create unexpected motion. The application is one layer
in a larger safety system; it is not a certified protective device.

Every physical rig requires an independent emergency-stop or propulsion-power
cutoff that does not depend on the host, Python process, USB link, ESP32 main
loop, game controller, or Lidar. The cutoff must be reachable by the operator
and a test observer.

## v0.1 safety boundary

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

These are software requirements. A release gate verifies them in tests;
physical qualification verifies the complete system.

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

## Lidar limitations

Obstacle assist is a supplemental input, not collision avoidance certification.
RPLidar may miss transparent, dark, absorbent, narrow, low, high, fast-moving,
or occluded objects. A threshold is only meaningful after confirming sensor
mounting, units, scan freshness, vehicle envelope, speed, stopping distance, and
surface conditions.

The default `auto_stop_distance_cm` is a conservative software starting value,
not a proven stopping distance. Qualification must derive a larger threshold
when measured worst-case stopping distance plus latency and margin require it.
Never reduce the threshold simply to avoid nuisance stops.

## Before every physical session

- inspect the chassis, wheels, steering, wiring, connectors, fuses, and battery;
- verify the physical cutoff with propulsion power enabled and the wheels clear;
- identify the exact host build and firmware version;
- make sure ESP32 and Lidar ports are not swapped;
- validate the controller mapping and neutral/dead-zone behavior;
- confirm the configured speed limit and Lidar threshold;
- keep bystanders outside the controlled test area;
- assign one operator and, for motion testing, one cutoff observer; and
- start in the wheels-off-ground configuration.

## Unexpected behavior

If motion is unexpected, do not troubleshoot through the GUI:

1. operate the independent physical cutoff;
2. isolate propulsion power when safe;
3. prevent reuse of the vehicle/build;
4. preserve logs, versions, configuration, and a timeline; and
5. report privately under [SECURITY.md](../SECURITY.md).

Do not resume because the symptom disappeared after a restart.

## Release blockers

A physical release is blocked when any of these is absent or failing:

- independent physical cutoff and documented electrical safe state;
- firmware source or immutable firmware artifact with provenance;
- controlled firmware identity records plus protocol-version and CRC validation;
- acknowledgement and timeout enforcement;
- firmware-side heartbeat watchdog tested by removing host communication;
- bounded RPLidar unplug/partial-response/worker-shutdown behavior with the
  pinned driver;
- automated safety transition and queue-priority tests;
- clean-machine packaged-build test; and
- completed [hardware qualification](hardware-qualification.md).
