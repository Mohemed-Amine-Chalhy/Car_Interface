# Hardware validation checklist

This checklist captures one exact combination of vehicle, electronics,
firmware, host build, configuration, controller, Lidar, and test environment.
It turns a hardware session into reproducible engineering evidence.

Stop immediately on unexpected motion or an ineffective cutoff, mark the run
failed, and follow the failure-capture process in
[Control-safety architecture](safety.md).

## Record identity

| Item | Recorded value |
| --- | --- |
| Date/time and location | |
| Test owner and cutoff observer | |
| Vehicle/rig identifier | |
| Chassis, motor, steering, brake, and driver revisions | |
| Battery type, nominal voltage, and protection | |
| Physical cutoff design/revision | |
| Host computer and Windows build | |
| Car Interface version/commit and artifact SHA-256 | |
| Configuration path and SHA-256 | |
| ESP32 board, firmware version/commit, artifact SHA-256 | |
| Protocol version | |
| Serial startup delay and measured firmware boot-ready time | |
| Controller model, connection, driver, and mapping revision | |
| RPLidar model, serial identity, driver, and mount revision | |

Attach wiring diagrams, relevant datasheets, build logs, and the final
configuration.

## Static inspection

- [ ] Mechanical fasteners, wheel retention, steering, and brake inspected.
- [ ] Wiring gauge, insulation, strain relief, polarity, and connectors inspected.
- [ ] Battery protection and fuse ratings reviewed.
- [ ] Logic and propulsion power domains match the electrical design.
- [ ] Physical cutoff is reachable and independent of host and ESP32 software.
- [ ] Vehicle defaults to non-propulsive outputs during ESP32 reset/unpowered I/O.
- [ ] Lidar is rigid, correctly oriented, and unobstructed.
- [ ] No cable can enter steering or wheel travel.

## Software-only evidence

- [ ] Locked clean install succeeds on a clean Windows machine.
- [ ] `scripts/dev.py check --ci` passes.
- [ ] `scripts/dev.py release-check` passes.
- [ ] The configured aggregate coverage gate passes.
- [ ] Simulator tests cover disconnect, stale input, timeout, NACK, malformed
      frame, worker failure, queue saturation, close, and repeated reconnect.
- [ ] Built application starts in simulation without Python or source checkout.
- [ ] Build checksums and release notes are archived.

## No-propulsion-power checks

- [ ] Hardware mode cannot start without the exact acknowledgement flag.
- [ ] Wrong/missing configuration and swapped ports are rejected safely.
- [ ] Flash records and SHA-256 identify the tested firmware build.
- [ ] Protocol-v1 frame/version validation succeeds; wrong protocol versions are
      rejected safely.
- [ ] Invalid CRC, sequence, opcode, range, and oversized frames receive safe
      rejection and cannot actuate output.
- [ ] Startup remains zero-speed and braked.
- [ ] The configured serial startup delay exceeds measured boot readiness with
      justified margin; opening the port/resetting never produces motion.
- [ ] Controller axes/buttons and neutral/dead zone match the recorded mapping.
- [ ] Lidar angles and centimetre conversion match measured targets.
- [ ] Unplugging each required device produces the expected latched fault.
- [ ] Reset cannot occur while a required device remains unhealthy/stale.
- [ ] Reconnect/reset never replays an old motion command.

## Wheels-clear powered checks

Use the lowest possible propulsion limit and keep the cutoff observer ready.

- [ ] Physical cutoff stops propulsion from minimum and maximum tested command.
- [ ] Software E-stop stops and latches from each state.
- [ ] Brake and disarm evict pending motion commands.
- [ ] Releasing brake alone remains `BRAKING`; fresh neutral plus explicit Arm is
      required before a later drive command may release the brake.
- [ ] Controller disconnect stops and latches within the measured requirement.
- [ ] ESP32 USB disconnect invokes the firmware watchdog and safe output.
- [ ] Host process termination invokes the firmware watchdog and safe output.
- [ ] Host suspension/freeze invokes the firmware watchdog and safe output.
- [ ] Lidar disconnect/staleness invokes the configured safe response.
- [ ] RPLidar unplug, partial response, blocked read, and shutdown latency are
      measured; the old driver cannot leave supervision or shutdown hung.
- [ ] ACK loss, repeated NACK, corrupted traffic, and queue saturation are safe.
- [ ] Closing the UI and normal shutdown end zero-speed and braked.
- [ ] Twenty connect/arm/disarm/disconnect cycles show no stale state or resource
      leak.

Record measured stop latency for each failure; a visual impression is not enough.

## Controlled-ground checks

Perform only after every prior item passes and the test environment, limits,
and observer roles are recorded.

- [ ] Exclusion zone, restraint/runoff, surface, lighting, and observer positions
      are documented.
- [ ] Worst-case mechanical stopping distance is measured at each tested
      speed/load/surface condition.
- [ ] Lidar assist threshold exceeds measured stopping distance plus sensing,
      host, protocol, actuation latency, vehicle envelope, and safety margin.
- [ ] Expected and difficult target materials/geometry are tested.
- [ ] Steering endpoints and direction are verified at low speed.
- [ ] Physical cutoff and software stop remain effective under motion/load.
- [ ] A sustained soak run shows no UI freeze, unbounded queue, missed heartbeat,
      resource leak, or thermal/electrical problem.

## Acceptance

Document every failed or skipped item. `N/A` requires a written rationale and
is not equivalent to pass.

| Run summary | Recorded value |
| --- | --- |
| Test owner | |
| Firmware revision | |
| Host commit | |
| Result | PASS / FAIL / INCOMPLETE |

Validated limits, observations, and follow-up work:

```text

```
