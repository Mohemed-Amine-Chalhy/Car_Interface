# Operator guide

Run simulation first. Vehicle testing uses the original firmware, verified
wiring, and a validated configuration, following
[Control-safety architecture](safety.md), [Hardware setup](hardware-setup.md),
and the wheels-clear section of the
[Hardware validation checklist](hardware-validation.md).

## Roles

For any test that can create motion:

- the **operator** controls the application and announces every state change;
- the **cutoff observer** remains at the independent physical E-stop and does
  not operate the GUI; and
- no bystander enters the controlled area.

One person must not try to operate the computer and guard a distant cutoff at
the same time.

## Simulation session

Start with:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py run-sim
```

Confirm the UI identifies every device as simulated. Exercise connect, arm,
drive, brake, E-stop, reset, Lidar obstacle, and simulated disconnect behavior.
Expected outcomes are:

- connect enters `SAFE_CONNECTED`, not `ARMED` or `DRIVING`;
- arm succeeds only when all required simulated devices are healthy;
- brake, E-stop, stale input, or disconnect immediately shows zero propulsion;
- E-stop and fault remain latched until an explicit reset; and
- reset returns to a safe state and never resumes the previous speed.

## Hardware preflight

Complete and record every item:

- [ ] Exact host release/version and firmware version are compatible.
- [ ] Configuration belongs to this vehicle and its hash is recorded.
- [ ] Propulsion limit and obstacle threshold match the test plan.
- [ ] Battery, wiring, chassis, steering, brakes, and sensor mount pass inspection.
- [ ] Driven wheels are clear of the ground for initial checks.
- [ ] Physical cutoff was tested during this session.
- [ ] Operator and cutoff observer agree on stop words and responsibilities.
- [ ] ESP32 and Lidar ports were identified by device, not guessed by COM number.
- [ ] Controller mapping, neutral position, and battery/connection are healthy.
- [ ] Test area is controlled and free of bystanders.

Do not continue with a failed or uncertain item.

## Start hardware mode

The explicit hardware flag prevents accidental startup and remains a manual
action:

```powershell
.\.venv\Scripts\python.exe scripts\dev.py run-hardware `
  --config C:\path\to\vehicle.toml `
  --i-understand-this-controls-real-hardware
```

The equivalent installed entry point is:

```powershell
car-interface run --mode hardware `
  --config C:\path\to\vehicle.toml `
  --i-understand-this-controls-real-hardware
```

Do not store the flag in a desktop shortcut, environment profile, scheduled
task, or automatic startup service.

## Connect and arm

1. Start with controller inputs neutral and the physical cutoff ready.
2. Connect the actuator. Confirm protocol-v1 frames are acknowledged and the
   state is `SAFE_CONNECTED`; verify firmware identity against the external
   controlled-flash/artifact-hash record because protocol v1 does not report it.
3. Connect the configured controller. Move each control individually while
   propulsion remains isolated and verify direction/range.
4. Connect the Lidar. Confirm fresh scans, forward angle, correct scale, and no
   unexplained blind region.
5. Verify the UI reports zero speed and brake asserted.
6. Leave throttle and steering at neutral, then explicitly arm. The neutral
   interlock rejects arming if the latest controller input is non-neutral. If
   arming is refused, diagnose the failed prerequisite; do not disable it.

Arming is permission to accept motion input, not a movement command.

## Driving

- Begin at the lowest validated speed.
- Keep the vehicle continuously visible.
- Keep hands clear of the vehicle and one person at the cutoff.
- Make one input change at a time during commissioning.
- Treat delayed, contradictory, stale, or unexpected UI feedback as a fault.
- Do not reconnect a controller, serial cable, or Lidar while the vehicle is
  free to move; stop and mechanically secure it first.

The configured maximum speed is a command clamp, not a speed guarantee. Actual
vehicle speed depends on firmware, supply, load, and mechanics.

## Brake and E-stop

Use ordinary brake/neutral for planned stops. Use software E-stop when immediate
software intervention is needed. Use the physical cutoff for unexpected motion,
loss of confidence, an unresponsive UI, or any situation where software status
cannot be trusted.

After braking, return throttle, steering, and brake inputs to neutral, then press
**Arm** explicitly. Releasing the brake trigger or selecting the GUI brake-
release control does not arm the vehicle: the state remains `BRAKING` and the
brake remains asserted. Only a fresh neutral input followed by explicit **Arm**
can enter `ARMED`; the next validated drive command releases the brake. No prior
motion command is replayed.

After a software E-stop:

1. verify the vehicle is physically stopped;
2. remove the hazard or fault cause;
3. check device health and logs;
4. explicitly reset the latch; and
5. verify the state returns to `SAFE_CONNECTED` before re-arming.

Reset must never return directly to `DRIVING` or replay queued inputs.
No controller button resets E-stop or fault state by default; use the confirmed
GUI reset after checking the vehicle and required-device health.

## Obstacle assist

An in-path point inside the configured threshold can request a stop. A clear
display does not prove a clear path. If the Lidar becomes stale or disconnects,
the default required-device policy faults and inhibits motion. Do not change the
policy or threshold during a run to suppress stops.

## Normal shutdown

1. Command neutral and brake.
2. Disarm and confirm `SAFE_CONNECTED`.
3. Use the physical cutoff or isolate propulsion power.
4. Disconnect devices in the UI.
5. Close the application and confirm all device activity stops.
6. Disconnect the battery according to the vehicle procedure.
7. Save logs and record anomalies before changing the configuration or build.

A shutdown message is not proof of a physical safe state. Verify the vehicle.
