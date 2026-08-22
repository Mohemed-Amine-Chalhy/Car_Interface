# Physical vehicle platform

Car Interface originated on a custom autonomous-driving car that was physically
built and demonstrated by a multidisciplinary engineering team. Mohamed Amine
Chalhy owned the vehicle software and AI-model integration within that team.
This section records what is known about the actual platform and makes the
remaining reconstruction work visible.

![Physical platform topology](../assets/hardware-topology.svg)

## Platform at a glance

| Subsystem | Identification | Evidence state |
| --- | --- | --- |
| Main embedded board | 38-pin ESP32 development board with an ESP-WROOM-32 module and CP2102 USB-to-UART bridge | **Confirmed component; candidate primary-controller role** |
| Secondary embedded board | Arduino Uno R3-form-factor board with DIP ATmega328P | **Confirmed component; exact role TBD** |
| Ranging | USB-connected RPLidar feeding a 2D front-corridor view | **Historical integration; exact model TBD** |
| Vision | Host camera feeding a YOLO11n object-detection view | **Historical integration; exact camera TBD** |
| Driver input | Xbox/PlayStation-compatible game controller through pygame | **Historical integration; exact controller TBD** |
| Vehicle | Custom chassis with propulsion, steering, brake actuation, motor drivers, and battery power | **Confirmed system; detailed BOM and measurements TBD** |
| Host | Windows desktop application with control, Lidar, camera, logging, and device connection views | **Historical and maintained implementations** |

The owner confirmed that both pictured boards were used in the vehicle. The old
host code used the words “Arduino” and “ESP32” inconsistently for its serial
endpoint, so board responsibilities cannot be reconstructed from class names
alone. Repository evidence most strongly supports the ESP32 as the primary
host-facing vehicle controller. A separate Arduino serial experiment suggests
the Uno may have handled an auxiliary actuator, but firmware or wiring evidence
is required before that becomes a specification.

## Documentation set

- [Component inventory](component-inventory.md) records every known subsystem,
  its evidence, and the missing identification data.
- [Vehicle specification](vehicle-specification.md) separates confirmed facts,
  historical software settings, and dimensions still to measure.
- [Configuration profile](configuration-profile.md) maps the reconstructed car
  settings to the maintained host configuration.
- [Firmware dossier](../firmware/README.md) explains the two boards and the
  current protocol boundary.
- [Perception dossier](../perception/README.md) documents RPLidar and YOLO11n
  work from the original prototype.

## What would complete the physical record

The next evidence pass should capture:

1. one overview photograph of the assembled car and close-ups of every board;
2. the RPLidar, camera, motor-driver, actuator, motor, and battery labels;
3. chassis length, width, height, wheelbase, track width, wheel diameter, and
   ready-to-run mass;
4. connector-to-connector wiring and every controller GPIO assignment;
5. the ESP32 and Uno firmware sources or binary hashes;
6. a serial transcript from the working car; and
7. controller, steering, speed, Lidar, camera, and AI calibration results.

That evidence can be added without changing the structure of this dossier: each
candidate or TBD row simply advances to confirmed with a linked artifact.
