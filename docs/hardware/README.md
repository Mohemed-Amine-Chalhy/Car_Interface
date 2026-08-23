# Physical vehicle platform

Car Interface runs on a custom autonomous-driving car that my multidisciplinary
engineering team physically built and demonstrated. Within that team, I was
responsible for the vehicle software and AI-model integration. This section
defines the installed platform, its software interfaces, and the
vehicle-specific values to confirm before a physical run.

![Physical platform topology](../assets/hardware-topology.svg)

## Platform at a glance

| Subsystem | Identification | Evidence state |
| --- | --- | --- |
| ESP32 board | 38-pin development board with an ESP-WROOM-32 module and CP2102 USB-to-UART bridge | **Confirmed component; candidate primary-controller role** |
| Arduino board | Uno R3-form-factor board with DIP ATmega328P | **Confirmed component; exact role TBD** |
| Ranging | USB-connected RPLidar feeding a 2D front-corridor view | **Project integration confirmed; exact model TBD** |
| Vision | Host camera feeding a YOLO11n object-detection view | **Project integration confirmed; exact camera TBD** |
| Driver input | Xbox/PlayStation-compatible game controller through pygame | **Project integration confirmed; exact controller TBD** |
| Vehicle | Custom chassis with propulsion, steering, brake actuation, motor drivers, and battery power | **Confirmed system; detailed BOM and measurements TBD** |
| Host | Windows desktop application with control, Lidar, camera, logging, and device connection views | **Confirmed software component** |

Both pictured boards were used in the vehicle. The host configuration targets
the ESP32 as the serial endpoint. The Uno's exact role remains part of the
firmware, wiring, and pin-map record for the car.

## Documentation set

- [Component inventory](component-inventory.md) records every known subsystem,
  its evidence, and the missing identification data.
- [Vehicle specification](vehicle-specification.md) separates confirmed facts,
  recorded project settings, and dimensions still to measure.
- [Configuration profile](configuration-profile.md) maps the vehicle settings
  to the host application configuration.
- [Firmware guide](../firmware/README.md) explains the two boards and the
  protocol boundary.
- [Perception guide](../perception/README.md) documents RPLidar and YOLO11n
  integration.

## Vehicle configuration checklist

Before the next physical run, record:

1. one overview photograph of the assembled car and close-ups of every board;
2. the RPLidar, camera, motor-driver, actuator, motor, and battery labels;
3. chassis length, width, height, wheelbase, track width, wheel diameter, and
   ready-to-run mass;
4. connector-to-connector wiring and every controller GPIO assignment;
5. the ESP32 and Uno firmware sources or binary hashes;
6. a serial transcript from the working car; and
7. controller, steering, speed, Lidar, camera, and AI calibration results.

Update each candidate or TBD row with the measured value and its supporting
artifact.
