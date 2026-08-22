# Component inventory and evidence register

This is the bill of materials that can be reconstructed today. It deliberately
records unknowns rather than substituting visually similar products.

## Installed controller boards

### ESP32 vehicle controller candidate

**Identification:** 38-pin DevKit-style carrier with a metal-shielded
ESP-WROOM-32 module, Micro-USB, EN and BOOT buttons, and a visibly marked Silicon
Labs CP2102 USB-to-UART bridge.

**Evidence:** the product photograph I supplied and historical host classes
named for ESP32 control.

**What is confirmed:**

- the installed component family is ESP32-WROOM-32;
- USB serial is provided by a CP2102 bridge;
- the carrier exposes the module I/O on two 19-pin headers; and
- the board has separate reset/enable and boot controls.

**What remains unknown:** carrier vendor/revision, module flash capacity,
installed ESP32 board package, flash/partition settings, USB VID/PID/serial,
GPIO assignments, power input, and firmware revision.

The closest official functional reference is Espressif's
[ESP32-DevKitC V2 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide_v2.html),
which documents the ESP32-WROOM-32 module, Micro-USB connection, EN/Boot
controls, and broken-out I/O. The photographed carrier must not be described as
an official DevKitC or a specific third-party revision until its silkscreen or
purchase record is captured.

### Arduino Uno auxiliary-controller candidate

**Identification:** Arduino Uno R3-form-factor board with a socketed DIP
ATmega328P, USB Type-B connector, barrel jack, ICSP headers, 14 digital headers,
and six analog-input headers. The exact vendor and board revision are not visible
in the supplied image.

**Evidence:** my confirmation that this board type was used and a historical
Python serial experiment targeting an “Arduino” at 115200 baud.

Arduino's official [UNO R3 reference](https://docs.arduino.cc/hardware/uno-rev3)
specifies the reference platform as an ATmega328P board with 14 digital I/O
pins, six PWM-capable outputs, six analog inputs, a 16 MHz clock, USB, barrel
power, and ICSP. The official datasheet records 32 KB flash, 2 KB SRAM, and 1 KB
EEPROM. Those are reference-board capabilities; they do not identify the
vehicle's wiring or the exact manufacturer of the installed board.

**Candidate role:** auxiliary brake or actuator controller. The only supporting
repository evidence is an experimental script that sent `S`, `D`, and `A`
commands to an Arduino endpoint. It does not prove that script or dialect was
used in the final assembled car.

## Full system inventory

| ID | Component | Known identification | Project role | Evidence | Missing record |
| --- | --- | --- | --- | --- | --- |
| ECU-01 | ESP32 development board | 38-pin ESP-WROOM-32, CP2102, Micro-USB | Candidate primary vehicle controller and host serial gateway | **Confirmed component / candidate role** | Vendor, revision, flash size, pins, firmware |
| ECU-02 | Arduino Uno | R3 form factor, DIP ATmega328P | Candidate auxiliary actuator controller | **Confirmed component / candidate role** | Vendor/revision, pins, firmware, link to ESP32 |
| SEN-01 | RPLidar | RPLidar family | 2D scan input and obstacle corridor | **Historical** | Exact model, serial, mount, scan rate |
| SEN-02 | Camera | Host camera index 0 | Video capture and YOLO inference | **Historical** | Make/model, lens, resolution, mount, calibration |
| HMI-01 | Game controller | Xbox/PS-compatible | Steering, throttle, brake, direction | **Historical** | Exact model, connection, verified mapping |
| HMI-02 | Windows computer | Windows host | GUI, device orchestration, perception | **Historical** | Model, CPU/GPU, OS build, USB topology |
| ACT-01 | Propulsion system | Motor(s) and driver(s) present | Vehicle traction | **Confirmed system / details TBD** | Models, count, voltage, current, gearing, pins |
| ACT-02 | Steering system | Steering actuator present | Front-wheel steering | **Confirmed system / details TBD** | Model, mechanism, range, PWM/position units |
| ACT-03 | Brake system | Commanded brake actuator present | Engage/release braking | **Historical behavior / hardware TBD** | Actuator, driver, fail state, pins |
| PWR-01 | Battery system | Battery-powered vehicle | Logic and actuator energy | **Confirmed system / details TBD** | Chemistry, voltage, capacity, BMS, fuse |
| MEC-01 | Chassis | Custom-built vehicle | Mechanical platform | **Confirmed** | Dimensions, mass, material, CAD |

## Evidence capture naming

Use stable names so future documentation can link directly to the physical
record:

```text
docs/hardware/evidence/
  vehicle-overview-front.jpg
  vehicle-overview-side.jpg
  ecu-01-esp32-front.jpg
  ecu-01-esp32-back.jpg
  ecu-02-uno-front.jpg
  sen-01-rplidar-label.jpg
  sen-02-camera-label.jpg
  act-01-motor-driver-label.jpg
  pwr-01-battery-label.jpg
  wiring-overview.jpg
```

Repository-native diagrams under [`docs/assets`](../assets/) communicate the
confirmed topology and keep the hardware record consistent with the rest of the
documentation.
