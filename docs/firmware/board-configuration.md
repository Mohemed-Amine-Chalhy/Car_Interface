# ESP32 and Arduino board configuration

This page turns the supplied board identification into a reproducible firmware
configuration plan. It separates settings that can be selected now from values
that must be recovered from the physical car.

## ESP32-WROOM-32 development board

### Confirmed hardware

- 38-pin DevKit-style carrier;
- ESP-WROOM-32 module;
- Silicon Labs CP2102 USB-to-UART bridge;
- Micro-USB connector;
- EN/reset and BOOT buttons; and
- two 19-pin I/O headers.

The carrier vendor and revision remain unknown. Espressif's
[ESP32-DevKitC V2 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide_v2.html)
is a useful functional reference, but it is not proof that the installed
carrier is an official DevKitC.

### Settings to recover before rebuilding

| Build setting | Current value | How to verify |
| --- | --- | --- |
| Framework | Candidate: Arduino framework | Recover source/includes or build log |
| Espressif board package | TBD | Recover IDE preferences or compile metadata |
| Board identifier | TBD; do not assume `esp32dev` from appearance | Read carrier marking and test a non-actuating firmware build |
| Module flash size | TBD | Query chip/flash ID, compare module marking |
| Flash mode/frequency | TBD | Recover build configuration or known-good binary metadata |
| Partition scheme | TBD | Recover build configuration |
| Upload speed | TBD | Recover IDE/PlatformIO settings; start conservatively |
| Serial monitor speed | 115200 candidate | Historical host configuration |
| USB driver | Silicon Labs CP210x | Confirm VID/PID and driver in Device Manager |
| Firmware libraries | TBD | Recover lock, `platformio.ini`, or Arduino library list |

`Serial.begin(115200)` is the historical compatibility target, but serial baud
alone does not select the command dialect. The integrated prototype used
newline commands; the maintained application uses [protocol v1](../protocol.md).

### Reproducible PlatformIO template

This template is intentionally incomplete until the exact carrier and toolchain
versions are confirmed:

```ini
[platformio]
default_envs = esp32_vehicle

[env:esp32_vehicle]
platform = espressif32@<PINNED_VERSION>
board = <CONFIRMED_BOARD_ID>
framework = arduino
monitor_speed = 115200
upload_speed = <CONFIRMED_UPLOAD_SPEED>

build_flags =
  -D VEHICLE_PROTOCOL_VERSION=<CONFIRMED_VERSION>
  -D VEHICLE_BOARD_REVISION=\"<CONFIRMED_REVISION>\"
```

Record the resolved platform packages, compiler version, binary size, and
SHA-256 for every firmware build.

## Arduino Uno R3-style board

### Confirmed platform capability

The pictured board follows the Uno R3 layout with a DIP ATmega328P. Arduino's
official [UNO R3 reference](https://docs.arduino.cc/hardware/uno-rev3) specifies:

- ATmega328P at 16 MHz;
- 14 digital I/O pins, six supporting PWM;
- six analog inputs;
- USB Type-B, barrel-jack power, and ICSP connections;
- 32 KB flash, 2 KB SRAM, and 1 KB EEPROM on the reference board.

The photograph does not establish genuine-versus-clone provenance, and those
capabilities do not identify how the car used the pins.

### Build configuration

| Setting | Selection |
| --- | --- |
| Arduino IDE board | `Arduino Uno` |
| PlatformIO board | `uno` |
| MCU | ATmega328P |
| Clock | 16 MHz, selected by the board definition |
| Framework | Arduino candidate; recover source to confirm |
| Serial speed | 115200 only if recovered firmware confirms the historical experiment |
| Port | Select by USB identity; do not commit a COM number |
| Libraries | TBD from recovered firmware |

```ini
[env:uno_aux]
platform = atmelavr@<PINNED_VERSION>
board = uno
framework = arduino
monitor_speed = 115200

build_flags =
  -D AUX_PROTOCOL_VERSION=<CONFIRMED_VERSION>
  -D AUX_BOARD_REVISION=\"<CONFIRMED_REVISION>\"
```

Pins 0/1 are the Uno hardware UART used by its USB-serial path. Do not also wire
them to the ESP32 until the inter-board topology and voltage/interface behavior
have been documented.

## Installed pin-map worksheet

No GPIO assignment can be inferred safely from the product photos. Populate
this table from firmware and continuity measurements before presenting a wiring
diagram as historical fact.

| Function | Board | GPIO/pin | Signal type | Active state / range | Boot/reset state | Evidence |
| --- | --- | --- | --- | --- | --- | --- |
| Propulsion enable | TBD | TBD | Digital/PWM TBD | TBD | TBD | Firmware + continuity |
| Propulsion command | TBD | TBD | PWM/digital TBD | `V 0..100` host candidate | TBD | Firmware + oscilloscope |
| Direction forward/reverse | TBD | TBD | Digital pair TBD | `D F` / `D R` host candidate | TBD | Firmware + continuity |
| Steering command | TBD | TBD | PWM/servo TBD | raw 200/1750/2900 candidate | TBD | Firmware + endpoint test |
| Brake engage | TBD | TBD | Digital/servo TBD | `S 1` host candidate | TBD | Firmware + continuity |
| Brake release | TBD | TBD | Digital/servo TBD | `Q 1` host candidate | TBD | Firmware + continuity |
| ESP32 ↔ Uno link | Both | TBD | UART/I²C/SPI TBD | TBD | TBD | Wiring + firmware |
| Status/diagnostic output | TBD | TBD | LED/serial TBD | TBD | TBD | Firmware |

## Recovery and bring-up sequence

1. Photograph both board faces and every connected header before unplugging
   anything.
2. Label every conductor at both ends and create a continuity map.
3. Record Windows USB VID, PID, serial number, driver, and current COM port for
   each USB device.
4. Read the ESP32 chip and flash identity without erasing it; archive any
   recoverable flash image and its SHA-256.
5. Archive the Uno flash/EEPROM only if the project owner has a known-good
   recovery method; record fuses and lock bits.
6. Recover source, libraries, board packages, IDE settings, and build logs from
   the original development computers.
7. Compile deterministic binaries without connecting actuator power.
8. Replay a captured [legacy protocol](legacy-protocol.md) transcript on a bench
   and compare serial responses and output waveforms.
9. Replace candidate pin, timing, range, and role entries with the verified
   values.
10. Tag the host, ESP32, Uno, configuration, and evidence report as one tested
    vehicle profile.

This process preserves compatibility with the already-built car while making
future changes traceable and repeatable.
