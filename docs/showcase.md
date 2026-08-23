# Software showcase

The walkthrough runs the Tkinter operator application in simulation mode with
the ESP32, controller, firmware, and RPLidar adapters. It covers the operator
flow from connection and arming to obstacle response and diagnostics.

## Run the guided walkthrough

Set up the locked environment first, then start the simulation with the showcase
flag:

~~~powershell
.\scripts\bootstrap.ps1
.\.venv\Scripts\python.exe scripts\dev.py run-sim --showcase
~~~

For free-form control, run `run-sim` without `--showcase`.

## What the walkthrough exercises

1. Opens the operator dashboard in a disconnected, braked state.
2. Connects the simulated ESP32 transport, game controller, and RPLidar.
3. Arms the vehicle when all required devices report ready.
4. Sets speed to 32% and steering to -18%.
5. Opens the live vehicle-relative Lidar view.
6. Moves the simulated obstacle from 180 cm to 35 cm.
7. Triggers the latched emergency stop when the obstacle enters the projected path.
8. Opens Diagnostics to show commands, acknowledgements, and events.
9. Clears the simulated obstacle, resets the latch, and returns to a safe state.

The guided sequence uses the same service methods as the manual controls and
advances when each expected state is observed.

## Regenerate the repository media

The capture command launches a visible 1200 x 800 application window, records
it at eight frames per second, saves six milestone screenshots, and writes a
960-pixel-wide, 128-color looping GIF:

~~~powershell
.\.venv\Scripts\python.exe scripts\dev.py capture-showcase
~~~

The command writes to `docs/assets/showcase/`:

~~~text
01-control-disconnected.png
02-control-connected.png
03-control-driving.png
04-lidar-monitoring.png
05-lidar-assisted-stop.png
06-diagnostics.png
app-walkthrough.gif
~~~

The window must remain visible while capture runs. The script prefers native
window-handle capture on Windows and falls back to the Tk client-area bounds.
Pillow is installed as a development dependency because it is only used by the
capture command.

## Demo modes

The guided walkthrough shows simulation mode. The README pairs it with a
repository-hosted drive-test excerpt and the full 2025 team showcase.
