# Software showcase

The README walkthrough is an authentic recording of the maintained Tkinter
application operating against the repository's deterministic simulated vehicle,
controller, firmware, and RPLidar adapters. I use it to demonstrate the complete
operator flow without requiring access to the physical car.

## Run the guided walkthrough

Set up the locked environment first, then start the simulation with the showcase
flag:

~~~powershell
.\scripts\bootstrap.ps1
.\.venv\Scripts\python.exe scripts\dev.py run-sim --showcase
~~~

The normal interactive simulator remains available through `run-sim` without
the flag.

## What the walkthrough exercises

1. Opens the operator dashboard in a disconnected, braked state.
2. Connects the simulated ESP32 transport, game controller, and RPLidar.
3. Arms the safety state machine only after all required devices are ready.
4. Applies 32% speed and -18% steering through the real control service.
5. Traverses the live vehicle-relative Lidar view.
6. Moves the simulated obstacle from 180 cm to 35 cm.
7. Lets the regular projected-path analysis trigger the latched emergency stop.
8. Opens Diagnostics to show the genuine commands, acknowledgements, and events.
9. Clears the simulated obstacle, resets the latch, and returns to a safe state.

The director does not write snapshots into the UI and does not bypass domain
transitions. It sequences the same operations an operator can invoke manually,
then waits for observable service states before advancing.

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
Pillow is locked as a development-only dependency; it is not part of the
desktop application's runtime dependency set.

## Media intent

The software walkthrough is explicitly a simulation-mode demonstration of the
maintained code. The separate video in the README shows the physical vehicle
that my team built and demonstrated. Keeping both makes the distinction between
repeatable software evidence and the completed real-world prototype clear.
