Of course, here is a README.md file suitable for your GitHub repository.

---

# ESP32 Remote Control Car with Lidar Assistance

This repository contains the Python-based graphical user interface (GUI) for controlling a custom-built, ESP32-powered car. The project was developed for a school assignment and features real-time control via a game controller, Lidar-based obstacle detection, and an advanced user interface for monitoring and interaction.

## Demonstration

Here is a brief look at our car in action and the control interface.

**Control Interface GUI**
[![Screenshot-2025-08-13-022142.png](https://i.postimg.cc/WbGvL6HC/Screenshot-2025-08-13-022142.png)](https://postimg.cc/Mvp41Rxm)

*A screenshot of the main application window, showing the Robot Control and Lidar Visualizer tabs.*

**Video Showcase**

![Demo](https://i.postimg.cc/vmDkBW0Q/ezgif-84aa40de80b339.gif)

## Features

*   **Dual-Mode Control:**
    *   **Controller Mode:** Drive the car using a standard Xbox or PlayStation controller, with analog steering and acceleration for precise control.
    *   **Manual GUI Mode:** Use sliders and buttons on the interface for direct control and testing.
*   **Advanced Lidar System:**
    *   **Real-time Visualization:** A dedicated tab displays a 2D plot of the Lidar's surroundings.
    *   **Obstacle Detection:** Identifies and highlights potential obstacles directly in the car's path.
    *   **Auto-Stop Assist:** An intelligent safety feature that automatically triggers an emergency stop if an obstacle gets too close.
    *   **Proximity Alerts:** Plays audible beeps that increase in frequency as the car approaches an obstacle.
*   **Robust Connectivity:**
    *   Communicates with the ESP32 via a stable serial connection.
    *   Automatic detection of available serial ports.
*   **Comprehensive User Interface:**
    *   **Robot Control Tab:** Manage connections, toggle control modes, and view live logs.
    *   **Lidar Visualizer Tab:** Activate/deactivate the Lidar, configure the auto-stop threshold, and monitor the environment.
    *   **Real-time Logging:** A console displays all communication with the ESP32, controller events, and system status messages.
*   **Safety First:**
    *   **Emergency Stop (E-Stop):** A prominent E-Stop button in the GUI and a failsafe that triggers if the controller disconnects during operation.
    *   **Manual Brake Override:** Engage or release the car's brake manually from both the GUI and the controller.

## Hardware Components

To replicate this project, you will need the following hardware:
*   **Car Chassis:** A custom-built car frame with motors and motor drivers.
*   **Main Microcontroller:** An ESP32 development board to control the motors and steering.
*   **Lidar Sensor:** An RPLidar (A1 or similar model) for environmental scanning.
*   **Power Source:** A battery pack suitable for powering the ESP32 and motors.
*   **Game Controller:** An Xbox or PS5-compatible controller.
*   **Host Computer:** A computer to run the Python control application.

## Software & Installation

This project is built with Python and relies on several external libraries.

### Prerequisites

*   **Python 3.7+**
*   **Arduino IDE** (for programming the ESP32)

### Installation Steps

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/your-username/your-repository-name.git
    cd your-repository-name
    ```

2.  **Set up the ESP32:**
    *   Open the Arduino IDE and install the necessary libraries for your motor driver and other components.
    *   Upload the corresponding `.ino` sketch (not included in this file) to your ESP32. This sketch should be able to interpret serial commands like `V [speed]`, `W [steer_value]`, `S 1` (Stop), `Q 1` (Release Stop), etc.

3.  **Install Python Libraries:**
    Install all the required Python packages using pip:
    ```bash
    pip install pyserial pygame rplidar-python
    ```
    *Note: `tkinter` is part of the standard Python library and does not need to be installed separately.*

## How to Run

1.  **Connect Hardware:**
    *   Connect the ESP32-powered car to your computer via USB.
    *   Connect the RPLidar to your computer via USB.
    *   Ensure your game controller is connected to the computer (via Bluetooth or USB).

2.  **Launch the Application:**
    Run the main Python script from the terminal:
    ```bash
    python your_main_script_name.py
    ```

3.  **Using the Interface:**
    *   **Connect to Car:** In the "Robot Control" tab, select the correct COM port for your ESP32 from the dropdown menu and click **Connect**.
    *   **Connect Controller:** Click the **Connect** button in the "Xbox Controller" section. The status should change to "Connected".
    *   **Activate Lidar:** Switch to the "Lidar Visualizer" tab, select the Lidar's COM port, and click **Activate LIDAR**.
    *   **Enable Control:** Return to the "Robot Control" tab and click **Enable Control**. This will switch the car to "AUTOMATIC" mode, allowing you to drive with the controller.
    *   **Drive!** Use the right stick for steering and the right trigger for acceleration. The left trigger acts as a manual brake.

## Future Improvements

*   **Path Planning:** Implement autonomous navigation algorithms that use the Lidar data to follow a path or avoid obstacles automatically.
*   **Web-Based UI:** Convert the Tkinter GUI into a web application using a framework like Flask or Django, allowing control from any device on the network.
*   **Camera Integration:** Add a camera to the car and stream the video feed to the control interface for FPV (First-Person View) driving.
*   **Better Controller Mapping:** Create a configuration file or GUI section to allow users to easily remap controller buttons and axes.

## Project Team


