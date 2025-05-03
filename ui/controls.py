import tkinter as tk
from tkinter import ttk
import math
import serial
import time
from core.robot_state import RobotState

class ControlsPanel(ttk.Frame):
    def __init__(self, parent, main_window):
        super().__init__(parent, style="Dark.TFrame")
        self.main_window = main_window
        self.speed = 0
        self.steering_angle = 0
        self.steering_active = False
        self.frein_a_main_active = False
        
        # Persistent serial connection
        self.serial_connection = None
        self.connect_serial()

        # Connection controls - separate buttons for different components
        ttk.Label(self, text="Connections", style="Title.TLabel").pack(pady=10)
        
        # LIDAR connection
        lidar_frame = ttk.Frame(self, style="Dark.TFrame")
        lidar_frame.pack(fill="x", padx=20, pady=5)
        self.toggle_lidar_button = ttk.Button(lidar_frame, text="Connect LIDAR", 
                                              command=self.main_window.toggle_lidar_connection)
        self.toggle_lidar_button.pack(side="left", fill="x", expand=True)
        self.lidar_status_label = ttk.Label(lidar_frame, text="LIDAR: Disconnected", style="Dark.TLabel")
        self.lidar_status_label.pack(side="right", padx=5)
        
        # Camera connection
        camera_frame = ttk.Frame(self, style="Dark.TFrame")
        camera_frame.pack(fill="x", padx=20, pady=5)
        self.toggle_camera_button = ttk.Button(camera_frame, text="Connect Camera", 
                                               command=self.main_window.toggle_camera_connection)
        self.toggle_camera_button.pack(side="left", fill="x", expand=True)
        self.camera_status_label = ttk.Label(camera_frame, text="Camera: Disconnected", style="Dark.TLabel")
        self.camera_status_label.pack(side="right", padx=5)
        
        # Control enable/disable
        control_frame = ttk.Frame(self, style="Dark.TFrame")
        control_frame.pack(fill="x", padx=20, pady=5)
        self.toggle_control_button = ttk.Button(control_frame, text="Enable Control", 
                                                command=self.main_window.toggle_control_status)
        self.toggle_control_button.pack(side="left", fill="x", expand=True)
        self.control_status_label = ttk.Label(control_frame, text="Control: Disabled", style="Dark.TLabel")
        self.control_status_label.pack(side="right", padx=5)
       
        ttk.Label(self, text="Control Mode", style="Title.TLabel").pack(pady=10)
        mode_frame = ttk.Frame(self, style="Dark.TFrame")
        mode_frame.pack(fill="x", padx=20, pady=5)
        self.manual_button = ttk.Button(mode_frame, text="Manual",
                                        command=lambda: self.set_control_mode(False),
                                        style="Toggle.TButton")
       
        self.manual_button.pack(side="left", fill="x", expand=True)
        self.auto_button = ttk.Button(mode_frame, text="Platform",
                                      command=lambda: self.set_control_mode(True),
                                      style="Toggle.TButton")
       
        self.auto_button.pack(side="right", fill="x", expand=True)
        self.mode_label = ttk.Label(self, text="Manual", style="Dark.TLabel")
        self.mode_label.pack(fill="x", padx=20, pady=5)
       
        # Auto-stopping checkbox
        self.auto_stop_var = tk.BooleanVar()  # Variable to hold the state of the checkbox
        auto_stop_check = ttk.Checkbutton(self, text="Enable Auto Stop",
                                          variable=self.auto_stop_var)
       
        auto_stop_check.pack(pady=5, padx=20, fill="x")
       
        ttk.Label(self, text="Steering Control", style="Title.TLabel").pack(pady=10)

        steering_frame = ttk.Frame(self, style="Dark.TFrame")
        steering_frame.pack(fill="x", padx=20, pady=5)

        self.steering_slider = ttk.Scale(
            steering_frame,
            from_=-100,
            to=100,
            orient="horizontal",
            command=self.set_steering,
            length=200
        )
        self.steering_slider.pack(side="left", fill="x", expand=True, pady=5)

        self.steering_label = ttk.Label(steering_frame, text="0°", style="Speed.TLabel")
        self.steering_label.pack(side="right", padx=5, pady=5)

        ttk.Label(self, text="Speed Control", style="Title.TLabel").pack(pady=10)
        speed_frame = ttk.Frame(self, style="Dark.TFrame")
        speed_frame.pack(fill="x", padx=20, pady=5)

        self.speed_slider = ttk.Scale(speed_frame, from_=100, to=-100, orient="vertical",
                                     command=self.set_speed, length=200)
        self.speed_slider.pack(side="left", fill="x", expand=True, pady=5)
        self.speed_label = ttk.Label(speed_frame, text="0%", style="Speed.TLabel")
        self.speed_label.pack(side="right", padx=5, pady=5)
       
        # Frein à main button
        self.frein_a_main_button = ttk.Button(self, text="Activate Frein",
                                              command=self.toggle_frein_a_main)
        self.frein_a_main_button.pack(pady=5, padx=20, fill="x")

        self.emergency_button = ttk.Button(self, text="EMERGENCY STOP", command=self.emergency_stop)
        self.emergency_button.pack(fill="x", padx=20, pady=20)
        self.emergency_button.configure(style="Dark.TButton")

        # Proximity alerts
        proximity_frame = ttk.Frame(self, style="Dark.TFrame")
        proximity_frame.pack(fill="x", pady=10)
        self.proximity_label = ttk.Label(proximity_frame, text="No obstacles detected", style="Dark.TLabel")
        self.proximity_label.pack(side="left", padx=5)
        
        # Rate limiting for slider updates
        self.last_speed_update = 0
        self.last_steering_update = 0
        self.update_interval = 0.1  # Update at most every 100ms
        
        # Clean up serial connection when window is destroyed
        self.bind("<Destroy>", self.cleanup)
        
    def connect_serial(self):
        """Establish a persistent serial connection"""
        if self.serial_connection is not None and self.serial_connection.is_open:
            return  # Already connected
            
        try:
            serial_port = RobotState().sp32port
            baud_rate = 115200
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(0.5)  # Shorter wait time for initial connection
            print(f"Serial connection established on {serial_port}")
        except serial.SerialException as e:
            print(f"Error establishing serial connection: {e}")
            self.serial_connection = None
            
    def send_command(self, command):
        """Send a command through the serial connection"""
        if not self.main_window.control_enabled:
            print("Control is disabled. Enable control first.")
            return False
            
        if self.serial_connection is None or not self.serial_connection.is_open:
            self.connect_serial()
            if self.serial_connection is None:
                return False
                
        try:
            # Add newline if not present
            if not command.endswith('\n'):
                command += '\n'
                
            self.serial_connection.write(command.encode())
            print(f"Sent command: {command.strip()}")
            
            # Optional: Read response (non-blocking)
            self.read_serial_response()
            return True
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            self.serial_connection = None  # Reset connection on error
            return False
            
    def read_serial_response(self):
        """Read available data from serial port without blocking"""
        if self.serial_connection is None or not self.serial_connection.is_open:
            return
            
        try:
            if self.serial_connection.in_waiting:
                response = self.serial_connection.readline().decode().strip()
                print(f"Received response: {response}")
                return response
        except serial.SerialException as e:
            print(f"Error reading from serial port: {e}")
            
    def cleanup(self, event=None):
        """Close serial connection when widget is destroyed"""
        if self.serial_connection is not None and self.serial_connection.is_open:
            try:
                self.serial_connection.close()
                print("Serial connection closed")
            except serial.SerialException as e:
                print(f"Error closing serial connection: {e}")
       
    def set_control_mode(self, auto_mode):
        """Set the control mode (Manual or Platform)"""
        # Check if control is enabled before proceeding
        if not self.main_window.control_enabled:
            print("Control is disabled. Enable control first.")
            return

        self.main_window.robot_state.is_auto_mode = auto_mode
        if auto_mode:
            self.auto_button.state(["selected"])
            self.manual_button.state(["!selected"])
            self.mode_label.config(text="Platform Control")
            print("Control mode set to Platform. Sending 'A' command.")
            
            self.send_command('a')
            self.send_command('D F')
            self.send_command('v 50')
            
            # Schedule a command to stop after delay
            self.after(10000, lambda: self.send_command('v 0'))
        else:
            self.manual_button.state(["selected"])
            self.auto_button.state(["!selected"])
            self.mode_label.config(text="Manual Control")
            print("Control mode set to Manual. Sending 'M' command.")
            self.send_command('M')

    def set_steering(self, value):
        """Set the steering angle with rate limiting"""
        # Check if control is enabled before proceeding
        if not self.main_window.control_enabled:
            print("Control is disabled. Enable control first.")
            return
        
        # Apply rate limiting to avoid overwhelming the serial connection
        current_time = time.time()
        if current_time - self.last_steering_update < self.update_interval:
            return
            
        self.last_steering_update = current_time
        
        angle = float(value)
        self.steering_angle = angle
        self.main_window.robot_state.steering_angle = angle
        self.steering_label.config(text=f"{angle:.0f}%")  # Update label

        command = f"O {int(angle)}"
        self.send_command(command)

    def set_speed(self, event):
        """Set the speed with rate limiting"""
        # Check if control is enabled before proceeding
        if not self.main_window.control_enabled:
            print("Control is disabled. Enable control first.")
            return

        # Apply rate limiting to avoid overwhelming the serial connection
        current_time = time.time()
        if current_time - self.last_speed_update < self.update_interval:
            return
            
        self.last_speed_update = current_time

        self.speed = int(self.speed_slider.get())
        self.main_window.robot_state.speed = self.speed  # Update robot state
        self.speed_label.config(text=f"{self.speed}%")

        command = f"V {self.speed}"
        self.send_command(command)

    def emergency_stop(self):
        """Emergency stop function"""
        if not self.main_window.control_enabled:
            return
            
        self.speed = 0
        self.steering_angle = 0
        self.main_window.robot_state.speed = 0
        self.main_window.robot_state.steering_angle = 0
        self.speed_label.config(text=f"{self.speed}%")
        self.steering_label.config(text=f"{self.steering_angle:.0f}%")
        
        # Reset sliders
        self.speed_slider.set(0)
        self.steering_slider.set(0)
        
        # Send emergency stop command
        self.send_command("V 0")
        
        # Also activate the brakes
        self.toggle_frein_a_main(activate=True)

    def toggle_frein_a_main(self, activate=None):
        """Toggle or set the brake state"""
        # Check if control is enabled before proceeding
        if not self.main_window.control_enabled:
            print("Control is disabled. Enable control first.")
            return

        # If activate is specified, use that value, otherwise toggle
        if activate is not None:
            self.frein_a_main_active = activate
        else:
            self.frein_a_main_active = not self.frein_a_main_active

        if self.frein_a_main_active:
            self.frein_a_main_button.config(text="Deactivate Frein")
            print("Frein à main activated! Sending 'F' command.")
            self.send_command('F')
        else:
            self.frein_a_main_button.config(text="Activate Frein")
            print("Frein à main deactivated! Sending 'Z' command.")
            self.send_command('Z')