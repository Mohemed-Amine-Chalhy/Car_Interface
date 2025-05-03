import sys
import time
import threading
import math
import tkinter as tk
import serial
from tkinter import font as tkfont, ttk, messagebox
from collections import deque
from rplidar import RPLidar
import queue

# Default Configuration
DEFAULT_PORT_NAME = 'COM6'  # Default LIDAR port
DEFAULT_ARDUINO_PORT = 'COM3'  # Default Arduino port
SERIAL_BAUD_RATE = 9600  # Default baud rate for Arduino
VEHICLE_WIDTH = 42  # Vehicle width in cm
OBSTACLE_THRESHOLD = 200  # Max distance to consider an obstacle (2m = 200cm)
STOP_ASSIST_THRESHOLD = 50  # Distance in cm to trigger stop assist
VIEW_ANGLE = 90  # 180° angle centered around 90° (0° to 180°)
WINDOW_SIZE = (900, 800)  # Increased size for new controls
MAX_DISTANCE = 2000  # Maximum distance to display (in cm)
HISTORY_SIZE = 5  # Number of frames to preserve for stability

# Colors
WHITE = "#FFFFFF"
BLACK = "#000000"
RED = "#FF0000"
GREEN = "#00FF00"
BLUE = "#0000FF"
YELLOW = "#FFFF00"
DARK_GRAY = "#1E1E1E"
LIGHT_GRAY = "#646464"
ORANGE = "#FFA500"

class SerialCommunicator:
    def __init__(self, port=DEFAULT_ARDUINO_PORT, baud_rate=SERIAL_BAUD_RATE):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = None
        self.connected = False
        self.f_sent = False
        self.z_sent = False
        self.command_queue = queue.Queue()
        self.serial_thread = None
        self.running = False
        
        # Simulation properties
        self.simulated_motor_state = "STOPPED"
        self.simulated_speed = 0
        self.simulated_direction = "STOP"
        self.simulated_steering_angle = 0
        self.simulated_mode = "MANUAL"  # or "AUTO"
        
    def connect(self):
        try:
            # Simulate connection
            self.connected = True
            self.running = True
            print(f"[SIMULATED] Connected to Arduino on {self.port}")
            
            # Start the command processing thread
            self.serial_thread = threading.Thread(target=self._process_command_queue)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            return True
        except Exception as e:
            print(f"[SIMULATED] Error connecting to Arduino on {self.port}: {e}")
            return False
            
    def disconnect(self):
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=1.0)
        self.connected = False
        print(f"[SIMULATED] Disconnected from Arduino on {self.port}")
            
    def _process_command_queue(self):
        """Process commands from the queue in a separate thread"""
        while self.running:
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get(block=False)
                    self._simulate_arduino_response(command)
                    self.command_queue.task_done()
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"[SIMULATED] Error in command processing thread: {e}")
    
    def _simulate_arduino_response(self, command):
        """Simulate how Arduino would respond to commands"""
        print(f"[SIMULATED] Received command: {command.strip()}")
        
        # Process the command
        if command == "F\n":
            self.f_sent = True
            self.z_sent = False
            self.simulated_motor_state = "RUNNING"
            print("[SIMULATED] Arduino: Motor started (F command)")
            
        elif command == "Z\n":
            self.z_sent = True
            self.f_sent = False
            self.simulated_motor_state = "STOPPED"
            print("[SIMULATED] Arduino: Motor stopped (Z command)")
            
        elif command.startswith("V "):
            try:
                speed = int(command[2:].strip())
                self.simulated_speed = speed
                print(f"[SIMULATED] Arduino: Speed set to {speed} (V command)")
            except ValueError:
                print("[SIMULATED] Arduino: Invalid speed value")
                
        elif command.startswith("O "):
            try:
                angle = int(command[2:].strip())
                self.simulated_steering_angle = angle
                print(f"[SIMULATED] Arduino: Steering angle set to {angle}° (O command)")
            except ValueError:
                print("[SIMULATED] Arduino: Invalid angle value")
                
        elif command.startswith("D "):
            direction = command[2:].strip()
            if direction in ["F", "R"]:
                self.simulated_direction = "FORWARD" if direction == "F" else "REVERSE"
                print(f"[SIMULATED] Arduino: Direction set to {self.simulated_direction} (D command)")
            else:
                print("[SIMULATED] Arduino: Invalid direction")
                
        elif command == "A\n":
            self.simulated_mode = "AUTO"
            print("[SIMULATED] Arduino: Switched to AUTOMATIC mode (A command)")
            
        elif command == "M\n":
            self.simulated_mode = "MANUAL"
            print("[SIMULATED] Arduino: Switched to MANUAL mode (M command)")
            
        else:
            print("[SIMULATED] Arduino: Unknown command")
    
    def send_command(self, command):
        """Queue command to be sent asynchronously"""
        if not self.connected:
            print("[SIMULATED] Not connected to Arduino")
            return False
        
        self.command_queue.put(command)
        return True

    def get_simulated_state(self):
        """Return the current simulated state of the Arduino"""
        return {
            "motor_state": self.simulated_motor_state,
            "speed": self.simulated_speed,
            "direction": self.simulated_direction,
            "steering_angle": self.simulated_steering_angle,
            "mode": self.simulated_mode,
            "f_sent": self.f_sent,
            "z_sent": self.z_sent
        }
# LidarProcessor class remains unchanged
class LidarProcessor:
    def __init__(self, port_name=DEFAULT_PORT_NAME):
        self.port_name = port_name
        self.lidar = None
        self.running = False
        self.scan_data = {}
        self.obstacle_points = []
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.history = deque(maxlen=HISTORY_SIZE)

        # Calculate vehicle passage zone
        self.angle_width = math.degrees(math.atan((VEHICLE_WIDTH/2) / 100))  # Half-angle in degrees

        # Mutex to protect data access
        self.data_lock = threading.Lock()

    def connect(self):
        try:
            self.lidar = RPLidar(self.port_name)
            info = self.lidar.get_info()
            print(f"LIDAR connected on {self.port_name}: {info}")
            health = self.lidar.get_health()
            print(f"LIDAR status: {health}")
            return True
        except Exception as e:
            print(f"Error connecting to LIDAR on {self.port_name}: {e}")
            return False

    def start(self):
        if not self.running:
            if self.connect():
                self.running = True
                self.scan_thread = threading.Thread(target=self._scan_thread)
                self.scan_thread.daemon = True
                self.scan_thread.start()
                return True
        return False

    def stop(self):
        self.running = False
        if hasattr(self, 'scan_thread') and self.scan_thread:
            self.scan_thread.join(timeout=1.0)
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print(f"LIDAR disconnected from {self.port_name}")
            except Exception as e:
                print(f"Error stopping LIDAR: {e}")

    def _scan_thread(self):
        try:
            if self.lidar:
                self.lidar.start_motor()
                time.sleep(1)  # Wait for motor to reach speed

                for scan in self.lidar.iter_scans():
                    if not self.running:
                        break

                    # Process scan data
                    scan_data = {}
                    for _, angle, distance in scan:
                        # Convert to cm
                        distance = distance / 10

                        # Shift angle so that 0° is the front of the vehicle
                        adjusted_angle = angle - 90
                        # Normalize angle between -180 and 180
                        if adjusted_angle > 180:
                            adjusted_angle -= 360
                        elif adjusted_angle < -180:
                            adjusted_angle += 360

                        scan_data[adjusted_angle] = distance

                    # Identify obstacles
                    obstacle_points = []
                    min_distance = float('inf')
                    min_angle = 0

                    for angle, distance in scan_data.items():
                        # Check if point is in area of interest (in front of vehicle)
                        if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2:
                            # Check if distance is less than threshold
                            if distance <= OBSTACLE_THRESHOLD:
                                # Calculate lateral position relative to central axis
                                lateral_position = distance * math.tan(math.radians(angle))

                                # Check if obstacle is in vehicle path
                                if abs(lateral_position) <= (VEHICLE_WIDTH / 2):
                                    obstacle_points.append((angle, distance, True))  # In path

                                    # Update minimum distance
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_angle = angle
                                else:
                                    obstacle_points.append((angle, distance, False))  # Out of path

                    # Add to history for stability
                    with self.data_lock:
                        self.scan_data = scan_data
                        self.obstacle_points = obstacle_points
                        if min_distance < float('inf'):
                            self.closest_distance = min_distance
                            self.closest_angle = min_angle
                        self.history.append((scan_data, obstacle_points, min_distance, min_angle))

        except Exception as e:
            print(f"Error in scan thread: {e}")
        finally:
            if self.running:
                self.stop()

    def get_stable_data(self):
        """Returns stabilized data using history"""
        with self.data_lock:
            if not self.history:
                return {}, [], float('inf'), 0

            # Average of latest measurements for closest distance
            closest_distances = [h[2] for h in self.history if h[2] < float('inf')]
            if closest_distances:
                avg_closest = sum(closest_distances) / len(closest_distances)
            else:
                avg_closest = float('inf')

            # Use most recent data for display
            return self.scan_data, self.obstacle_points, avg_closest, self.closest_angle

class LidarVisualizer:
    def __init__(self, master, lidar_processor, serial_communicator):
        self.master = master
        self.lidar_processor = lidar_processor
        self.serial_communicator = serial_communicator
        self.running = False
        self.lidar_assist_enabled = tk.BooleanVar(value=True)  # Default to enabled
        self.stop_assist_active = tk.BooleanVar(value=False)  # Stop assist functionality
        self.stop_assist_triggered = False  # Flag to track if stop assist has been triggered
        self.auto_mode = tk.BooleanVar(value=False)  # Default to manual mode
        
        # Add Arduino Simulation Panel
        self.create_simulation_panel()

        master.title("RPLidar - Robot Control Interface")
        master.geometry(f"{WINDOW_SIZE[0]}x{WINDOW_SIZE[1]}")
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Create a main content frame
        self.main_content = ttk.Frame(master)
        self.main_content.pack(fill=tk.BOTH, expand=True)
        
        # Left panel for LIDAR and controls
        self.left_panel = ttk.Frame(self.main_content)
        self.left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Right panel for robot controls
        self.right_panel = ttk.Frame(self.main_content)
        self.right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5, ipadx=10)

        # UI Controls Frame in left panel
        self.controls_frame = ttk.LabelFrame(self.left_panel, text="LIDAR Control")
        self.controls_frame.pack(pady=10, padx=10, fill=tk.X)

        # LIDAR Port Controls
        ttk.Label(self.controls_frame, text="LIDAR Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_entry = ttk.Entry(self.controls_frame, width=15)
        self.port_entry.insert(0, DEFAULT_PORT_NAME)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # Arduino Port Controls
        ttk.Label(self.controls_frame, text="Arduino Port:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.arduino_port_entry = ttk.Entry(self.controls_frame, width=15)
        self.arduino_port_entry.insert(0, DEFAULT_ARDUINO_PORT)
        self.arduino_port_entry.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)

        # LIDAR Activation Buttons
        self.activate_button = ttk.Button(self.controls_frame, text="Activate LIDAR", command=self.start_lidar)
        self.activate_button.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W+tk.E)

        self.deactivate_button = ttk.Button(self.controls_frame, text="Deactivate LIDAR", command=self.stop_lidar, state=tk.DISABLED)
        self.deactivate_button.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W+tk.E)

        # Arduino Connection Buttons
        self.connect_arduino_button = ttk.Button(self.controls_frame, text="Connect Arduino", command=self.connect_arduino)
        self.connect_arduino_button.grid(row=1, column=2, padx=5, pady=5, sticky=tk.W+tk.E)

        self.disconnect_arduino_button = ttk.Button(self.controls_frame, text="Disconnect Arduino", command=self.disconnect_arduino, state=tk.DISABLED)
        self.disconnect_arduino_button.grid(row=1, column=3, padx=5, pady=5, sticky=tk.W+tk.E)

        # Add a new frame for the command controls
        self.command_frame = ttk.LabelFrame(self.left_panel, text="Command Controls")
        self.command_frame.pack(pady=10, padx=10, fill=tk.X)

        # Add Stop Assist checkbox
        self.stop_assist_checkbox = ttk.Checkbutton(
            self.command_frame, 
            text="Stop Assist (Auto-stop when obstacle < 50cm)", 
            variable=self.stop_assist_active,
            state=tk.DISABLED
        )
        self.stop_assist_checkbox.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # Add F Command Button
        self.f_command_button = ttk.Button(
            self.command_frame, 
            text="Send F Command", 
            command=self.send_f_command,
            state=tk.DISABLED
        )
        self.f_command_button.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W+tk.E)

        # Add Z Command Button
        self.z_command_button = ttk.Button(
            self.command_frame, 
            text="Send Z Command", 
            command=self.send_z_command,
            state=tk.DISABLED
        )
        self.z_command_button.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W+tk.E)

        # Lidar Assist Checkbox
        self.assist_checkbox = ttk.Checkbutton(
            self.controls_frame, 
            text="Lidar Assist", 
            variable=self.lidar_assist_enabled
        )
        self.assist_checkbox.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky=tk.W)

        # Canvas for Lidar Display
        self.canvas = tk.Canvas(self.left_panel, width=WINDOW_SIZE[0]-300, height=WINDOW_SIZE[1]-300, bg=BLACK)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # ----- NEW ROBOT CONTROL INTERFACE -----
        # Create robot control frame in right panel
        self.robot_control_frame = ttk.LabelFrame(self.right_panel, text="Robot Controls")
        self.robot_control_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Mode toggle button
        self.mode_frame = ttk.Frame(self.robot_control_frame)
        self.mode_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(self.mode_frame, text="Driving Mode:").pack(side=tk.LEFT, padx=5)
        
        self.mode_indicator = ttk.Label(self.mode_frame, text="MANUAL", foreground="blue", font=("Arial", 12, "bold"))
        self.mode_indicator.pack(side=tk.LEFT, padx=5)
        
        self.mode_toggle_button = ttk.Button(
            self.mode_frame,
            text="Toggle Mode",
            command=self.toggle_mode,
            state=tk.DISABLED
        )
        self.mode_toggle_button.pack(side=tk.RIGHT, padx=5)
        
        # Separator
        ttk.Separator(self.robot_control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=10, pady=5)
        
        # Steering control (horizontal slider)
        self.steering_frame = ttk.Frame(self.robot_control_frame)
        self.steering_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(self.steering_frame, text="Steering Control:").pack(anchor=tk.W)
        
        self.steering_value = tk.IntVar(value=0)
        self.steering_slider = ttk.Scale(
            self.steering_frame,
            from_=-90,
            to=90,
            orient=tk.HORIZONTAL,
            variable=self.steering_value,
            command=self.on_steering_change,
            length=250,
            state=tk.DISABLED
        )
        self.steering_slider.pack(fill=tk.X, pady=5)
        
        self.steering_label = ttk.Label(self.steering_frame, text="Angle: 0°")
        self.steering_label.pack(anchor=tk.E)
        
        # Separator
        ttk.Separator(self.robot_control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=10, pady=5)
        
        # Direction control buttons
        self.direction_frame = ttk.Frame(self.robot_control_frame)
        self.direction_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Label(self.direction_frame, text="Direction Control:").pack(anchor=tk.W)
        
        self.direction_buttons_frame = ttk.Frame(self.direction_frame)
        self.direction_buttons_frame.pack(fill=tk.X, pady=5)
        
        self.forward_button = ttk.Button(
            self.direction_buttons_frame,
            text="Forward",
            command=lambda: self.send_direction("F"),
            state=tk.DISABLED
        )
        self.forward_button.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        self.reverse_button = ttk.Button(
            self.direction_buttons_frame,
            text="Reverse",
            command=lambda: self.send_direction("R"),
            state=tk.DISABLED
        )
        self.reverse_button.pack(side=tk.RIGHT, padx=5, expand=True, fill=tk.X)
        
        # Separator
        ttk.Separator(self.robot_control_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=10, pady=5)
        
        # Acceleration control (vertical slider)
        self.accel_frame = ttk.LabelFrame(self.robot_control_frame, text="Acceleration")
        self.accel_frame.pack(fill=tk.BOTH, padx=10, pady=10, expand=True)
        
        self.accel_value = tk.IntVar(value=0)
        self.accel_slider = ttk.Scale(
            self.accel_frame,
            from_=100,  # Top of slider is max value
            to=0,       # Bottom of slider is min value
            orient=tk.VERTICAL,
            variable=self.accel_value,
            command=self.on_acceleration_change,
            length=200,
            state=tk.DISABLED
        )
        self.accel_slider.pack(side=tk.LEFT, padx=20, expand=True)
        
        self.accel_label = ttk.Label(self.accel_frame, text="Speed: 0")
        self.accel_label.pack(side=tk.LEFT, padx=5)
        
        # Emergency stop button
        self.emergency_stop_button = ttk.Button(
            self.robot_control_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            style="Emergency.TButton",
            state=tk.DISABLED
        )
        self.emergency_stop_button.pack(fill=tk.X, padx=10, pady=10, ipady=10)
        
        # Create a custom style for the emergency button
        style = ttk.Style()
        style.configure("Emergency.TButton", background="red", foreground="white", font=("Arial", 12, "bold"))

        # Status Bar
        self.status_frame = ttk.Frame(master)
        self.status_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.status_label = ttk.Label(self.status_frame, text="Status: Ready")
        self.status_label.pack(side=tk.LEFT, padx=5, pady=2)

        # Define fonts
        self.font = tkfont.Font(family="Arial", size=14, weight="bold")
        self.small_font = tkfont.Font(family="Arial", size=10)

        # Center of screen (vehicle position)
        self.center_x = (WINDOW_SIZE[0] - 300) // 2
        self.center_y = (WINDOW_SIZE[1] - 300) - 50

        # Scale factor for display
        self.scale = min(WINDOW_SIZE[0] - 300, WINDOW_SIZE[1] - 300) / (MAX_DISTANCE * 1.5)

        # Text elements
        self.distance_text = self.canvas.create_text(10, 10, anchor=tk.NW,
                                                    text="Distance: -- m",
                                                    fill=WHITE, font=self.font)
        self.warning_text = self.canvas.create_text(10, 50, anchor=tk.NW,
                                                   text="", fill=RED, font=self.font)
        self.vehicle_info = self.canvas.create_text(10, (WINDOW_SIZE[1] - 300) - 30, anchor=tk.NW,
                                                   text=f"Width: {VEHICLE_WIDTH} cm",
                                                   fill=WHITE, font=self.small_font)

        # Create update loop
        self.update_id = None
        

    
    # ... [rest of initialization]

def create_simulation_panel(self):
    """Create a panel to display the simulated Arduino state"""
    self.simulation_frame = ttk.LabelFrame(self.right_panel, text="Arduino Simulation")
    self.simulation_frame.pack(fill=tk.X, padx=10, pady=10)
    
    # Motor State
    ttk.Label(self.simulation_frame, text="Motor State:").grid(row=0, column=0, padx=5, sticky=tk.W)
    self.motor_state_label = ttk.Label(self.simulation_frame, text="STOPPED", foreground=RED)
    self.motor_state_label.grid(row=0, column=1, padx=5, sticky=tk.W)
    
    # Current Speed
    ttk.Label(self.simulation_frame, text="Current Speed:").grid(row=1, column=0, padx=5, sticky=tk.W)
    self.speed_label = ttk.Label(self.simulation_frame, text="0")
    self.speed_label.grid(row=1, column=1, padx=5, sticky=tk.W)
    
    # Direction
    ttk.Label(self.simulation_frame, text="Direction:").grid(row=2, column=0, padx=5, sticky=tk.W)
    self.direction_label = ttk.Label(self.simulation_frame, text="STOP")
    self.direction_label.grid(row=2, column=1, padx=5, sticky=tk.W)
    
    # Steering Angle
    ttk.Label(self.simulation_frame, text="Steering Angle:").grid(row=3, column=0, padx=5, sticky=tk.W)
    self.steering_angle_label = ttk.Label(self.simulation_frame, text="0°")
    self.steering_angle_label.grid(row=3, column=1, padx=5, sticky=tk.W)
    
    # Mode
    ttk.Label(self.simulation_frame, text="Mode:").grid(row=4, column=0, padx=5, sticky=tk.W)
    self.mode_label = ttk.Label(self.simulation_frame, text="MANUAL", foreground="blue")
    self.mode_label.grid(row=4, column=1, padx=5, sticky=tk.W)
    
    # Last Command
    ttk.Label(self.simulation_frame, text="Last Command:").grid(row=5, column=0, padx=5, sticky=tk.W)
    self.last_command_label = ttk.Label(self.simulation_frame, text="None")
    self.last_command_label.grid(row=5, column=1, padx=5, sticky=tk.W)

def update_simulation_panel(self):
    """Update the simulation panel with current Arduino state"""
    state = self.serial_communicator.get_simulated_state()
    
    # Update motor state
    self.motor_state_label.config(
        text=state["motor_state"],
        foreground=GREEN if state["motor_state"] == "RUNNING" else RED
    )
    
    # Update speed
    self.speed_label.config(text=str(state["speed"]))
    
    # Update direction
    self.direction_label.config(text=state["direction"])
    
    # Update steering angle
    self.steering_angle_label.config(text=f"{state['steering_angle']}°")
    
    # Update mode
    self.mode_label.config(
        text=state["mode"],
        foreground="green" if state["mode"] == "AUTO" else "blue"
    )

    def toggle_mode(self):
        """Toggle between automatic and manual driving modes"""
        new_mode = not self.auto_mode.get()
        self.auto_mode.set(new_mode)
        
        if new_mode:  # Switching to automatic mode
            self.serial_communicator.send_command("A\n")
            self.serial_communicator.send_command("V 0\n")
            self.update_status("Switched to AUTOMATIC mode")
            self.mode_indicator.config(text="AUTOMATIC", foreground="green")
        else:  # Switching to manual mode
            self.serial_communicator.send_command("M\n")
            self.update_status("Switched to MANUAL mode")
            self.mode_indicator.config(text="MANUAL", foreground="blue")
        
        # Update UI controls based on the new mode
        self.update_controls_state()

    def update_controls_state(self):
        """Update the enabled/disabled state of controls based on the current mode"""
        state = tk.NORMAL if self.auto_mode.get() else tk.DISABLED
        
        # Update control states
        self.steering_slider.config(state=state)
        self.accel_slider.config(state=state)
        self.forward_button.config(state=state)
        self.reverse_button.config(state=state)
        self.emergency_stop_button.config(state=state)
        
        # F command button is enabled only in automatic mode and if not sent already
        if self.auto_mode.get() and not self.serial_communicator.f_sent:
            self.f_command_button.config(state=tk.NORMAL)
        else:
            self.f_command_button.config(state=tk.DISABLED)
        
        # Z command button is enabled only in automatic mode and if F has been sent
        if self.auto_mode.get() and self.serial_communicator.f_sent:
            self.z_command_button.config(state=tk.NORMAL)
        else:
            self.z_command_button.config(state=tk.DISABLED)

    def on_steering_change(self, value):
        """Handle steering slider changes"""
        angle = int(float(value))
        self.steering_label.config(text=f"Angle: {angle}°")
        
        # Only send command if not actively dragging (when released)
        if not self.steering_slider.state == ['pressed']:
            self.master.after(100, lambda: self.send_steering_command(angle))

    def send_steering_command(self, angle):
        """Send steering command with throttling"""
        if self.serial_communicator.connected:
            self.serial_communicator.send_command(f"O {angle}\n")

    def on_acceleration_change(self, value):
        """Handle acceleration slider changes"""
        speed = int(float(value))
        self.accel_label.config(text=f"Speed: {speed}")
        
        # Only send command if not actively dragging (when released)
        if not self.accel_slider.state == ['pressed']:
            self.master.after(100, lambda: self.send_acceleration_command(speed))

    def send_acceleration_command(self, speed):
        """Send acceleration command with throttling"""
        if self.serial_communicator.connected:
            self.serial_communicator.send_command(f"V {speed}\n")

    def send_direction(self, direction):
        """Send direction command (F or R)"""
        if self.serial_communicator.connected:
            self.serial_communicator.send_command(f"D {direction}\n")
            self.update_status(f"Direction set to: {'Forward' if direction == 'F' else 'Reverse'}")

    def emergency_stop(self):
        """Handle emergency stop"""
        if self.serial_communicator.connected:
            # Send emergency stop command
            self.serial_communicator.send_command("V 0\n")
            self.update_status("EMERGENCY STOP activated")
            
            # Reset acceleration slider
            self.accel_value.set(0)
            self.accel_label.config(text="Speed: 0")
            
            # Toggle F command state
            if not self.serial_communicator.f_sent:
                self.send_f_command()
            else:
                self.send_z_command()

    def connect_arduino(self):
        port = self.arduino_port_entry.get()
        if port:
            self.serial_communicator.port = port
            if self.serial_communicator.connect():
                self.connect_arduino_button.config(state=tk.DISABLED)
                self.disconnect_arduino_button.config(state=tk.NORMAL)
                self.stop_assist_checkbox.config(state=tk.NORMAL)
                self.mode_toggle_button.config(state=tk.NORMAL)
                self.update_controls_state()  # Update other controls based on mode
                self.update_status(f"Arduino connected on {port}")
            else:
                messagebox.showerror("Error", f"Could not connect to Arduino on port {port}")
        else:
            messagebox.showerror("Error", "Please enter an Arduino port.")

    def disconnect_arduino(self):
        self.serial_communicator.disconnect()
        self.connect_arduino_button.config(state=tk.NORMAL)
        self.disconnect_arduino_button.config(state=tk.DISABLED)
        self.stop_assist_checkbox.config(state=tk.DISABLED)
        self.mode_toggle_button.config(state=tk.DISABLED)
        
        # Disable all control buttons
        self.f_command_button.config(state=tk.DISABLED)
        self.z_command_button.config(state=tk.DISABLED)
        self.steering_slider.config(state=tk.DISABLED)
        self.accel_slider.config(state=tk.DISABLED)
        self.forward_button.config(state=tk.DISABLED)
        self.reverse_button.config(state=tk.DISABLED)
        self.emergency_stop_button.config(state=tk.DISABLED)
        
        self.update_status("Arduino disconnected")

    def trigger_stop_assist(self):
        """Function to send stop assist commands when obstacle is detected"""
        if not self.stop_assist_triggered and self.serial_communicator.connected:
            # Send 'V 0\n' command
            self.serial_communicator.send_command("V 0\n")
            
            # Reset acceleration slider
            self.accel_value.set(0)
            self.accel_label.config(text="Speed: 0")
            
            # Send 'F\n' only if it hasn't been sent before
            if not self.serial_communicator.f_sent:
                self.serial_communicator.send_command("F\n")
                self.update_command_buttons()
                self.update_status("STOP ASSIST TRIGGERED: V 0 and F commands sent")
                self.stop_assist_triggered = True

    def send_f_command(self):
        if self.serial_communicator.connected and not self.serial_communicator.f_sent:
            self.serial_communicator.send_command("F\n")
            self.update_command_buttons()
            self.update_status("F command sent")

    def send_z_command(self):
        if self.serial_communicator.connected and self.serial_communicator.f_sent and not self.serial_communicator.z_sent:
            self.serial_communicator.send_command("Z\n")
            self.update_command_buttons()
            self.update_status("Z command sent")
            self.stop_assist_triggered = False  # Reset the trigger state

    def update_command_buttons(self):
        # Update button states based on the current state of F and Z commands
        # and considering the auto_mode state
        if self.auto_mode.get():
            if self.serial_communicator.f_sent:
                self.f_command_button.config(state=tk.DISABLED)
                self.z_command_button.config(state=tk.NORMAL)
            elif self.serial_communicator.z_sent:
                self.f_command_button.config(state=tk.NORMAL)
                self.z_command_button.config(state=tk.DISABLED)
            else:
                self.f_command_button.config(state=tk.NORMAL)
                self.z_command_button.config(state=tk.DISABLED)
        else:
            # In manual mode, both buttons are disabled
            self.f_command_button.config(state=tk.DISABLED)
            self.z_command_button.config(state=tk.DISABLED)

    def update_status(self, message):
        self.status_label.config(text=f"Status: {message}")

    def start_lidar(self):
        port = self.port_entry.get()
        if port:
            self.lidar_processor.port_name = port
            if self.lidar_processor.start():
                self.running = True
                self.activate_button.config(state=tk.DISABLED)
                self.deactivate_button.config(state=tk.NORMAL)
                self.update_display()
                self.update_status(f"LIDAR activated on {port}")
            else:
                messagebox.showerror("Error", f"Could not connect to LIDAR on port {port}")
        else:
            messagebox.showerror("Error", "Please enter a LIDAR port.")

    def stop_lidar(self):
        if self.running:
            self.running = False
            self.lidar_processor.stop()
            self.activate_button.config(state=tk.NORMAL)
            self.deactivate_button.config(state=tk.DISABLED)
            if self.update_id:
                self.master.after_cancel(self.update_id)
            self.canvas.delete("scan", "obstacle", "closest")
            self.canvas.itemconfig(self.distance_text, text="Distance: -- cm")
            self.canvas.itemconfig(self.warning_text, text="")
            self.update_status("LIDAR deactivated")

    def on_closing(self):
        self.running = False
        self.lidar_processor.stop()
        self.serial_communicator.disconnect()
        self.master.quit()
        self.master.destroy()

    def update_display(self):
        if not self.running:
            return

        # Get stabilized data
        scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()

        # Check for stop assist condition
        if (self.stop_assist_active.get() and 
            closest_distance < STOP_ASSIST_THRESHOLD and 
            not self.stop_assist_triggered and
            self.auto_mode.get()):  # Only in auto mode
            self.trigger_stop_assist()
        
        # If obstacle is no longer close enough, reset the trigger flag
        if closest_distance > STOP_ASSIST_THRESHOLD + 20:  # Add a buffer to prevent oscillation
            self.stop_assist_triggered = False

        # Clear canvas
        self.canvas.delete("scan", "obstacle", "closest")

        # Draw reference elements
        self.draw_reference_elements()

        # Draw scan points
        for angle, distance in scan_data.items():
            if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2 and distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill=WHITE, outline=WHITE, tags="scan")

        # Draw obstacles based on 'Lidar Assist' checkbox
        if self.lidar_assist_enabled.get():
            for angle, distance, in_path in obstacle_points:
                x, y = self._polar_to_cartesian(angle, distance)
                color = RED if in_path else YELLOW
                self.canvas.create_oval(x-4, y-4, x+4, y+4, fill=color, outline=color, tags="obstacle")

        # Display closest distance
        if closest_distance < float('inf'):
            # Draw line to closest obstacle
            x, y = self._polar_to_cartesian(closest_angle, closest_distance)
            self.canvas.create_line(self.center_x, self.center_y, x, y, fill=GREEN, width=2, tags="closest")

            # Update distance text
            self.canvas.itemconfig(self.distance_text, text=f"Distance: {closest_distance:.2f} cm")

            # Display warning if necessary
            if closest_distance < STOP_ASSIST_THRESHOLD and self.lidar_assist_enabled.get():  # Only warn if assist is on
                self.canvas.itemconfig(self.warning_text, text="BRAKING RECOMMENDED!")
            else:
                self.canvas.itemconfig(self.warning_text, text="")
        else:
            self.canvas.itemconfig(self.distance_text, text="Distance: -- cm")
            self.canvas.itemconfig(self.warning_text, text="")

        # Schedule next update
        self.update_id = self.master.after(33, self.update_display)  # ~30 FPS

    def draw_reference_elements(self):
        # Clear previous reference elements
        self.canvas.delete("reference")

        # Draw distance reference circles
        for r in range(100, MAX_DISTANCE + 1, 100):
            radius = int(r * self.scale)
            self.canvas.create_oval(
                self.center_x - radius, self.center_y - radius,
                self.center_x + radius, self.center_y + radius,
                outline=DARK_GRAY, tags="reference"
            )
            # Distance label
            self.canvas.create_text(
                self.center_x + 5, self.center_y - radius - 10,
                text=f"{r* 0.01} m", fill=LIGHT_GRAY, anchor=tk.W,
                font=self.small_font, tags="reference"
            )

        # Draw reference angles
        half_view = VIEW_ANGLE/2
        for angle in range(int(-half_view), int(half_view) + 1, 30):
            rads = math.radians(angle)
            x = self.center_x + int(MAX_DISTANCE * self.scale * math.sin(rads))
            y = self.center_y - int(MAX_DISTANCE * self.scale * math.cos(rads))
            self.canvas.create_line(
                self.center_x, self.center_y, x, y,
                fill=DARK_GRAY, tags="reference"
            )

            # Angle label with adjusted values to show that 0° is front (90° in LIDAR's reference)
            if angle != 0:
                lidar_angle = angle + 90  # Convert back to LIDAR's original angle
                self.canvas.create_text(
                    x + 5, y + 5, text=f"{lidar_angle}°",
                    fill=LIGHT_GRAY, anchor=tk.NW,
                    font=self.small_font, tags="reference"
                )
            else:
                # Mark front (90° in LIDAR's reference)
                self.canvas.create_text(
                    x, y - 15, text="90° (FRONT)",
                    fill=GREEN, anchor=tk.S,
                    font=self.small_font, tags="reference"
                )

        # Draw vehicle (simple rectangle)
        half_width = int((VEHICLE_WIDTH / 2) * self.scale)
        self.canvas.create_rectangle(
            self.center_x - half_width, self.center_y - 10,
            self.center_x + half_width, self.center_y + 10,
            fill=BLUE, outline=BLUE, tags="reference"
        )

        # Draw vehicle path as two parallel lines
        half_width = int((VEHICLE_WIDTH / 2) * self.scale)

        # Left parallel line
        self.canvas.create_line(
            self.center_x - half_width, self.center_y,
            self.center_x - half_width, self.center_y - int(MAX_DISTANCE * self.scale),
            fill=BLUE, width=1, tags="reference"
        )

        # Right parallel line
        self.canvas.create_line(
            self.center_x + half_width, self.center_y,
            self.center_x + half_width, self.center_y - int(MAX_DISTANCE * self.scale),
            fill=BLUE, width=1, tags="reference"
        )

        # Draw stop assist threshold line
        stop_radius = int(STOP_ASSIST_THRESHOLD * self.scale)
        self.canvas.create_oval(
            self.center_x - stop_radius, self.center_y - stop_radius,
            self.center_x + stop_radius, self.center_y + stop_radius,
            outline=RED, width=2, dash=(5, 5), tags="reference"
        )
        self.canvas.create_text(
            self.center_x + 10, self.center_y - stop_radius - 5,
            text=f"Stop Assist Threshold ({STOP_ASSIST_THRESHOLD} cm)",
            fill=RED, anchor=tk.W, font=self.small_font, tags="reference"
        )

    def _polar_to_cartesian(self, angle, distance):
        """Converts polar coordinates to cartesian coordinates for display"""
        rads = math.radians(angle)
        x = self.center_x + int(distance * self.scale * math.sin(rads))
        y = self.center_y - int(distance * self.scale * math.cos(rads))
        return x, y


def main():
    try:
        # Initialize Serial Communicator
        serial_communicator = SerialCommunicator()
        
        # Initialize LIDAR processor with default port
        lidar_processor = LidarProcessor()

        # Initialize Tkinter root window
        root = tk.Tk()

        # Initialize visualizer, passing the root window
        visualizer = LidarVisualizer(root, lidar_processor, serial_communicator)

        # Start the Tkinter main loop
        root.mainloop()

    except Exception as e:
        print(f"Error in main program: {e}")
    finally:
        # Clean up
        if 'serial_communicator' in locals():
            serial_communicator.disconnect()
        if 'lidar_processor' in locals():
            lidar_processor.stop()
        if 'root' in locals():
            root.destroy()
        sys.exit()
    

if __name__ == "__main__":
    main()