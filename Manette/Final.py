import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, font as tkfont
import threading
import queue
import serial
import serial.tools.list_ports
import time
import pygame  # For Xbox controller support
import sys
import math
from collections import deque
from rplidar import RPLidar

# --- Constants (Consolidated) ---
# Robot Control Defaults
DEFAULT_BAUD_RATE = 115200  # Default for ESP32

# Lidar Defaults
DEFAULT_LIDAR_PORT = 'COM6'  # Default LIDAR port (Adjust as needed)
DEFAULT_ARDUINO_PORT = 'COM3' # Default Arduino Port (Adjust as needed) - Used as initial suggestion
VEHICLE_WIDTH = 42  # Vehicle width in cm
OBSTACLE_THRESHOLD = 500  # Max distance to consider an obstacle (cm)
VIEW_ANGLE = 180  # Lidar view angle (degrees) - Front facing
MAX_DISTANCE = 2000  # Maximum distance to display (in cm)
HISTORY_SIZE = 5  # Number of Lidar frames for stability
AUTO_BRAKE_THRESHOLD = 50 # Distance threshold for auto-braking (in cm)

# Colors
WHITE = "#FFFFFF"
BLACK = "#000000"
RED = "#FF0000"
GREEN = "#00FF00"
BLUE = "#0000FF"
YELLOW = "#FFFF00"
DARK_GRAY = "#1E1E1E"
LIGHT_GRAY = "#646464"

# --- Arduino Communicator (from RobotControlGUI) ---
class ArduinoCommunicator:
   def __init__(self, command_queue, response_queue, log_callback):
       self.serial_conn = None
       self.is_connected = False
       self.port = None
       self.baud_rate = None
       self.command_queue = command_queue
       self.response_queue = response_queue
       self.log_callback = log_callback # Function to log messages to GUI
       self.running = True

       # Start command processing thread
       self.command_thread = threading.Thread(target=self._process_commands, daemon=True)
       self.command_thread.start()

       # Start response reading thread
       self.response_thread = threading.Thread(target=self._read_responses, daemon=True)
       self.response_thread.start()

   def connect(self, port, baud_rate=DEFAULT_BAUD_RATE):
       try:
           self.disconnect() # Ensure previous connection is closed

           self.port = port
           self.baud_rate = baud_rate
           self.serial_conn = serial.Serial(port, baud_rate, timeout=0.5)
           time.sleep(1) # Allow time for ESP32 to reset/initialize
           self.is_connected = True
           self.log_callback(f"Successfully connected to Arduino on {port} at {baud_rate} baud.")
           return True
       except Exception as e:
           self.log_callback(f"Arduino Connection Error on {port}: {e}", is_error=True)
           self.is_connected = False
           self.serial_conn = None
           self.port = None
           self.baud_rate = None
           return False

   def disconnect(self):
       if self.serial_conn and self.is_connected:
           try:
               # Send a stop command before disconnecting if needed
               # self.send_command("V 0") # Example: Stop motors
               # time.sleep(0.1)
               self.serial_conn.close()
           except Exception as e:
               self.log_callback(f"Error during Arduino disconnect: {e}", is_error=True)
           finally:
               self.serial_conn = None
               self.is_connected = False
               self.port = None
               self.baud_rate = None
               self.log_callback("Arduino disconnected.")
       # Clear queues on disconnect? Maybe not, commands might be pending reconnection.
       # while not self.command_queue.empty(): self.command_queue.get_nowait(); self.command_queue.task_done()

   def send_command(self, command):
       if self.is_connected and self.serial_conn:
           # Ensure command ends with newline for Arduino Serial.println() compatibility
           if not command.endswith('\n'):
               command += '\n'
           self.command_queue.put(command)
           # self.log_callback(f"Queued Cmd: {command.strip()}") # Optional: Log queued commands
           return True
       else:
           # self.log_callback("Cannot send command: Arduino not connected.", is_error=True)
           return False

   def _process_commands(self):
       last_command_time = 0
       while self.running:
           try:
               command = self.command_queue.get(timeout=0.1) # Wait briefly for a command
               if self.is_connected and self.serial_conn:
                   current_time = time.time()
                   # Add delay for ESP32 stability if commands are sent too rapidly
                   if current_time - last_command_time < 0.05: # 50ms min interval
                       time.sleep(0.05 - (current_time - last_command_time))

                   try:
                       self.serial_conn.write(command.encode('utf-8'))
                       self.serial_conn.flush()
                       last_command_time = time.time()
                       # Don't log every single command sent here if it's high frequency (like steering)
                       # Log important commands or state changes elsewhere
                       if command.strip() not in ["V 0", "S 1", "Q 1"] and not command.startswith("W ") and not command.startswith("V "):
                            self.log_callback(f"Sent: {command.strip()}")
                       elif command.strip() in ["S 1", "Q 1", "A", "M", "D F", "D R"]: # Log key commands
                            self.log_callback(f"Sent: {command.strip()}")

                   except serial.SerialException as se:
                       self.log_callback(f"Serial Write Error: {se}. Disconnecting.", is_error=True)
                       self.disconnect() # Disconnect on serial error
                   except Exception as e:
                       self.log_callback(f"Error sending command '{command.strip()}': {e}", is_error=True)
               else:
                   # If not connected, maybe re-queue or discard? For now, discard.
                   self.log_callback(f"Discarded command (not connected): {command.strip()}", is_warning=True)

               self.command_queue.task_done()

           except queue.Empty:
               continue # No command in queue, loop again
           except Exception as e:
               # Catch potential errors in the loop itself
               self.log_callback(f"Error in command processing loop: {e}", is_error=True)
               time.sleep(0.1)


   def _read_responses(self):
       buffer = ""
       while self.running:
           if self.is_connected and self.serial_conn:
               try:
                   if self.serial_conn.in_waiting > 0:
                       data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='replace')
                       buffer += data

                       while '\n' in buffer:
                           line, buffer = buffer.split('\n', 1)
                           line = line.strip()
                           if line:
                               # self.log_callback(f"Raw Recv: {line}") # Optional: Log raw lines
                               self.response_queue.put(line)
               except serial.SerialException as se:
                   self.log_callback(f"Serial Read Error: {se}. Disconnecting.", is_error=True)
                   self.disconnect() # Disconnect on serial error
               except Exception as e:
                   # Log error but don't necessarily disconnect unless it's persistent
                   self.log_callback(f"Error reading response: {e}", is_error=True)
                   # Add a small delay to prevent spamming errors if port closed unexpectedly
                   time.sleep(0.5)
           else:
                # If not connected, wait a bit longer before checking again
               time.sleep(0.2)
           time.sleep(0.01) # Small delay when connected to prevent high CPU


   def shutdown(self):
       self.running = False
       self.log_callback("Shutting down Arduino communicator...")
       # Give threads time to exit cleanly
       if self.command_thread.is_alive():
           # Don't wait indefinitely if queue is blocked
           # Clear queue to allow thread to potentially exit faster
           while not self.command_queue.empty():
                try:
                    self.command_queue.get_nowait()
                    self.command_queue.task_done()
                except queue.Empty:
                    break
           self.command_thread.join(timeout=1.0)
       if self.response_thread.is_alive():
           self.response_thread.join(timeout=1.0)
       self.disconnect()
       self.log_callback("Arduino communicator shut down.")

# --- Xbox Controller (from RobotControlGUI) ---
# --- Xbox Controller (Modified for PS5 D-Pad Button Support) ---
# --- Xbox Controller (Modified for PS5 D-Pad Button Support & X Button E-Stop Release) ---
class XboxController:
    def __init__(self, controller_id=0, log_callback=None):
        pygame.init() # Initialize Pygame subsystems
        pygame.joystick.init() # Initialize Joystick subsystem specifically

        self.log_callback = log_callback or (lambda msg, **kwargs: print(msg)) # Basic print logger if none provided
        self.connected = False
        self.controller = None
        self.controller_id = controller_id
        self.controller_name = "None"

        # --- Controller Mapping ---
        # Stick/Trigger Axes (Defaults, potentially overridden in _determine_mappings)
        self.AXIS_RIGHT_STICK_X_XBOX = 0
        self.AXIS_RT_XBOX = 5
        self.AXIS_LT_XBOX = 4
        self.AXIS_RIGHT_STICK_X_PS5 = 0
        self.AXIS_RT_PS5 = 5
        self.AXIS_LT_PS5 = 4

        # D-Pad (Hat for Xbox, Buttons for PS5)
        self.HAT_DPAD_XBOX = 0
        self.BUTTON_DPAD_UP = 11    # Placeholder - Check your controller!
        self.BUTTON_DPAD_DOWN = 12  # Placeholder - Check your controller!
        self.BUTTON_DPAD_LEFT = 13  # Placeholder - Check your controller!
        self.BUTTON_DPAD_RIGHT = 14 # Placeholder - Check your controller!

        # Action Buttons (Common Mappings - *** VERIFY THESE ***)
        self.BUTTON_X = 0 # Usually 0 for Cross(PS)/A(Xbox), or maybe Square(PS)/X(Xbox)? Check debug logs!
                          # PS Cross = 0, Circle = 1, Square = 2, Triangle = 3
                          # Xbox A = 0, B = 1, X = 2, Y = 3
                          # *** MODIFY self.BUTTON_X BASED ON YOUR CONTROLLER AND DESIRED BUTTON ***

        # --- Dynamically Determined Mappings ---
        self.AXIS_RIGHT_STICK_X = self.AXIS_RIGHT_STICK_X_XBOX # Default to Xbox
        self.AXIS_RT = self.AXIS_RT_XBOX
        self.AXIS_LT = self.AXIS_LT_XBOX
        self.USE_HAT_DPAD = True # Assume hat D-Pad initially

        # Status variables
        self.right_stick_x = 0.0
        self.rt_value = -1.0
        self.lt_value = -1.0
        self.hat_values = (0, 0)

        # State tracking
        self.dpad_button_state = {'up': False, 'down': False, 'left': False, 'right': False}
        self.last_dpad_tuple = (0, 0)
        self.lt_pressed_state = False

        self.DEADZONE_STICK = 0.15
        self.DEADZONE_TRIGGER = 0.1

        # Value mapping ranges
        self.STEERING_MIN = 200
        self.STEERING_MAX = 2900
        self.STEERING_CENTER = (self.STEERING_MAX + self.STEERING_MIN) // 2
        self.STEERING_CENTER = 1750
        self.ACCEL_MAX = 100

        # Cache last sent values
        self.last_sent_steering = self.STEERING_CENTER
        self.last_sent_accel = 0

        self.connect()

    # _determine_mappings, connect, disconnect remain the same as before...

    def _determine_mappings(self):
        """Tries to guess mappings based on controller name or capabilities."""
        if not self.controller: return

        name = self.controller.get_name().lower()
        num_hats = self.controller.get_numhats()
        num_buttons = self.controller.get_numbuttons()

        self.log_callback(f"Controller: {name}, Hats: {num_hats}, Buttons: {num_buttons}")

        # Simple heuristic: PS controllers often lack hats or have many buttons
        if "dual" in name or "playstation" in name or num_hats == 0 and num_buttons > 10:
            self.log_callback("Controller detected as potentially PS-like. Using PS5 mappings.")
            self.AXIS_RIGHT_STICK_X = self.AXIS_RIGHT_STICK_X_PS5
            self.AXIS_RT = self.AXIS_RT_PS5
            self.AXIS_LT = self.AXIS_LT_PS5
            self.USE_HAT_DPAD = False # Assume D-Pad is buttons
            self.log_callback(f"Using Button D-Pad (Up:{self.BUTTON_DPAD_UP}, Down:{self.BUTTON_DPAD_DOWN}, Left:{self.BUTTON_DPAD_LEFT}, Right:{self.BUTTON_DPAD_RIGHT})")
            self.log_callback("NOTE: Verify these button numbers!")
            # PS5 button mapping confirmation:
            self.log_callback(f"Using 'X' button ID: {self.BUTTON_X} (PS: Cross=0, Circle=1, Square=2, Tri=3) - PLEASE VERIFY!")
        else:
            self.log_callback("Controller detected as potentially Xbox-like. Using Xbox mappings.")
            self.AXIS_RIGHT_STICK_X = self.AXIS_RIGHT_STICK_X_XBOX
            self.AXIS_RT = self.AXIS_RT_XBOX
            self.AXIS_LT = self.AXIS_LT_XBOX
            self.USE_HAT_DPAD = True
            self.log_callback(f"Using Hat D-Pad (Hat ID: {self.HAT_DPAD_XBOX})")
            # Xbox button mapping confirmation:
            self.log_callback(f"Using 'X' button ID: {self.BUTTON_X} (Xbox: A=0, B=1, X=2, Y=3) - PLEASE VERIFY!")

        # Verify axis numbers
        num_axes = self.controller.get_numaxes()
        if self.AXIS_RIGHT_STICK_X >= num_axes:
            self.log_callback(f"Warning: AXIS_RIGHT_STICK_X ({self.AXIS_RIGHT_STICK_X}) out of range (Num Axes: {num_axes}). Setting to 0.", is_warning=True)
            self.AXIS_RIGHT_STICK_X = 0
        if self.AXIS_RT >= num_axes:
            self.log_callback(f"Warning: AXIS_RT ({self.AXIS_RT}) out of range. Setting to axis {num_axes-1} if possible.", is_warning=True)
            self.AXIS_RT = max(0, num_axes - 1)
        if self.AXIS_LT >= num_axes:
            self.log_callback(f"Warning: AXIS_LT ({self.AXIS_LT}) out of range. Setting to axis {num_axes-2} if possible.", is_warning=True)
            self.AXIS_LT = max(0, num_axes - 2)

    def connect(self):
        joystick_count = pygame.joystick.get_count()
        if joystick_count <= self.controller_id:
            self.log_callback(f"Controller ID {self.controller_id} not found (Count: {joystick_count}).", is_warning=True)
            self.connected = False
            return False
        try:
            if self.controller: self.controller.quit()
            self.controller = pygame.joystick.Joystick(self.controller_id)
            self.controller.init()
            self.controller_name = self.controller.get_name()
            self.connected = True
            self.log_callback(f"Connected to controller: {self.controller_name}")
            self._determine_mappings()
            # Reset states
            self.right_stick_x = self.controller.get_axis(self.AXIS_RIGHT_STICK_X) if self.controller.get_numaxes() > self.AXIS_RIGHT_STICK_X else 0.0
            self.rt_value = self.controller.get_axis(self.AXIS_RT) if self.controller.get_numaxes() > self.AXIS_RT else -1.0
            self.lt_value = self.controller.get_axis(self.AXIS_LT) if self.controller.get_numaxes() > self.AXIS_LT else -1.0
            self.hat_values = self.controller.get_hat(self.HAT_DPAD_XBOX) if self.controller.get_numhats() > self.HAT_DPAD_XBOX else (0,0)
            self.dpad_button_state = {'up': False, 'down': False, 'left': False, 'right': False}
            self.last_dpad_tuple = self.get_hat_values()
            self.lt_pressed_state = self.lt_value > 0.5
            self.last_sent_steering = self.STEERING_CENTER
            self.last_sent_accel = 0
            return True
        except pygame.error as e:
            self.log_callback(f"Pygame error connecting controller: {e}", is_error=True)
            self.connected = False; self.controller = None; return False
        except Exception as e:
            self.log_callback(f"Unexpected error connecting controller: {e}", is_error=True)
            self.connected = False; self.controller = None; return False

    def disconnect(self):
        if self.controller:
            try: self.controller.quit()
            except Exception as e: self.log_callback(f"Error during controller quit: {e}", is_error=True)
        self.connected = False; self.controller = None; self.controller_name = "None"
        self.log_callback("Controller disconnected.")


    def update(self):
        """Pumps Pygame events, reads controller state, handles D-pad and X button."""
        if not self.connected or not self.controller:
             return False, None

        event_occurred = None # Track primary event (LT, DPad, X)
        dpad_input_changed = False

        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.log_callback("Pygame quit event received.")
                    return False, None # Signal app to handle quit

                elif event.type == pygame.JOYDEVICEADDED and event.instance_id == self.controller_id:
                    self.log_callback("Controller (re)detected by event.")
                    if not self.controller.get_init(): self.connect()

                elif event.type == pygame.JOYDEVICEREMOVED and event.instance_id == self.controller_id:
                    self.log_callback("Controller disconnected by event.", is_warning=True)
                    
                    self.disconnect()
                    return False, None

                elif event.type == pygame.JOYBUTTONDOWN:
                    button = event.button
                    # --- Add this Debug Print to find your button numbers ---
                    self.log_callback(f"DEBUG: Button {button} Pressed")
                    # ---
                    pressed = True
                    # Check for 'X' button press
                    if button == self.BUTTON_X:
                        event_occurred = "X_PRESSED" # Assign specific event
                    # Check D-Pad buttons if using button mode
                    elif not self.USE_HAT_DPAD:
                        if button == self.BUTTON_DPAD_UP: self.dpad_button_state['up'] = pressed; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_DOWN: self.dpad_button_state['down'] = pressed; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_LEFT: self.dpad_button_state['left'] = pressed; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_RIGHT: self.dpad_button_state['right'] = pressed; dpad_input_changed = True


                elif event.type == pygame.JOYBUTTONUP:
                    button = event.button
                    # self.log_callback(f"DEBUG: Button {button} Released") # Keep for debugging if needed
                    pressed = False
                    # Only need button state for D-Pad in button mode
                    if not self.USE_HAT_DPAD:
                        if button == self.BUTTON_DPAD_UP: self.dpad_button_state['up'] = pressed; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_DOWN: self.dpad_button_state['down'] = pressed; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_LEFT: self.dpad_button_state['left'] = pressed; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_RIGHT: self.dpad_button_state['right'] = pressed; dpad_input_changed = True


                elif event.type == pygame.JOYHATMOTION:
                     if self.USE_HAT_DPAD and event.hat == self.HAT_DPAD_XBOX:
                          self.hat_values = event.value
                          dpad_input_changed = True

            # --- Read Current States (Axes) AFTER processing events ---
            num_axes = self.controller.get_numaxes()
            self.right_stick_x = self.controller.get_axis(self.AXIS_RIGHT_STICK_X) if num_axes > self.AXIS_RIGHT_STICK_X else 0.0
            self.rt_value = self.controller.get_axis(self.AXIS_RT) if num_axes > self.AXIS_RT else -1.0
            self.lt_value = self.controller.get_axis(self.AXIS_LT) if num_axes > self.AXIS_LT else -1.0

            # --- Determine Combined D-Pad State ---
            current_dpad_tuple = self.get_hat_values() # Calculates based on mode

            # --- Detect Events based on State Changes (Prioritize X > LT > DPad) ---
            # Note: We already set event_occurred = "X_PRESSED" in the event loop if X was pressed.

            # D-Pad Change Event (only if X wasn't pressed)
            if event_occurred != "X_PRESSED" and dpad_input_changed and current_dpad_tuple != self.last_dpad_tuple:
                 if self.last_dpad_tuple != (0,0) or current_dpad_tuple != (0,0):
                     event_occurred = "HAT_CHANGED" # Reuse name
                 self.last_dpad_tuple = current_dpad_tuple

            # LT Press/Release Event (only if X and DPad didn't change/activate)
            lt_currently_pressed = self.lt_value > 0.5
            if event_occurred is None: # Only check LT if no other event detected yet
                if lt_currently_pressed and not self.lt_pressed_state:
                    event_occurred = "LT_PRESSED"
                    self.lt_pressed_state = True
                elif not lt_currently_pressed and self.lt_pressed_state:
                    event_occurred = "LT_RELEASED"
                    self.lt_pressed_state = False
            elif lt_currently_pressed != self.lt_pressed_state:
                # Update LT state even if another event takes precedence this cycle
                 self.lt_pressed_state = lt_currently_pressed


            return True, event_occurred # Success, optional event detected

        except pygame.error as e:
            self.log_callback(f"Pygame error reading controller: {e}. Disconnecting.", is_error=True)
            self.disconnect()
            return False, None
        except Exception as e:
            self.log_callback(f"Unexpected error reading controller: {e}. Disconnecting.", is_error=True)
            self.disconnect()
            return False, None

    # get_hat_values, get_steering_value, get_acceleration_value,
    # has_steering_changed, has_accel_changed, is_connected
    # remain the same as the previous D-Pad fix version...

    def get_hat_values(self):
        """Returns the current D-pad state as a tuple (x, y), compatible with hat output."""
        if self.USE_HAT_DPAD:
            if self.connected and self.controller and self.controller.get_numhats() > self.HAT_DPAD_XBOX:
                return self.controller.get_hat(self.HAT_DPAD_XBOX)
            else: return (0, 0)
        else:
            dpad_x = 0; dpad_y = 0
            if self.dpad_button_state['right']: dpad_x = 1
            elif self.dpad_button_state['left']: dpad_x = -1
            if self.dpad_button_state['up']: dpad_y = 1
            elif self.dpad_button_state['down']: dpad_y = -1
            return (dpad_x, dpad_y)

    def get_steering_value(self):
        """Maps right stick X (-1 to 1) to steering value (e.g., 1000-2000) with deadzone."""
        raw_value = self.right_stick_x
        raw_value = -raw_value

        if abs(raw_value) < self.DEADZONE_STICK: return self.STEERING_CENTER
        if raw_value > 0:
            scaled_value = (raw_value - self.DEADZONE_STICK) / (1 - self.DEADZONE_STICK)
            steering = self.STEERING_CENTER + scaled_value * (self.STEERING_MAX - self.STEERING_CENTER)
        else:
            scaled_value = (raw_value + self.DEADZONE_STICK) / (1 - self.DEADZONE_STICK)
            steering = self.STEERING_CENTER + scaled_value * (self.STEERING_CENTER - self.STEERING_MIN)
        return max(self.STEERING_MIN, min(self.STEERING_MAX, int(steering)))

    def get_acceleration_value(self):
        """Maps RT trigger (-1 to 1) to acceleration (0-50) with deadzone."""
        raw_value = self.rt_value
        normalized = (raw_value + 1) / 2
        if normalized < self.DEADZONE_TRIGGER: return 0
        scaled_value = (normalized - self.DEADZONE_TRIGGER) / (1 - self.DEADZONE_TRIGGER)
        accel = int(scaled_value * self.ACCEL_MAX)
        return max(0, min(self.ACCEL_MAX, accel))

    def has_steering_changed(self):
        current_steering = self.get_steering_value()
        if abs(current_steering - self.last_sent_steering) > 5:
            self.last_sent_steering = current_steering; return True
        return False

    def has_accel_changed(self):
        current_accel = self.get_acceleration_value()
        if current_accel != self.last_sent_accel:
            self.last_sent_accel = current_accel; return True
        return False

    def is_connected(self):
        try:
            return self.connected and self.controller and self.controller.get_init() and pygame.joystick.get_count() > self.controller_id and pygame.joystick.Joystick(self.controller_id).get_instance_id() == self.controller.get_instance_id()
        except pygame.error: return False
# --- Lidar Processor (from LidarVisualizer) ---
class LidarProcessor:
    def __init__(self, port_name=DEFAULT_LIDAR_PORT, log_callback=None):
        self.port_name = port_name
        self.log_callback = log_callback or (lambda msg, **kwargs: print(msg))
        self.lidar = None
        self.running = False
        self.scan_thread = None
        self.history = deque(maxlen=HISTORY_SIZE)
        self.data_lock = threading.Lock()

        # Data accessible via get_stable_data()
        self._scan_data = {}
        self._obstacle_points = []
        self._closest_distance = float('inf')
        self._closest_angle = 0

        # Calculate vehicle passage zone angles relative to front (0 degrees)
        # Using small angle approximation: angle ≈ tan(angle) = (opposite/adjacent)
        # Opposite = VEHICLE_WIDTH / 2, Adjacent = distance. Let's use a reference distance like 100cm.
        # self.half_vehicle_angle_rad = math.atan((VEHICLE_WIDTH / 2) / 100.0)
        # self.half_vehicle_angle_deg = math.degrees(self.half_vehicle_angle_rad)
        # Using a fixed angle might be simpler or use dynamic calculation based on distance below.

    def connect(self):
        if self.lidar and self.running:
             self.log_callback("Lidar already connected and running.", is_warning=True)
             return True # Already connected?

        try:
            self.log_callback(f"Attempting to connect to LIDAR on {self.port_name}...")
            # Ensure any previous instance is properly closed
            if self.lidar:
                try:
                     self.lidar.disconnect()
                except: pass # Ignore errors disconnecting old instance
            self.lidar = RPLidar(self.port_name, timeout=3) # Increase timeout?
            # Test connection by getting info
            info = self.lidar.get_info()
            self.log_callback(f"LIDAR Info: {info}")
            health = self.lidar.get_health()
            self.log_callback(f"LIDAR Health: {health}")
            if health[0] == 'Error':
                 raise Exception(f"Lidar health status error: {health[1]}")
            self.log_callback(f"LIDAR connected successfully on {self.port_name}.")
            return True
        except serial.SerialException as se:
             self.log_callback(f"Serial Error connecting to LIDAR on {self.port_name}: {se}", is_error=True)
             self.lidar = None
             return False
        except Exception as e:
            self.log_callback(f"Error connecting to LIDAR on {self.port_name}: {e}", is_error=True)
            if self.lidar: # Clean up if partial connection occurred
                 try: self.lidar.disconnect()
                 except: pass
            self.lidar = None
            return False

    def start_scan(self):
        if self.running:
            self.log_callback("Lidar scan already running.", is_warning=True)
            return True
        if not self.lidar:
            self.log_callback("Cannot start scan: Lidar not connected.", is_error=True)
            return False

        try:
            self.log_callback("Starting Lidar motor and scan...")
            self.lidar.start_motor()
            time.sleep(2) # Give motor time to spin up
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.scan_thread.start()
            self.log_callback("Lidar scan started.")
            return True
        except Exception as e:
            self.log_callback(f"Error starting Lidar scan: {e}", is_error=True)
            self.running = False
            # Try to stop motor if it started
            if self.lidar:
                 try: self.lidar.stop_motor()
                 except: pass
            return False

    def stop_scan(self):
        if not self.running:
            # self.log_callback("Lidar scan already stopped.") # Can be noisy
            return

        self.running = False
        self.log_callback("Stopping Lidar scan...")
        if self.scan_thread and self.scan_thread.is_alive():
            try:
                self.scan_thread.join(timeout=2.0) # Wait for thread to finish
            except Exception as e:
                 self.log_callback(f"Error joining scan thread: {e}", is_warning=True)

        if self.lidar:
            try:
                self.lidar.stop() # Stop scan iterator
                self.lidar.stop_motor()
                self.log_callback("Lidar motor stopped.")
            except Exception as e:
                self.log_callback(f"Error stopping Lidar motor/scan: {e}", is_warning=True)
        self.scan_thread = None
        # Don't disconnect here, just stop scanning. Disconnect separately.
        self.log_callback("Lidar scan stopped.")

    def disconnect_lidar(self):
        self.stop_scan() # Ensure scan is stopped first
        if self.lidar:
             try:
                 self.lidar.disconnect()
                 self.log_callback(f"LIDAR disconnected from {self.port_name}.")
             except Exception as e:
                  self.log_callback(f"Error disconnecting Lidar: {e}", is_warning=True)
             finally:
                  self.lidar = None


    def _scan_loop(self):
        try:
            # Use iter_scans for potentially cleaner data
            for i, scan in enumerate(self.lidar.iter_scans(max_buf_meas=5000, min_len=5)):
                if not self.running:
                    break # Exit loop if stopped

                current_scan_data = {}
                current_obstacle_points = []
                min_dist_in_scan = float('inf')
                min_angle_in_scan = 0

                # --- Process one full 360 scan ---
                for quality, angle, distance in scan:
                    # Filter low quality points? (e.g., if quality > 10)
                    # if quality < 10: continue

                    dist_cm = distance / 10.0 # Convert mm to cm

                    # Adjust angle: 0 degrees = Lidar front. We want 0 degrees = Robot front.
                    # Assuming Lidar front (0 deg) points to Robot's right (90 deg).
                    # Robot front is at Lidar's 270 degrees.
                    # robot_angle = angle - 270 # Shift by 270
                    # Normalize to -180 to 180
                    # if robot_angle < -180: robot_angle += 360
                    # elif robot_angle > 180: robot_angle -= 360

                    # **** Simpler Adjustment: Assume Lidar's 90 degrees IS the robot's front ****
                    # This matches the original LidarVisualizer assumption
                    robot_angle = angle - 90
                    if robot_angle < -180: robot_angle += 360
                    elif robot_angle > 180: robot_angle -= 360


                    # Store adjusted angle and distance
                    current_scan_data[robot_angle] = dist_cm

                    # --- Obstacle Detection within VIEW_ANGLE and THRESHOLD ---
                    if -VIEW_ANGLE/2 <= robot_angle <= VIEW_ANGLE/2 and 0 < dist_cm <= OBSTACLE_THRESHOLD:
                        # Calculate if the point is within the vehicle's projected path width
                        # Lateral distance = distance * sin(angle) (using robot_angle)
                        lateral_dist = abs(dist_cm * math.sin(math.radians(robot_angle)))

                        in_path = lateral_dist <= (VEHICLE_WIDTH / 2.0)

                        current_obstacle_points.append((robot_angle, dist_cm, in_path))

                        # Track closest obstacle *within the path*
                        if in_path and dist_cm < min_dist_in_scan:
                            min_dist_in_scan = dist_cm
                            min_angle_in_scan = robot_angle

                # --- Update Shared Data Safely ---
                with self.data_lock:
                    self._scan_data = current_scan_data
                    self._obstacle_points = current_obstacle_points
                    # Update closest only if a valid obstacle was found in this scan
                    # Reset if no obstacle found in this scan? Or keep last known? Let's keep last known for stability.
                    if min_dist_in_scan < float('inf'):
                        self._closest_distance = min_dist_in_scan
                        self._closest_angle = min_angle_in_scan
                    # Add current state to history (consider adding only key metrics for memory)
                    self.history.append({
                        'obstacles': current_obstacle_points,
                        'closest_dist': min_dist_in_scan, # Use scan's min for history calc
                        'closest_angle': min_angle_in_scan
                        })

        except serial.SerialException as se:
             self.log_callback(f"Serial Error during Lidar scan: {se}. Stopping scan.", is_error=True)
             self.running = False # Ensure running flag is false so stop_scan works cleanly
             # Don't disconnect here, let the main app handle potential reconnection attempts.
        except Exception as e:
            self.log_callback(f"Error in Lidar scan loop: {e}", is_error=True)
            self.running = False # Stop on unexpected errors
        finally:
            # This block runs even if loop breaks or errors occur
            # Ensure motor is stopped if the thread exits unexpectedly while 'running' was true
             if self.lidar and self.running: # Check if we exited due to error while 'running'
                 self.log_callback("Scan loop terminated unexpectedly. Attempting to stop motor.", is_warning=True)
                 try:
                     self.lidar.stop()
                     self.lidar.stop_motor()
                 except: pass
             self.running = False # Explicitly set running to false
             self.log_callback("Lidar scan loop finished.")


    def get_stable_data(self):
        """Returns the latest scan data, obstacle points, and a stabilized closest distance/angle."""
        with self.data_lock:
            # Use the most recent full scan data for visualization
            latest_scan = self._scan_data
            latest_obstacles = self._obstacle_points

            # Calculate stable closest distance from history
            valid_distances = [h['closest_dist'] for h in self.history if h['closest_dist'] < float('inf')]
            if valid_distances:
                # Use average or median? Average is simpler.
                stable_closest_dist = sum(valid_distances) / len(valid_distances)
                # Find the angle corresponding to the *most recent* minimum distance in history
                # This is tricky, maybe just return the angle from the *absolute latest* scan?
                stable_closest_angle = self._closest_angle # Use the latest angle for simplicity
            else:
                stable_closest_dist = float('inf')
                stable_closest_angle = 0

            return latest_scan, latest_obstacles, stable_closest_dist, stable_closest_angle


# --- Lidar Visualizer Frame (Adapted) ---
class LidarVisualizerFrame(ttk.Frame):
    def __init__(self, master, main_app, lidar_processor, arduino_communicator, log_callback, **kwargs):
        super().__init__(master, **kwargs)
        self.main_app = main_app # Reference to the main application instance
        self.lidar_processor = lidar_processor
        self.arduino_communicator = arduino_communicator
        self.log_callback = log_callback

        # Lidar State
        self.lidar_running = False
        self.update_id = None # For canceling Tkinter's after() loop
        
        # Proximity Alert Settings
        self.proximity_alert_active = False
        self.last_beep_time = 0
        self.proximity_levels = {
            "far": 150,    # cm - first warning
            "medium": 100, # cm - medium warning
            "close": 50    # cm - urgent warning
        }
        self.beep_intervals = {
            "far": 1.0,    # seconds between beeps
            "medium": 0.5, # seconds between beeps
            "close": 0.2   # seconds between beeps
        }

        # UI Elements specific to Lidar Tab
        self._create_widgets()
        self._setup_canvas()

    def _create_widgets(self):
        # Frame for Lidar Controls
        controls_frame = ttk.LabelFrame(self, text="LIDAR Control")
        controls_frame.pack(pady=5, padx=10, fill=tk.X)

        # LIDAR Port Controls
        ttk.Label(controls_frame, text="LIDAR Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar(value=DEFAULT_LIDAR_PORT)
        self.port_entry = ttk.Entry(controls_frame, textvariable=self.port_var, width=15)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        # TODO: Add refresh/scan for Lidar ports? More complex.

        # LIDAR Activation Buttons
        self.activate_button = ttk.Button(controls_frame, text="Activate LIDAR", command=self.start_lidar)
        self.activate_button.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        self.deactivate_button = ttk.Button(controls_frame, text="Deactivate LIDAR", command=self.stop_lidar, state=tk.DISABLED)
        self.deactivate_button.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        # Frame for Lidar Assist / Commands (interacting with Arduino)
        assist_frame = ttk.LabelFrame(self, text="Lidar Assist Commands (Requires Arduino Connection)")
        assist_frame.pack(pady=5, padx=10, fill=tk.X)

        # Stop Assist Checkbox
        self.stop_assist_var = tk.BooleanVar(value=False)
        self.stop_assist_checkbox = ttk.Checkbutton(
            assist_frame,
            text="Enable Auto-Stop Assist",
            variable=self.stop_assist_var,
            command=self.on_stop_assist_changed,
            state=tk.DISABLED # Enabled when Arduino is connected
        )
        self.stop_assist_checkbox.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # Threshold entry for auto-braking
        ttk.Label(assist_frame, text="Brake Threshold (cm):").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.threshold_var = tk.StringVar(value=str(AUTO_BRAKE_THRESHOLD))
        self.threshold_entry = ttk.Entry(assist_frame, textvariable=self.threshold_var, width=6)
        self.threshold_entry.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)
        # TODO: Add validation for threshold entry?

        # Proximity Alert Checkbox
        self.proximity_alert_var = tk.BooleanVar(value=False)
        self.proximity_alert_checkbox = ttk.Checkbutton(
            assist_frame,
            text="Enable Proximity Alert Sound",
            variable=self.proximity_alert_var,
            command=self.on_proximity_alert_changed,
            state=tk.NORMAL  # Always available
        )
        self.proximity_alert_checkbox.grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)

        # Volume slider for proximity alert
        ttk.Label(assist_frame, text="Alert Volume:").grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)
        self.volume_var = tk.DoubleVar(value=50)
        self.volume_slider = ttk.Scale(
            assist_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.volume_var,
            length=100
        )
        self.volume_slider.grid(row=2, column=2, padx=5, pady=5, sticky="ew")

        # Button to manually trigger brake (S 1) via Lidar tab (useful for testing)
        self.manual_brake_button = ttk.Button(
             assist_frame,
             text="Manual Brake (S 1)",
             command=lambda: self.arduino_communicator.send_command("S 1"),
             state=tk.DISABLED
        )
        self.manual_brake_button.grid(row=0, column=2, padx=15, pady=5, sticky="ew")

        # Button to manually release brake (Q 1)
        self.manual_release_button = ttk.Button(
             assist_frame,
             text="Manual Release (Q 1)",
             command=lambda: self.arduino_communicator.send_command("Q 1"),
             state=tk.DISABLED
        )
        self.manual_release_button.grid(row=1, column=2, padx=15, pady=5, sticky="ew")

        # Lidar Canvas Frame
        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(5,10))

        # Canvas for Lidar Display
        # Make canvas size dynamic or fixed? Fixed for now.
        self.canvas = tk.Canvas(canvas_frame, width=600, height=400, bg=BLACK, relief=tk.SUNKEN, borderwidth=1)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    def _setup_canvas(self):
         # Define fonts
        self.font = tkfont.Font(family="Arial", size=12, weight="bold")
        self.small_font = tkfont.Font(family="Arial", size=9)

        # Get current canvas dimensions (might change if window resized)
        self.canvas.update_idletasks() # Ensure dimensions are calculated
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width <= 1 or canvas_height <= 1: # Initial setup might have small size
             canvas_width = 600
             canvas_height = 400

        # Center of screen (vehicle position) - near bottom center
        self.center_x = canvas_width // 2
        self.center_y = canvas_height - 50 # 50 pixels from bottom

        # Scale factor based on canvas size and max distance
        # Fit MAX_DISTANCE vertically or horizontally? Use minimum dimension.
        drawable_height = canvas_height - 60 # Space for vehicle/text at bottom
        drawable_width = canvas_width
        self.scale = min(drawable_width / (MAX_DISTANCE * 2), drawable_height / MAX_DISTANCE) * 0.9 # Use 90% of scale

        # Text elements - Create or update
        if hasattr(self, 'distance_text'):
             self.canvas.coords(self.distance_text, 10, 10)
             self.canvas.itemconfig(self.distance_text, text="Dist: -- cm")
        else:
             self.distance_text = self.canvas.create_text(10, 10, anchor=tk.NW,
                                                        text="Dist: -- cm",
                                                        fill=WHITE, font=self.font)
        if hasattr(self, 'warning_text'):
             self.canvas.coords(self.warning_text, 10, 35)
             self.canvas.itemconfig(self.warning_text, text="", fill=RED)
        else:
             self.warning_text = self.canvas.create_text(10, 35, anchor=tk.NW,
                                                       text="", fill=RED, font=self.font)
        if hasattr(self, 'vehicle_info'):
             self.canvas.coords(self.vehicle_info, self.center_x, canvas_height - 5, anchor=tk.S)
             self.canvas.itemconfig(self.vehicle_info, text=f"Veh Width: {VEHICLE_WIDTH} cm")
        else:
             self.vehicle_info = self.canvas.create_text(self.center_x, canvas_height - 5, anchor=tk.S,
                                                        text=f"Veh Width: {VEHICLE_WIDTH} cm",
                                                        fill=WHITE, font=self.small_font)

        # Initial drawing of reference elements
        self.draw_reference_elements()

    # --- Lidar Control Methods ---
    def start_lidar(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Lidar Error", "Please enter a LIDAR port.")
            return

        # Set port and attempt connection in LidarProcessor
        self.lidar_processor.port_name = port
        if self.lidar_processor.connect():
             # Connection successful, now start scanning
             if self.lidar_processor.start_scan():
                 self.lidar_running = True
                 self.activate_button.config(state=tk.DISABLED)
                 self.deactivate_button.config(state=tk.NORMAL)
                 self.port_entry.config(state=tk.DISABLED)
                 self.start_display_update() # Start canvas updates
                 self.log_callback(f"LIDAR activated on {port}")
             else:
                 messagebox.showerror("Lidar Error", f"Failed to start Lidar scan on {port}.")
                 # Disconnect if connection was made but scan failed
                 self.lidar_processor.disconnect_lidar()
        else:
            messagebox.showerror("Lidar Error", f"Could not connect to LIDAR on port {port}.")

    def stop_lidar(self):
        if not self.lidar_running:
            return

        self.stop_display_update() # Stop canvas updates first
        self.lidar_processor.stop_scan()
        # Optional: Disconnect lidar completely when stopping? Or just stop scan?
        # Let's just stop the scan for now. Disconnect handled on app close or manual disconnect.
        # self.lidar_processor.disconnect_lidar()

        self.lidar_running = False
        self.activate_button.config(state=tk.NORMAL)
        self.deactivate_button.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.NORMAL)

        # Clear canvas elements related to scan data
        self.canvas.delete("scan", "obstacle", "closest")
        self.canvas.itemconfig(self.distance_text, text="Dist: -- cm")
        self.canvas.itemconfig(self.warning_text, text="")
        self.log_callback("LIDAR deactivated.")

    # --- Canvas Update Methods ---
    def start_display_update(self):
        if self.update_id is None:
            self.update_display() # Call immediately once
            self.log_callback("Lidar display update loop started.")

    def stop_display_update(self):
        if self.update_id is not None:
            self.master.after_cancel(self.update_id)
            self.update_id = None
            self.log_callback("Lidar display update loop stopped.")

    def update_display(self):
        if not self.lidar_running:
            self.update_id = None # Ensure loop stops if lidar stops externally
            return

        # Get latest data from processor
        scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()

        # Handle Auto-Stop Assist Logic (triggers E-Stop if needed)
        self.handle_auto_stop_assist(closest_distance)

        # --- Drawing on Canvas ---
        self.canvas.delete("scan", "obstacle", "closest") # Clear dynamic elements

        # ... (Drawing scan points, obstacles, closest line - keep this section) ...
        for angle, distance in scan_data.items():
            # Limit drawing to view angle and max distance for clarity
            if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2 and 0 < distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                # Draw small dots for scan points
                self.canvas.create_oval(x-1, y-1, x+1, y+1, fill=LIGHT_GRAY, outline="", tags="scan")

        # Draw obstacles
        for angle, distance, in_path in obstacle_points:
             if 0 < distance <= MAX_DISTANCE: # Ensure distance is valid
                x, y = self._polar_to_cartesian(angle, distance)
                color = RED if in_path else YELLOW
                size = 4 if in_path else 3
                self.canvas.create_oval(x-size, y-size, x+size, y+size, fill=color, outline="", tags="obstacle")

        # Display closest distance and warning
        if closest_distance < float('inf'):
            # Draw line to closest obstacle *within the path*
            x_close, y_close = self._polar_to_cartesian(closest_angle, closest_distance)
            self.canvas.create_line(self.center_x, self.center_y, x_close, y_close, fill=GREEN, width=2, arrow=tk.LAST, tags="closest")

            self.canvas.itemconfig(self.distance_text, text=f"Dist: {closest_distance:.1f} cm")
            
            # Handle proximity alert sound if enabled
            if self.proximity_alert_active:
                # Determine proximity level based on distance
                if closest_distance <= self.proximity_levels["close"]:
                    self.play_proximity_beep("close")
                elif closest_distance <= self.proximity_levels["medium"]:
                    self.play_proximity_beep("medium")
                elif closest_distance <= self.proximity_levels["far"]:
                    self.play_proximity_beep("far")

            # --- Update warning text based on threshold and assist status ---
            try:
                threshold = float(self.threshold_var.get())
            except ValueError:
                threshold = AUTO_BRAKE_THRESHOLD # Fallback

            if closest_distance < threshold:
                is_braking = self.main_app.brake_active
                brake_reason = self.main_app.brake_reason

                if is_braking and brake_reason == "E-Stop":
                     # E-Stop is active (could be manual or auto)
                     # Check if auto-assist is enabled to refine the message
                     assist_enabled = self.stop_assist_var.get()
                     if assist_enabled:
                          # If E-Stop is active AND assist is ON, it was likely triggered by assist
                          self.canvas.itemconfig(self.warning_text, text="AUTO E-STOP ACTIVE!", fill=RED)
                     else:
                          # E-Stop active, but assist is OFF (must have been manual E-Stop click)
                          self.canvas.itemconfig(self.warning_text, text="MANUAL E-STOP ACTIVE!", fill=RED)
                elif is_braking:
                    # Brake is active for other reasons (Manual button, Controller LT)
                    self.canvas.itemconfig(self.warning_text, text=f"BRAKE ACTIVE ({brake_reason})!", fill=YELLOW) # Show specific reason
                else:
                    # Not braking, but below threshold -> Advise action
                    self.canvas.itemconfig(self.warning_text, text="E-STOP ADVISED!", fill=YELLOW) # Changed from "BRAKE ADVISED"
            else:
                # Distance is greater than threshold, clear warning
                self.canvas.itemconfig(self.warning_text, text="")
        else:
            # No valid closest distance found
            self.canvas.itemconfig(self.distance_text, text="Dist: -- cm")
            self.canvas.itemconfig(self.warning_text, text="")

        # Schedule next update
        self.update_id = self.master.after(100, self.update_display) # Update ~10 FPS

    def draw_reference_elements(self):
        # Clear previous reference elements
        self.canvas.delete("reference")

        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width <= 1 or canvas_height <= 1: return # Avoid drawing if canvas not ready

        # Update center and scale based on current size (important for resize)
        self.center_x = canvas_width // 2
        self.center_y = canvas_height - 50
        drawable_height = canvas_height - 60
        drawable_width = canvas_width
        self.scale = min(drawable_width / (MAX_DISTANCE * 2), drawable_height / MAX_DISTANCE) * 0.9

        # Draw distance rings (e.g., every 50cm up to MAX_DISTANCE)
        for r_cm in range(50, int(MAX_DISTANCE) + 1, 50):
            radius_px = int(r_cm * self.scale)
            if radius_px < 5: continue # Don't draw tiny rings

            # Draw arcs instead of full circles if VIEW_ANGLE < 360
            if VIEW_ANGLE < 359:
                 # Calculate start angle and extent based on robot's forward direction (0 deg)
                 start_angle_tk = 90 - (-VIEW_ANGLE / 2) # Tkinter angles: 0=East, 90=North
                 extent_tk = -VIEW_ANGLE # Negative for clockwise
                 self.canvas.create_arc(
                    self.center_x - radius_px, self.center_y - radius_px,
                    self.center_x + radius_px, self.center_y + radius_px,
                    start=start_angle_tk, extent=extent_tk,
                    outline=DARK_GRAY, style=tk.ARC, tags="reference"
                 )
            else: # Draw full circle
                 self.canvas.create_oval(
                    self.center_x - radius_px, self.center_y - radius_px,
                    self.center_x + radius_px, self.center_y + radius_px,
                    outline=DARK_GRAY, tags="reference"
                 )

            # Distance label (only draw if space permits)
            if radius_px > 15:
                 # Position label slightly above the ring on the center line (y-axis)
                 self.canvas.create_text(self.center_x, self.center_y - radius_px,
                                     text=f"{r_cm}cm", fill=LIGHT_GRAY, anchor=tk.S,
                                     font=self.small_font, tags="reference")

        # Draw angle reference lines (e.g., every 30 degrees within VIEW_ANGLE)
        half_view = VIEW_ANGLE / 2
        for angle in range(int(-half_view), int(half_view) + 1, 30):
             # Calculate end point at max distance for the line
             x_end, y_end = self._polar_to_cartesian(angle, MAX_DISTANCE)
             # Draw line from center
             self.canvas.create_line(self.center_x, self.center_y, x_end, y_end,
                                    fill=DARK_GRAY, dash=(2, 4), tags="reference")
             # Angle label near the end of the line
             x_lbl, y_lbl = self._polar_to_cartesian(angle, MAX_DISTANCE * 1.05) # Slightly outside max dist
             self.canvas.create_text(x_lbl, y_lbl, text=f"{angle}°", fill=LIGHT_GRAY,
                                     font=self.small_font, anchor=tk.CENTER, tags="reference")


        # Draw vehicle representation (simple rectangle)
        vehicle_width_px = int(VEHICLE_WIDTH * self.scale)
        half_width_px = max(2, vehicle_width_px // 2) # Ensure min width of 4px
        vehicle_height_px = max(4, int(20 * self.scale)) # Approx 20cm long vehicle representation
        self.canvas.create_rectangle(
            self.center_x - half_width_px, self.center_y - (vehicle_height_px / 2),
            self.center_x + half_width_px, self.center_y + (vehicle_height_px / 2),
            fill=BLUE, outline=WHITE, tags="reference"
        )
        # Add arrow pointing forward
        self.canvas.create_line(self.center_x, self.center_y, self.center_x, self.center_y - vehicle_height_px * 0.7,
                               fill=WHITE, width=max(1, int(self.scale*2)), arrow=tk.LAST, tags="reference")


        # Draw vehicle path width lines
        path_half_width_px = int((VEHICLE_WIDTH / 2) * self.scale)
        # Calculate end points for path lines
        _, y_end_path = self._polar_to_cartesian(0, MAX_DISTANCE) # Y coordinate at max distance forward

        # Left path line
        self.canvas.create_line(self.center_x - path_half_width_px, self.center_y,
                                self.center_x - path_half_width_px, y_end_path,
                                fill=BLUE, width=1, dash=(4, 4), tags="reference")
        # Right path line
        self.canvas.create_line(self.center_x + path_half_width_px, self.center_y,
                                self.center_x + path_half_width_px, y_end_path,
                                fill=BLUE, width=1, dash=(4, 4), tags="reference")


        # Draw braking threshold arc
        try:
            threshold_cm = float(self.threshold_var.get())
            if 0 < threshold_cm <= MAX_DISTANCE:
                threshold_radius_px = int(threshold_cm * self.scale)

                start_angle_tk = 90 - (-VIEW_ANGLE / 2) # Tkinter angles: 0=East, 90=North
                extent_tk = -VIEW_ANGLE # Negative for clockwise

                self.canvas.create_arc(
                    self.center_x - threshold_radius_px, self.center_y - threshold_radius_px,
                    self.center_x + threshold_radius_px, self.center_y + threshold_radius_px,
                    start=start_angle_tk, extent=extent_tk,
                    style=tk.ARC, outline=RED, width=2, dash=(6,3), tags="reference"
                )
                # Threshold label
                self.canvas.create_text(
                    self.center_x, self.center_y - threshold_radius_px - 5, # Position above arc center
                    text=f"Brake: {threshold_cm:.0f}cm",
                    fill=RED, anchor=tk.S,
                    font=self.small_font, tags="reference"
                )
        except ValueError:
            pass # Ignore if threshold is not a valid number

    def _polar_to_cartesian(self, angle_deg, distance_cm):
        """Converts robot-centric polar coordinates (angle 0=forward) to canvas coordinates."""
        # Convert robot angle (0=fwd) to math angle (0=East, 90=North)
        # Robot 0 deg = Math 90 deg; Robot 90 deg = Math 0 deg; Robot -90 deg = Math 180 deg
        math_angle_rad = math.radians(90 - angle_deg)
        dist_px = distance_cm * self.scale

        x = self.center_x + dist_px * math.cos(math_angle_rad)
        y = self.center_y - dist_px * math.sin(math_angle_rad) # Y is inverted in Tkinter canvas
        return int(x), int(y)

    # --- Event Handlers and State Updates ---
    def update_arduino_connection_state(self, connected):
        """Enable/disable Lidar assist controls based on Arduino connection."""
        state = tk.NORMAL if connected else tk.DISABLED
        self.stop_assist_checkbox.config(state=state)
        self.threshold_entry.config(state=state)
        self.manual_brake_button.config(state=state)
        self.manual_release_button.config(state=state)
        if not connected:
            self.stop_assist_var.set(False) # Turn off assist if Arduino disconnects
            self.main_app.stop_assist_active = False # Update central state

    def on_stop_assist_changed(self):
        """Handle change in the stop assist checkbox."""
        is_active = self.stop_assist_var.get()
        self.main_app.stop_assist_active = is_active # Update central state
        if is_active:
            # Optional: Send V 0 immediately when activating?
            # self.arduino_communicator.send_command("V 0")
            self.log_callback("Lidar Auto-Stop Assist ENABLED.")
        else:
            # When disabling, ensure brake isn't stuck ON due to assist
            # Check if brake is active and *was* applied by assist? Hard to know for sure.
            # Maybe just log that it's disabled. User must manually release if needed.
            self.log_callback("Lidar Auto-Stop Assist DISABLED.")
            # Force warning text clear if it was showing auto-brake message
            if "AUTO-BRAKING" in self.canvas.itemcget(self.warning_text, "text"):
                self.canvas.itemconfig(self.warning_text, text="")

    def on_proximity_alert_changed(self):
        """Handle change in the proximity alert checkbox."""
        is_active = self.proximity_alert_var.get()
        self.proximity_alert_active = is_active
        if is_active:
            self.log_callback("Proximity Alert Sound ENABLED.")
        else:
            self.log_callback("Proximity Alert Sound DISABLED.")

    def play_proximity_beep(self, proximity_level):
        """Play a beep sound based on proximity level."""
        import winsound
        import time
        
        # Get current time
        current_time = time.time()
        
        # Check if enough time has passed since last beep
        required_interval = self.beep_intervals.get(proximity_level, 1.0)
        if current_time - self.last_beep_time < required_interval:
            return
        
        # Update last beep time
        self.last_beep_time = current_time
        
        # Different frequencies for different proximity levels
        frequencies = {
            "far": 800,     # Hz - lower tone
            "medium": 1200, # Hz - medium tone
            "close": 2000   # Hz - high tone
        }
        
        # Different durations for different proximity levels
        durations = {
            "far": 100,     # ms - short beep
            "medium": 150,  # ms - medium beep
            "close": 200    # ms - longer beep
        }
        
        # Calculate volume as percentage of maximum (0-100% to 0-65535)
        volume = int((self.volume_var.get() / 100) * 65535)
        
        # Play the beep based on proximity level
        freq = frequencies.get(proximity_level, 1000)
        duration = durations.get(proximity_level, 100)
        
        try:
            # On Windows, use winsound
            winsound.Beep(freq, duration)
        except:
            # Fallback for non-Windows platforms using system beep
            print("\a")  # System bell

    def handle_auto_stop_assist(self, closest_distance):
        """
        Checks if an obstacle is too close and assist is active.
        If so, triggers the main application's emergency_stop function.
        """
        if not self.main_app.stop_assist_active or not self.arduino_communicator.is_connected:
            return

        try:
            threshold = float(self.threshold_var.get())
        except ValueError:
            threshold = AUTO_BRAKE_THRESHOLD

        # --- Auto-E-Stop Logic ---
        if 0 < closest_distance < threshold:
             # Obstacle is too close, check if brake needs to be applied (E-Stop handles this check internally too, but doesn't hurt)
             # We trigger E-Stop even if brake is already active, ensuring V 0 is also sent.
             # Check if brake is ALREADY active due to E-Stop to prevent spamming logs/commands if distance remains low
             if not (self.main_app.brake_active and self.main_app.brake_reason == "E-Stop"):
                 self.log_callback(f"Auto-Stop Assist: Obstacle detected at {closest_distance:.1f}cm (Threshold: {threshold}cm). Triggering EMERGENCY STOP.", is_error=True) # Log as error for E-Stop
                 # ***** CHANGE HERE: Call main app's emergency_stop *****
                 self.main_app.emergency_stop()
                 # emergency_stop() handles sending V 0, S 1, logging, and setting brake_active/brake_reason
# --- Main Application Class ---
class CombinedRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Robot Control & Lidar Interface")
        # Try to make window resizable (Canvas needs adjustments)
        self.root.geometry("850x700") # Initial size
        # self.root.resizable(True, True) # Allow resizing

        # --- Core Components ---
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        # Pass self.log_message as the callback for logging
        self.arduino_communicator = ArduinoCommunicator(self.command_queue, self.response_queue, self.log_message)
        self.xbox_controller = XboxController(log_callback=self.log_message)
        self.lidar_processor = LidarProcessor(log_callback=self.log_message)

        # --- Shared State Variables ---
        self.arduino_connected = False
        self.is_automatic_mode = False # True = Auto, False = Manual (M command)
        self.brake_active = False # True = Brake applied (S 1), False = Released (Q 1)
        self.brake_reason = "" # Store why brake was activated (e.g., "Manual", "Auto-Stop", "E-Stop")
        self.controller_polling_active = False # Is the Xbox controller actively sending commands?
        self.stop_assist_active = False # Is Lidar auto-braking enabled? (Controlled by Lidar tab checkbox)

        # --- UI Setup ---
        self._create_styles()
        self._create_main_ui()

        # --- Start Background Tasks ---
        self.check_responses() # Start checking for Arduino responses
        self.poll_controller() # Start polling the Xbox controller

        # --- Bind Resize Event ---
        # self.root.bind("<Configure>", self.on_resize) # Handle window resize

    def _create_styles(self):
        self.style = ttk.Style()
        # ('winnative', 'clam', 'alt', 'default', 'classic', 'vista', 'xpnative')
        try:
             self.style.theme_use('clam') # Or 'vista', 'xpnative' on Windows
        except tk.TclError:
             print("Clam theme not available, using default.")

        self.style.configure("Emergency.TButton", foreground="white", background="red", font=("Arial", 12, "bold"))
        self.style.configure("TNotebook.Tab", padding=[10, 5], font=('Arial', 10))
        self.style.configure("TLabelframe.Label", font=('Arial', 10, 'bold'))
        # Style for indicating active state?
        self.style.configure("Active.TButton", foreground="black", background="#A0FFA0") # Light green background


    def _create_main_ui(self):
        # Use PanedWindow for adjustable sections? Or just Notebook? Notebook is simpler.
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create frames for each tab
        self.robot_control_frame = ttk.Frame(self.notebook)
        self.lidar_frame = ttk.Frame(self.notebook) # We'll place the LidarVisualizerFrame inside this

        self.notebook.add(self.robot_control_frame, text="Robot Control")
        self.notebook.add(self.lidar_frame, text="Lidar Visualizer")

        # Populate the Robot Control Tab
        self._create_robot_control_tab(self.robot_control_frame)

        # Populate the Lidar Tab by instantiating LidarVisualizerFrame
        self.lidar_visualizer = LidarVisualizerFrame(self.lidar_frame, self, self.lidar_processor, self.arduino_communicator, self.log_message)
        self.lidar_visualizer.pack(fill=tk.BOTH, expand=True)

        # Status Bar at the bottom
        self.status_bar = ttk.Label(self.root, text="Status: Ready", relief=tk.SUNKEN, anchor=tk.W, padding=5)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _create_robot_control_tab(self, parent_frame):
        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(parent_frame, text="Arduino Connection")
        conn_frame.pack(fill=tk.X, padx=10, pady=(10, 5))

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky='w')
        self.port_var = tk.StringVar()
        self.port_combobox = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self.refresh_ports() # Populate ports initially

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=(10, 5), pady=5, sticky='w')
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD_RATE))
        baud_rates = ["9600", "19200", "38400", "57600", "115200", "230400", "250000"]
        self.baud_combobox = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=baud_rates, width=8)
        self.baud_combobox.grid(row=0, column=3, padx=5, pady=5, sticky='w')

        self.refresh_button = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports, width=8)
        self.refresh_button.grid(row=0, column=4, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect_arduino, width=8)
        self.connect_button.grid(row=0, column=5, padx=5, pady=5)
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_arduino, width=10, state=tk.DISABLED)
        self.disconnect_button.grid(row=0, column=6, padx=5, pady=5)

        # --- Controller Frame ---
        controller_frame = ttk.LabelFrame(parent_frame, text="Xbox Controller")
        controller_frame.pack(fill=tk.X, padx=10, pady=5)

        self.controller_status_label = ttk.Label(controller_frame, text="Status: Disconnected", width=40)
        self.controller_status_label.pack(side=tk.LEFT, padx=10, pady=5)
        self.connect_controller_button = ttk.Button(controller_frame, text="Connect", command=self.connect_controller, width=10)
        self.connect_controller_button.pack(side=tk.LEFT, padx=5, pady=5)
        self.toggle_controller_button = ttk.Button(controller_frame, text="Enable Control", command=self.toggle_controller_polling, width=15, state=tk.DISABLED)
        self.toggle_controller_button.pack(side=tk.LEFT, padx=5, pady=5)


        # Frame to hold Controls and Console side-by-side
        main_area = ttk.Frame(parent_frame)
        main_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # --- Manual Controls Frame (Left) ---
        manual_controls_frame = ttk.LabelFrame(main_area, text="Manual Robot Controls")
        manual_controls_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # Mode Toggle Button
        mode_frame = ttk.Frame(manual_controls_frame)
        mode_frame.pack(fill=tk.X, pady=5)
        self.mode_button = ttk.Button(mode_frame, text="Switch to AUTOMATIC", command=self.toggle_mode, state=tk.DISABLED)
        self.mode_button.pack(fill=tk.X, padx=10, pady=5)

        # Steering Slider (Horizontal)
        steer_frame = ttk.Frame(manual_controls_frame)
        steer_frame.pack(fill=tk.X, pady=5)
        ttk.Label(steer_frame, text="Steer:", width=6).pack(side=tk.LEFT, padx=(10,0))
        # Use the controller's range for consistency
        self.steering_slider = ttk.Scale(steer_frame, from_=self.xbox_controller.STEERING_MIN,
                                         to=self.xbox_controller.STEERING_MAX, orient="horizontal",
                                         command=self.on_steering_change, state=tk.DISABLED, length=200)
        self.steering_slider.set(self.xbox_controller.STEERING_CENTER)
        self.steering_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.steering_value_label = ttk.Label(steer_frame, text=str(self.xbox_controller.STEERING_CENTER), width=5)
        self.steering_value_label.pack(side=tk.LEFT, padx=(0,10))

        # Acceleration Slider (Vertical)
        # Put accel slider and direction/brake buttons in a sub-frame
        drive_frame = ttk.Frame(manual_controls_frame)
        drive_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        accel_subframe = ttk.Frame(drive_frame)
        accel_subframe.pack(side=tk.LEFT, fill=tk.Y, padx=10)
        ttk.Label(accel_subframe, text="Accel:").pack(anchor='n')
        self.accel_slider = ttk.Scale(accel_subframe, from_=self.xbox_controller.ACCEL_MAX, to=0,
                                      orient="vertical", command=self.on_accel_change, state=tk.DISABLED, length=150)
        self.accel_slider.set(0)
        self.accel_slider.pack(fill=tk.Y, expand=True, pady=5)
        self.accel_value_label = ttk.Label(accel_subframe, text="0", width=3)
        self.accel_value_label.pack(anchor='s')


        # Buttons (Direction, Brake, E-Stop) Frame
        buttons_subframe = ttk.Frame(drive_frame)
        buttons_subframe.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        dir_frame = ttk.LabelFrame(buttons_subframe, text="Direction")
        dir_frame.pack(fill=tk.X, pady=5)
        self.forward_btn = ttk.Button(dir_frame, text="Forward (D F)", command=lambda: self.send_direction("F"), state=tk.DISABLED)
        self.forward_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=2)
        self.reverse_btn = ttk.Button(dir_frame, text="Reverse (D R)", command=lambda: self.send_direction("R"), state=tk.DISABLED)
        self.reverse_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=2)

        brake_frame = ttk.LabelFrame(buttons_subframe, text="Brake Control")
        brake_frame.pack(fill=tk.X, pady=5)
        self.brake_btn = ttk.Button(brake_frame, text="Engage Brake (S 1)", command=self.toggle_brake, state=tk.DISABLED)
        self.brake_btn.pack(fill=tk.X, padx=5, pady=2)

        estop_frame = ttk.LabelFrame(buttons_subframe, text="Emergency")
        estop_frame.pack(fill=tk.X, pady=10)
        self.stop_btn = ttk.Button(estop_frame, text="EMERGENCY STOP",
                                  command=self.emergency_stop, style="Emergency.TButton", state=tk.DISABLED)
        self.stop_btn.pack(fill=tk.X, padx=5, pady=5)

        # --- Response Console (Right) ---
        console_frame = ttk.LabelFrame(main_area, text="Arduino/System Log")
        console_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

        self.response_console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD, height=15, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1)
        self.response_console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        # Configure tags for different message types
        self.response_console.tag_configure("INFO", foreground="black")
        self.response_console.tag_configure("ERROR", foreground="red")
        self.response_console.tag_configure("WARNING", foreground="#FF8C00") # DarkOrange
        self.response_console.tag_configure("SUCCESS", foreground="green")
        self.response_console.tag_configure("CMD", foreground="blue")
        self.response_console.tag_configure("RECV", foreground="#551A8B") # Purple

        clear_button = ttk.Button(console_frame, text="Clear Log", command=self.clear_console)
        clear_button.pack(fill=tk.X, padx=5, pady=(0, 5))

    # --- UI Update and State Management ---

    def update_ui_based_on_connection(self):
        """Enable/disable controls based on Arduino connection status."""
        self.arduino_connected = self.arduino_communicator.is_connected
        state = tk.NORMAL if self.arduino_connected else tk.DISABLED
        conn_state = tk.DISABLED if self.arduino_connected else tk.NORMAL

        # Connection buttons
        self.connect_button.config(state=conn_state)
        self.port_combobox.config(state=conn_state)
        self.baud_combobox.config(state=conn_state)
        self.refresh_button.config(state=conn_state)
        self.disconnect_button.config(state=state)

        # Mode button
        self.mode_button.config(state=state)

        # Enable controller toggle only if controller hardware is connected AND Arduino is connected
        controller_hw_connected = self.xbox_controller.is_connected()
        self.toggle_controller_button.config(state=tk.NORMAL if (controller_hw_connected and self.arduino_connected) else tk.DISABLED)
        if not controller_hw_connected or not self.arduino_connected:
             self.controller_polling_active = False # Ensure polling stops if disconnected
             self.toggle_controller_button.config(text="Enable Control") # Reset text


        # Update Lidar tab controls
        if hasattr(self, 'lidar_visualizer'):
            self.lidar_visualizer.update_arduino_connection_state(self.arduino_connected)

        # Update controls based on mode (Manual/Auto) as well
        self.update_ui_based_on_mode()

    def update_ui_based_on_mode(self):
        """Enable/disable sliders/buttons based on Manual/Automatic mode."""
        # Only enable controls if Arduino is connected
        if not self.arduino_connected:
            state_auto = tk.DISABLED
            state_manual = tk.DISABLED
            state_always = tk.DISABLED # For E-Stop
        else:
            state_auto = tk.NORMAL if self.is_automatic_mode else tk.DISABLED
            state_manual = tk.DISABLED # Manual controls generally disabled if controlled by code
            state_always = tk.NORMAL # For E-Stop

        # --- Controls enabled in AUTOMATIC mode ---
        # Sliders are only controlled by code/controller in Auto mode, so disable manual interaction
        self.steering_slider.config(state=tk.DISABLED) # Always disabled for manual drag
        self.accel_slider.config(state=tk.DISABLED)  # Always disabled for manual drag
        # Direction buttons
        self.forward_btn.config(state=state_auto)
        self.reverse_btn.config(state=state_auto)
        # Brake button
        self.brake_btn.config(state=state_auto)
        # Emergency stop button should always be enabled when connected
        self.stop_btn.config(state=state_always)

        # Update mode button text
        if self.arduino_connected:
            self.mode_button.config(text=f"Switch to {'MANUAL' if self.is_automatic_mode else 'AUTOMATIC'}")

    def update_brake_button_text(self):
         if self.brake_active:
              self.brake_btn.config(text=f"Release Brake (Q 1) [{self.brake_reason}]")
         else:
              self.brake_btn.config(text="Engage Brake (S 1)")

    # --- Action Handlers ---

    def refresh_ports(self):
        self.log_message("Refreshing serial ports...")
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combobox['values'] = ports
            if ports:
                # Try to find the default or previously used port
                current_val = self.port_var.get()
                if current_val in ports:
                    self.port_combobox.set(current_val)
                elif DEFAULT_ARDUINO_PORT in ports:
                    self.port_combobox.set(DEFAULT_ARDUINO_PORT)
                else:
                    self.port_combobox.current(0)
                self.log_message(f"Available ports: {', '.join(ports)}")
            else:
                self.log_message("No serial ports found.", is_warning=True)
        except Exception as e:
            self.log_message(f"Error refreshing ports: {e}", is_error=True)


    def connect_arduino(self):
        port = self.port_var.get()
        baud = self.baud_var.get()
        if not port:
            messagebox.showwarning("Connect", "Please select a serial port.")
            return
        if not baud:
             messagebox.showwarning("Connect", "Please select a baud rate.")
             return

        # Disconnect first if already connected to another port
        if self.arduino_communicator.is_connected:
             self.disconnect_arduino()
             # Short delay to allow disconnect to settle?
             # time.sleep(0.2)

        self.update_status(f"Connecting to {port}...")
        if self.arduino_communicator.connect(port, int(baud)):
            # Success handled by communicator's log callback
            self.update_status(f"Connected to {port}")
            # Assume Manual mode on connection? Send 'M'? Or wait for user?
            # Let's wait for user to select mode.
            self.is_automatic_mode = False # Reset mode state on connect
            self.brake_active = False # Reset brake state on connect
            self.update_brake_button_text()
            self.update_ui_based_on_connection()
            # Automatically try to connect controller?
            if not self.xbox_controller.is_connected():
                self.connect_controller()
        else:
            # Failure logged by communicator
            messagebox.showerror("Connection Failed", f"Could not connect to Arduino on {port}.")
            self.update_status("Connection failed")
            self.update_ui_based_on_connection()


    def disconnect_arduino(self):
        # Stop Lidar assist first if active?
        if self.stop_assist_active:
             self.stop_assist_var.set(False)
             self.on_stop_assist_changed() # Ensure state is updated

        self.arduino_communicator.disconnect()
        self.update_status("Disconnected")
        self.is_automatic_mode = False
        self.brake_active = False
        # Stop controller polling if it was active
        self.controller_polling_active = False
        self.update_ui_based_on_connection()
        # Reset button texts
        self.update_brake_button_text()
        self.toggle_controller_button.config(text="Enable Control")

    def connect_controller(self):
        self.update_status("Connecting to Xbox controller...")
        if self.xbox_controller.connect():
             self.controller_status_label.config(text=f"Status: Connected ({self.xbox_controller.controller_name})")
             self.connect_controller_button.config(state=tk.DISABLED)
             self.update_status("Xbox controller connected")
             self.update_ui_based_on_connection() # Re-check toggle button state
        else:
             self.controller_status_label.config(text="Status: Connection Failed")
             self.connect_controller_button.config(state=tk.NORMAL)
             self.update_status("Xbox controller connection failed")
             messagebox.showerror("Controller Error", "Failed to connect to Xbox controller.")
             self.update_ui_based_on_connection()


    def toggle_controller_polling(self):
        if not self.arduino_connected or not self.xbox_controller.is_connected():
             self.log_message("Cannot enable controller: Arduino or Xbox controller not connected.", is_warning=True)
             self.controller_polling_active = False # Ensure it's off
             self.toggle_controller_button.config(text="Enable Control")
             return

        self.controller_polling_active = not self.controller_polling_active
        if self.controller_polling_active:
             # Ensure we are in Automatic mode when enabling controller
             if not self.is_automatic_mode:
                  self.log_message("Switching to AUTOMATIC mode to enable controller.", is_info=True)
                  self.set_mode(automatic=True) # Switch to Auto mode

             self.log_message("Xbox controller polling ENABLED.")
             self.toggle_controller_button.config(text="Disable Control")
             self.update_status("Controller Active")
             # Disable manual UI elements that controller overrides? (Sliders already disabled)
             # self.forward_btn.config(state=tk.DISABLED)
             # self.reverse_btn.config(state=tk.DISABLED)
             # self.brake_btn.config(state=tk.DISABLED) # Brake via LT?

        else:
             self.log_message("Xbox controller polling DISABLED.")
             self.toggle_controller_button.config(text="Enable Control")
             self.update_status("Controller Inactive")
             # Re-enable manual buttons if in Auto mode
             self.update_ui_based_on_mode()
             # Send V 0 when disabling controller?
             self.arduino_communicator.send_command("V 0")
             self.accel_slider.set(0) # Update slider UI
             self.accel_value_label.config(text="0")


    def set_mode(self, automatic: bool):
        """Sets the robot mode (Manual/Automatic) and sends command."""
        if not self.arduino_connected: return

        target_mode = "A" if automatic else "M"
        current_mode = "A" if self.is_automatic_mode else "M"

        if target_mode == current_mode:
            # self.log_message(f"Already in {'Automatic' if automatic else 'Manual'} mode.")
            return # Already in the desired mode

        # Send appropriate command
        if self.arduino_communicator.send_command(target_mode):
             self.is_automatic_mode = automatic
             mode_name = "Automatic" if automatic else "Manual"
             self.log_message(f"Switched to {mode_name} mode ({target_mode}).", is_success=True)

             # Send V 0 and center steering when switching modes for safety
             self.arduino_communicator.send_command("V 0")
             self.arduino_communicator.send_command(f"W {self.xbox_controller.STEERING_CENTER}")
             # If switching to Auto, maybe set default direction?
             if automatic:
                  self.arduino_communicator.send_command("D F") # Default to Forward in Auto?

             # Update UI elements
             self.update_ui_based_on_mode()
             # Update slider displays to reflect reset state
             self.accel_slider.set(0)
             self.accel_value_label.config(text="0")
             self.steering_slider.set(self.xbox_controller.STEERING_CENTER)
             self.steering_value_label.config(text=str(self.xbox_controller.STEERING_CENTER))
             self.update_status(f"Mode: {mode_name}")

             # If switching *away* from Auto while controller was active, disable controller polling
             if not automatic and self.controller_polling_active:
                 self.controller_polling_active = False
                 self.toggle_controller_button.config(text="Enable Control")
                 self.log_message("Controller polling disabled (switched to Manual mode).")

        else:
             self.log_message(f"Failed to send {target_mode} command.", is_error=True)


    def toggle_mode(self):
         """Toggles between Manual and Automatic mode."""
         self.set_mode(not self.is_automatic_mode)


    def on_steering_change(self, value_str):
        """Callback when steering slider is manually moved (only for display update)."""
        if not self.arduino_connected: return
        value = int(float(value_str))
        self.steering_value_label.config(text=str(value))
        # Manual slider drag should NOT send commands if in Auto mode or controller active
        # if self.is_automatic_mode and not self.controller_polling_active:
        #      # Send command only if in Auto AND controller is off (fully manual Auto?)
        #      self.arduino_communicator.send_command(f"W {value}")


    def on_accel_change(self, value_str):
        """Callback when acceleration slider is manually moved (only for display update)."""
        if not self.arduino_connected: return
        value = int(float(value_str))
        self.accel_value_label.config(text=str(value))
        # Manual slider drag should NOT send commands if in Auto mode or controller active
        # if self.is_automatic_mode and not self.controller_polling_active:
        #      # Send command only if in Auto AND controller is off
        #      self.arduino_communicator.send_command(f"V {value}")

    def send_direction(self, direction):
        """Sends D F or D R command."""
        if self.is_automatic_mode and self.arduino_connected:
            if self.arduino_communicator.send_command(f"D {direction}"):
                 self.log_message(f"Direction set to {'Forward' if direction == 'F' else 'Reverse'}.")
                 # Highlight active direction button?
                 # self.forward_btn.config(style="Active.TButton" if direction == "F" else "TButton")
                 # self.reverse_btn.config(style="Active.TButton" if direction == "R" else "TButton")
        else:
            self.log_message("Cannot set direction: Not in Automatic mode or not connected.", is_warning=True)

    def update_brake_state(self, active, reason="Unknown"):
        """Central method to update brake status."""
        self.brake_active = active
        self.brake_reason = reason if active else ""
        self.update_brake_button_text()
        # If activating brake, ensure accel slider shows 0
        if active:
             self.accel_slider.set(0)
             self.accel_value_label.config(text="0")


    def toggle_brake(self):
        """Manually toggles the brake state (S 1 / Q 1)."""
        if not self.is_automatic_mode or not self.arduino_connected:
            self.log_message("Cannot toggle brake: Not in Automatic mode or not connected.", is_warning=True)
            return

        if self.brake_active:
            # Currently active, send release command (Q 1)
            self.log_message("Manually releasing brake (Q 1)...")
            if self.arduino_communicator.send_command("Q 1"):
                self.update_brake_state(False, "Manual")
                self.log_message("Brake released manually.")
            else:
                self.log_message("Failed to send Q 1 command.", is_error=True)
        else:
            # Currently inactive, send engage command (S 1)
            self.log_message("Manually engaging brake (S 1)...")
            if self.arduino_communicator.send_command("S 1"):
                self.update_brake_state(True, "Manual")
                self.log_message("Brake engaged manually.")
                 # Optional: Send V 0 explicitly after S 1? S 1 should handle this.
                 # self.arduino_communicator.send_command("V 0")
            else:
                self.log_message("Failed to send S 1 command.", is_error=True)


    def emergency_stop(self):
        """Sends immediate stop commands (V 0 and S 1)."""
        if not self.arduino_connected:
             self.log_message("Cannot E-STOP: Arduino not connected.", is_error=True)
             return

        self.log_message("EMERGENCY STOP ACTIVATED!", is_error=True)
        self.update_status("EMERGENCY STOP")
        # Send V 0 first, then S 1 for immediate braking
        self.arduino_communicator.send_command("V 0")
        time.sleep(0.05) # Short delay
        if self.arduino_communicator.send_command("S 1"):
             self.update_brake_state(True, "E-Stop")
             # Update UI immediately
             self.accel_slider.set(0)
             self.accel_value_label.config(text="0")
             self.log_message("E-Stop: V 0 and S 1 sent.")
             # Optional: Automatically release after a delay? Or require manual release?
             # self.root.after(5000, self.release_emergency_stop) # Example: Release after 5s
        else:
             self.log_message("E-Stop: Failed to send S 1 command.", is_error=True)


    
    def release_emergency_stop(self):
         """Sends Q 1 to release brake after an E-Stop."""
         # Check if E-Stop is actually the reason for the brake being active
         if self.brake_active and self.brake_reason == "E-Stop":
              self.log_message("Releasing E-Stop brake (Q 1)...")
              if self.arduino_communicator.send_command("Q 1"):
                   self.update_brake_state(False, "E-Stop Release") # Update reason to show it was released
                   self.log_message("E-Stop brake released successfully.", is_success=True)
              else:
                   self.log_message("Failed to send Q 1 command for E-Stop release.", is_error=True)
         elif self.brake_active:
             # Brake is on, but not for E-Stop reason
             self.log_message(f"Cannot release E-stop: Brake is active but reason is '{self.brake_reason}'. Use appropriate release method.", is_warning=True)
         else:
             # Brake is not active
             self.log_message("Cannot release E-stop: Brake is not active.", is_warning=True)


    # --- Background Tasks ---

    def check_responses(self):
        """Periodically check the response queue and log messages."""
        try:
            while not self.response_queue.empty():
                response = self.response_queue.get_nowait()
                self.log_message(f"Recv: {response}", is_recv=True)
                # TODO: Parse responses if needed (e.g., status updates from Arduino)
                self.response_queue.task_done()
        except queue.Empty:
            pass
        except Exception as e:
             self.log_message(f"Error processing response queue: {e}", is_error=True)
        finally:
            self.root.after(50, self.check_responses) # Check frequently

    def poll_controller(self):
        """Periodically poll the Xbox controller and send commands if active."""
        controller_was_active_before_poll = self.controller_polling_active # Store state before update

        try:
            if self.xbox_controller.is_connected():
                success, event = self.xbox_controller.update()

                if not success:
                    # Controller disconnected or error during update handling
                    self.log_message("Controller update failed or disconnected.", is_warning=True)
                    
                    # --- Safety Feature: Trigger E-Stop if controller disconnects while active ---
                    if controller_was_active_before_poll and self.arduino_connected:
                        self.log_message("Controller disconnected while active! Triggering EMERGENCY STOP.", is_error=True)
                        self.emergency_stop() # Send V 0 and S 1
                    # --- End Safety Feature ---
                    
                    # Update UI and internal state regardless of E-Stop
                    self.controller_status_label.config(text="Status: Disconnected (Error)")
                    self.connect_controller_button.config(state=tk.NORMAL)
                    self.toggle_controller_button.config(state=tk.DISABLED, text="Enable Control")
                    self.controller_polling_active = False # Ensure polling stops sending commands
                    self.update_status("Xbox controller disconnected")
                    # Don't immediately reschedule poll if disconnected, let connect handle it?
                    # Or keep polling to detect reconnection? Let's keep polling.

                elif self.controller_polling_active and self.is_automatic_mode and self.arduino_connected:
                    # --- Controller is active, send commands ---

                    # Steering
                    if self.xbox_controller.has_steering_changed():
                        steer_val = self.xbox_controller.last_sent_steering
                        self.arduino_communicator.send_command(f"W {steer_val}")
                        # Update UI only if slider exists
                        if hasattr(self, 'steering_slider'):
                            self.steering_slider.set(steer_val)
                        if hasattr(self, 'steering_value_label'):
                            self.steering_value_label.config(text=str(steer_val))


                    # Acceleration (only send if brake is not active)
                    if self.xbox_controller.has_accel_changed():
                        accel_val = self.xbox_controller.last_sent_accel
                        if not self.brake_active:
                             # Only send V command if brake is OFF
                             self.arduino_communicator.send_command(f"V {accel_val}")
                        else:
                            # If brake is ON, ensure accel command is 0, even if trigger pulled
                            if accel_val > 0:
                                # Check if we need to send V 0 explicitly?
                                # E-stop/brake engage should already handle this.
                                # Maybe log a warning?
                                # self.log_message("Controller accel ignored, brake is active.", is_warning=True) # Can be noisy
                                pass
                        # Always update UI slider regardless of brake state
                        if hasattr(self, 'accel_slider'):
                            self.accel_slider.set(accel_val)
                        if hasattr(self, 'accel_value_label'):
                            self.accel_value_label.config(text=str(accel_val))


                    # --- Handle Specific Controller Events ---
                    if event == "HAT_CHANGED":
                        hat_x, hat_y = self.xbox_controller.get_hat_values()
                        if hat_y == 1: self.send_direction("F")
                        elif hat_y == -1: self.send_direction("R")
                        # Add handling for hat_x if needed (e.g., fine steering adjustments?)

                    elif event == "LT_PRESSED":
                        if not self.brake_active:
                              self.log_message("Controller: LT pressed, engaging brake (S 1)...")
                              if self.arduino_communicator.send_command("S 1"):
                                   self.update_brake_state(True, "Controller LT")
                              else:
                                   self.log_message("Failed to send S 1 from controller.", is_error=True)

                    elif event == "LT_RELEASED":
                         # Only release if the *reason* for the brake was the LT press
                         if self.brake_active and self.brake_reason == "Controller LT":
                              self.log_message("Controller: LT released, releasing brake (Q 1)...")
                              if self.arduino_communicator.send_command("Q 1"):
                                   self.update_brake_state(False, "") # Clear specific reason
                              else:
                                   self.log_message("Failed to send Q 1 from controller.", is_error=True)
                         elif self.brake_active:
                              # LT released, but brake is active for another reason (E-Stop, Manual, Auto-Stop)
                              self.log_message(f"Controller: LT released, but brake remains active (Reason: {self.brake_reason}).", is_info=True)

                    elif event == "X_PRESSED":
                         # Use X button to release E-Stop only
                         if self.brake_active and self.brake_reason == "E-Stop":
                              self.log_message("Controller: X button pressed, attempting to release E-Stop...", is_info=True)
                              self.release_emergency_stop() # Call the dedicated release function
                         elif self.brake_active:
                              self.log_message(f"Controller: X button pressed, but brake is active for another reason ('{self.brake_reason}'). No action taken.", is_warning=True)
                         else:
                              self.log_message("Controller: X button pressed, but brake is not active. No action taken.", is_info=True)
            else:
                # Controller hardware is not connected according to is_connected()
                if controller_was_active_before_poll:
                    # It *was* active according to our flag, but now hardware is gone
                    self.log_message("Controller hardware lost while polling was active! Triggering EMERGENCY STOP.", is_error=True)
                    if self.arduino_connected:
                        self.emergency_stop()

                    # Update UI and state
                    self.controller_status_label.config(text="Status: Disconnected (Lost)")
                    self.connect_controller_button.config(state=tk.NORMAL)
                    self.toggle_controller_button.config(state=tk.DISABLED, text="Enable Control")
                    self.controller_polling_active = False
                    self.update_status("Xbox controller lost")


            # Schedule next poll only if the root window still exists
            if self.root.winfo_exists():
                self.root.after(50, self.poll_controller) # Poll ~20 times per second

        except Exception as e:
            self.log_message(f"Error in controller poll loop: {e}", is_error=True)
            # Try to recover and continue polling if the window exists
            if self.root.winfo_exists():
                self.root.after(100, self.poll_controller) # Poll less frequently after error

    # --- Logging and Status ---
    # ... (log_message, clear_console, update_status remain the same) ...

   

    


    # --- Logging and Status ---

    def log_message(self, message, is_error=False, is_warning=False, is_success=False, is_info=False, is_cmd=False, is_recv=False):
        """Add a timestamped message to the console with appropriate tag."""
        timestamp = time.strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"

        # Determine tag
        tag = "INFO" # Default
        if is_error: tag = "ERROR"
        elif is_warning: tag = "WARNING"
        elif is_success: tag = "SUCCESS"
        elif is_cmd: tag = "CMD"
        elif is_recv: tag = "RECV"
        elif is_info: tag = "INFO" # Explicit info

        try:
            if self.response_console:
                self.response_console.config(state=tk.NORMAL)
                self.response_console.insert(tk.END, full_message, (tag,))
                self.response_console.see(tk.END) # Auto-scroll
                self.response_console.config(state=tk.DISABLED)
        except tk.TclError:
             # Handle case where console might be destroyed during shutdown
             print(f"LOG ({tag}): {full_message.strip()}") # Fallback to console print
        except Exception as e:
             print(f"LOGGING ERROR: {e}")
             print(f"Original Msg ({tag}): {full_message.strip()}")


    def clear_console(self):
        if self.response_console:
            self.response_console.config(state=tk.NORMAL)
            self.response_console.delete(1.0, tk.END)
            self.response_console.config(state=tk.DISABLED)
            self.log_message("Log cleared.", is_info=True)

    def update_status(self, message):
        """Update the bottom status bar."""
        if self.status_bar:
            timestamp = time.strftime("%H:%M:%S")
            self.status_bar.config(text=f"Status: {message} ({timestamp})")

    # --- Resize Handling ---
    # def on_resize(self, event):
    #     # Debounce resize events?
    #     # Check if the event source is the main window
    #     if event.widget == self.root:
    #         # Check if size actually changed significantly?
    #         # Schedule the canvas redraw/setup after a short delay
    #         if hasattr(self, '_resize_job'):
    #             self.root.after_cancel(self._resize_job)
    #         self._resize_job = self.root.after(250, self._handle_resize_end)

    # def _handle_resize_end(self):
    #      # This function is called after resizing stops
    #      self.log_message("Window resized.", is_info=True)
    #      if hasattr(self, 'lidar_visualizer'):
    #          self.lidar_visualizer._setup_canvas() # Re-calculate scale and redraw reference
    #          # No need to restart update_display, it uses current scale/center


    # --- Application Shutdown ---
    def on_closing(self):
        """Handle cleanup when the application window is closed."""
        self.log_message("----- Application Closing -----", is_warning=True)
        self.update_status("Closing...")

        # 1. Stop Lidar Processing and Scanning
        if hasattr(self, 'lidar_visualizer'):
             self.lidar_visualizer.stop_display_update()
        if self.lidar_processor:
             self.lidar_processor.disconnect_lidar() # Stops scan and disconnects

        # 2. Stop Controller Polling (implicitly stopped by root destroy, but good practice)
        # No explicit stop needed for `after` loops usually, but stop sending commands

        # 3. Disconnect Arduino (sends V 0 etc. if implemented in disconnect)
        if self.arduino_communicator:
            # Send stop commands before final shutdown
            if self.arduino_communicator.is_connected:
                 self.arduino_communicator.send_command("V 0")
                 self.arduino_communicator.send_command("S 1") # End in safe state
                 time.sleep(0.1) # Allow commands to be sent
            self.arduino_communicator.shutdown()

        # 4. Disconnect Xbox Controller and Quit Pygame
        if self.xbox_controller:
            self.xbox_controller.disconnect()
        pygame.quit()
        self.log_message("Pygame shut down.")

        # 5. Destroy the Tkinter window
        self.log_message("Destroying GUI...")
        self.root.destroy()
        print("Application closed.") # Final message to console


# --- Main Execution ---
if __name__ == "__main__":
    try:
        # Initialize Pygame explicitly before Tkinter (sometimes helps with focus/events)
        # pygame.init() - Moved inside XboxController init

        # Create the main Tkinter window
        root = tk.Tk()

        # Create the main application instance
        app = CombinedRobotApp(root)

        # Set the close window protocol
        root.protocol("WM_DELETE_WINDOW", app.on_closing)

        # Start the Tkinter main loop
        root.mainloop()

    except Exception as e:
        print(f"\n--- UNHANDLED ERROR IN MAIN ---")
        import traceback
        traceback.print_exc()
        print(f"Error: {e}")
        print("Exiting.")
        # Attempt cleanup even on error
        try:
             if 'app' in locals() and app:
                 if app.lidar_processor: app.lidar_processor.disconnect_lidar()
                 if app.arduino_communicator: app.arduino_communicator.shutdown()
                 if app.xbox_controller: app.xbox_controller.disconnect()
                 pygame.quit()
             elif 'pygame' in sys.modules:
                 pygame.quit() # Quit pygame if initialized but app failed
        except:
             print("Error during final cleanup.")
        sys.exit(1)

