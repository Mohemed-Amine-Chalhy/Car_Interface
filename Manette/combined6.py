

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, font as tkfont
import threading
import queue
import serial
import serial.tools.list_ports
import time
import pygame  # For Xbox controller AND sound
import sys
import math
from collections import deque
from rplidar import RPLidar
import colorsys # For HSV to RGB conversion for gradient

# --- Constants (Consolidated & Updated) ---
# Robot Control Defaults
DEFAULT_BAUD_RATE = 115200  # Default for ESP32

# Lidar Defaults
DEFAULT_LIDAR_PORT = 'COM6'  # Default LIDAR port (Adjust as needed)
DEFAULT_ARDUINO_PORT = 'COM3' # Default Arduino Port (Adjust as needed) - Used as initial suggestion
VEHICLE_WIDTH = 42  # Vehicle width in cm
OBSTACLE_THRESHOLD = 500  # Max distance to consider an obstacle (cm)
VIEW_ANGLE = 180  # Lidar view angle (degrees) - Front facing
MAX_DISTANCE = 2000  # Maximum distance to display (in cm) - Should be larger than OBSTACLE_THRESHOLD
HISTORY_SIZE = 5  # Number of Lidar frames for stability
AUTO_BRAKE_THRESHOLD = 50 # Distance threshold for auto-braking (in cm)

# Sound Defaults (NEW)
BEEP_SOUND_FILE = 'beep.wav' # Name of your beep sound file
BEEP_MAX_DISTANCE = 150     # Start beeping when obstacle is within this distance (cm)
BEEP_MIN_DISTANCE = 25      # Beep fastest at this distance or closer (cm)
BEEP_MAX_INTERVAL = 0.8     # Longest interval between beeps (seconds)
BEEP_MIN_INTERVAL = 0.1     # Shortest interval between beeps (seconds)

# Colors (Updated)
WHITE = "#FFFFFF"
BLACK = "#000000"
RED = "#FF0000"
GREEN = "#00FF00"
BLUE = "#0000FF"
YELLOW = "#FFFF00"
DARK_GRAY = "#303030" # Slightly lighter dark gray for grid
LIGHT_GRAY = "#808080" # Lighter gray for far scan points
OBSTACLE_COLOR_PATH = RED     # Obstacles in path
OBSTACLE_COLOR_OFFPATH = YELLOW # Obstacles near, but off path
CLOSEST_POINT_COLOR = GREEN
VEHICLE_COLOR = BLUE
BRAKE_ARC_COLOR = RED
# Color gradient (Hue: Red=0, Yellow=60, Green=120) - Using Hue for smooth transition
COLOR_GRADIENT_HUE_START = 0   # Hue for closest points (Red)
COLOR_GRADIENT_HUE_END = 120 # Hue for farthest points (Green)


# --- Arduino Communicator Class (No changes needed) ---
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


# --- Xbox Controller Class (No changes needed) ---
class XboxController:
    def __init__(self, controller_id=0, log_callback=None):
        # pygame.init() # Initialize Pygame subsystems - MOVED to CombinedRobotApp.__init__
        # pygame.joystick.init() # Initialize Joystick subsystem specifically - MOVED

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
        self.STEERING_MIN = 0
        self.STEERING_MAX = 3590
        self.STEERING_CENTER = (self.STEERING_MAX + self.STEERING_MIN) // 2
        self.ACCEL_MAX = 50

        # Cache last sent values
        self.last_sent_steering = self.STEERING_CENTER
        self.last_sent_accel = 0

        # self.connect() # Connection attempt moved to main app

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
        # Check if joystick subsystem is initialized
        if not pygame.joystick.get_init():
             self.log_callback("Joystick subsystem not initialized. Initializing...", is_warning=True)
             pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count <= self.controller_id:
            self.log_callback(f"Controller ID {self.controller_id} not found (Count: {joystick_count}).", is_warning=True)
            self.connected = False
            return False
        try:
            # Quit previous instance if exists
            if self.controller and self.controller.get_init():
                self.controller.quit()

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
            self.lt_pressed_state = self.lt_value > 0.5 # Check initial state
            self.last_sent_steering = self.STEERING_CENTER
            self.last_sent_accel = 0
            return True
        except pygame.error as e:
            self.log_callback(f"Pygame error connecting controller: {e}", is_error=True)
            self.connected = False; self.controller = None; self.controller_name = "None"; return False
        except Exception as e:
            self.log_callback(f"Unexpected error connecting controller: {e}", is_error=True)
            self.connected = False; self.controller = None; self.controller_name = "None"; return False

    def disconnect(self):
        if self.controller:
            try: self.controller.quit()
            except Exception as e: self.log_callback(f"Error during controller quit: {e}", is_error=True)
        self.connected = False; self.controller = None; self.controller_name = "None"
        self.log_callback("Controller disconnected.")


    def update(self):
        """Pumps Pygame events, reads controller state, handles D-pad and X button."""
        if not self.connected or not self.controller or not self.controller.get_init():
             # Attempt reconnect if joystick seems available? Or just signal failure.
             if pygame.joystick.get_count() > self.controller_id:
                 self.log_callback("Controller connection lost, attempting reconnect...", is_warning=True)
                 if self.connect(): return True, None # Reconnected, proceed next cycle
                 else: return False, None # Reconnect failed
             else:
                 self.disconnect() # Ensure state is clean
                 return False, None # No joysticks available


        event_occurred = None # Track primary event (LT, DPad, X)
        dpad_input_changed = False

        try:
            # Pump Pygame events FIRST
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.log_callback("Pygame quit event received.")
                    # This event should be handled by the main application loop ideally
                    return False, "QUIT" # Signal app to handle quit?

                elif event.type == pygame.JOYDEVICEADDED and event.instance_id == self.controller_id:
                    self.log_callback("Controller (re)detected by event.")
                    # Re-initialize if it's not initialized
                    if not self.controller or not self.controller.get_init():
                        self.connect()

                elif event.type == pygame.JOYDEVICEREMOVED and event.instance_id == self.controller_id:
                    self.log_callback("Controller disconnected by event.", is_warning=True)
                    self.disconnect()
                    return False, None

                elif event.type == pygame.JOYBUTTONDOWN:
                    button = event.button
                    # self.log_callback(f"DEBUG: Button {button} Pressed") # Keep for debugging if needed
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

                elif event.type == pygame.JOYAXISMOTION:
                    # Axis events are frequent, we read them directly below anyway
                    pass

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
            # Check dpad_input_changed first (covers both HAT and BUTTON modes)
            if event_occurred != "X_PRESSED" and dpad_input_changed and current_dpad_tuple != self.last_dpad_tuple:
                 event_occurred = "HAT_CHANGED" # Reuse name for simplicity
                 self.last_dpad_tuple = current_dpad_tuple

            # LT Press/Release Event (check based on threshold crossing)
            lt_currently_pressed = self.lt_value > 0.5 # Use a threshold, raw axis > 0 not reliable
            if lt_currently_pressed != self.lt_pressed_state:
                if event_occurred is None: # Only prioritize LT if X/DPad didn't happen
                     event_occurred = "LT_PRESSED" if lt_currently_pressed else "LT_RELEASED"
                self.lt_pressed_state = lt_currently_pressed # Update state regardless


            return True, event_occurred # Success, optional event detected

        except pygame.error as e:
            self.log_callback(f"Pygame error reading controller: {e}. Disconnecting.", is_error=True)
            self.disconnect()
            return False, None
        except Exception as e:
            self.log_callback(f"Unexpected error reading controller: {e}. Disconnecting.", is_error=True)
            self.disconnect()
            return False, None

    def get_hat_values(self):
        """Returns the current D-pad state as a tuple (x, y), compatible with hat output."""
        if not self.connected or not self.controller or not self.controller.get_init():
            return (0,0)

        if self.USE_HAT_DPAD:
            try:
                if self.controller.get_numhats() > self.HAT_DPAD_XBOX:
                    return self.controller.get_hat(self.HAT_DPAD_XBOX)
                else: return (0, 0)
            except pygame.error: # Handle case where controller disconnects between checks
                self.log_callback("Pygame error getting hat value.", is_warning=True)
                self.disconnect()
                return (0,0)
        else:
            # Check button states directly
            dpad_x = 0; dpad_y = 0
            try:
                if self.dpad_button_state['right']: dpad_x = 1
                elif self.dpad_button_state['left']: dpad_x = -1
                if self.dpad_button_state['up']: dpad_y = 1
                elif self.dpad_button_state['down']: dpad_y = -1
                return (dpad_x, dpad_y)
            except Exception as e: # General catch if state dictionary is bad somehow
                 self.log_callback(f"Error reading D-Pad button state: {e}", is_error=True)
                 return (0,0)

    def get_steering_value(self):
        """Maps right stick X (-1 to 1) to steering value (e.g., 0-3590) with deadzone."""
        raw_value = self.right_stick_x
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
        # RT axis range is often -1 (released) to 1 (fully pressed)
        normalized = (raw_value + 1) / 2 # Map to 0..1 range
        if normalized < self.DEADZONE_TRIGGER: return 0
        # Scale considering deadzone
        scaled_value = (normalized - self.DEADZONE_TRIGGER) / (1 - self.DEADZONE_TRIGGER)
        accel = int(scaled_value * self.ACCEL_MAX)
        return max(0, min(self.ACCEL_MAX, accel))

    def has_steering_changed(self):
        current_steering = self.get_steering_value()
        # Threshold to prevent sending commands for tiny fluctuations
        if abs(current_steering - self.last_sent_steering) > 5: # Adjust sensitivity if needed
            self.last_sent_steering = current_steering; return True
        # Also update if it returns to center from non-center
        if current_steering == self.STEERING_CENTER and self.last_sent_steering != self.STEERING_CENTER:
             self.last_sent_steering = current_steering; return True
        return False

    def has_accel_changed(self):
        current_accel = self.get_acceleration_value()
        if current_accel != self.last_sent_accel:
            self.last_sent_accel = current_accel; return True
        return False

    def is_connected(self):
        try:
            # Re-check pygame and joystick status
            if not pygame.get_init() or not pygame.joystick.get_init(): return False
            if not self.connected or not self.controller or not self.controller.get_init(): return False
            # Verify the controller instance ID still matches (less critical but safer)
            # And check the joystick count
            if pygame.joystick.get_count() <= self.controller_id: return False
            # Attempt a simple call to check if it raises an error
            self.controller.get_name()
            return True
        except pygame.error:
            # If a pygame error occurs, it's likely disconnected
            # self.disconnect() # Let the update loop handle full disconnect logic
            return False
        except AttributeError: # If self.controller is None
            return False


# --- Lidar Processor Class (No changes needed) ---
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

    def connect(self):
        if self.lidar and self.running:
             self.log_callback("Lidar already connected and running.", is_warning=True)
             return True # Already connected?

        try:
            self.log_callback(f"Attempting to connect to LIDAR on {self.port_name}...")
            # Ensure any previous instance is properly closed
            if self.lidar:
                try:
                     self.lidar.stop()
                     self.lidar.stop_motor()
                     self.lidar.disconnect()
                except: pass # Ignore errors disconnecting old instance
            self.lidar = RPLidar(self.port_name, timeout=3) # Increase timeout?
            # Test connection by getting info
            info = self.lidar.get_info()
            self.log_callback(f"LIDAR Info: {info}")
            health = self.lidar.get_health()
            self.log_callback(f"LIDAR Health: {health}")
            if health[0] != 'Good': # Check for 'Good' status specifically
                 raise Exception(f"Lidar health status not 'Good': {health}")
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
            # Clear old data on start
            self.history.clear()
            with self.data_lock:
                 self._scan_data = {}
                 self._obstacle_points = []
                 self._closest_distance = float('inf')
                 self._closest_angle = 0

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
        scan_iterator = None
        try:
            scan_iterator = self.lidar.iter_scans(max_buf_meas=5000, min_len=5)
            for scan in scan_iterator:
                if not self.running:
                    break # Exit loop if stopped

                current_scan_data = {}
                current_obstacle_points = []
                min_dist_in_scan = float('inf')
                min_angle_in_scan = 0

                # --- Process one full 360 scan ---
                # scan is a list of tuples: (quality, angle, distance)
                for quality, angle, distance in scan:
                    # Filter low quality points? (e.g., if quality > 10)
                    # if quality == 0: continue # RPLidar A1 gives 0 for bad points

                    dist_cm = distance / 10.0 # Convert mm to cm

                    # **** Assume Lidar's 90 degrees IS the robot's front ****
                    robot_angle = angle - 90
                    # Normalize to -180 to 180
                    if robot_angle < -180: robot_angle += 360
                    elif robot_angle > 180: robot_angle -= 360

                    # Store adjusted angle and distance (only if within reasonable range)
                    if 0 < dist_cm <= MAX_DISTANCE * 1.1 : # Store slightly beyond max for gradient
                        current_scan_data[robot_angle] = dist_cm

                    # --- Obstacle Detection within VIEW_ANGLE and THRESHOLD ---
                    # Use robot_angle for view check
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
                    # Update overall closest distance (might be outside path)
                    # self._closest_distance_overall = min((d for d in current_scan_data.values()), default=float('inf'))

                    # Update closest *in-path* distance
                    if min_dist_in_scan < float('inf'):
                        self._closest_distance = min_dist_in_scan # This is the critical one for braking/beeping
                        self._closest_angle = min_angle_in_scan
                    else:
                         # If no obstacles *in path* this scan, keep the last known angle, but maybe reset distance?
                         # Or, average from history. Let's use history for stability.
                         pass # Rely on get_stable_data for calculation

                    # Add current state to history
                    self.history.append({
                        'obstacles': current_obstacle_points, # Might be memory intensive?
                        'closest_dist_inpath': min_dist_in_scan, # Store in-path min for this scan
                        'closest_angle_inpath': min_angle_in_scan
                        })

        except serial.SerialException as se:
             self.log_callback(f"Serial Error during Lidar scan: {se}. Stopping scan.", is_error=True)
             self.running = False # Ensure running flag is false so stop_scan works cleanly
        except Exception as e:
            self.log_callback(f"Error in Lidar scan loop: {e}", is_error=True)
            # import traceback # Debugging
            # traceback.print_exc() # Debugging
            self.running = False # Stop on unexpected errors
        finally:
            # Ensure lidar.stop() is called to clean up iterator resources
            if hasattr(self.lidar, 'stop'): # Check if method exists
                try:
                    self.lidar.stop()
                except Exception as e:
                     self.log_callback(f"Minor error calling lidar.stop(): {e}", is_warning=True)

            # Ensure motor is stopped if the thread exits unexpectedly
            if self.lidar and self.running: # Check if we exited due to error while 'running' was true
                 self.log_callback("Scan loop terminated unexpectedly. Attempting to stop motor.", is_warning=True)
                 try:
                     self.lidar.stop_motor()
                 except: pass
            self.running = False # Explicitly set running to false
            self.log_callback("Lidar scan loop finished.")


    def get_stable_data(self):
        """Returns the latest scan data, obstacle points, and a stabilized closest distance/angle."""
        with self.data_lock:
            # Use the most recent full scan data for visualization
            latest_scan = self._scan_data.copy() # Return copies
            latest_obstacles = list(self._obstacle_points)

            # Calculate stable closest distance *within path* from history
            valid_distances = [h['closest_dist_inpath'] for h in self.history if h['closest_dist_inpath'] < float('inf')]
            if valid_distances:
                # Use average or median? Average is simpler. Median less sensitive to outliers.
                # stable_closest_dist = sum(valid_distances) / len(valid_distances)
                stable_closest_dist = sorted(valid_distances)[len(valid_distances)//2] # Median

                # Find the angle corresponding to the *most recent* minimum distance in history
                # Find the history entry matching the median (or closest to it if average used)
                # Or simpler: use the angle from the latest scan that had an in-path obstacle?
                last_valid_angle = 0
                for h in reversed(self.history):
                    if h['closest_dist_inpath'] < float('inf'):
                        last_valid_angle = h['closest_angle_inpath']
                        break
                stable_closest_angle = last_valid_angle
            else:
                # If no recent valid distances, use the absolute latest value even if inf
                stable_closest_dist = self._closest_distance
                stable_closest_angle = self._closest_angle

            return latest_scan, latest_obstacles, stable_closest_dist, stable_closest_angle


# --- Lidar Visualizer Frame (Adapted & Enhanced) ---
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

        # Sound State (NEW)
        self.beep_sound = None
        self.last_beep_time = 0
        self._load_beep_sound() # Attempt to load sound on init

        # UI Elements specific to Lidar Tab
        self._create_widgets()
        self._setup_canvas()

    def _load_beep_sound(self):
        """Attempts to load the beep sound file."""
        if not pygame.mixer.get_init():
            self.log_callback("Initializing Pygame mixer for sound...", is_info=True)
            try:
                pygame.mixer.init() # Initialize if not already done
            except pygame.error as e:
                self.log_callback(f"Failed to initialize Pygame mixer: {e}", is_error=True)
                return

        try:
            self.beep_sound = pygame.mixer.Sound(BEEP_SOUND_FILE)
            self.log_callback(f"Beep sound '{BEEP_SOUND_FILE}' loaded successfully.", is_success=True)
        except pygame.error as e:
            self.log_callback(f"Warning: Could not load beep sound '{BEEP_SOUND_FILE}'. Pygame error: {e}", is_warning=True)
            self.beep_sound = None
        except FileNotFoundError:
            self.log_callback(f"Warning: Beep sound file not found: '{BEEP_SOUND_FILE}'. Place it in the script directory.", is_warning=True)
            self.beep_sound = None
        except Exception as e:
             self.log_callback(f"Warning: An unexpected error occurred loading beep sound '{BEEP_SOUND_FILE}': {e}", is_warning=True)
             self.beep_sound = None

    def _create_widgets(self):
        # Frame for Lidar Controls
        controls_frame = ttk.LabelFrame(self, text="LIDAR Control")
        controls_frame.pack(pady=5, padx=10, fill=tk.X)
        controls_frame.columnconfigure(2, weight=1) # Allow spacing column to expand

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
        assist_frame = ttk.LabelFrame(self, text="Lidar Assist & Feedback")
        assist_frame.pack(pady=5, padx=10, fill=tk.X)
        assist_frame.columnconfigure(3, weight=1) # Allow spacer column to expand


        # Stop Assist Checkbox
        self.stop_assist_var = tk.BooleanVar(value=False)
        self.stop_assist_checkbox = ttk.Checkbutton(
            assist_frame,
            text="Enable Auto-Stop",
            variable=self.stop_assist_var,
            command=self.on_stop_assist_changed,
            state=tk.DISABLED # Enabled when Arduino is connected
        )
        self.stop_assist_checkbox.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # Threshold entry for auto-braking
        ttk.Label(assist_frame, text="Brake Thresh (cm):").grid(row=0, column=1, padx=(10, 2), pady=5, sticky=tk.W)
        self.threshold_var = tk.StringVar(value=str(AUTO_BRAKE_THRESHOLD))
        # Add validation command
        vcmd = (self.register(self._validate_threshold), '%P') # Pass prospective value
        self.threshold_entry = ttk.Entry(assist_frame, textvariable=self.threshold_var, width=6, validate='key', validatecommand=vcmd)
        self.threshold_entry.grid(row=0, column=2, padx=(0, 5), pady=5, sticky=tk.W)

        # Beep Enable Checkbox (NEW)
        self.beep_enabled_var = tk.BooleanVar(value=False)
        self.beep_checkbox = ttk.Checkbutton(
            assist_frame,
            text="Enable Proximity Beep",
            variable=self.beep_enabled_var,
            state=tk.NORMAL if self.beep_sound else tk.DISABLED # Enable only if sound loaded
        )
        self.beep_checkbox.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W, columnspan=3)

        # Manual Brake/Release Buttons (moved right)
        self.manual_brake_button = ttk.Button(
             assist_frame,
             text="Manual Brake (S 1)",
             command=lambda: self.arduino_communicator.send_command("S 1"),
             state=tk.DISABLED
        )
        self.manual_brake_button.grid(row=0, column=4, padx=15, pady=5, sticky="ew")

        self.manual_release_button = ttk.Button(
             assist_frame,
             text="Manual Release (Q 1)",
             command=lambda: self.arduino_communicator.send_command("Q 1"),
             state=tk.DISABLED
        )
        self.manual_release_button.grid(row=1, column=4, padx=15, pady=5, sticky="ew")


        # Lidar Canvas Frame
        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(5,10))

        # Canvas for Lidar Display
        self.canvas = tk.Canvas(canvas_frame, width=600, height=450, bg=BLACK, # Increased height slightly
                                relief=tk.SUNKEN, borderwidth=1, highlightthickness=0) # Remove highlight border
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Bind resize event to canvas frame or canvas itself
        self.canvas.bind("<Configure>", self.on_canvas_resize)


    def _validate_threshold(self, P):
        """Validate threshold entry to allow only numbers."""
        if P == "" or (P.isdigit() and 0 < int(P) <= OBSTACLE_THRESHOLD):
            # If valid or empty, redraw the brake arc
            # Use after(1) to avoid issues during validation itself
            self.after(1, self.draw_reference_elements)
            return True
        else:
            return False

    def on_canvas_resize(self, event):
        """Handle canvas resize events to redraw reference elements."""
        # Schedule redraw to avoid excessive calls during resize drag
        if hasattr(self, '_resize_job_canvas'):
            self.after_cancel(self._resize_job_canvas)
        self._resize_job_canvas = self.after(200, self._setup_canvas) # Delay redraw slightly


    def _setup_canvas(self):
         # Define fonts
        self.font = tkfont.Font(family="Arial", size=11, weight="bold")
        self.small_font = tkfont.Font(family="Arial", size=9)

        # Get current canvas dimensions
        # self.canvas.update_idletasks() # Ensure dimensions are calculated - Configure event handles this
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width <= 1 or canvas_height <= 1: # Initial setup might have small size
             # print("Canvas size 0, skipping setup")
             return # Don't draw if canvas isn't ready


        # Center of screen (vehicle position) - near bottom center
        self.center_x = canvas_width // 2
        self.center_y = canvas_height - 50 # 50 pixels from bottom

        # Scale factor based on canvas size and max distance
        # Fit MAX_DISTANCE vertically (considering space at bottom) or horizontally
        drawable_height = canvas_height - 60 # Space for vehicle/text at bottom
        drawable_width = canvas_width - 20 # Add some padding on sides
        # Scale to fit MAX_DISTANCE within the drawable area
        self.scale = min(drawable_width / (MAX_DISTANCE * 2), drawable_height / MAX_DISTANCE)
        self.scale *= 0.95 # Use 95% of scale for margin


        # Text elements - Create or update
        if hasattr(self, 'distance_text_id'):
             # Reposition and reset text
             self.canvas.coords(self.distance_text_id, 10, 10)
             self.canvas.itemconfig(self.distance_text_id, text="Closest: -- cm @ --°")
        else:
             # Create new text item
             self.distance_text_id = self.canvas.create_text(10, 10, anchor=tk.NW,
                                                        text="Closest: -- cm @ --°", # Show angle too
                                                        fill=WHITE, font=self.font, tags="info_text")
        if hasattr(self, 'warning_text_id'):
             # Reposition and reset text
             self.canvas.coords(self.warning_text_id, 10, 35)
             self.canvas.itemconfig(self.warning_text_id, text="", fill=RED)
        else:
             # Create new text item
             self.warning_text_id = self.canvas.create_text(10, 35, anchor=tk.NW,
                                                       text="", fill=RED, font=self.font, tags="info_text")

        # Vehicle info text (bottom) - create or update
        vehicle_info_text = f"Veh Width: {VEHICLE_WIDTH} cm | View: {VIEW_ANGLE}° | Max Dist: {MAX_DISTANCE} cm"
        if hasattr(self, 'vehicle_info_id'):
             self.canvas.coords(self.vehicle_info_id, self.center_x, canvas_height - 5, anchor=tk.S)
             self.canvas.itemconfig(self.vehicle_info_id, text=vehicle_info_text)
        else:
             self.vehicle_info_id = self.canvas.create_text(self.center_x, canvas_height - 5, anchor=tk.S,
                                                        text=vehicle_info_text,
                                                        fill=WHITE, font=self.small_font, tags="info_text")


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
        # Don't disconnect Lidar device here, just stop scanning
        # Disconnect is handled on app close or if connection fails

        self.lidar_running = False
        self.activate_button.config(state=tk.NORMAL)
        self.deactivate_button.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.NORMAL)

        # Clear canvas elements related to scan data
        self.canvas.delete("scan", "obstacle", "closest")
        self.canvas.itemconfig(self.distance_text_id, text="Closest: -- cm @ --°")
        self.canvas.itemconfig(self.warning_text_id, text="")
        self.log_callback("LIDAR deactivated.")


    # --- Canvas Update Methods ---
    def start_display_update(self):
        if self.update_id is None:
            self.log_callback("Lidar display update loop starting.")
            self.update_display() # Call immediately once
        else:
             self.log_callback("Lidar display update loop already running.", is_warning=True)

    def stop_display_update(self):
        if self.update_id is not None:
            self.after_cancel(self.update_id)
            self.update_id = None
            self.log_callback("Lidar display update loop stopped.")

    def _get_distance_color(self, distance_cm):
        """Calculates a color based on distance using HSV gradient."""
        # Normalize distance (0 = close, 1 = max_dist)
        # Cap distance at MAX_DISTANCE for color calculation
        norm_dist = min(1.0, max(0.0, distance_cm / MAX_DISTANCE))

        # Interpolate Hue (Green=120, Red=0) -> Closer = lower Hue
        hue = (COLOR_GRADIENT_HUE_END - (norm_dist * (COLOR_GRADIENT_HUE_END - COLOR_GRADIENT_HUE_START))) / 360.0
        # Use high saturation and value for visibility
        saturation = 1.0
        value = 1.0 # Brightness

        # Convert HSV to RGB
        rgb = colorsys.hsv_to_rgb(hue, saturation, value)

        # Convert RGB (0-1 floats) to Hex string #RRGGBB
        hex_color = "#{:02x}{:02x}{:02x}".format(int(rgb[0]*255), int(rgb[1]*255), int(rgb[2]*255))
        return hex_color

    def update_display(self):
        # Reschedule first, so it continues even if errors occur below
        # Use 'self.after' instead of 'self.master.after' to tie it to the frame widget
        if self.lidar_running:
             self.update_id = self.after(100, self.update_display) # Update ~10 FPS
        else:
             self.update_id = None # Ensure loop stops if lidar stops externally
             return

        # Get latest data from processor
        try:
             scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()
        except Exception as e:
             self.log_callback(f"Error getting stable data from Lidar processor: {e}", is_error=True)
             return # Skip update cycle on error

        # Handle Proximity Beep (moved before braking logic for immediate feedback)
        self.handle_proximity_beep(closest_distance)

        # Handle Auto-Stop Assist Logic (triggers E-Stop if needed)
        self.handle_auto_stop_assist(closest_distance)

        # --- Drawing on Canvas ---
        self.canvas.delete("scan", "obstacle", "closest") # Clear dynamic elements

        # Draw general scan points with color gradient
        for angle, distance in scan_data.items():
            # Only draw within the defined VIEW_ANGLE
            if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2:
                # Check if this point is already marked as an obstacle (to avoid double drawing)
                # is_obstacle = any(abs(o[0] - angle) < 0.1 and abs(o[1] - distance) < 0.1 for o in obstacle_points) # Tolerance check
                # if not is_obstacle: # Draw only if not already drawn as specific obstacle
                    x, y = self._polar_to_cartesian(angle, distance)
                    color = self._get_distance_color(distance)
                    # Draw small dots for scan points
                    self.canvas.create_oval(x-1, y-1, x+1, y+1, fill=color, outline="", tags=("scan",)) # Tuple of tags


        # Draw obstacles (potentially overriding some scan points)
        for angle, distance, in_path in obstacle_points:
             # Draw only within the defined VIEW_ANGLE (redundant check, but safe)
             if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2:
                x, y = self._polar_to_cartesian(angle, distance)
                color = OBSTACLE_COLOR_PATH if in_path else OBSTACLE_COLOR_OFFPATH
                size = 4 if in_path else 3 # Make in-path obstacles slightly larger
                self.canvas.create_oval(x-size, y-size, x+size, y+size, fill=color, outline="", tags=("obstacle",))


        # Display closest distance and warning text
        if closest_distance < float('inf'):
            # Draw line to closest obstacle *within the path*
            x_close, y_close = self._polar_to_cartesian(closest_angle, closest_distance)
            self.canvas.create_line(self.center_x, self.center_y, x_close, y_close,
                                    fill=CLOSEST_POINT_COLOR, width=2, arrow=tk.LAST, tags="closest")

            # Update distance text
            self.canvas.itemconfig(self.distance_text_id, text=f"Closest: {closest_distance:.1f} cm @ {closest_angle:.1f}°")

            # --- Update warning text based on threshold and assist status ---
            try:
                threshold = float(self.threshold_var.get())
            except ValueError:
                threshold = AUTO_BRAKE_THRESHOLD # Fallback

            # Update warning text (uses main_app state)
            if closest_distance < threshold:
                is_braking = self.main_app.brake_active
                brake_reason = self.main_app.brake_reason

                if is_braking and brake_reason == "E-Stop":
                     # E-Stop is active (could be manual or auto)
                     assist_enabled = self.stop_assist_var.get()
                     if assist_enabled:
                          # If E-Stop is active AND assist is ON, it was likely triggered by assist
                          self.canvas.itemconfig(self.warning_text_id, text="AUTO E-STOP ACTIVE!", fill=RED)
                     else:
                          # E-Stop active, but assist is OFF (must have been manual E-Stop click)
                          self.canvas.itemconfig(self.warning_text_id, text="MANUAL E-STOP ACTIVE!", fill=RED)
                elif is_braking:
                    # Brake is active for other reasons (Manual button, Controller LT)
                    self.canvas.itemconfig(self.warning_text_id, text=f"BRAKE ACTIVE ({brake_reason})!", fill=YELLOW) # Show specific reason
                else:
                    # Not braking, but below threshold -> Advise action
                    self.canvas.itemconfig(self.warning_text_id, text="E-STOP ADVISED!", fill=YELLOW)
            else:
                # Distance is greater than threshold, clear warning
                self.canvas.itemconfig(self.warning_text_id, text="")
        else:
            # No valid closest distance *in path* found
            self.canvas.itemconfig(self.distance_text_id, text="Closest: -- cm @ --°")
            self.canvas.itemconfig(self.warning_text_id, text="")

        # Ensure info text is on top
        self.canvas.tag_raise("info_text")


    def draw_reference_elements(self):
        """Draws grid lines, vehicle shape, path lines, and brake arc."""
        # Clear previous reference elements
        self.canvas.delete("reference")

        # Check canvas dimensions again (might be called before fully configured)
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        if canvas_width <= 1 or canvas_height <= 1 or self.scale <= 0:
             # print("Canvas not ready for reference drawing")
             return # Avoid drawing if canvas not ready or scale is invalid

        # Update center based on current size (scale calculated in _setup_canvas)
        self.center_x = canvas_width // 2
        self.center_y = canvas_height - 50

        # Draw distance rings (arcs for limited view angle)
        for r_cm in range(50, int(MAX_DISTANCE) + 1, 100): # Every 100cm for less clutter
            radius_px = int(r_cm * self.scale)
            if radius_px < 5: continue # Don't draw tiny rings

            # Calculate start angle and extent based on robot's forward direction (0 deg) for Tkinter arc
            # Tkinter angles: 0=East(3 o'clock), 90=North(12 o'clock), 180=West(9 o'clock), 270=South(6 o'clock)
            # Robot front (0 deg) corresponds to Tkinter North (90 deg)
            # Robot right (+90 deg) corresponds to Tkinter East (0 deg)
            # Robot left (-90 deg) corresponds to Tkinter West (180 deg)
            start_angle_tk = 90 - (-VIEW_ANGLE / 2) # Angle of the right edge of view
            extent_tk = -VIEW_ANGLE # Sweep angle (negative for clockwise)

            self.canvas.create_arc(
                self.center_x - radius_px, self.center_y - radius_px,
                self.center_x + radius_px, self.center_y + radius_px,
                start=start_angle_tk, extent=extent_tk,
                outline=DARK_GRAY, width=1, style=tk.ARC, tags=("reference",)
            )

            # Distance label (only draw if space permits)
            if radius_px > 15:
                 # Position label slightly above the ring on the center line (y-axis)
                 self.canvas.create_text(self.center_x, self.center_y - radius_px - 2, # Nudge up slightly
                                     text=f"{r_cm}cm", fill=LIGHT_GRAY, anchor=tk.S,
                                     font=self.small_font, tags=("reference",))

        # Draw angle reference lines (e.g., every 30 degrees within VIEW_ANGLE)
        half_view = VIEW_ANGLE / 2
        # Ensure 0 degree line is always drawn if view angle allows
        angles_to_draw = set(range(0, int(half_view) + 1, 30)) | set(range(0, int(-half_view) - 1, -30))
        if 0 <= half_view: angles_to_draw.add(0)

        for angle in sorted(list(angles_to_draw)):
            if abs(angle) > half_view : continue # Ensure within view

            # Calculate end point at max distance for the line
            x_end, y_end = self._polar_to_cartesian(angle, MAX_DISTANCE)
            # Draw line from center
            self.canvas.create_line(self.center_x, self.center_y, x_end, y_end,
                                    fill=DARK_GRAY, dash=(2, 4), tags=("reference",))
            # Angle label near the end of the line
            # Avoid label overlap at 0 degrees with distance label
            if radius_px > 15 and abs(angle) > 5: # Check radius for distance label space
                 x_lbl, y_lbl = self._polar_to_cartesian(angle, MAX_DISTANCE * 1.05) # Slightly outside max dist
                 self.canvas.create_text(x_lbl, y_lbl, text=f"{angle}°", fill=LIGHT_GRAY,
                                        font=self.small_font, anchor=tk.CENTER, tags=("reference",))

        # Draw vehicle representation (simple rectangle + arrow)
        vehicle_width_px = int(VEHICLE_WIDTH * self.scale)
        half_width_px = max(3, vehicle_width_px // 2) # Ensure min width of 6px
        vehicle_height_px = max(5, int(25 * self.scale)) # Approx 25cm long vehicle representation
        self.canvas.create_rectangle(
            self.center_x - half_width_px, self.center_y - (vehicle_height_px * 0.4), # Shifted slightly forward
            self.center_x + half_width_px, self.center_y + (vehicle_height_px * 0.6),
            fill=VEHICLE_COLOR, outline=WHITE, width=1, tags=("reference", "vehicle")
        )
        # Add arrow pointing forward
        arrow_len = vehicle_height_px * 0.6
        arrow_tip_y = self.center_y - arrow_len
        self.canvas.create_line(self.center_x, self.center_y, self.center_x, arrow_tip_y,
                               fill=WHITE, width=max(1, int(self.scale*2)), arrow=tk.LAST, tags=("reference", "vehicle"))

        # Draw vehicle path width lines
        path_half_width_px = int((VEHICLE_WIDTH / 2) * self.scale)
        # Calculate end points for path lines
        _, y_end_path = self._polar_to_cartesian(0, MAX_DISTANCE) # Y coordinate at max distance forward

        # Left path line
        self.canvas.create_line(self.center_x - path_half_width_px, self.center_y,
                                self.center_x - path_half_width_px, y_end_path,
                                fill=VEHICLE_COLOR, width=1, dash=(4, 4), tags=("reference",))
        # Right path line
        self.canvas.create_line(self.center_x + path_half_width_px, self.center_y,
                                self.center_x + path_half_width_px, y_end_path,
                                fill=VEHICLE_COLOR, width=1, dash=(4, 4), tags=("reference",))


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
                    style=tk.ARC, outline=BRAKE_ARC_COLOR, width=2, dash=(6,3), tags=("reference", "brake_arc")
                )
                # Threshold label (Only if arc is reasonably large)
                if threshold_radius_px > 10:
                    self.canvas.create_text(
                        self.center_x, self.center_y - threshold_radius_px - 5, # Position above arc center
                        text=f"Stop: {threshold_cm:.0f}cm",
                        fill=BRAKE_ARC_COLOR, anchor=tk.S,
                        font=self.small_font, tags=("reference", "brake_arc")
                    )
        except ValueError:
            pass # Ignore if threshold is not a valid number yet

        # Raise vehicle representation above grid lines but below points? Optional.
        self.canvas.tag_raise("vehicle")


    def _polar_to_cartesian(self, angle_deg, distance_cm):
        """Converts robot-centric polar coordinates (angle 0=forward) to canvas coordinates."""
        if self.scale <= 0: return self.center_x, self.center_y # Avoid division by zero if scale not ready
        # Convert robot angle (0=fwd) to math angle (0=East, 90=North)
        math_angle_rad = math.radians(90.0 - angle_deg)
        dist_px = distance_cm * self.scale

        x = self.center_x + dist_px * math.cos(math_angle_rad)
        y = self.center_y - dist_px * math.sin(math_angle_rad) # Y is inverted in Tkinter canvas
        return int(x), int(y)

    # --- Event Handlers and State Updates ---
    def update_arduino_connection_state(self, connected):
        """Enable/disable Lidar assist controls based on Arduino connection."""
        state = tk.NORMAL if connected else tk.DISABLED
        self.stop_assist_checkbox.config(state=state)
        self.threshold_entry.config(state=state) # Keep threshold entry always editable? Or disable? Let's disable.
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
            self.log_callback("Lidar Auto-Stop Assist ENABLED.")
        else:
            self.log_callback("Lidar Auto-Stop Assist DISABLED.")
            # Force warning text clear if it was showing auto-brake message? Maybe not, let distance check handle it.
            # if "AUTO E-STOP ACTIVE!" in self.canvas.itemcget(self.warning_text_id, "text"):
            #    self.canvas.itemconfig(self.warning_text_id, text="")


    def handle_auto_stop_assist(self, closest_distance_in_path):
        """
        Checks if an obstacle *in path* is too close and assist is active.
        If so, triggers the main application's emergency_stop function.
        """
        if not self.main_app.stop_assist_active or not self.arduino_communicator.is_connected:
            return

        try:
            threshold = float(self.threshold_var.get())
        except ValueError:
            threshold = AUTO_BRAKE_THRESHOLD

        # --- Auto-E-Stop Logic ---
        # Check if closest distance is valid (not inf) and below threshold
        if 0 < closest_distance_in_path < threshold:
             # Obstacle is too close *in the path*
             # Check if brake is ALREADY active due to E-Stop to prevent spamming logs/commands
             if not (self.main_app.brake_active and self.main_app.brake_reason == "E-Stop"):
                 self.log_callback(f"Auto-Stop Assist: Obstacle in path at {closest_distance_in_path:.1f}cm (Threshold: {threshold}cm). Triggering EMERGENCY STOP.", is_error=True)
                 # Call main app's emergency_stop
                 self.main_app.emergency_stop() # Handles sending commands and setting state
             # else: Optional debug log
                 # print(f"Debug: Auto-Stop condition persists {closest_distance_in_path:.1f}cm, E-Stop already active.")

    def handle_proximity_beep(self, closest_distance_in_path):
        """Plays a beep sound with frequency based on the closest obstacle distance *in path*."""
        if not self.beep_enabled_var.get() or not self.beep_sound or not self.lidar_running:
             return

        # Check if distance is valid and within beeping range
        if 0 < closest_distance_in_path <= BEEP_MAX_DISTANCE:
            # Calculate beep interval (linear interpolation)
            if closest_distance_in_path <= BEEP_MIN_DISTANCE:
                 interval = BEEP_MIN_INTERVAL
            else:
                 # Normalize distance within the beeping range (0 at max dist, 1 at min dist)
                 norm_dist = (BEEP_MAX_DISTANCE - closest_distance_in_path) / (BEEP_MAX_DISTANCE - BEEP_MIN_DISTANCE)
                 norm_dist = max(0.0, min(1.0, norm_dist)) # Clamp
                 # Interpolate interval (shorter interval for higher norm_dist)
                 interval = BEEP_MAX_INTERVAL - norm_dist * (BEEP_MAX_INTERVAL - BEEP_MIN_INTERVAL)

            # Check if enough time has passed since the last beep
            now = time.time()
            if now - self.last_beep_time >= interval:
                 try:
                     self.beep_sound.play()
                     self.last_beep_time = now
                 except pygame.error as e:
                     self.log_callback(f"Error playing beep sound: {e}", is_warning=True)
                     # Disable beeping temporarily? Or just log error.
        # else: No obstacle in range, do nothing


# --- Main Application Class ---
class CombinedRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Robot Control & Lidar Interface")
        self.root.geometry("850x750") # Increased height slightly for Lidar view

        # --- Initialize Pygame Early (Crucial for Mixer) ---
        try:
            pygame.init() # Initializes all modules, including mixer and joystick
            self.log_message("Pygame initialized successfully.", is_success=True, console_only=True) # Log before console exists
        except Exception as e:
             print(f"FATAL: Pygame initialization failed: {e}") # Print critical error
             # Optionally show a popup and exit
             messagebox.showerror("Initialization Error", f"Pygame failed to initialize: {e}\nApplication cannot continue.")
             root.destroy()
             sys.exit(1)


        # --- Core Components ---
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.arduino_communicator = ArduinoCommunicator(self.command_queue, self.response_queue, self.log_message)
        self.xbox_controller = XboxController(log_callback=self.log_message) # Pygame already init
        self.lidar_processor = LidarProcessor(log_callback=self.log_message)

        # --- Shared State Variables ---
        self.arduino_connected = False
        self.is_automatic_mode = False # True = Auto, False = Manual (M command)
        self.brake_active = False # True = Brake applied (S 1), False = Released (Q 1)
        self.brake_reason = "" # Store why brake was activated (e.g., "Manual", "Auto-Stop", "E-Stop", "Controller LT")
        self.controller_polling_active = False # Is the Xbox controller actively sending commands?
        self.stop_assist_active = False # Is Lidar auto-braking enabled? (Controlled by Lidar tab checkbox)

        # --- UI Setup ---
        self._create_styles()
        self._create_main_ui() # Creates console, log_message is now safe

        # --- Start Background Tasks ---
        self.check_responses_id = self.root.after(50, self.check_responses) # Store ID
        self.poll_controller_id = self.root.after(50, self.poll_controller) # Store ID

        # --- Bind Resize Event ---
        # self.root.bind("<Configure>", self.on_resize) # Handle window resize (optional)

    def _create_styles(self):
        self.style = ttk.Style()
        try:
             self.style.theme_use('clam') # Or 'vista', 'xpnative' on Windows
        except tk.TclError:
             print("Clam theme not available, using default.")

        self.style.configure("Emergency.TButton", foreground="white", background="#CC0000", font=("Arial", 12, "bold")) # Darker Red
        self.style.map("Emergency.TButton",
                       background=[('active', '#FF4444')]) # Lighter red when pressed
        self.style.configure("TNotebook.Tab", padding=[10, 5], font=('Arial', 10))
        self.style.configure("TLabelframe.Label", font=('Arial', 10, 'bold'))
        # Style for indicating active state (e.g., controller polling)
        self.style.configure("Active.TButton", foreground="black", background="#A0FFA0") # Light green background
        self.style.map("Active.TButton", background=[('active', '#C0FFC0')])


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
        # Pass necessary components
        self.lidar_visualizer = LidarVisualizerFrame(
            self.lidar_frame,
            main_app=self, # Pass self (the main app instance)
            lidar_processor=self.lidar_processor,
            arduino_communicator=self.arduino_communicator,
            log_callback=self.log_message
        )
        self.lidar_visualizer.pack(fill=tk.BOTH, expand=True)

        # Status Bar at the bottom
        self.status_bar = ttk.Label(self.root, text="Status: Initializing...", relief=tk.SUNKEN, anchor=tk.W, padding=5)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        self.update_status("Ready") # Set initial ready status

    def _create_robot_control_tab(self, parent_frame):
        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(parent_frame, text="Arduino Connection")
        conn_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        conn_frame.columnconfigure(7, weight=1) # Allow space at end to expand

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky='w')
        self.port_var = tk.StringVar()
        self.port_combobox = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12, state='readonly') # Readonly prevents typing
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky='w')
        self.refresh_ports() # Populate ports initially

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=(10, 5), pady=5, sticky='w')
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD_RATE))
        baud_rates = ["9600", "19200", "38400", "57600", "115200", "230400", "250000"]
        self.baud_combobox = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=baud_rates, width=8, state='readonly')
        self.baud_combobox.grid(row=0, column=3, padx=5, pady=5, sticky='w')

        self.refresh_button = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports, width=8)
        self.refresh_button.grid(row=0, column=4, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect_arduino, width=8)
        self.connect_button.grid(row=0, column=5, padx=5, pady=5)
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_arduino, width=10, state=tk.DISABLED)
        self.disconnect_button.grid(row=0, column=6, padx=5, pady=5)

        # --- Controller Frame ---
        controller_frame = ttk.LabelFrame(parent_frame, text="Controller")
        controller_frame.pack(fill=tk.X, padx=10, pady=5)

        self.controller_status_label = ttk.Label(controller_frame, text="Status: Disconnected", width=35, anchor='w')
        self.controller_status_label.pack(side=tk.LEFT, padx=10, pady=5)
        self.connect_controller_button = ttk.Button(controller_frame, text="Connect Ctrl", command=self.connect_controller, width=12)
        self.connect_controller_button.pack(side=tk.LEFT, padx=5, pady=5)
        self.toggle_controller_button = ttk.Button(controller_frame, text="Enable Control", command=self.toggle_controller_polling, width=15, state=tk.DISABLED)
        self.toggle_controller_button.pack(side=tk.LEFT, padx=5, pady=5)

        # Frame to hold Controls and Console side-by-side
        main_area = ttk.Frame(parent_frame)
        main_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # --- Manual Controls Frame (Left) ---
        manual_controls_frame = ttk.LabelFrame(main_area, text="Robot Controls")
        manual_controls_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5), anchor='nw')


        # Top Row: Mode, Steering
        top_row_frame = ttk.Frame(manual_controls_frame)
        top_row_frame.pack(fill=tk.X, pady=5)

        self.mode_button = ttk.Button(top_row_frame, text="Switch to AUTO", command=self.toggle_mode, state=tk.DISABLED, width=15)
        self.mode_button.pack(side=tk.LEFT, padx=10, pady=5)

        steer_frame = ttk.Frame(top_row_frame)
        steer_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        ttk.Label(steer_frame, text="Steer:", width=6).pack(side=tk.LEFT, padx=(0,2))
        # Use the controller's range for consistency
        self.steering_slider = ttk.Scale(steer_frame, from_=self.xbox_controller.STEERING_MIN,
                                         to=self.xbox_controller.STEERING_MAX, orient="horizontal",
                                         command=self.on_steering_change, state=tk.DISABLED, length=180) # Adjusted length
        self.steering_slider.set(self.xbox_controller.STEERING_CENTER)
        self.steering_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.steering_value_label = ttk.Label(steer_frame, text=str(self.xbox_controller.STEERING_CENTER), width=5, anchor='e')
        self.steering_value_label.pack(side=tk.LEFT, padx=(0,5))

        # Bottom Row: Accel, Buttons
        bottom_row_frame = ttk.Frame(manual_controls_frame)
        bottom_row_frame.pack(fill=tk.BOTH, expand=True, pady=5)


        # Acceleration Slider (Vertical, Left)
        accel_subframe = ttk.Frame(bottom_row_frame)
        accel_subframe.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=(5,0))
        ttk.Label(accel_subframe, text="Accel:").pack(anchor='n')
        self.accel_slider = ttk.Scale(accel_subframe, from_=self.xbox_controller.ACCEL_MAX, to=0,
                                      orient="vertical", command=self.on_accel_change, state=tk.DISABLED, length=120) # Adjusted length
        self.accel_slider.set(0)
        self.accel_slider.pack(fill=tk.Y, expand=True, pady=5)
        self.accel_value_label = ttk.Label(accel_subframe, text=" 0", width=3, anchor='s') # Added space for alignment
        self.accel_value_label.pack(anchor='s')


        # Buttons (Direction, Brake, E-Stop) Frame (Right)
        buttons_subframe = ttk.Frame(bottom_row_frame)
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

        self.response_console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD, height=15, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1, font=("Consolas", 9)) # Monospace font
        self.response_console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        # Configure tags for different message types
        self.response_console.tag_configure("INFO", foreground="black")
        self.response_console.tag_configure("ERROR", foreground=RED, font=("Consolas", 9, "bold"))
        self.response_console.tag_configure("WARNING", foreground="#FF8C00") # DarkOrange
        self.response_console.tag_configure("SUCCESS", foreground=GREEN)
        self.response_console.tag_configure("CMD", foreground=BLUE)
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
             if self.controller_polling_active:
                 self.log_message("Controller or Arduino disconnected, disabling controller polling.", is_warning=True)
                 self.controller_polling_active = False # Ensure polling stops if disconnected
                 self.toggle_controller_button.config(text="Enable Control", style="TButton") # Reset text & style


        # Update Lidar tab controls
        if hasattr(self, 'lidar_visualizer'):
            self.lidar_visualizer.update_arduino_connection_state(self.arduino_connected)

        # Update controls based on mode (Manual/Auto) as well
        self.update_ui_based_on_mode()

    def update_ui_based_on_mode(self):
        """Enable/disable sliders/buttons based on Manual/Automatic mode."""
        if not self.arduino_connected:
            state_auto = tk.DISABLED
            state_manual = tk.DISABLED
            state_always = tk.DISABLED # For E-Stop
        else:
            # In Auto mode, manual buttons are generally active (unless controller polling overrides)
            # In Manual mode, most buttons are disabled (control via specific M commands if implemented)
            is_auto = self.is_automatic_mode
            # E-Stop is always active when connected
            state_always = tk.NORMAL

            # Determine state for Auto mode controls (Direction, Brake)
            # They are active in Auto mode UNLESS controller polling is also active
            state_auto_manual_override = tk.NORMAL if is_auto and not self.controller_polling_active else tk.DISABLED

            # Manual mode controls (placeholders if M mode had specific UI controls)
            state_manual_only = tk.NORMAL if not is_auto else tk.DISABLED


        # --- Update Control States ---
        # Sliders are only for display, always disabled for manual drag in this setup
        self.steering_slider.config(state=tk.DISABLED)
        self.accel_slider.config(state=tk.DISABLED)

        # Direction buttons (Active in Auto mode, unless Controller is polling)
        self.forward_btn.config(state=state_auto_manual_override)
        self.reverse_btn.config(state=state_auto_manual_override)

        # Brake button (Active in Auto mode, unless Controller is polling)
        self.brake_btn.config(state=state_auto_manual_override)
        # Update brake button text/state based on actual brake status
        self.update_brake_button_text()

        # Emergency stop button (Always active when connected)
        self.stop_btn.config(state=state_always)

        # Update mode button text
        if self.arduino_connected:
            self.mode_button.config(text=f"Switch to {'MANUAL' if self.is_automatic_mode else 'AUTO'}")

    def update_brake_button_text(self):
         # Only enable manual toggle if connected, in auto, and controller NOT polling
         can_manually_toggle = self.arduino_connected and self.is_automatic_mode and not self.controller_polling_active
         button_state = tk.NORMAL if can_manually_toggle else tk.DISABLED

         if self.brake_active:
              # Show release text, include reason, enable if possible
              reason_text = f" [{self.brake_reason}]" if self.brake_reason else ""
              self.brake_btn.config(text=f"Release Brake (Q 1){reason_text}", state=button_state)
         else:
              # Show engage text, enable if possible
              self.brake_btn.config(text="Engage Brake (S 1)", state=button_state)


    # --- Action Handlers ---

    def refresh_ports(self):
        self.log_message("Refreshing serial ports...")
        try:
            ports = sorted([port.device for port in serial.tools.list_ports.comports()])
            self.port_combobox['values'] = ports
            if ports:
                # Try to find the default or previously used port
                current_val = self.port_var.get()
                if current_val in ports:
                    self.port_combobox.set(current_val)
                elif DEFAULT_ARDUINO_PORT in ports:
                     # Select default Arduino port if available and nothing else selected
                     self.port_combobox.set(DEFAULT_ARDUINO_PORT)
                     self.log_message(f"Default Arduino port '{DEFAULT_ARDUINO_PORT}' found and selected.")
                else:
                    # Otherwise, just select the first available port
                    self.port_combobox.current(0)
                self.log_message(f"Available ports: {', '.join(ports)}")
            else:
                self.port_combobox.set('') # Clear selection if no ports
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

        # Disconnect first if already connected to another port/same port
        if self.arduino_communicator.is_connected:
             self.log_message(f"Disconnecting from {self.arduino_communicator.port} before reconnecting...")
             self.disconnect_arduino()
             time.sleep(0.3) # Allow time for serial port to release/settle

        self.update_status(f"Connecting to {port}...")
        if self.arduino_communicator.connect(port, int(baud)):
            # Success handled by communicator's log callback
            self.update_status(f"Connected to {port}")
            # Reset states on successful connection
            self.is_automatic_mode = False # Default to Manual mode? Let's try Auto.
            self.brake_active = False
            self.brake_reason = ""
            self.set_mode(automatic=True) # Attempt to switch to Auto mode on connect
            self.update_brake_button_text()
            self.update_ui_based_on_connection()

            # Automatically try to connect controller if not already connected
            if not self.xbox_controller.is_connected():
                self.connect_controller() # Attempt connection
            else:
                 # Controller was already connected, re-check UI state
                 self.update_ui_based_on_connection()

        else:
            # Failure logged by communicator
            messagebox.showerror("Connection Failed", f"Could not connect to Arduino on {port}. Check port, power, and wiring. Ensure Serial Monitor is closed.")
            self.update_status("Connection failed")
            self.update_ui_based_on_connection()


    def disconnect_arduino(self):
        # Stop Lidar assist first if active
        if self.stop_assist_active:
             # Access the var directly from the visualizer instance
             if hasattr(self, 'lidar_visualizer') and self.lidar_visualizer:
                 self.lidar_visualizer.stop_assist_var.set(False)
                 self.lidar_visualizer.on_stop_assist_changed() # Ensure state is updated

        # Send stop command before disconnecting if connected
        if self.arduino_communicator.is_connected:
             self.log_message("Sending V 0 before disconnect...")
             self.arduino_communicator.send_command("V 0")
             time.sleep(0.1) # Allow command to send

        self.arduino_communicator.disconnect()
        self.update_status("Disconnected")
        # Reset states
        self.is_automatic_mode = False
        self.brake_active = False
        self.brake_reason = ""
        # Stop controller polling if it was active
        if self.controller_polling_active:
            self.controller_polling_active = False
            self.toggle_controller_button.config(text="Enable Control", style="TButton")

        self.update_ui_based_on_connection()
        # Reset button texts after UI update
        self.update_brake_button_text()
        self.mode_button.config(text="Switch to AUTO")


    def connect_controller(self):
        if not pygame.joystick.get_init():
            self.log_message("Pygame joystick module not initialized. Cannot connect controller.", is_error=True)
            messagebox.showerror("Controller Error", "Pygame joystick module not initialized.")
            return

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            self.log_message("No joystick/controller detected by Pygame.", is_warning=True)
            messagebox.showwarning("Controller", "No joystick/controller detected. Ensure it's plugged in and recognized by the system.")
            self.controller_status_label.config(text="Status: Not Detected")
            self.connect_controller_button.config(state=tk.NORMAL)
            self.toggle_controller_button.config(state=tk.DISABLED, text="Enable Control", style="TButton")
            self.update_ui_based_on_connection() # Re-check toggle button state
            return

        self.update_status("Connecting to controller...")
        if self.xbox_controller.connect(): # Connect will log details
             self.controller_status_label.config(text=f"Status: Connected ({self.xbox_controller.controller_name})")
             self.connect_controller_button.config(state=tk.DISABLED) # Disable connect btn after success
             self.update_status("Controller connected")
             self.update_ui_based_on_connection() # Re-check toggle button state based on Arduino conn too
        else:
             self.controller_status_label.config(text="Status: Connection Failed")
             self.connect_controller_button.config(state=tk.NORMAL) # Re-enable connect btn on failure
             self.update_status("Controller connection failed")
             # messagebox.showerror("Controller Error", "Failed to connect to controller. See log for details.") # Connect() already logs
             self.update_ui_based_on_connection()


    def toggle_controller_polling(self):
        if not self.arduino_connected or not self.xbox_controller.is_connected():
             self.log_message("Cannot enable controller: Arduino or Controller not connected.", is_warning=True)
             self.controller_polling_active = False # Ensure it's off
             self.toggle_controller_button.config(text="Enable Control", style="TButton")
             self.update_ui_based_on_mode() # Refresh UI state
             return

        self.controller_polling_active = not self.controller_polling_active
        if self.controller_polling_active:
             # Ensure we are in Automatic mode when enabling controller
             if not self.is_automatic_mode:
                  self.log_message("Switching to AUTOMATIC mode to enable controller.", is_info=True)
                  self.set_mode(automatic=True) # Switch to Auto mode

             self.log_message("Xbox controller polling ENABLED.", is_success=True)
             self.toggle_controller_button.config(text="Disable Control", style="Active.TButton") # Use Active style
             self.update_status("Controller Active")
             # UI update to disable conflicting manual controls
             self.update_ui_based_on_mode()


        else:
             self.log_message("Xbox controller polling DISABLED.")
             self.toggle_controller_button.config(text="Enable Control", style="TButton") # Reset style
             self.update_status("Controller Inactive")
             # Send V 0 when disabling controller for safety
             self.log_message("Sending V 0 (Controller polling disabled).")
             self.arduino_communicator.send_command("V 0")
             self.accel_slider.set(0) # Update slider UI
             self.accel_value_label.config(text=" 0")
             # Re-enable manual buttons based on mode
             self.update_ui_based_on_mode()


    def set_mode(self, automatic: bool):
        """Sets the robot mode (Manual/Automatic) and sends command."""
        if not self.arduino_connected: return False

        target_mode = "A" if automatic else "M"
        current_mode_str = "Automatic" if self.is_automatic_mode else "Manual"
        target_mode_str = "Automatic" if automatic else "Manual"

        if self.is_automatic_mode == automatic:
            # self.log_message(f"Already in {current_mode_str} mode.")
            return True # Already in the desired mode

        self.log_message(f"Attempting to switch to {target_mode_str} mode ({target_mode})...")
        # Send appropriate command
        if self.arduino_communicator.send_command(target_mode):
             self.is_automatic_mode = automatic
             self.log_message(f"Switched to {target_mode_str} mode ({target_mode}) successfully.", is_success=True)

             # Send V 0 and center steering when switching modes for safety
             self.arduino_communicator.send_command("V 0")
             self.arduino_communicator.send_command(f"W {self.xbox_controller.STEERING_CENTER}")
             # If switching to Auto, set default direction to Forward
             if automatic:
                  self.arduino_communicator.send_command("D F")

             # Update UI elements
             self.update_ui_based_on_mode()
             # Update slider displays to reflect reset state
             self.accel_slider.set(0)
             self.accel_value_label.config(text=" 0")
             self.steering_slider.set(self.xbox_controller.STEERING_CENTER)
             self.steering_value_label.config(text=str(self.xbox_controller.STEERING_CENTER))
             self.update_status(f"Mode: {target_mode_str}")

             # If switching *away* from Auto while controller was active, disable controller polling
             if not automatic and self.controller_polling_active:
                 self.log_message("Controller polling disabled (switched to Manual mode).", is_warning=True)
                 self.controller_polling_active = False
                 self.toggle_controller_button.config(text="Enable Control", style="TButton")
                 self.update_ui_based_on_mode() # Re-enable manual buttons if applicable

             return True # Success
        else:
             self.log_message(f"Failed to send {target_mode} command.", is_error=True)
             # Revert UI text if mode change failed? Maybe not needed as state wasn't updated.
             return False # Failure


    def toggle_mode(self):
         """Toggles between Manual and Automatic mode."""
         self.set_mode(not self.is_automatic_mode)


    def on_steering_change(self, value_str):
        """Callback when steering slider is manually moved (only for display update)."""
        # This should only update the label, as the slider state is disabled for input
        try:
             value = int(float(value_str))
             self.steering_value_label.config(text=str(value))
        except ValueError:
             pass # Ignore non-numeric values during drag


    def on_accel_change(self, value_str):
        """Callback when acceleration slider is manually moved (only for display update)."""
        # This should only update the label, as the slider state is disabled for input
        try:
            value = int(float(value_str))
             # Map slider value (max->0) back to accel value (0->max) for display
            display_value = self.xbox_controller.ACCEL_MAX - value
            self.accel_value_label.config(text=f"{display_value:2d}") # Format to 2 digits
        except ValueError:
             pass


    def send_direction(self, direction):
        """Sends D F or D R command. Only active in Auto mode without controller polling."""
        if self.is_automatic_mode and self.arduino_connected and not self.controller_polling_active:
            if self.arduino_communicator.send_command(f"D {direction}"):
                 dir_text = 'Forward' if direction == 'F' else 'Reverse'
                 self.log_message(f"Manual Direction set to {dir_text}.")
                 # Highlight active direction button? (Optional)
                 # self.forward_btn.config(style="Active.TButton" if direction == "F" else "TButton")
                 # self.reverse_btn.config(style="Active.TButton" if direction == "R" else "TButton")
        elif not self.is_automatic_mode:
             self.log_message("Cannot set direction: Not in Automatic mode.", is_warning=True)
        elif self.controller_polling_active:
             self.log_message("Cannot set manual direction: Controller is active.", is_warning=True)
        else:
             self.log_message("Cannot set direction: Not connected.", is_warning=True)


    def update_brake_state(self, active, reason="Unknown"):
        """Central method to update brake status and related UI."""
        if self.brake_active == active: # No change
            # If the reason is different (e.g. LT release then E-Stop), update reason
            if active and self.brake_reason != reason:
                 self.brake_reason = reason
                 self.update_brake_button_text() # Update text to show new reason
            return

        self.brake_active = active
        self.brake_reason = reason if active else ""
        self.update_brake_button_text() # Update button text/state

        # If activating brake, ensure accel slider UI shows 0
        if active:
             self.accel_slider.set(0) # Slider value 0 = max accel reading
             # Update label to show 0 accel
             self.accel_value_label.config(text=" 0")
             # If controller is polling, also reset its internal accel state
             if self.controller_polling_active:
                  self.xbox_controller.last_sent_accel = 0


    def toggle_brake(self):
        """Manually toggles the brake state (S 1 / Q 1).
           Only callable when connected, in auto mode, and controller NOT active."""
        if not self.arduino_connected:
             self.log_message("Cannot toggle brake: Not connected.", is_warning=True)
             return
        if not self.is_automatic_mode:
             self.log_message("Cannot toggle brake: Not in Automatic mode.", is_warning=True)
             return
        if self.controller_polling_active:
             self.log_message("Cannot toggle manual brake: Controller is active.", is_warning=True)
             return


        if self.brake_active:
            # Currently active, send release command (Q 1)
            self.log_message("Manually releasing brake (Q 1)...")
            if self.arduino_communicator.send_command("Q 1"):
                # Update state AFTER command success
                self.update_brake_state(False, "Manual Release") # Use specific reason
                self.log_message("Brake released manually.")
            else:
                self.log_message("Failed to send Q 1 command.", is_error=True)
        else:
            # Currently inactive, send engage command (S 1)
            self.log_message("Manually engaging brake (S 1)...")
            # Send V 0 first for safety before engaging brake
            self.arduino_communicator.send_command("V 0")
            time.sleep(0.05) # Small delay
            if self.arduino_communicator.send_command("S 1"):
                # Update state AFTER command success
                self.update_brake_state(True, "Manual Button")
                self.log_message("Brake engaged manually.")
            else:
                self.log_message("Failed to send S 1 command.", is_error=True)


    def emergency_stop(self):
        """Sends immediate stop commands (V 0 and S 1). Central point for all E-Stops."""
        if not self.arduino_connected:
             self.log_message("Cannot E-STOP: Arduino not connected.", is_error=True)
             return

        # Check if already E-Stopped to prevent log/command spam
        if self.brake_active and self.brake_reason == "E-Stop":
             # self.log_message("E-Stop condition persists.", is_info=True)
             return

        self.log_message("EMERGENCY STOP ACTIVATED!", is_error=True)
        self.update_status("EMERGENCY STOP")
        # Send V 0 first, then S 1 for immediate braking
        self.arduino_communicator.send_command("V 0")
        time.sleep(0.05) # Short delay

        # Update state immediately to E-Stop, even before S1 confirmation
        # This prevents race conditions with Lidar check or controller input
        self.update_brake_state(True, "E-Stop")
        # Send the brake command
        if self.arduino_communicator.send_command("S 1"):
             self.log_message("E-Stop: V 0 and S 1 sent.")
        else:
             self.log_message("E-Stop: Failed to send S 1 command (V 0 sent).", is_error=True)
             # State remains E-Stop, user must manually release or retry

        # Update accel UI display
        self.accel_slider.set(0)
        self.accel_value_label.config(text=" 0")

        # E-Stop requires MANUAL release via Controller X button or potentially a dedicated GUI button in future.

    def release_emergency_stop(self):
         """Sends Q 1 to release brake ONLY IF brake is active DUE TO E-Stop."""
         if not self.arduino_connected:
             self.log_message("Cannot release E-Stop: Not connected.", is_warning=True)
             return

         # Check if E-Stop is actually the reason for the brake being active
         if self.brake_active and self.brake_reason == "E-Stop":
              self.log_message("Attempting to release E-Stop brake (Q 1)...")
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
             self.log_message("Cannot release E-stop: Brake is not currently active.", is_info=True)


    # --- Background Tasks ---

    def check_responses(self):
        """Periodically check the response queue and log messages."""
        # Check if root window still exists
        if not self.root.winfo_exists(): return

        try:
            while not self.response_queue.empty():
                response = self.response_queue.get_nowait()
                self.log_message(f"Recv: {response}", is_recv=True)
                # TODO: Parse responses if needed (e.g., status updates, sensor readings from Arduino)
                self.response_queue.task_done()
        except queue.Empty:
            pass
        except Exception as e:
             self.log_message(f"Error processing response queue: {e}", is_error=True)
        finally:
            # Reschedule using the stored ID
            if hasattr(self, 'check_responses_id'):
                 self.check_responses_id = self.root.after(50, self.check_responses)

    def poll_controller(self):
        """Periodically poll the Xbox controller and send commands if active."""
        if not self.root.winfo_exists(): return # Stop polling if window closed

        controller_just_connected = False
        try:
            # --- Check connection status first ---
            if not self.xbox_controller.is_connected():
                 # If polling was active, log disconnect and update UI
                 if self.controller_polling_active:
                      self.log_message("Controller disconnected during polling.", is_warning=True)
                      self.controller_polling_active = False
                      self.controller_status_label.config(text="Status: Disconnected")
                      self.connect_controller_button.config(state=tk.NORMAL)
                      self.toggle_controller_button.config(state=tk.DISABLED, text="Enable Control", style="TButton")
                      self.update_ui_based_on_mode() # Update other controls
                      self.update_status("Controller disconnected")
                 # else: # Controller not connected, and polling wasn't active - do nothing special
            else:
                 # --- Controller is connected, update its state ---
                 success, event = self.xbox_controller.update()

                 if not success and event != "QUIT": # Update failed (and wasn't a quit signal)
                      # Controller disconnected during update? Logged by update()
                      if self.controller_polling_active: # Log only if we expected it to be active
                          self.controller_status_label.config(text="Status: Disconnected (Error)")
                          self.connect_controller_button.config(state=tk.NORMAL)
                          self.toggle_controller_button.config(state=tk.DISABLED, text="Enable Control", style="TButton")
                          self.controller_polling_active = False
                          self.update_ui_based_on_mode()
                          self.update_status("Controller disconnected (Error)")

                 # --- Process Controller Input if Polling Active ---
                 elif self.controller_polling_active and self.is_automatic_mode and self.arduino_connected:

                     # Steering (Send only if changed significantly)
                     if self.xbox_controller.has_steering_changed():
                         steer_val = self.xbox_controller.last_sent_steering
                         self.arduino_communicator.send_command(f"W {steer_val}")
                         # Update UI display
                         self.steering_slider.set(steer_val)
                         self.steering_value_label.config(text=str(steer_val))

                     # Acceleration (Send only if changed and brake NOT active)
                     if self.xbox_controller.has_accel_changed():
                         accel_val = self.xbox_controller.last_sent_accel
                         # Only send V command if brake is NOT active
                         if not self.brake_active:
                             self.arduino_communicator.send_command(f"V {accel_val}")
                         else:
                              # If brake is active, ensure we send V 0 if accel tries to increase
                              if accel_val > 0:
                                   self.arduino_communicator.send_command("V 0")
                         # Update UI display regardless
                         # Map accel 0-50 back to slider 50-0
                         slider_val = self.xbox_controller.ACCEL_MAX - accel_val
                         self.accel_slider.set(slider_val)
                         self.accel_value_label.config(text=f"{accel_val:2d}")


                     # --- Handle Specific Controller Events ---
                     if event == "HAT_CHANGED":
                         hat_x, hat_y = self.xbox_controller.get_hat_values()
                         if hat_y == 1: self.send_direction("F") # Request Forward
                         elif hat_y == -1: self.send_direction("R") # Request Reverse
                         # Could handle hat_x for other functions?

                     elif event == "LT_PRESSED":
                         # Engage brake only if not already engaged for another reason
                         if not self.brake_active:
                               self.log_message("Controller: LT pressed, engaging brake (V 0, S 1)...")
                               self.arduino_communicator.send_command("V 0") # Send V0 first
                               time.sleep(0.05)
                               if self.arduino_communicator.send_command("S 1"):
                                    self.update_brake_state(True, "Controller LT")
                               else:
                                    self.log_message("Failed to send S 1 from controller.", is_error=True)
                         else:
                              self.log_message(f"Controller: LT pressed, but brake already active (Reason: {self.brake_reason}).", is_info=True)

                     elif event == "LT_RELEASED":
                          # Release brake ONLY if it was engaged by the LT controller button
                          if self.brake_active and self.brake_reason == "Controller LT":
                               self.log_message("Controller: LT released, releasing brake (Q 1)...")
                               if self.arduino_communicator.send_command("Q 1"):
                                    self.update_brake_state(False) # Reason cleared by update_brake_state
                               else:
                                    self.log_message("Failed to send Q 1 from controller.", is_error=True)
                          # elif self.brake_active: # LT released but brake active for other reason
                          #      self.log_message(f"Controller: LT released, but brake remains active (Reason: {self.brake_reason}).", is_info=True)

                     elif event == "X_PRESSED":
                          # Use X button to attempt releasing an E-Stop ONLY
                          if self.brake_active and self.brake_reason == "E-Stop":
                               self.log_message("Controller: X button pressed, attempting to release E-Stop...", is_info=True)
                               self.release_emergency_stop() # Call the dedicated release function
                          elif self.brake_active:
                               self.log_message(f"Controller: X button pressed, but brake is active for reason '{self.brake_reason}'. E-Stop release only.", is_warning=True)
                          # else: # X pressed, brake not active
                          #      self.log_message("Controller: X button pressed, brake not active.", is_info=True)


        except Exception as e:
            self.log_message(f"Error in controller poll loop: {e}", is_error=True)
            # import traceback # Debugging
            # traceback.print_exc() # Debugging
            # Attempt to disable polling on error?
            if self.controller_polling_active:
                self.log_message("Disabling controller polling due to error.", is_error=True)
                self.controller_polling_active = False
                self.toggle_controller_button.config(text="Enable Control", style="TButton")
                self.update_ui_based_on_mode()


        finally:
            # Schedule next poll if the window still exists
            if self.root.winfo_exists() and hasattr(self, 'poll_controller_id'):
                self.poll_controller_id = self.root.after(30, self.poll_controller) # Poll slightly faster ~33Hz

    # --- Logging and Status ---

    def log_message(self, message, is_error=False, is_warning=False, is_success=False, is_info=False, is_cmd=False, is_recv=False, console_only=False):
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

        # Fallback print for early messages or console issues
        if console_only or not hasattr(self, 'response_console') or not self.response_console:
            print(f"LOG ({tag}): {message}") # Print without newline if already included
            return

        try:
            # Check if widget exists and is valid
             if self.response_console.winfo_exists():
                self.response_console.config(state=tk.NORMAL)
                self.response_console.insert(tk.END, full_message, (tag,))
                self.response_console.see(tk.END) # Auto-scroll
                self.response_console.config(state=tk.DISABLED)
        except tk.TclError as e:
             # Handle case where console might be destroyed during shutdown
             print(f"LOG ({tag}): {full_message.strip()} (TclError: {e})") # Fallback to console print
        except Exception as e:
             print(f"LOGGING ERROR: {e}")
             print(f"Original Msg ({tag}): {full_message.strip()}")


    def clear_console(self):
        if hasattr(self, 'response_console') and self.response_console.winfo_exists():
            self.response_console.config(state=tk.NORMAL)
            self.response_console.delete(1.0, tk.END)
            self.response_console.config(state=tk.DISABLED)
            self.log_message("Log cleared.", is_info=True)

    def update_status(self, message):
        """Update the bottom status bar."""
        if hasattr(self, 'status_bar') and self.status_bar.winfo_exists():
            timestamp = time.strftime("%H:%M:%S")
            self.status_bar.config(text=f"Status: {message} ({timestamp})")


    # --- Application Shutdown ---
    def on_closing(self):
        """Handle cleanup when the application window is closed."""
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            self.log_message("----- Application Closing Initiated -----", is_warning=True)
            self.update_status("Closing...")
            self.root.protocol("WM_DELETE_WINDOW", lambda: None) # Prevent closing again

            # 1. Stop Lidar Processing and Scanning
            self.log_message("Stopping Lidar...", is_info=True)
            if hasattr(self, 'lidar_visualizer') and self.lidar_visualizer:
                self.lidar_visualizer.stop_display_update()
            if self.lidar_processor:
                self.lidar_processor.disconnect_lidar() # Stops scan and disconnects

            # 2. Stop Background Tasks (`after` loops)
            self.log_message("Stopping background tasks...", is_info=True)
            if hasattr(self, 'check_responses_id'): self.root.after_cancel(self.check_responses_id)
            if hasattr(self, 'poll_controller_id'): self.root.after_cancel(self.poll_controller_id)
            # Cancel any pending canvas resize jobs
            if hasattr(self.lidar_visualizer, '_resize_job_canvas'):
                self.lidar_visualizer.after_cancel(self.lidar_visualizer._resize_job_canvas)


            # 3. Disconnect Arduino (sends V 0 etc. if implemented in disconnect)
            self.log_message("Shutting down Arduino connection...", is_info=True)
            if self.arduino_communicator:
                # Send final stop commands if connected
                if self.arduino_communicator.is_connected:
                    self.log_message("Sending final Stop commands (V 0, S 1)...", is_info=True)
                    # Use queue to ensure sent if possible, but with timeout
                    self.arduino_communicator.send_command("V 0")
                    self.arduino_communicator.send_command("S 1") # End in safe state
                    # Wait briefly for queue to process, communicator shutdown handles join
                    time.sleep(0.2)
                self.arduino_communicator.shutdown()

            # 4. Disconnect Xbox Controller and Quit Pygame
            self.log_message("Disconnecting controller and shutting down Pygame...", is_info=True)
            if self.xbox_controller:
                self.xbox_controller.disconnect()

            # Check if pygame is initialized before quitting
            if pygame.get_init():
                 pygame.quit()
                 self.log_message("Pygame shut down.", is_success=True)
            else:
                 self.log_message("Pygame was not initialized.", is_info=True)


            # 5. Destroy the Tkinter window
            self.log_message("Destroying GUI...", is_info=True)
            self.update_status("Exited")
            self.root.destroy()
            print("----- Application Closed -----") # Final message to console

# --- Main Execution ---
if __name__ == "__main__":
    app = None # Initialize app to None
    root = None # Initialize root to None
    try:
        # Create the main Tkinter window
        root = tk.Tk()
        root.withdraw() # Hide window initially until app is created

        # Create the main application instance
        # Pygame is initialized inside CombinedRobotApp constructor now
        app = CombinedRobotApp(root)

        # Set the close window protocol
        root.protocol("WM_DELETE_WINDOW", app.on_closing)

        # Make window visible now
        root.deiconify()

        # Start the Tkinter main loop
        root.mainloop()

    except Exception as e:
        print(f"\n--- UNHANDLED ERROR IN MAIN EXECUTION ---")
        import traceback
        traceback.print_exc()
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Details: {e}")
        print("Attempting emergency cleanup...")
        # Attempt cleanup even on error
        try:
             if app:
                 print("Closing App Components...")
                 if hasattr(app, 'lidar_processor') and app.lidar_processor: app.lidar_processor.disconnect_lidar()
                 if hasattr(app, 'arduino_communicator') and app.arduino_communicator: app.arduino_communicator.shutdown()
                 if hasattr(app, 'xbox_controller') and app.xbox_controller: app.xbox_controller.disconnect()
             if pygame.get_init():
                 print("Quitting Pygame...")
                 pygame.quit()
             if root and root.winfo_exists():
                 print("Destroying Tkinter root...")
                 root.destroy()
        except Exception as cleanup_e:
             print(f"Error during final cleanup: {cleanup_e}")
        finally:
             print("Exiting due to error.")
             sys.exit(1) # Exit with error code
