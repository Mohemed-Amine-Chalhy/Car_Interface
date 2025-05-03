import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, font as tkfont
import threading
import queue
import serial
import serial.tools.list_ports
import time
import pygame # For Xbox controller AND sound
import sys
import math
import os # To check for sound file existence
from collections import deque
from rplidar import RPLidar

# --- Constants ---
# Robot Control
DEFAULT_BAUD_RATE = 115200
# Lidar
DEFAULT_LIDAR_PORT = 'COM6' # Adjust as needed
DEFAULT_ARDUINO_PORT = 'COM3' # Adjust as needed
VEHICLE_WIDTH = 42
OBSTACLE_THRESHOLD = 500
VIEW_ANGLE = 180
MAX_DISTANCE = 2000
HISTORY_SIZE = 5
AUTO_BRAKE_THRESHOLD = 50
# Sound
BEEP_SOUND_FILE = "beep.wav" # <<< Name of your beep sound file
# Colors
WHITE = "#FFFFFF"
BLACK = "#000000"
RED = "#FF0000"
GREEN = "#00FF00"
BLUE = "#0000FF"
YELLOW = "#FFFF00"
DARK_GRAY = "#1E1E1E"
MEDIUM_GRAY = "#3C3C3C"
LIGHT_GRAY = "#A0A0A0"
SCAN_POINT_COLOR = "#6495ED"
OBSTACLE_COLOR_PATH = "#FF4500"
OBSTACLE_COLOR_NEAR = "#FFD700"
CLOSEST_POINT_COLOR = "#32CD32"
VEHICLE_OUTLINE_COLOR = "#4682B4"
BRAKE_ARC_COLOR = "#DC143C"

# --- Arduino Communicator Class (No changes needed) ---
class ArduinoCommunicator:
   def __init__(self, command_queue, response_queue, log_callback):
       self.serial_conn = None
       self.is_connected = False
       self.port = None
       self.baud_rate = None
       self.command_queue = command_queue
       self.response_queue = response_queue
       self.log_callback = log_callback
       self.running = True
       self.command_thread = threading.Thread(target=self._process_commands, daemon=True)
       self.command_thread.start()
       self.response_thread = threading.Thread(target=self._read_responses, daemon=True)
       self.response_thread.start()

   def connect(self, port, baud_rate=DEFAULT_BAUD_RATE):
       try:
           self.disconnect()
           self.port = port
           self.baud_rate = baud_rate
           self.serial_conn = serial.Serial(port, baud_rate, timeout=0.5)
           time.sleep(1)
           self.is_connected = True
           self.log_callback(f"Successfully connected to Arduino on {port} at {baud_rate} baud.", is_success=True)
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
               # self.send_command("V 0") # Optional: Send stop before disconnect
               # self.send_command("S 1")
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

   def send_command(self, command):
       if self.is_connected and self.serial_conn:
           if not command.endswith('\n'):
               command += '\n'
           self.command_queue.put(command)
           # Log non-trivial commands
           log_this = True
           cmd_strip = command.strip()
           if cmd_strip.startswith("W ") or cmd_strip.startswith("V "): # Don't log frequent steering/velocity
               log_this = False
           # if cmd_strip in ["V 0", "S 1", "Q 1"]: # Log important state changes
               # log_this = True

           if log_this:
               self.log_callback(f"Queued Cmd: {cmd_strip}", is_cmd=True)
           return True
       else:
           # self.log_callback("Cannot send command: Arduino not connected.", is_error=True) # Can be noisy
           return False

   def _process_commands(self):
       last_command_time = 0
       min_interval = 0.05 # 50ms
       while self.running:
           try:
               command = self.command_queue.get(timeout=0.1)
               if self.is_connected and self.serial_conn:
                   current_time = time.time()
                   time_since_last = current_time - last_command_time
                   if time_since_last < min_interval:
                       time.sleep(min_interval - time_since_last)

                   try:
                       self.serial_conn.write(command.encode('utf-8'))
                       self.serial_conn.flush()
                       last_command_time = time.time()
                       # Log only specific sent commands if needed (already logged when queued mostly)
                       # if command.strip() in ["S 1", "Q 1", "A", "M", "D F", "D R", "V 0"]:
                       #     self.log_callback(f"Sent: {command.strip()}")
                   except serial.SerialException as se:
                       self.log_callback(f"Serial Write Error: {se}. Disconnecting.", is_error=True)
                       self.disconnect()
                   except Exception as e:
                       self.log_callback(f"Error sending command '{command.strip()}': {e}", is_error=True)
               else:
                   self.log_callback(f"Discarded command (not connected): {command.strip()}", is_warning=True)
               self.command_queue.task_done()
           except queue.Empty:
               continue
           except Exception as e:
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
                               self.response_queue.put(line)
               except serial.SerialException as se:
                   self.log_callback(f"Serial Read Error: {se}. Disconnecting.", is_error=True)
                   self.disconnect()
               except Exception as e:
                   self.log_callback(f"Error reading response: {e}", is_error=True)
                   time.sleep(0.5)
           else:
                time.sleep(0.2)
           time.sleep(0.01)


   def shutdown(self):
       self.running = False
       self.log_callback("Shutting down Arduino communicator...")
       if self.command_thread.is_alive():
           while not self.command_queue.empty():
                try: self.command_queue.get_nowait(); self.command_queue.task_done()
                except queue.Empty: break
           self.command_thread.join(timeout=1.0)
       if self.response_thread.is_alive():
           self.response_thread.join(timeout=1.0)
       self.disconnect()
       self.log_callback("Arduino communicator shut down.")

# --- Xbox Controller Class (No changes needed from previous version with X button) ---
class XboxController:
    def __init__(self, controller_id=0, log_callback=None):
        # Ensure Pygame display is initialized minimally if not done elsewhere, needed for events
        if not pygame.display.get_init():
            pygame.display.init()
        # Initialize subsystems
        # pygame.init() # Covered by display/joystick init? Avoid multiple calls.
        pygame.joystick.init()

        self.log_callback = log_callback or (lambda msg, **kwargs: print(msg))
        self.connected = False
        self.controller = None
        self.controller_id = controller_id
        self.controller_name = "None"

        # Mappings (Defaults, adjust/verify as needed)
        self.AXIS_RIGHT_STICK_X_XBOX = 0; self.AXIS_RT_XBOX = 5; self.AXIS_LT_XBOX = 4
        self.AXIS_RIGHT_STICK_X_PS5 = 2; self.AXIS_RT_PS5 = 5; self.AXIS_LT_PS5 = 4
        self.HAT_DPAD_XBOX = 0
        self.BUTTON_DPAD_UP = 11; self.BUTTON_DPAD_DOWN = 12; self.BUTTON_DPAD_LEFT = 13; self.BUTTON_DPAD_RIGHT = 14 # PS5 D-Pad Placeholders
        self.BUTTON_X = 0 # PS Cross / Xbox A (Check your controller)

        # Dynamic mappings
        self.AXIS_RIGHT_STICK_X = self.AXIS_RIGHT_STICK_X_XBOX
        self.AXIS_RT = self.AXIS_RT_XBOX; self.AXIS_LT = self.AXIS_LT_XBOX
        self.USE_HAT_DPAD = True

        # Status & State
        self.right_stick_x = 0.0; self.rt_value = -1.0; self.lt_value = -1.0
        self.hat_values = (0, 0)
        self.dpad_button_state = {'up': False, 'down': False, 'left': False, 'right': False}
        self.last_dpad_tuple = (0, 0); self.lt_pressed_state = False

        self.DEADZONE_STICK = 0.15; self.DEADZONE_TRIGGER = 0.1
        self.STEERING_MIN = 0; self.STEERING_MAX = 3590
        self.STEERING_CENTER = (self.STEERING_MAX + self.STEERING_MIN) // 2
        self.ACCEL_MAX = 50

        self.last_sent_steering = self.STEERING_CENTER; self.last_sent_accel = 0
        self.connect() # Attempt initial connection

    def _determine_mappings(self):
        if not self.controller: return
        name = self.controller.get_name().lower()
        num_hats = self.controller.get_numhats(); num_buttons = self.controller.get_numbuttons()
        self.log_callback(f"Controller: {name}, Hats: {num_hats}, Buttons: {num_buttons}")
        if "dual" in name or "playstation" in name or num_hats == 0 and num_buttons > 10:
            self.log_callback("Controller -> PS-like mappings.")
            self.AXIS_RIGHT_STICK_X = self.AXIS_RIGHT_STICK_X_PS5; self.AXIS_RT = self.AXIS_RT_PS5; self.AXIS_LT = self.AXIS_LT_PS5
            self.USE_HAT_DPAD = False
            self.log_callback(f"Using Button D-Pad (U:{self.BUTTON_DPAD_UP}, D:{self.BUTTON_DPAD_DOWN}, L:{self.BUTTON_DPAD_LEFT}, R:{self.BUTTON_DPAD_RIGHT}) - VERIFY!")
            self.log_callback(f"Using 'X' button ID: {self.BUTTON_X} (PS: Cross=0, Circle=1, Square=2, Tri=3) - VERIFY!")
        else:
            self.log_callback("Controller -> Xbox-like mappings.")
            self.AXIS_RIGHT_STICK_X = self.AXIS_RIGHT_STICK_X_XBOX; self.AXIS_RT = self.AXIS_RT_XBOX; self.AXIS_LT = self.AXIS_LT_XBOX
            self.USE_HAT_DPAD = True
            self.log_callback(f"Using Hat D-Pad (Hat ID: {self.HAT_DPAD_XBOX})")
            self.log_callback(f"Using 'X' button ID: {self.BUTTON_X} (Xbox: A=0, B=1, X=2, Y=3) - VERIFY!")
        # Verify axis numbers
        num_axes = self.controller.get_numaxes()
        if self.AXIS_RIGHT_STICK_X >= num_axes: self.log_callback(f"Warn: AXIS_RIGHT_STICK_X ({self.AXIS_RIGHT_STICK_X}) OOR", is_warning=True); self.AXIS_RIGHT_STICK_X = 0
        if self.AXIS_RT >= num_axes: self.log_callback(f"Warn: AXIS_RT ({self.AXIS_RT}) OOR", is_warning=True); self.AXIS_RT = max(0, num_axes - 1)
        if self.AXIS_LT >= num_axes: self.log_callback(f"Warn: AXIS_LT ({self.AXIS_LT}) OOR", is_warning=True); self.AXIS_LT = max(0, num_axes - 2)

    def connect(self):
        joystick_count = pygame.joystick.get_count()
        if joystick_count <= self.controller_id: self.log_callback(f"Controller ID {self.controller_id} not found (Count: {joystick_count}).", is_warning=True); self.connected = False; return False
        try:
            if self.controller: self.controller.quit()
            self.controller = pygame.joystick.Joystick(self.controller_id); self.controller.init()
            self.controller_name = self.controller.get_name(); self.connected = True
            self.log_callback(f"Connected to controller: {self.controller_name}", is_success=True)
            self._determine_mappings()
            # Reset states
            num_axes = self.controller.get_numaxes()
            self.right_stick_x = self.controller.get_axis(self.AXIS_RIGHT_STICK_X) if num_axes > self.AXIS_RIGHT_STICK_X else 0.0
            self.rt_value = self.controller.get_axis(self.AXIS_RT) if num_axes > self.AXIS_RT else -1.0
            self.lt_value = self.controller.get_axis(self.AXIS_LT) if num_axes > self.AXIS_LT else -1.0
            self.hat_values = self.controller.get_hat(self.HAT_DPAD_XBOX) if self.controller.get_numhats() > self.HAT_DPAD_XBOX else (0,0)
            self.dpad_button_state = {'up': False, 'down': False, 'left': False, 'right': False}
            self.last_dpad_tuple = self.get_hat_values(); self.lt_pressed_state = self.lt_value > 0.5
            self.last_sent_steering = self.STEERING_CENTER; self.last_sent_accel = 0
            return True
        except pygame.error as e: self.log_callback(f"Pygame error connecting controller: {e}", is_error=True); self.connected = False; self.controller = None; return False
        except Exception as e: self.log_callback(f"Unexpected error connecting controller: {e}", is_error=True); self.connected = False; self.controller = None; return False

    def disconnect(self):
        if self.controller:
            try: self.controller.quit()
            except Exception as e: self.log_callback(f"Error during controller quit: {e}", is_error=True)
        self.connected = False; self.controller = None; self.controller_name = "None"
        self.log_callback("Controller disconnected.")

    def update(self):
        if not self.connected or not self.controller: return False, None
        event_occurred = None; dpad_input_changed = False
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.log_callback("Pygame quit event."); return False, None
                elif event.type == pygame.JOYDEVICEADDED and event.instance_id == self.controller_id: self.log_callback("Controller re-added."); self.connect()
                elif event.type == pygame.JOYDEVICEREMOVED and event.instance_id == self.controller_id: self.log_callback("Controller removed.", is_warning=True); self.disconnect(); return False, None
                elif event.type == pygame.JOYBUTTONDOWN:
                    button = event.button
                    #self.log_callback(f"DEBUG: Button {button} Pressed") # Keep for debugging button IDs
                    if button == self.BUTTON_X: event_occurred = "X_PRESSED"
                    elif not self.USE_HAT_DPAD:
                        if button == self.BUTTON_DPAD_UP: self.dpad_button_state['up'] = True; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_DOWN: self.dpad_button_state['down'] = True; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_LEFT: self.dpad_button_state['left'] = True; dpad_input_changed = True
                        elif button == self.BUTTON_DPAD_RIGHT: self.dpad_button_state['right'] = True; dpad_input_changed = True
                elif event.type == pygame.JOYBUTTONUP:
                    button = event.button
                    if not self.USE_HAT_DPAD:
                         if button == self.BUTTON_DPAD_UP: self.dpad_button_state['up'] = False; dpad_input_changed = True
                         elif button == self.BUTTON_DPAD_DOWN: self.dpad_button_state['down'] = False; dpad_input_changed = True
                         elif button == self.BUTTON_DPAD_LEFT: self.dpad_button_state['left'] = False; dpad_input_changed = True
                         elif button == self.BUTTON_DPAD_RIGHT: self.dpad_button_state['right'] = False; dpad_input_changed = True
                elif event.type == pygame.JOYHATMOTION:
                     if self.USE_HAT_DPAD and event.hat == self.HAT_DPAD_XBOX: self.hat_values = event.value; dpad_input_changed = True
            # Read Axes
            num_axes = self.controller.get_numaxes()
            self.right_stick_x = self.controller.get_axis(self.AXIS_RIGHT_STICK_X) if num_axes > self.AXIS_RIGHT_STICK_X else 0.0
            self.rt_value = self.controller.get_axis(self.AXIS_RT) if num_axes > self.AXIS_RT else -1.0
            self.lt_value = self.controller.get_axis(self.AXIS_LT) if num_axes > self.AXIS_LT else -1.0
            # Determine Events (Priority: X > DPad > LT)
            current_dpad_tuple = self.get_hat_values()
            if event_occurred != "X_PRESSED" and dpad_input_changed and current_dpad_tuple != self.last_dpad_tuple:
                 if self.last_dpad_tuple != (0,0) or current_dpad_tuple != (0,0): event_occurred = "HAT_CHANGED"
                 self.last_dpad_tuple = current_dpad_tuple
            lt_currently_pressed = self.lt_value > 0.5
            if event_occurred is None:
                if lt_currently_pressed and not self.lt_pressed_state: event_occurred = "LT_PRESSED"; self.lt_pressed_state = True
                elif not lt_currently_pressed and self.lt_pressed_state: event_occurred = "LT_RELEASED"; self.lt_pressed_state = False
            elif lt_currently_pressed != self.lt_pressed_state: self.lt_pressed_state = lt_currently_pressed # Update state even if other event happened
            return True, event_occurred
        except pygame.error as e: self.log_callback(f"Pygame error reading controller: {e}. Disconnecting.", is_error=True); self.disconnect(); return False, None
        except Exception as e: self.log_callback(f"Unexpected error reading controller: {e}. Disconnecting.", is_error=True); self.disconnect(); return False, None

    def get_hat_values(self):
        if self.USE_HAT_DPAD:
            if self.connected and self.controller and self.controller.get_numhats() > self.HAT_DPAD_XBOX: return self.controller.get_hat(self.HAT_DPAD_XBOX)
            else: return (0, 0)
        else:
            dpad_x = 1 if self.dpad_button_state['right'] else -1 if self.dpad_button_state['left'] else 0
            dpad_y = 1 if self.dpad_button_state['up'] else -1 if self.dpad_button_state['down'] else 0
            return (dpad_x, dpad_y)

    def get_steering_value(self):
        raw_value = self.right_stick_x
        if abs(raw_value) < self.DEADZONE_STICK: return self.STEERING_CENTER
        if raw_value > 0: scaled_value = (raw_value - self.DEADZONE_STICK) / (1 - self.DEADZONE_STICK); steering = self.STEERING_CENTER + scaled_value * (self.STEERING_MAX - self.STEERING_CENTER)
        else: scaled_value = (raw_value + self.DEADZONE_STICK) / (1 - self.DEADZONE_STICK); steering = self.STEERING_CENTER + scaled_value * (self.STEERING_CENTER - self.STEERING_MIN)
        return max(self.STEERING_MIN, min(self.STEERING_MAX, int(steering)))

    def get_acceleration_value(self):
        raw_value = self.rt_value; normalized = (raw_value + 1) / 2
        if normalized < self.DEADZONE_TRIGGER: return 0
        scaled_value = (normalized - self.DEADZONE_TRIGGER) / (1 - self.DEADZONE_TRIGGER)
        accel = int(scaled_value * self.ACCEL_MAX)
        return max(0, min(self.ACCEL_MAX, accel))

    def has_steering_changed(self):
        current_steering = self.get_steering_value()
        if abs(current_steering - self.last_sent_steering) > 5: self.last_sent_steering = current_steering; return True
        return False

    def has_accel_changed(self):
        current_accel = self.get_acceleration_value()
        if current_accel != self.last_sent_accel: self.last_sent_accel = current_accel; return True
        return False

    def is_connected(self):
        try: return self.connected and self.controller and self.controller.get_init() and pygame.joystick.get_count() > self.controller_id and pygame.joystick.Joystick(self.controller_id).get_instance_id() == self.controller.get_instance_id()
        except pygame.error: return False

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
        self._scan_data = {}
        self._obstacle_points = []
        self._closest_distance = float('inf')
        self._closest_angle = 0

    def connect(self):
        if self.lidar and self.running: self.log_callback("Lidar already connected.", is_warning=True); return True
        try:
            self.log_callback(f"Connecting to LIDAR on {self.port_name}...")
            if self.lidar:
                 try: self.lidar.disconnect()
                 except: pass
            self.lidar = RPLidar(self.port_name, timeout=3)
            info = self.lidar.get_info(); self.log_callback(f"LIDAR Info: {info}")
            health = self.lidar.get_health(); self.log_callback(f"LIDAR Health: {health}")
            if health[0] == 'Error': raise Exception(f"Lidar health status error: {health[1]}")
            self.log_callback(f"LIDAR connected successfully.", is_success=True)
            return True
        except serial.SerialException as se: self.log_callback(f"Serial Error connecting LIDAR: {se}", is_error=True); self.lidar = None; return False
        except Exception as e: self.log_callback(f"Error connecting LIDAR: {e}", is_error=True); self.lidar = None; return False

    def start_scan(self):
        if self.running: self.log_callback("Lidar scan already running.", is_warning=True); return True
        if not self.lidar: self.log_callback("Cannot start scan: Lidar not connected.", is_error=True); return False
        try:
            self.log_callback("Starting Lidar motor and scan...")
            self.lidar.start_motor(); time.sleep(2)
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True); self.scan_thread.start()
            self.log_callback("Lidar scan started.")
            return True
        except Exception as e:
            self.log_callback(f"Error starting Lidar scan: {e}", is_error=True); self.running = False
            #if self.lidar: try: self.lidar.stop_motor() catch: pass
            return False

    def stop_scan(self):
        if not self.running: return
        self.running = False; self.log_callback("Stopping Lidar scan...")
        if self.scan_thread and self.scan_thread.is_alive():
            try: self.scan_thread.join(timeout=2.0)
            except Exception as e: self.log_callback(f"Error joining scan thread: {e}", is_warning=True)
        if self.lidar:
            try: self.lidar.stop(); self.lidar.stop_motor(); self.log_callback("Lidar motor stopped.")
            except Exception as e: self.log_callback(f"Error stopping Lidar: {e}", is_warning=True)
        self.scan_thread = None
        self.log_callback("Lidar scan stopped.")

    def disconnect_lidar(self):
        self.stop_scan()
        if self.lidar:
             try: self.lidar.disconnect(); self.log_callback(f"LIDAR disconnected from {self.port_name}.")
             except Exception as e: self.log_callback(f"Error disconnecting Lidar: {e}", is_warning=True)
             finally: self.lidar = None

    def _scan_loop(self):
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=5000, min_len=10): # Increased min_len
                if not self.running: break
                current_scan_data = {}
                current_obstacle_points = []
                min_dist_in_scan = float('inf'); min_angle_in_scan = 0

                for quality, angle, distance in scan:
                    # if quality < 10: continue # Optional quality filter
                    dist_cm = distance / 10.0
                    # Assume Lidar 0 deg is Robot 90 deg (adjust if needed)
                    robot_angle = angle - 90
                    if robot_angle < -180: robot_angle += 360
                    elif robot_angle > 180: robot_angle -= 360

                    current_scan_data[robot_angle] = dist_cm

                    if -VIEW_ANGLE/2 <= robot_angle <= VIEW_ANGLE/2 and 0 < dist_cm <= OBSTACLE_THRESHOLD:
                        lateral_dist = abs(dist_cm * math.sin(math.radians(robot_angle)))
                        in_path = lateral_dist <= (VEHICLE_WIDTH / 2.0)
                        current_obstacle_points.append((robot_angle, dist_cm, in_path))
                        if in_path and dist_cm < min_dist_in_scan:
                            min_dist_in_scan = dist_cm
                            min_angle_in_scan = robot_angle

                with self.data_lock:
                    self._scan_data = current_scan_data
                    self._obstacle_points = current_obstacle_points
                    if min_dist_in_scan < float('inf'):
                        self._closest_distance = min_dist_in_scan
                        self._closest_angle = min_angle_in_scan
                    # Maybe reset closest if no obstacle seen for a while?
                    # else: self._closest_distance = float('inf')

                    self.history.append({'obstacles': current_obstacle_points, 'closest_dist': min_dist_in_scan, 'closest_angle': min_angle_in_scan})

        except serial.SerialException as se: self.log_callback(f"Serial Error during Lidar scan: {se}. Stopping scan.", is_error=True)
        except Exception as e: self.log_callback(f"Error in Lidar scan loop: {e}", is_error=True)
        finally:
             if self.lidar and self.running: # Ensure motor stops if thread exits unexpectedly
                 self.log_callback("Scan loop terminated unexpectedly. Stopping motor.", is_warning=True)
                 try: self.lidar.stop(); self.lidar.stop_motor()
                 except: pass
             self.running = False # Explicitly set flag
             self.log_callback("Lidar scan loop finished.")

    def get_stable_data(self):
        with self.data_lock:
            latest_scan = self._scan_data.copy() # Return copies
            latest_obstacles = list(self._obstacle_points)
            # Stability logic (optional - using latest for simplicity now)
            # valid_distances = [h['closest_dist'] for h in self.history if h['closest_dist'] < float('inf')]
            # if valid_distances:
            #     stable_closest_dist = sum(valid_distances) / len(valid_distances)
            #     stable_closest_angle = self._closest_angle # Use latest angle
            # else:
            #     stable_closest_dist = float('inf'); stable_closest_angle = 0
            stable_closest_dist = self._closest_distance
            stable_closest_angle = self._closest_angle
            return latest_scan, latest_obstacles, stable_closest_dist, stable_closest_angle

# --- Lidar Visualizer Frame (Added Beep Toggle and Logic) ---
class LidarVisualizerFrame(ttk.Frame):
    def __init__(self, master, main_app, lidar_processor, arduino_communicator, log_callback, **kwargs):
        super().__init__(master, **kwargs)
        self.main_app = main_app
        self.lidar_processor = lidar_processor
        self.arduino_communicator = arduino_communicator
        self.log_callback = log_callback

        # Lidar State
        self.lidar_running = False
        self.update_id = None

        # --- Beep State ---
        self.proximity_beep_enabled_var = tk.BooleanVar(value=False)
        self.proximity_beep_enabled = False
        self.was_too_close = False # Track previous state for beep trigger

        # UI Elements
        self._create_widgets()
        self.master.bind("<Map>", self._initial_canvas_setup, add="+")


    def _initial_canvas_setup(self, event=None):
         self.master.unbind("<Map>", self._initial_map_bind_id)
         self._setup_canvas()


    def _create_widgets(self):
        # Controls Frame (Top)
        controls_frame = ttk.LabelFrame(self, text="LIDAR Control")
        controls_frame.pack(pady=5, padx=10, fill=tk.X, side=tk.TOP)
        ttk.Label(controls_frame, text="LIDAR Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_var = tk.StringVar(value=DEFAULT_LIDAR_PORT)
        self.port_entry = ttk.Entry(controls_frame, textvariable=self.port_var, width=15)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.activate_button = ttk.Button(controls_frame, text="Activate LIDAR", command=self.start_lidar)
        self.activate_button.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.deactivate_button = ttk.Button(controls_frame, text="Deactivate LIDAR", command=self.stop_lidar, state=tk.DISABLED)
        self.deactivate_button.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        # Assist Frame (Middle)
        assist_frame = ttk.LabelFrame(self, text="Lidar Assist / Audio") # Renamed slightly
        assist_frame.pack(pady=5, padx=10, fill=tk.X, side=tk.TOP)

        # Auto-Stop Controls (Row 0)
        self.stop_assist_var = tk.BooleanVar(value=False)
        self.stop_assist_checkbox = ttk.Checkbutton(assist_frame, text="Enable Auto-Stop", variable=self.stop_assist_var, command=self.on_stop_assist_changed, state=tk.DISABLED)
        self.stop_assist_checkbox.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Label(assist_frame, text="Brake Threshold (cm):").grid(row=0, column=1, padx=(10, 5), pady=5, sticky=tk.W)
        self.threshold_var = tk.StringVar(value=str(AUTO_BRAKE_THRESHOLD))
        self.threshold_entry = ttk.Entry(assist_frame, textvariable=self.threshold_var, width=6)
        self.threshold_entry.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)

        # --- Proximity Beep Control (Row 1) ---
        self.beep_checkbox = ttk.Checkbutton(
            assist_frame,
            text="Enable Proximity Beep",
            variable=self.proximity_beep_enabled_var,
            command=self.on_beep_toggle,
            state=tk.DISABLED # Initially disabled until sound check
        )
        self.beep_checkbox.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        # Check if sound loaded and enable checkbox if it did
        if self.main_app.beep_sound:
             self.beep_checkbox.config(state=tk.NORMAL)
        else:
             self.log_callback("Beep sound file not loaded. Proximity beep disabled.", is_warning=True)


        # Manual Brake Buttons (Right side, spanning rows?)
        self.manual_brake_button = ttk.Button(assist_frame, text="Manual Brake (S 1)", command=lambda: self.arduino_communicator.send_command("S 1"), state=tk.DISABLED)
        self.manual_brake_button.grid(row=0, column=3, padx=15, pady=5, sticky="ew", rowspan=1) # Adjust layout if needed
        self.manual_release_button = ttk.Button(assist_frame, text="Manual Release (Q 1)", command=lambda: self.arduino_communicator.send_command("Q 1"), state=tk.DISABLED)
        self.manual_release_button.grid(row=1, column=3, padx=15, pady=5, sticky="ew", rowspan=1)

        # Configure column weights for assist_frame if needed for layout
        assist_frame.columnconfigure(0, weight=1)
        assist_frame.columnconfigure(1, weight=0)
        assist_frame.columnconfigure(2, weight=0)
        assist_frame.columnconfigure(3, weight=0)


        # Canvas Frame (Bottom, Expanding)
        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(5,10), side=tk.BOTTOM)
        self.canvas = tk.Canvas(canvas_frame, bg=DARK_GRAY, relief=tk.SUNKEN, borderwidth=1, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Bind initial setup
        self._initial_map_bind_id = self.master.bind("<Map>", self._initial_canvas_setup, add="+")


    def _setup_canvas(self):
         self.canvas.update_idletasks()
         canvas_width = self.canvas.winfo_width(); canvas_height = self.canvas.winfo_height()
         if canvas_width < 50 or canvas_height < 50: return

         self.title_font = tkfont.Font(family="Arial", size=14, weight="bold")
         self.info_font = tkfont.Font(family="Arial", size=10)
         self.warning_font = tkfont.Font(family="Arial", size=12, weight="bold")

         self.center_x = canvas_width // 2; self.center_y = canvas_height * 0.85
         drawable_height = self.center_y * 0.95; drawable_width = canvas_width * 0.95
         self.scale = min(drawable_width / (MAX_DISTANCE * 2), drawable_height / MAX_DISTANCE)

         self.canvas.delete("text_info")
         self.distance_text = self.canvas.create_text(15, 15, anchor=tk.NW, text="Dist: -- cm", fill=LIGHT_GRAY, font=self.title_font, tags="text_info")
         self.warning_text = self.canvas.create_text(canvas_width / 2, 15, anchor=tk.N, text="", fill=RED, font=self.warning_font, tags="text_info")
         self.vehicle_info = self.canvas.create_text(self.center_x, canvas_height - 10, anchor=tk.S, text=f"W:{VEHICLE_WIDTH}cm V:{VIEW_ANGLE}° T:{self.threshold_var.get()}cm", fill=LIGHT_GRAY, font=self.info_font, tags="text_info")

         self.draw_reference_elements()


    def start_lidar(self):
        port = self.port_var.get()
        if not port: messagebox.showerror("Lidar Error", "Please enter a LIDAR port."); return
        self.lidar_processor.port_name = port
        if self.lidar_processor.connect():
             if self.lidar_processor.start_scan():
                 self.lidar_running = True
                 self.activate_button.config(state=tk.DISABLED); self.deactivate_button.config(state=tk.NORMAL)
                 self.port_entry.config(state=tk.DISABLED)
                 self.start_display_update()
                 self.log_callback(f"LIDAR activated on {port}", is_success=True)
                 self._setup_canvas() # Ensure redraw on activation
             else:
                 messagebox.showerror("Lidar Error", f"Failed to start Lidar scan on {port}.")
                 self.lidar_processor.disconnect_lidar()
        else:
            messagebox.showerror("Lidar Error", f"Could not connect to LIDAR on port {port}.")

    def stop_lidar(self):
        if not self.lidar_running: return
        self.stop_display_update()
        self.lidar_processor.stop_scan()
        self.lidar_running = False
        self.activate_button.config(state=tk.NORMAL); self.deactivate_button.config(state=tk.DISABLED)
        self.port_entry.config(state=tk.NORMAL)
        self.canvas.delete("scan", "obstacle", "closest", "closest_text")
        self.canvas.itemconfig(self.distance_text, text="Dist: -- cm"); self.canvas.itemconfig(self.warning_text, text="")
        self.log_callback("LIDAR deactivated.")
        self.was_too_close = False # Reset beep state when stopping


    def start_display_update(self):
        if not hasattr(self, 'scale'): self._setup_canvas()
        if hasattr(self, 'scale') and self.update_id is None:
            self.update_display()

    def stop_display_update(self):
        if self.update_id is not None:
            self.master.after_cancel(self.update_id); self.update_id = None

    def update_display(self):
        if not self.lidar_running or not self.canvas.winfo_exists():
            self.update_id = None; return

        scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()
        self.handle_auto_stop_assist(closest_distance)

        # --- Clear dynamic elements ---
        self.canvas.delete("scan", "obstacle", "closest", "closest_text")

        # --- Drawing Scan Points ---
        scan_point_size = 2
        for angle, distance in scan_data.items():
            if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2 and 0 < distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                self.canvas.create_oval(x - scan_point_size, y - scan_point_size, x + scan_point_size, y + scan_point_size, fill=SCAN_POINT_COLOR, outline="", tags="scan")

        # --- Drawing Obstacles ---
        obstacle_size_path = 5; obstacle_size_near = 4
        for angle, distance, in_path in obstacle_points:
             if 0 < distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                color = OBSTACLE_COLOR_PATH if in_path else OBSTACLE_COLOR_NEAR; size = obstacle_size_path if in_path else obstacle_size_near
                self.canvas.create_oval(x-size, y-size, x+size, y+size, fill=color, outline=DARK_GRAY, width=1, tags="obstacle")

        # --- Closest Distance & Warning & Beep Logic ---
        warn_msg = ""; warn_color = RED
        is_too_close = False # Current state for beep logic

        if closest_distance < float('inf'):
            x_close, y_close = self._polar_to_cartesian(closest_angle, closest_distance)
            self.canvas.create_line(self.center_x, self.center_y, x_close, y_close, fill=CLOSEST_POINT_COLOR, width=2, arrow=tk.LAST, tags="closest")
            self.canvas.create_text(x_close + 10, y_close, anchor=tk.W, text=f"{closest_distance:.1f}cm", fill=CLOSEST_POINT_COLOR, font=self.info_font, tags="closest_text")
            self.canvas.itemconfig(self.distance_text, text=f"Dist: {closest_distance:.1f} cm")

            try: threshold = float(self.threshold_var.get())
            except ValueError: threshold = AUTO_BRAKE_THRESHOLD

            is_too_close = 0 < closest_distance < threshold # Check if currently too close

            if is_too_close:
                is_braking = self.main_app.brake_active; brake_reason = self.main_app.brake_reason
                assist_enabled = self.stop_assist_var.get()
                if is_braking and brake_reason == "E-Stop": warn_msg = "AUTO E-STOP ACTIVE!" if assist_enabled else "MANUAL E-STOP ACTIVE!"
                elif is_braking: warn_msg = f"BRAKE ACTIVE ({brake_reason})!"; warn_color = YELLOW
                else: warn_msg = "E-STOP ADVISED!"; warn_color = YELLOW

            # --- Beep Logic: Play sound on transition INTO the threshold ---
            if self.proximity_beep_enabled and is_too_close and not self.was_too_close:
                if self.main_app.beep_sound:
                    try:
                        self.main_app.beep_sound.play()
                        # self.log_callback("Beep!", is_info=True) # Optional: Log beep event
                    except pygame.error as e:
                        self.log_callback(f"Error playing sound: {e}", is_error=True)
                else:
                    # Should not happen if checkbox is enabled, but check anyway
                    self.log_callback("Beep enabled but sound not loaded.", is_warning=True)

            # Update the 'previous state' tracker for the *next* iteration
            self.was_too_close = is_too_close

        else: # No closest distance
            self.canvas.itemconfig(self.distance_text, text="Dist: -- cm")
            self.was_too_close = False # Reset beep state if no obstacles

        # Update warning text
        self.canvas.itemconfig(self.warning_text, text=warn_msg, fill=warn_color)
        # Update vehicle info text
        self.canvas.itemconfig(self.vehicle_info, text=f"W:{VEHICLE_WIDTH}cm V:{VIEW_ANGLE}° T:{self.threshold_var.get()}cm")

        # Schedule next update
        self.update_id = self.master.after(100, self.update_display)


    def draw_reference_elements(self):
        self.canvas.delete("reference")
        canvas_width = self.canvas.winfo_width(); canvas_height = self.canvas.winfo_height()
        if not hasattr(self, 'scale') or canvas_width < 10 or canvas_height < 10: return

        # Distance Rings
        ring_interval_cm = 50 if MAX_DISTANCE > 200 else 25
        for r_cm in range(ring_interval_cm, int(MAX_DISTANCE) + 1, ring_interval_cm):
            radius_px = int(r_cm * self.scale); start_angle_tk = 90 - (-VIEW_ANGLE / 2); extent_tk = -VIEW_ANGLE
            if radius_px < 5: continue
            self.canvas.create_arc(self.center_x - radius_px, self.center_y - radius_px, self.center_x + radius_px, self.center_y + radius_px, start=start_angle_tk, extent=extent_tk, outline=MEDIUM_GRAY, style=tk.ARC, dash=(2, 4), tags="reference")
            if radius_px > 15: self.canvas.create_text(self.center_x, self.center_y - radius_px, text=f"{r_cm}cm", fill=LIGHT_GRAY, anchor=tk.S, font=self.info_font, tags="reference")

        # Angle Lines
        half_view = VIEW_ANGLE / 2; angle_interval = 30
        for angle in range(int(-half_view // angle_interval * angle_interval), int(half_view // angle_interval * angle_interval) + 1, angle_interval):
             if angle == 0 and VIEW_ANGLE < 359 : continue
             x_end, y_end = self._polar_to_cartesian(angle, MAX_DISTANCE); self.canvas.create_line(self.center_x, self.center_y, x_end, y_end, fill=MEDIUM_GRAY, dash=(2, 4), tags="reference")
             x_lbl, y_lbl = self._polar_to_cartesian(angle, MAX_DISTANCE * 1.05); anchor_val = tk.S if abs(angle) < 5 else tk.CENTER
             self.canvas.create_text(x_lbl, y_lbl, text=f"{angle}°", fill=LIGHT_GRAY, font=self.info_font, anchor=anchor_val, tags="reference")

        # Vehicle Outline
        vehicle_width_px = max(4, int(VEHICLE_WIDTH * self.scale)); half_width_px = vehicle_width_px // 2
        vehicle_height_px = max(6, int(25 * self.scale)); arrow_len = vehicle_height_px * 0.7
        self.canvas.create_rectangle(self.center_x - half_width_px, self.center_y - (vehicle_height_px * 0.4), self.center_x + half_width_px, self.center_y + (vehicle_height_px * 0.6), fill="", outline=VEHICLE_OUTLINE_COLOR, width=2, tags="reference")
        self.canvas.create_line(self.center_x, self.center_y + (vehicle_height_px * 0.1), self.center_x, self.center_y - arrow_len, fill=VEHICLE_OUTLINE_COLOR, width=2, arrow=tk.LAST, tags="reference")

        # Path Lines
        path_half_width_px = int((VEHICLE_WIDTH / 2) * self.scale)
        if path_half_width_px > 0:
            y_path_end = self._polar_to_cartesian(0, MAX_DISTANCE)[1]; x_path_left = self.center_x - path_half_width_px; x_path_right = self.center_x + path_half_width_px
            self.canvas.create_line(x_path_left, self.center_y, x_path_left, y_path_end, fill=VEHICLE_OUTLINE_COLOR, width=1, dash=(3, 6), tags="reference")
            self.canvas.create_line(x_path_right, self.center_y, x_path_right, y_path_end, fill=VEHICLE_OUTLINE_COLOR, width=1, dash=(3, 6), tags="reference")

        # Brake Threshold Arc
        try:
            threshold_cm = float(self.threshold_var.get())
            if 0 < threshold_cm <= MAX_DISTANCE:
                threshold_radius_px = int(threshold_cm * self.scale)
                if threshold_radius_px > 2:
                    start_angle_tk = 90 - (-VIEW_ANGLE / 2); extent_tk = -VIEW_ANGLE
                    self.canvas.create_arc(self.center_x - threshold_radius_px, self.center_y - threshold_radius_px, self.center_x + threshold_radius_px, self.center_y + threshold_radius_px, start=start_angle_tk, extent=extent_tk, style=tk.ARC, outline=BRAKE_ARC_COLOR, width=2, dash=(8, 4), tags="reference")
                    if threshold_radius_px > 30: self.canvas.create_text(self.center_x - threshold_radius_px * 0.707, self.center_y - threshold_radius_px * 0.707, text=f"{threshold_cm:.0f}cm", fill=BRAKE_ARC_COLOR, anchor=tk.NE, font=self.info_font, tags="reference")
        except ValueError: pass

    def _polar_to_cartesian(self, angle_deg, distance_cm):
        if not hasattr(self, 'scale'): return self.center_x, self.center_y
        math_angle_rad = math.radians(90 - angle_deg); dist_px = distance_cm * self.scale
        x = self.center_x + dist_px * math.cos(math_angle_rad); y = self.center_y - dist_px * math.sin(math_angle_rad)
        return int(x), int(y)

    def update_arduino_connection_state(self, connected):
        state = tk.NORMAL if connected else tk.DISABLED
        self.stop_assist_checkbox.config(state=state); self.threshold_entry.config(state=state)
        self.manual_brake_button.config(state=state); self.manual_release_button.config(state=state)
        # Beep checkbox depends on sound loaded AND arduino connected? Just sound loaded.
        # Re-enable/disable based on sound file status if needed here, but initial check should suffice.
        if not connected: self.stop_assist_var.set(False); self.main_app.stop_assist_active = False

    def on_stop_assist_changed(self):
        is_active = self.stop_assist_var.get()
        self.main_app.stop_assist_active = is_active
        self.log_callback(f"Lidar Auto-Stop Assist {'ENABLED' if is_active else 'DISABLED'}.")
        if not is_active and "AUTO E-STOP ACTIVE!" in self.canvas.itemcget(self.warning_text, "text"):
             self.canvas.itemconfig(self.warning_text, text="")

    # --- New method for beep toggle ---
    def on_beep_toggle(self):
        self.proximity_beep_enabled = self.proximity_beep_enabled_var.get()
        self.log_callback(f"Proximity Beep {'ENABLED' if self.proximity_beep_enabled else 'DISABLED'}.")
        if not self.proximity_beep_enabled:
            self.was_too_close = False # Reset state if disabled

    def handle_auto_stop_assist(self, closest_distance):
        if not self.main_app.stop_assist_active or not self.arduino_communicator.is_connected: return
        try: threshold = float(self.threshold_var.get())
        except ValueError: threshold = AUTO_BRAKE_THRESHOLD
        if 0 < closest_distance < threshold:
             if not (self.main_app.brake_active and self.main_app.brake_reason == "E-Stop"):
                 self.log_callback(f"Auto-Stop: Obstacle {closest_distance:.1f}cm < {threshold}cm. E-STOP!", is_error=True)
                 self.main_app.emergency_stop()


# --- Main Application Class (Added Sound Init/Quit) ---
class CombinedRobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Robot Control & Lidar Interface")
        self.root.geometry("1000x800") # Larger default size
        self.root.resizable(True, True)

        # --- Initialize Pygame and Mixer ---
        try:
            pygame.init() # General init
            pygame.mixer.init() # <<< Initialize mixer
            self.log_message("Pygame and Mixer initialized.", is_info=True)
            # --- Load Beep Sound ---
            self.beep_sound = None
            if os.path.exists(BEEP_SOUND_FILE):
                try:
                    self.beep_sound = pygame.mixer.Sound(BEEP_SOUND_FILE)
                    self.log_message(f"Loaded sound: {BEEP_SOUND_FILE}", is_success=True)
                except pygame.error as e:
                    self.log_message(f"Error loading sound '{BEEP_SOUND_FILE}': {e}", is_error=True)
            else:
                self.log_message(f"Sound file not found: {BEEP_SOUND_FILE}", is_warning=True)

        except pygame.error as e:
             self.log_message(f"Pygame initialization error: {e}", is_error=True)
             # Optionally exit or disable sound features
             self.beep_sound = None # Ensure it's None if init failed
        except Exception as e:
             self.log_message(f"Error during sound setup: {e}", is_error=True)
             self.beep_sound = None


        # --- Core Components ---
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.arduino_communicator = ArduinoCommunicator(self.command_queue, self.response_queue, self.log_message)
        self.xbox_controller = XboxController(log_callback=self.log_message) # Pygame init done above now
        self.lidar_processor = LidarProcessor(log_callback=self.log_message)

        # --- Shared State ---
        self.arduino_connected = False; self.is_automatic_mode = False
        self.brake_active = False; self.brake_reason = ""
        self.controller_polling_active = False; self.stop_assist_active = False

        # --- UI Setup ---
        self._create_styles()
        self._create_main_ui()

        # --- Background Tasks ---
        self.check_responses()
        self.poll_controller()

        # --- Resize Handling ---
        self._resize_job = None
        self.root.bind("<Configure>", self.on_resize)

    def _create_styles(self):
        self.style = ttk.Style()
        try: self.style.theme_use('clam')
        except tk.TclError: print("Clam theme not available.")
        self.style.configure("Emergency.TButton", foreground="white", background="red", font=("Arial", 12, "bold"))
        self.style.configure("TNotebook.Tab", padding=[10, 5], font=('Arial', 10))
        self.style.configure("TLabelframe.Label", font=('Arial', 10, 'bold'))
        # self.style.configure("Active.TButton", foreground="black", background="#A0FFA0")

    def _create_main_ui(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.robot_control_frame = ttk.Frame(self.notebook)
        self.lidar_frame = ttk.Frame(self.notebook)

        self.notebook.add(self.robot_control_frame, text="Robot Control")
        self.notebook.add(self.lidar_frame, text="Lidar Visualizer")

        self._create_robot_control_tab(self.robot_control_frame)

        # Instantiate LidarVisualizerFrame (pass self for access to beep_sound)
        self.lidar_visualizer = LidarVisualizerFrame(self.lidar_frame, self, self.lidar_processor, self.arduino_communicator, self.log_message)
        self.lidar_visualizer.pack(fill=tk.BOTH, expand=True)

        self.status_bar = ttk.Label(self.root, text="Status: Ready", relief=tk.SUNKEN, anchor=tk.W, padding=5)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)


    def _create_robot_control_tab(self, parent_frame):
        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(parent_frame, text="Arduino Connection")
        conn_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky='w')
        self.port_var = tk.StringVar(); self.port_combobox = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky='w')
        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=(10, 5), pady=5, sticky='w')
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD_RATE)); baud_rates = ["9600", "19200", "38400", "57600", "115200", "230400", "250000"]
        self.baud_combobox = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=baud_rates, width=8); self.baud_combobox.grid(row=0, column=3, padx=5, pady=5, sticky='w')
        self.refresh_button = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports, width=8); self.refresh_button.grid(row=0, column=4, padx=5, pady=5)
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect_arduino, width=8); self.connect_button.grid(row=0, column=5, padx=5, pady=5)
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_arduino, width=10, state=tk.DISABLED); self.disconnect_button.grid(row=0, column=6, padx=5, pady=5)
        self.refresh_ports()

        # --- Controller Frame ---
        controller_frame = ttk.LabelFrame(parent_frame, text="Xbox Controller")
        controller_frame.pack(fill=tk.X, padx=10, pady=5)
        self.controller_status_label = ttk.Label(controller_frame, text="Status: Disconnected", width=40); self.controller_status_label.pack(side=tk.LEFT, padx=10, pady=5)
        self.connect_controller_button = ttk.Button(controller_frame, text="Connect", command=self.connect_controller, width=10); self.connect_controller_button.pack(side=tk.LEFT, padx=5, pady=5)
        self.toggle_controller_button = ttk.Button(controller_frame, text="Enable Control", command=self.toggle_controller_polling, width=15, state=tk.DISABLED); self.toggle_controller_button.pack(side=tk.LEFT, padx=5, pady=5)

        # --- Main Area (Controls | Console) ---
        main_area = ttk.Frame(parent_frame); main_area.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # --- Manual Controls (Left) ---
        manual_controls_frame = ttk.LabelFrame(main_area, text="Manual Robot Controls"); manual_controls_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        mode_frame = ttk.Frame(manual_controls_frame); mode_frame.pack(fill=tk.X, pady=5)
        self.mode_button = ttk.Button(mode_frame, text="Switch to AUTOMATIC", command=self.toggle_mode, state=tk.DISABLED); self.mode_button.pack(fill=tk.X, padx=10, pady=5)
        steer_frame = ttk.Frame(manual_controls_frame); steer_frame.pack(fill=tk.X, pady=5)
        ttk.Label(steer_frame, text="Steer:", width=6).pack(side=tk.LEFT, padx=(10,0))
        self.steering_slider = ttk.Scale(steer_frame, from_=self.xbox_controller.STEERING_MIN, to=self.xbox_controller.STEERING_MAX, orient="horizontal", command=self.on_steering_change, state=tk.DISABLED, length=200); self.steering_slider.set(self.xbox_controller.STEERING_CENTER); self.steering_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.steering_value_label = ttk.Label(steer_frame, text=str(self.xbox_controller.STEERING_CENTER), width=5); self.steering_value_label.pack(side=tk.LEFT, padx=(0,10))
        drive_frame = ttk.Frame(manual_controls_frame); drive_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        accel_subframe = ttk.Frame(drive_frame); accel_subframe.pack(side=tk.LEFT, fill=tk.Y, padx=10)
        ttk.Label(accel_subframe, text="Accel:").pack(anchor='n')
        self.accel_slider = ttk.Scale(accel_subframe, from_=self.xbox_controller.ACCEL_MAX, to=0, orient="vertical", command=self.on_accel_change, state=tk.DISABLED, length=150); self.accel_slider.set(0); self.accel_slider.pack(fill=tk.Y, expand=True, pady=5)
        self.accel_value_label = ttk.Label(accel_subframe, text="0", width=3); self.accel_value_label.pack(anchor='s')
        buttons_subframe = ttk.Frame(drive_frame); buttons_subframe.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        dir_frame = ttk.LabelFrame(buttons_subframe, text="Direction"); dir_frame.pack(fill=tk.X, pady=5)
        self.forward_btn = ttk.Button(dir_frame, text="Forward (D F)", command=lambda: self.send_direction("F"), state=tk.DISABLED); self.forward_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=2)
        self.reverse_btn = ttk.Button(dir_frame, text="Reverse (D R)", command=lambda: self.send_direction("R"), state=tk.DISABLED); self.reverse_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=2)
        brake_frame = ttk.LabelFrame(buttons_subframe, text="Brake Control"); brake_frame.pack(fill=tk.X, pady=5)
        self.brake_btn = ttk.Button(brake_frame, text="Engage Brake (S 1)", command=self.toggle_brake, state=tk.DISABLED); self.brake_btn.pack(fill=tk.X, padx=5, pady=2)
        estop_frame = ttk.LabelFrame(buttons_subframe, text="Emergency"); estop_frame.pack(fill=tk.X, pady=10)
        self.stop_btn = ttk.Button(estop_frame, text="EMERGENCY STOP", command=self.emergency_stop, style="Emergency.TButton", state=tk.DISABLED); self.stop_btn.pack(fill=tk.X, padx=5, pady=5)

        # --- Console (Right) ---
        console_frame = ttk.LabelFrame(main_area, text="Arduino/System Log"); console_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        self.response_console = scrolledtext.ScrolledText(console_frame, wrap=tk.WORD, height=15, state=tk.DISABLED, relief=tk.SUNKEN, borderwidth=1); self.response_console.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.response_console.tag_configure("INFO", foreground="black"); self.response_console.tag_configure("ERROR", foreground="red"); self.response_console.tag_configure("WARNING", foreground="#FF8C00"); self.response_console.tag_configure("SUCCESS", foreground="green"); self.response_console.tag_configure("CMD", foreground="blue"); self.response_console.tag_configure("RECV", foreground="#551A8B")
        clear_button = ttk.Button(console_frame, text="Clear Log", command=self.clear_console); clear_button.pack(fill=tk.X, padx=5, pady=(0, 5))

    # --- UI Update and State Management ---
    def update_ui_based_on_connection(self):
        self.arduino_connected = self.arduino_communicator.is_connected
        state = tk.NORMAL if self.arduino_connected else tk.DISABLED
        conn_state = tk.DISABLED if self.arduino_connected else tk.NORMAL
        self.connect_button.config(state=conn_state); self.port_combobox.config(state=conn_state); self.baud_combobox.config(state=conn_state); self.refresh_button.config(state=conn_state); self.disconnect_button.config(state=state)
        self.mode_button.config(state=state)
        controller_hw_connected = self.xbox_controller.is_connected()
        self.toggle_controller_button.config(state=tk.NORMAL if (controller_hw_connected and self.arduino_connected) else tk.DISABLED)
        if not controller_hw_connected or not self.arduino_connected: self.controller_polling_active = False; self.toggle_controller_button.config(text="Enable Control")
        if hasattr(self, 'lidar_visualizer'): self.lidar_visualizer.update_arduino_connection_state(self.arduino_connected)
        self.update_ui_based_on_mode()

    def update_ui_based_on_mode(self):
        if not self.arduino_connected: state_auto = state_manual = state_always = tk.DISABLED
        else: state_auto = tk.NORMAL if self.is_automatic_mode else tk.DISABLED; state_manual = tk.DISABLED; state_always = tk.NORMAL # E-Stop always enabled
        self.steering_slider.config(state=tk.DISABLED); self.accel_slider.config(state=tk.DISABLED)
        self.forward_btn.config(state=state_auto); self.reverse_btn.config(state=state_auto)
        self.brake_btn.config(state=state_auto); self.stop_btn.config(state=state_always)
        if self.arduino_connected: self.mode_button.config(text=f"Switch to {'MANUAL' if self.is_automatic_mode else 'AUTOMATIC'}")

    def update_brake_button_text(self):
         self.brake_btn.config(text=f"Release Brake (Q 1) [{self.brake_reason}]" if self.brake_active else "Engage Brake (S 1)")

    # --- Action Handlers ---
    def refresh_ports(self):
        self.log_message("Refreshing serial ports...")
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combobox['values'] = ports
            if ports:
                current_val = self.port_var.get()
                default_port = DEFAULT_ARDUINO_PORT # Use constant
                if current_val in ports: self.port_combobox.set(current_val)
                elif default_port in ports: self.port_combobox.set(default_port)
                else: self.port_combobox.current(0)
                self.log_message(f"Available ports: {', '.join(ports)}")
            else: self.log_message("No serial ports found.", is_warning=True)
        except Exception as e: self.log_message(f"Error refreshing ports: {e}", is_error=True)

    def connect_arduino(self):
        port = self.port_var.get(); baud = self.baud_var.get()
        if not port: messagebox.showwarning("Connect", "Select serial port."); return
        if not baud: messagebox.showwarning("Connect", "Select baud rate."); return
        if self.arduino_communicator.is_connected: self.disconnect_arduino(); # time.sleep(0.2)
        self.update_status(f"Connecting to {port}...")
        if self.arduino_communicator.connect(port, int(baud)):
            self.update_status(f"Connected to {port}")
            self.is_automatic_mode = False; self.brake_active = False
            self.update_brake_button_text(); self.update_ui_based_on_connection()
            if not self.xbox_controller.is_connected(): self.connect_controller()
        else: messagebox.showerror("Connection Failed", f"Could not connect to Arduino on {port}."); self.update_status("Connection failed"); self.update_ui_based_on_connection()

    def disconnect_arduino(self):
        if hasattr(self, 'lidar_visualizer') and self.lidar_visualizer.stop_assist_var.get():
             self.lidar_visualizer.stop_assist_var.set(False)
             self.lidar_visualizer.on_stop_assist_changed()
        self.arduino_communicator.disconnect()
        self.update_status("Disconnected"); self.is_automatic_mode = False; self.brake_active = False
        self.controller_polling_active = False
        self.update_ui_based_on_connection(); self.update_brake_button_text()
        self.toggle_controller_button.config(text="Enable Control")

    def connect_controller(self):
        self.update_status("Connecting Xbox controller...")
        if self.xbox_controller.connect():
             self.controller_status_label.config(text=f"Status: Connected ({self.xbox_controller.controller_name})")
             self.connect_controller_button.config(state=tk.DISABLED); self.update_status("Xbox controller connected")
        else:
             self.controller_status_label.config(text="Status: Connection Failed"); self.connect_controller_button.config(state=tk.NORMAL)
             self.update_status("Xbox controller connection failed"); messagebox.showerror("Controller Error", "Failed to connect controller.")
        self.update_ui_based_on_connection()

    def toggle_controller_polling(self):
        if not self.arduino_connected or not self.xbox_controller.is_connected():
             self.log_message("Cannot enable controller: Arduino or controller not connected.", is_warning=True); self.controller_polling_active = False; self.toggle_controller_button.config(text="Enable Control"); return
        self.controller_polling_active = not self.controller_polling_active
        if self.controller_polling_active:
             if not self.is_automatic_mode: self.log_message("Switching to AUTOMATIC mode for controller.", is_info=True); self.set_mode(automatic=True)
             self.log_message("Xbox controller polling ENABLED.", is_success=True); self.toggle_controller_button.config(text="Disable Control"); self.update_status("Controller Active")
        else:
             self.log_message("Xbox controller polling DISABLED."); self.toggle_controller_button.config(text="Enable Control"); self.update_status("Controller Inactive")
             self.update_ui_based_on_mode()
             self.arduino_communicator.send_command("V 0")
             self.accel_slider.set(0); self.accel_value_label.config(text="0")

    def set_mode(self, automatic: bool):
        if not self.arduino_connected: return
        target_mode = "A" if automatic else "M"; current_mode = "A" if self.is_automatic_mode else "M"
        if target_mode == current_mode: return
        if self.arduino_communicator.send_command(target_mode):
             self.is_automatic_mode = automatic; mode_name = "Automatic" if automatic else "Manual"
             self.log_message(f"Switched to {mode_name} mode ({target_mode}).", is_success=True)
             self.arduino_communicator.send_command("V 0"); self.arduino_communicator.send_command(f"W {self.xbox_controller.STEERING_CENTER}")
             if automatic: self.arduino_communicator.send_command("D F")
             self.update_ui_based_on_mode()
             self.accel_slider.set(0); self.accel_value_label.config(text="0")
             self.steering_slider.set(self.xbox_controller.STEERING_CENTER); self.steering_value_label.config(text=str(self.xbox_controller.STEERING_CENTER))
             self.update_status(f"Mode: {mode_name}")
             if not automatic and self.controller_polling_active: self.controller_polling_active = False; self.toggle_controller_button.config(text="Enable Control"); self.log_message("Controller polling disabled (Manual mode).")
        else: self.log_message(f"Failed to send {target_mode} command.", is_error=True)

    def toggle_mode(self): self.set_mode(not self.is_automatic_mode)
    def on_steering_change(self, value_str): # Manual slider drag does not send commands
        if not self.arduino_connected: return
        self.steering_value_label.config(text=str(int(float(value_str))))
    def on_accel_change(self, value_str): # Manual slider drag does not send commands
        if not self.arduino_connected: return
        self.accel_value_label.config(text=str(int(float(value_str))))

    def send_direction(self, direction):
        if self.is_automatic_mode and self.arduino_connected:
            if self.arduino_communicator.send_command(f"D {direction}"): self.log_message(f"Dir set: {'Fwd' if direction == 'F' else 'Rev'}.")
        else: self.log_message("Cannot set dir: Not Auto or not connected.", is_warning=True)

    def update_brake_state(self, active, reason="Unknown"):
        self.brake_active = active; self.brake_reason = reason if active else ""
        self.update_brake_button_text()
        if active: self.accel_slider.set(0); self.accel_value_label.config(text="0")

    def toggle_brake(self):
        if not self.is_automatic_mode or not self.arduino_connected: self.log_message("Cannot toggle brake: Not Auto or not connected.", is_warning=True); return
        if self.brake_active:
            self.log_message("Manually releasing brake (Q 1)...")
            if self.arduino_communicator.send_command("Q 1"): self.update_brake_state(False, "Manual"); self.log_message("Brake released manually.")
            else: self.log_message("Failed to send Q 1.", is_error=True)
        else:
            self.log_message("Manually engaging brake (S 1)...")
            if self.arduino_communicator.send_command("S 1"): self.update_brake_state(True, "Manual"); self.log_message("Brake engaged manually."); self.arduino_communicator.send_command("V 0") # Ensure stop
            else: self.log_message("Failed to send S 1.", is_error=True)

    def emergency_stop(self):
        if not self.arduino_connected: self.log_message("Cannot E-STOP: Arduino disconnected.", is_error=True); return
        self.log_message("EMERGENCY STOP ACTIVATED!", is_error=True); self.update_status("EMERGENCY STOP")
        self.arduino_communicator.send_command("V 0"); time.sleep(0.05) # Stop accel first
        if self.arduino_communicator.send_command("S 1"):
             self.update_brake_state(True, "E-Stop")
             self.accel_slider.set(0); self.accel_value_label.config(text="0")
             self.log_message("E-Stop: V 0 and S 1 sent.")
        else: self.log_message("E-Stop: Failed to send S 1.", is_error=True)

    def release_emergency_stop(self):
         if self.brake_active and self.brake_reason == "E-Stop":
              self.log_message("Releasing E-Stop brake (Q 1)...")
              if self.arduino_communicator.send_command("Q 1"): self.update_brake_state(False, "E-Stop Release"); self.log_message("E-Stop brake released.", is_success=True)
              else: self.log_message("Failed to send Q 1 for E-Stop release.", is_error=True)
         elif self.brake_active: self.log_message(f"Cannot release E-stop: Brake active (Reason: '{self.brake_reason}').", is_warning=True)
         else: self.log_message("Cannot release E-stop: Brake not active.", is_warning=True)

    # --- Background Tasks ---
    def check_responses(self):
        try:
            while not self.response_queue.empty():
                response = self.response_queue.get_nowait()
                self.log_message(f"Recv: {response}", is_recv=True)
                self.response_queue.task_done()
        except queue.Empty: pass
        except Exception as e: self.log_message(f"Response queue error: {e}", is_error=True)
        finally: self.root.after(50, self.check_responses)

    def poll_controller(self):
        try:
            if self.xbox_controller.is_connected():
                success, event = self.xbox_controller.update()
                if not success:
                    if self.controller_polling_active: # Log only if disconnect was unexpected
                        self.controller_status_label.config(text="Status: Disconnected (Error)"); self.connect_controller_button.config(state=tk.NORMAL); self.toggle_controller_button.config(state=tk.DISABLED, text="Enable Control"); self.controller_polling_active = False; self.update_status("Xbox controller disconnected")
                elif self.controller_polling_active and self.is_automatic_mode and self.arduino_connected:
                    # Steering/Accel
                    if self.xbox_controller.has_steering_changed():
                        steer_val = self.xbox_controller.last_sent_steering; self.arduino_communicator.send_command(f"W {steer_val}"); self.steering_slider.set(steer_val); self.steering_value_label.config(text=str(steer_val))
                    if self.xbox_controller.has_accel_changed():
                        accel_val = self.xbox_controller.last_sent_accel
                        if not self.brake_active: self.arduino_communicator.send_command(f"V {accel_val}")
                        self.accel_slider.set(accel_val); self.accel_value_label.config(text=str(accel_val))
                    # Events
                    if event == "HAT_CHANGED":
                        hat_x, hat_y = self.xbox_controller.get_hat_values();
                        if hat_y == 1: self.send_direction("F")
                        elif hat_y == -1: self.send_direction("R")
                    elif event == "LT_PRESSED":
                        if not self.brake_active: self.log_message("Ctrl: LT -> Brake ON (S 1)"); self.arduino_communicator.send_command("S 1"); self.update_brake_state(True, "Controller LT")
                    elif event == "LT_RELEASED":
                         if self.brake_active and self.brake_reason == "Controller LT": self.log_message("Ctrl: LT -> Brake OFF (Q 1)"); self.arduino_communicator.send_command("Q 1"); self.update_brake_state(False, "Controller LT")
                         elif self.brake_active: self.log_message(f"Ctrl: LT Released, brake remains ON (Reason: {self.brake_reason}).", is_info=True)
                    elif event == "X_PRESSED":
                         if self.brake_active and self.brake_reason == "E-Stop": self.log_message("Ctrl: X -> Release E-Stop..."); self.release_emergency_stop()
                         elif self.brake_active: self.log_message(f"Ctrl: X pressed, brake ON (Reason: {self.brake_reason}). No action.", is_warning=True)
                         else: self.log_message("Ctrl: X pressed, brake OFF. No action.", is_info=True)
            self.root.after(50, self.poll_controller) # Poll ~20/sec
        except Exception as e: self.log_message(f"Controller poll error: {e}", is_error=True); self.root.after(100, self.poll_controller)

    # --- Logging and Status ---
    def log_message(self, message, is_error=False, is_warning=False, is_success=False, is_info=False, is_cmd=False, is_recv=False):
        timestamp = time.strftime("%H:%M:%S"); full_message = f"[{timestamp}] {message}\n"
        tag = "INFO"; # Default
        if is_error: tag = "ERROR"; print(f"ERROR: {message}") # Also print errors to console
        elif is_warning: tag = "WARNING"
        elif is_success: tag = "SUCCESS"
        elif is_cmd: tag = "CMD"
        elif is_recv: tag = "RECV"
        elif is_info: tag = "INFO"
        try:
            if self.response_console and self.response_console.winfo_exists():
                self.response_console.config(state=tk.NORMAL); self.response_console.insert(tk.END, full_message, (tag,)); self.response_console.see(tk.END); self.response_console.config(state=tk.DISABLED)
        except tk.TclError: print(f"LOG ({tag}): {full_message.strip()}") # Fallback
        except Exception as e: print(f"LOGGING ERROR: {e}\nOriginal Msg ({tag}): {full_message.strip()}")
    def clear_console(self):
        if self.response_console: self.response_console.config(state=tk.NORMAL); self.response_console.delete(1.0, tk.END); self.response_console.config(state=tk.DISABLED); self.log_message("Log cleared.", is_info=True)
    def update_status(self, message):
        if self.status_bar: self.status_bar.config(text=f"Status: {message} ({time.strftime('%H:%M:%S')})")

    # --- Resize Handling ---
    def on_resize(self, event):
        if event.widget == self.root:
            if self._resize_job is not None: self.root.after_cancel(self._resize_job)
            self._resize_job = self.root.after(250, self._handle_resize_end)
    def _handle_resize_end(self):
         self._resize_job = None
         if hasattr(self, 'lidar_visualizer') and self.lidar_visualizer.winfo_exists():
             self.lidar_visualizer._setup_canvas()

    # --- Application Shutdown ---
    def on_closing(self):
        self.log_message("----- Application Closing -----", is_warning=True); self.update_status("Closing...")
        if self._resize_job is not None: self.root.after_cancel(self._resize_job)
        if hasattr(self, 'lidar_visualizer'): self.lidar_visualizer.stop_display_update()
        if self.lidar_processor: self.lidar_processor.disconnect_lidar()
        if self.arduino_communicator:
            if self.arduino_communicator.is_connected: self.arduino_communicator.send_command("V 0"); self.arduino_communicator.send_command("S 1"); time.sleep(0.1)
            self.arduino_communicator.shutdown()
        if self.xbox_controller: self.xbox_controller.disconnect()
        # --- Quit Pygame Mixer and Pygame ---
        if pygame.mixer.get_init():
             pygame.mixer.quit()
             self.log_message("Pygame Mixer shut down.")
        if pygame.get_init():
             pygame.quit()
             self.log_message("Pygame shut down.")
        # --- Destroy GUI ---
        self.log_message("Destroying GUI..."); self.root.destroy(); print("Application closed.")

# --- Main Execution ---
if __name__ == "__main__":
    root = tk.Tk()
    app = CombinedRobotApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    # Check if controller connected initially and update status label
    if app.xbox_controller.is_connected():
         app.controller_status_label.config(text=f"Status: Connected ({app.xbox_controller.controller_name})")
         app.connect_controller_button.config(state=tk.DISABLED)
         app.update_ui_based_on_connection() # Ensure toggle button state is correct
    root.mainloop()