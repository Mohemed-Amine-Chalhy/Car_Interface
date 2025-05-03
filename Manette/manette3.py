import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import queue
import serial
import serial.tools.list_ports
import time
import pygame  # For Xbox controller support

class ArduinoCommunicator:
   def __init__(self, command_queue, response_queue):
       self.serial_conn = None
       self.is_connected = False
       self.command_queue = command_queue
       self.response_queue = response_queue
       self.running = True
       
       # Start command processing thread
       self.command_thread = threading.Thread(target=self._process_commands)
       self.command_thread.daemon = True
       self.command_thread.start()
       
       # Start response reading thread
       self.response_thread = threading.Thread(target=self._read_responses)
       self.response_thread.daemon = True
       self.response_thread.start()
   
   def connect(self, port, baud_rate=115200):  # Higher baud rate for ESP32
       try:
           if self.serial_conn and self.is_connected:
               self.serial_conn.close()
               self.is_connected = False
           
           self.serial_conn = serial.Serial(port, baud_rate, timeout=0.5)  # Shorter timeout
           time.sleep(1)  # Reduced wait time after connection
           self.is_connected = True
           return True
       except Exception as e:
           print(f"Connection error: {e}")
           self.is_connected = False
           return False
   
   def disconnect(self):
       if self.serial_conn and self.is_connected:
           self.serial_conn.close()
           self.is_connected = False
   
   def send_command(self, command):
       if self.is_connected:
           self.command_queue.put(command)
           return True
       return False
   
   def _process_commands(self):
       last_command_time = 0
       while self.running:
           if not self.command_queue.empty() and self.is_connected:
               current_time = time.time()
               # Add a small delay between commands for ESP32
               if current_time - last_command_time < 0.05:  # 50ms between commands
                   time.sleep(0.01)
                   continue
                   
               command = self.command_queue.get()
               try:
                   self.serial_conn.write(f"{command}\n".encode())
                   self.serial_conn.flush()  # Ensure data is sent immediately
                   last_command_time = time.time()
                   print(f"Sent: {command}")
               except Exception as e:
                   print(f"Error sending command: {e}")
               finally:
                   self.command_queue.task_done()
           time.sleep(0.01)  # Small delay to prevent CPU hogging
   
   def _read_responses(self):
       buffer = ""
       while self.running:
           if self.is_connected and self.serial_conn:
               try:
                   if self.serial_conn.in_waiting > 0:
                       data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='replace')
                       buffer += data
                       
                       # Process complete lines
                       while '\n' in buffer:
                           line, buffer = buffer.split('\n', 1)
                           line = line.strip()
                           if line:
                               print(f"Received: {line}")
                               self.response_queue.put(line)
               except Exception as e:
                   print(f"Error reading response: {e}")
           time.sleep(0.01)  # Small delay to prevent CPU hogging
   
   def shutdown(self):
       self.running = False
       if self.command_thread.is_alive():
           self.command_thread.join(timeout=1.0)
       if self.response_thread.is_alive():
           self.response_thread.join(timeout=1.0)
       self.disconnect()


class XboxController:
    """Xbox controller input handler for robot control"""
    
    def __init__(self, controller_id=0):
        # Initialize pygame for controller support
        pygame.init()
        pygame.joystick.init()
        
        self.connected = False
        self.controller = None
        self.controller_id = controller_id
    
        # Controller mapping for Xbox 360
        self.AXIS_RIGHT_STICK_X = 0  # Right stick horizontal axis
        self.AXIS_RT = 5  # Right trigger
        self.AXIS_LT = 4  # Left trigger
        
        # Status variables
        self.right_stick_x = 0.0
        self.rt_value = 0.0
        self.lt_value = 0.0
        self.lt_pressed = False
        
        # Add deadzone to prevent noise from idle controller
        self.DEADZONE = 0.15
        
        # Cache the last sent values to prevent duplicates
        self.last_steering = 1500
        self.last_accel = 0
        
            
        
        
        # Add hat mapping for D-pad
        self.HAT_DPAD = 0  # D-pad is typically hat 0
        
        # Add hat status tracking
        self.hat_values = (0, 0)
        self.last_hat_values = (0, 0)
        
        # Connect to the controller
        self.connect()
        
    def connect(self):
        """Connect to the Xbox controller"""
        try:
            if pygame.joystick.get_count() > self.controller_id:
                self.controller = pygame.joystick.Joystick(self.controller_id)
                self.controller.init()
                self.connected = True
                print(f"Connected to {self.controller.get_name()}")
                return True
            else:
                print("Xbox controller not found")
                return False
        except Exception as e:
            print(f"Controller connection error: {e}")
            self.connected = False
            return False
            
    def disconnect(self):
        """Disconnect from the controller"""
        if self.controller:
            self.controller.quit()
        self.connected = False
        
    def update(self):
        """Update controller input values"""
        if not self.connected:
            return False
            
        # Process events to get fresh data
        pygame.event.pump()
        
        # Get right stick X axis (steering)
        try:
            self.right_stick_x = self.controller.get_axis(self.AXIS_RIGHT_STICK_X)
            # Get trigger values
            self.rt_value = self.controller.get_axis(self.AXIS_RT)
            self.lt_value = self.controller.get_axis(self.AXIS_LT)
            # Get hat (D-pad) values
            self.last_hat_values = self.hat_values
            self.hat_values = self.controller.get_hat(self.HAT_DPAD)
            
            
            
            
            
            
            # Detect LT press/release for brake toggle
            lt_pressed_now = self.lt_value > 0.5
            if lt_pressed_now and not self.lt_pressed:
                # LT just pressed
                self.lt_pressed = True
                return True, "LT_PRESSED"
            elif not lt_pressed_now and self.lt_pressed:
                # LT just released
                self.lt_pressed = False
                return True, "LT_RELEASED"
            if self.hat_values != self.last_hat_values:
                return True, "HAT_CHANGED"
                
            return True, None
        except Exception as e:
            print(f"Error reading controller: {e}")
            self.connected = False
            return False, None
        
        
    def get_hat_values(self):
        """Return the current hat (D-pad) values"""
        return self.hat_values
    
    def get_steering_value(self):
        """Map right stick X axis to steering value (0-3000) with deadzone"""
        # Apply deadzone to eliminate small movements around center
        if abs(self.right_stick_x) < self.DEADZONE:
            return 1500  # Center position
            
        # Convert -1.0 to 1.0 range to 0-3000 range with deadzone adjustment
        # Scale the remaining range to still use the full output range
        adjusted_value = (abs(self.right_stick_x) - self.DEADZONE) / (1 - self.DEADZONE)
        if self.right_stick_x < 0:
            adjusted_value = -adjusted_value
            
        steering = int(1500 + (adjusted_value * 1500))
        
        # Store as last value for change detection
        self.last_steering = steering
        return steering
    
    def get_acceleration_value(self):
        """Map RT to acceleration value (0-50) with deadzone"""
        # RT goes from -1 (released) to 1 (fully pressed)
        # Apply deadzone to RT
        normalized = (self.rt_value + 1) / 2  # Convert from -1,1 to 0,1
        
        if normalized < self.DEADZONE:
            return 0  # Zero acceleration in deadzone
            
        # Adjust the range to still use full output range
        adjusted_value = (normalized - self.DEADZONE) / (1 - self.DEADZONE)
        accel = int(adjusted_value * 50)
        
        # Store last value for change detection
        self.last_accel = accel
        return accel
    
    def is_connected(self):
        """Check if controller is connected"""
        return self.connected


class RobotControlGUI:
   def __init__(self, root):
       self.root = root
       self.root.title("ESP32 Robot Control Interface with Xbox 360 Support")
       self.root.geometry("800x600")  # Increased size to accommodate response console
       
       self.command_queue = queue.Queue()
       self.response_queue = queue.Queue()
       self.arduino = ArduinoCommunicator(self.command_queue, self.response_queue)
       
       # Initialize Xbox controller
       self.xbox = XboxController()
       self.controller_active = False
       
       self.is_automatic_mode = False
       self.brake_active = False
       
       self.create_widgets()
       self.update_ui_based_on_mode()
       
       # Start a periodic task to check for responses
       self.check_responses()
       
       # Start controller polling if connected
       if self.xbox.connected:
           self.controller_active = True
           self.update_xbox_status("Xbox controller connected")
           self.poll_controller()
   
   def create_widgets(self):
       # Frame for connection settings
       connection_frame = ttk.LabelFrame(self.root, text="Connection Settings")
       connection_frame.pack(fill="x", padx=10, pady=10)
       
       # Serial port selection
       ttk.Label(connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
       self.port_var = tk.StringVar()
       self.port_combobox = ttk.Combobox(connection_frame, textvariable=self.port_var)
       self.port_combobox.grid(row=0, column=1, padx=5, pady=5)
       
       # Baud rate selection
       ttk.Label(connection_frame, text="Baud Rate:").grid(row=0, column=2, padx=5, pady=5)
       self.baud_var = tk.StringVar(value="115200")  # Default for ESP32
       baud_rates = ["9600", "19200", "38400", "57600", "115200"]
       self.baud_combobox = ttk.Combobox(connection_frame, textvariable=self.baud_var, values=baud_rates, width=10)
       self.baud_combobox.grid(row=0, column=3, padx=5, pady=5)
       
       self.refresh_ports()
       
       ttk.Button(connection_frame, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=4, padx=5, pady=5)
       ttk.Button(connection_frame, text="Connect", command=self.connect_arduino).grid(row=0, column=5, padx=5, pady=5)
       ttk.Button(connection_frame, text="Disconnect", command=self.disconnect_arduino).grid(row=0, column=6, padx=5, pady=5)
       
       # Controller connection
       controller_frame = ttk.LabelFrame(self.root, text="Controller Settings")
       controller_frame.pack(fill="x", padx=10, pady=5)
       
       self.controller_status = ttk.Label(controller_frame, text="Xbox controller: Disconnected")
       self.controller_status.pack(side="left", padx=10, pady=5)
       
       ttk.Button(controller_frame, text="Connect Controller", command=self.connect_controller).pack(side="left", padx=10, pady=5)
       ttk.Button(controller_frame, text="Toggle Controller", command=self.toggle_controller).pack(side="left", padx=10, pady=5)
       
       # Mode toggle
       mode_frame = ttk.LabelFrame(self.root, text="Drive Mode")
       mode_frame.pack(fill="x", padx=10, pady=5)
       
       self.mode_var = tk.StringVar(value="Manual")
       self.mode_toggle = ttk.Button(mode_frame, textvariable=self.mode_var, command=self.toggle_mode)
       self.mode_toggle.pack(fill="x", padx=10, pady=10)
       
       # Main content frame with controls and response console
       main_frame = ttk.Frame(self.root)
       main_frame.pack(fill="both", expand=True, padx=10, pady=10)
       
       # Control frame (left side)
       control_frame = ttk.LabelFrame(main_frame, text="Robot Controls")
       control_frame.pack(side="left", fill="both", expand=True, padx=(0, 5), pady=0)
       
       # Steering slider (horizontal)
       steering_frame = ttk.Frame(control_frame)
       steering_frame.pack(fill="x", pady=10)
       ttk.Label(steering_frame, text="Steering:").pack(side="left", padx=10)
       self.steering_slider = ttk.Scale(steering_frame, from_=0, to=3000, orient="horizontal", 
                                        command=self.on_steering_change)
       self.steering_slider.pack(side="left", fill="x", expand=True, padx=10)
       self.steering_slider.set(1500)  # Center position
       
       # Acceleration slider (vertical)
       accel_frame = ttk.Frame(control_frame)
       accel_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
       ttk.Label(accel_frame, text="Acceleration:").pack(anchor="n")
       self.accel_slider = ttk.Scale(accel_frame, from_=50, to=0, orient="vertical", 
                                     command=self.on_accel_change)
       self.accel_slider.pack(fill="both", expand=True, pady=5)
       self.accel_slider.set(0)
       
       # Direction and emergency controls
       buttons_frame = ttk.Frame(control_frame)
       buttons_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)
       
       # Direction buttons
       dir_frame = ttk.LabelFrame(buttons_frame, text="Direction")
       dir_frame.pack(fill="x", pady=5)
       self.forward_btn = ttk.Button(dir_frame, text="Forward", command=lambda: self.send_direction("F"))
       self.forward_btn.pack(fill="x", pady=2)
       self.reverse_btn = ttk.Button(dir_frame, text="Reverse", command=lambda: self.send_direction("R"))
       self.reverse_btn.pack(fill="x", pady=2)
       
       # Brake control
       brake_frame = ttk.LabelFrame(buttons_frame, text="Brake")
       brake_frame.pack(fill="x", pady=5)
       self.brake_btn = ttk.Button(brake_frame, text="Activate Brake", command=self.toggle_brake)
       self.brake_btn.pack(fill="x", pady=2)
       
       # Emergency stop
       emergency_frame = ttk.LabelFrame(buttons_frame, text="Emergency")
       emergency_frame.pack(fill="x", pady=5)
       self.stop_btn = ttk.Button(emergency_frame, text="EMERGENCY STOP", 
                                 command=self.emergency_stop, style="Emergency.TButton")
       self.stop_btn.pack(fill="x", pady=5)
       
       # Response console (right side)
       response_frame = ttk.LabelFrame(main_frame, text="ESP32 Responses")
       response_frame.pack(side="right", fill="both", expand=True, padx=(5, 0), pady=0)
       
       # Console with scrollbar
       self.response_console = scrolledtext.ScrolledText(response_frame, wrap=tk.WORD, height=15)
       self.response_console.pack(fill="both", expand=True, padx=5, pady=5)
       self.response_console.config(state=tk.DISABLED)  # Make it read-only
       
       # Clear console button
       ttk.Button(response_frame, text="Clear Console", command=self.clear_console).pack(fill="x", padx=5, pady=5)
       
       # Create a custom style for the emergency button
       style = ttk.Style()
       style.configure("Emergency.TButton", foreground="white", background="red", font=("Arial", 12, "bold"))
   
   def connect_controller(self):
       """Attempt to connect to Xbox controller"""
       if self.xbox.connect():
           self.controller_active = True
           self.update_xbox_status("Xbox controller connected")
           self.poll_controller()
       else:
           self.update_xbox_status("Xbox controller not found")
   
   def toggle_controller(self):
       """Enable/disable controller input"""
       self.controller_active = not self.controller_active
       status = "enabled" if self.controller_active else "disabled"
       self.update_xbox_status(f"Xbox controller {status}")
   
   def update_xbox_status(self, message):
       """Update controller status label"""
       self.controller_status.config(text=message)
       self.log_message(message)
   
   def poll_controller(self):
       """Poll Xbox controller for input"""
       if not self.controller_active or not self.is_automatic_mode:
           # Still keep polling even if inactive
           self.root.after(50, self.poll_controller)
           return
           
       success, event = self.xbox.update()
       
       if not success:
           self.update_xbox_status("Xbox controller disconnected")
           self.controller_active = False
       else:
           # Cache previous values to detect real changes
           prev_steering = int(self.steering_slider.get())
           prev_accel = int(self.accel_slider.get())
           
           # Process steering input from right stick - only send if changed significantly
           steering_value = self.xbox.get_steering_value()
           if abs(steering_value - prev_steering) > 10:  # Only update if changed by more than 10 units
               self.steering_slider.set(steering_value)
               self.arduino.send_command(f"W {steering_value}")
           
           # Process acceleration from RT - only send if changed
           accel_value = self.xbox.get_acceleration_value()
           if accel_value != prev_accel:  # Only update if actually changed
               self.accel_slider.set(accel_value)
               self.arduino.send_command(f"V {accel_value}")
               
               
           if event == "HAT_CHANGED":
                hat_x, hat_y = self.xbox.get_hat_values()
                
                # hat_y = 1 means up/forward, hat_y = -1 means down/reverse
                if hat_y == 1:
                    self.send_direction("F")
                    self.log_message("D-pad: Forward")
                elif hat_y == -1:
                    self.send_direction("R")
                    self.log_message("D-pad: Reverse")
           
           # Process brake toggle from LT
           if event == "LT_PRESSED":
               self.toggle_brake()
            
        
       
       # Schedule next poll - slow down polling rate to reduce command spam
       self.root.after(100, self.poll_controller)
   
   def clear_console(self):
       self.response_console.config(state=tk.NORMAL)
       self.response_console.delete(1.0, tk.END)
       self.response_console.config(state=tk.DISABLED)
   
   def log_message(self, message):
       """Add a message to the response console"""
       self.response_console.config(state=tk.NORMAL)
       self.response_console.insert(tk.END, f"{message}\n")
       self.response_console.see(tk.END)  # Auto-scroll to end
       self.response_console.config(state=tk.DISABLED)
   
   def check_responses(self):
       # Process any available responses
       while not self.response_queue.empty():
           response = self.response_queue.get()
           self.log_message(response)
       
       # Schedule this method to run again after 50ms (more responsive for ESP32)
       self.root.after(50, self.check_responses)
   
   def refresh_ports(self):
       ports = [port.device for port in serial.tools.list_ports.comports()]
       self.port_combobox['values'] = ports
       if ports:
           self.port_combobox.current(0)
   
   def connect_arduino(self):
       port = self.port_var.get()
       baud_rate = int(self.baud_var.get())
       
       if port:
           success = self.arduino.connect(port, baud_rate)
           if success:
               messagebox.showinfo("Connection", f"Connected to {port} at {baud_rate} baud")
               self.log_message(f"Connected to {port} at {baud_rate} baud")
           else:
               messagebox.showerror("Connection Error", f"Failed to connect to {port}")
       else:
           messagebox.showwarning("Connection Warning", "Please select a port first")
   
   def disconnect_arduino(self):
       self.arduino.disconnect()
       messagebox.showinfo("Disconnection", "ESP32 disconnected")
       self.log_message("ESP32 disconnected")
   
   def toggle_mode(self):
       self.is_automatic_mode = not self.is_automatic_mode
       
       if self.is_automatic_mode:
           # Switch to Automatic mode
           self.mode_var.set("Automatic")
           self.arduino.send_command("A")
           self.arduino.send_command("D F")
           self.arduino.send_command("V 0")  # Stop the robot when switching to auto
           self.log_message("Switched to Automatic mode")
       else:
           # Switch to Manual mode
           self.arduino.send_command("V 0")
           self.mode_var.set("Manual")
           self.arduino.send_command("M")
           self.log_message("Switched to Manual mode")
           
           # Reset controls
           self.steering_slider.set(1500)  # Center position
           self.accel_slider.set(0)
           self.brake_active = False
           self.brake_btn.config(text="Activate Brake")
       
       self.update_ui_based_on_mode()
   
   def update_ui_based_on_mode(self):
       # For ttk widgets like buttons, we use the state method correctly
       state = "!disabled" if self.is_automatic_mode else "disabled"
       
       # For ttk.Scale widgets, we need to configure the state differently
       slider_state = "normal" if self.is_automatic_mode else "disabled"
       
       # Update button states
       self.forward_btn.state([state])
       self.reverse_btn.state([state])
       self.brake_btn.state([state])
       self.stop_btn.state([state])
       
       # Update slider states
       self.steering_slider.configure(state=slider_state)
       self.accel_slider.configure(state=slider_state)
   
   def on_steering_change(self, value):
       if self.is_automatic_mode:
           value = int(float(value))
           self.arduino.send_command(f"W {value}")
   
   def on_accel_change(self, value):
       if self.is_automatic_mode:
           value = int(float(value))
           self.arduino.send_command(f"V {value}")
   
   def send_direction(self, direction):
       if self.is_automatic_mode:
           self.arduino.send_command(f"D {direction}")
   
   def toggle_brake(self):
       if self.is_automatic_mode:
           if self.brake_active:
               # Deactivate brake
               self.arduino.send_command("Q 1")
               self.brake_btn.config(text="Activate Brake")
               self.log_message("Brake deactivated")
           else:
               # Activate brake
               self.arduino.send_command("S 1")
               #self.accel_slider.set(0)  # Update slider to match the zero velocity
               self.brake_btn.config(text="Deactivate Brake")
               self.log_message("Brake activated")
           
           self.brake_active = not self.brake_active
   
   def emergency_stop(self):
       if self.is_automatic_mode:
           self.arduino.send_command("V 0")
           self.arduino.send_command("S 1")
           self.log_message("EMERGENCY STOP ACTIVATED")
           time.sleep(5)
           self.arduino.send_command("Q 1")
           self.log_message("Emergency stop released")
           # Update slider to match the zero velocity
           self.accel_slider.set(0)
   
   def on_closing(self):
       # Clean up pygame
       if self.xbox:
           self.xbox.disconnect()
       pygame.quit()
       
       # Shutdown Arduino communication
       self.arduino.shutdown()
       self.root.destroy()


if __name__ == "__main__":
   # Initialize pygame
   pygame.init()
   
   # Create and run the application
   root = tk.Tk()
   app = RobotControlGUI(root)
   root.protocol("WM_DELETE_WINDOW", app.on_closing)
   root.mainloop()