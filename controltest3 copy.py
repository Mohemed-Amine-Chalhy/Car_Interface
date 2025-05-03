import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import queue
import serial
import serial.tools.list_ports
import time

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
    
    def connect(self, port, baud_rate=115200):
        try:
            if self.serial_conn and self.is_connected:
                self.serial_conn.close()
                self.is_connected = False
            
            self.serial_conn = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)  # Allow time for Arduino to reset after connection
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
        while self.running:
            if not self.command_queue.empty() and self.is_connected:
                command = self.command_queue.get()
                try:
                    self.serial_conn.write(f"{command}\n".encode())
                    print(f"Sent: {command}")
                except Exception as e:
                    print(f"Error sending command: {e}")
                finally:
                    self.command_queue.task_done()
            time.sleep(0.01)  # Small delay to prevent CPU hogging
    
    def _read_responses(self):
        while self.running:
            if self.is_connected and self.serial_conn:
                try:
                    if self.serial_conn.in_waiting > 0:
                        response = self.serial_conn.readline().decode('utf-8').strip()
                        if response:
                            print(f"Received: {response}")
                            self.response_queue.put(response)
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


class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("800x600")  # Increased size to accommodate response console
        
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.arduino = ArduinoCommunicator(self.command_queue, self.response_queue)
        
        self.is_automatic_mode = False
        
        self.create_widgets()
        self.update_ui_based_on_mode()
        
        # Start a periodic task to check for responses
        self.check_responses()
    
    def create_widgets(self):
        # Frame for connection settings
        connection_frame = ttk.LabelFrame(self.root, text="Connection Settings")
        connection_frame.pack(fill="x", padx=10, pady=10)
        
        # Serial port selection
        ttk.Label(connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combobox = ttk.Combobox(connection_frame, textvariable=self.port_var)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5)
        self.refresh_ports()
        
        ttk.Button(connection_frame, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(connection_frame, text="Connect", command=self.connect_arduino).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(connection_frame, text="Disconnect", command=self.disconnect_arduino).grid(row=0, column=4, padx=5, pady=5)
        
        # Mode toggle
        mode_frame = ttk.LabelFrame(self.root, text="Drive Mode")
        mode_frame.pack(fill="x", padx=10, pady=10)
        
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
        self.steering_slider.set(1800)
        
        # Acceleration slider (vertical)
        accel_frame = ttk.Frame(control_frame)
        accel_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        ttk.Label(accel_frame, text="Acceleration:").pack(anchor="n")
        self.accel_slider = ttk.Scale(accel_frame, from_=100, to=0, orient="vertical", 
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
        self.brake_active = False
        
        # Emergency stop
        emergency_frame = ttk.LabelFrame(buttons_frame, text="Emergency")
        emergency_frame.pack(fill="x", pady=5)
        self.stop_btn = ttk.Button(emergency_frame, text="EMERGENCY STOP", 
                                  command=self.emergency_stop, style="Emergency.TButton")
        self.stop_btn.pack(fill="x", pady=5)
        
        # Response console (right side)
        response_frame = ttk.LabelFrame(main_frame, text="Arduino Responses")
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
    
    def clear_console(self):
        self.response_console.config(state=tk.NORMAL)
        self.response_console.delete(1.0, tk.END)
        self.response_console.config(state=tk.DISABLED)
    
    def check_responses(self):
        # Process any available responses
        while not self.response_queue.empty():
            response = self.response_queue.get()
            self.response_console.config(state=tk.NORMAL)
            self.response_console.insert(tk.END, f"{response}\n")
            self.response_console.see(tk.END)  # Auto-scroll to end
            self.response_console.config(state=tk.DISABLED)
        
        # Schedule this method to run again after 100ms
        self.root.after(100, self.check_responses)
    
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0)
    
    def connect_arduino(self):
        port = self.port_var.get()
        if port:
            success = self.arduino.connect(port)
            if success:
                messagebox.showinfo("Connection", f"Connected to {port}")
                self.response_console.config(state=tk.NORMAL)
                self.response_console.insert(tk.END, f"Connected to {port}\n")
                self.response_console.see(tk.END)
                self.response_console.config(state=tk.DISABLED)
            else:
                messagebox.showerror("Connection Error", f"Failed to connect to {port}")
        else:
            messagebox.showwarning("Connection Warning", "Please select a port first")
    
    def disconnect_arduino(self):
        self.arduino.disconnect()
        messagebox.showinfo("Disconnection", "Arduino disconnected")
        self.response_console.config(state=tk.NORMAL)
        self.response_console.insert(tk.END, "Arduino disconnected\n")
        self.response_console.see(tk.END)
        self.response_console.config(state=tk.DISABLED)
    
    def toggle_mode(self):
        self.is_automatic_mode = not self.is_automatic_mode
        
        if self.is_automatic_mode:
            # Switch to Automatic mode
            self.mode_var.set("Automatic")
            self.arduino.send_command("A")
            self.arduino.send_command("D F")
            self.arduino.send_command("V 0")# Stop the robot when switching to auto
        else:
            # Switch to Manual mode
            self.mode_var.set("Manual")
            self.arduino.send_command("M")
            
            # Reset controls
            self.steering_slider.set(0)
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
                self.arduino.send_command("Z")
                self.brake_btn.config(text="Activate Brake")
            else:
                # Activate brake
                self.arduino.send_command("V 0")
                self.arduino.send_command("F")
                self.accel_slider.set(0)  # Update slider to match the zero velocity
                self.brake_btn.config(text="Deactivate Brake")
            
            self.brake_active = not self.brake_active
    
    def emergency_stop(self):
        if self.is_automatic_mode:
            self.arduino.send_command("V 0")  # Stop the robot
            #self.arduino.send_command("5")    # Emergency signal
            self.accel_slider.set(0)  # Update slider to match the zero velocity
    
    def on_closing(self):
        self.arduino.shutdown()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()