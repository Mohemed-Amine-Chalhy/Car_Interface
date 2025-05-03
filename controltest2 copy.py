import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import queue
import time

class ArduinoHandler:
    def __init__(self, port=None, baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.connected = False
        self.command_queue = queue.Queue()
        self.running = False
        self.thread = None
    
    def connect(self, port, baud_rate=9600):
        try:
            self.port = port
            self.baud_rate = baud_rate
            # Simulate connection instead of creating a real serial connection
            print(f"Simulating connection to {port} at {baud_rate} baud")
            self.connected = True
            self.running = True
            
            # Start command processing thread
            self.thread = threading.Thread(target=self._process_commands, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"Simulated connection error: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        if self.connected:
            print("Simulating disconnection")
            self.connected = False
    
    def send_command(self, command):
        if self.connected:
            self.command_queue.put(command)
            return True
        return False
    
    def _process_commands(self):
        while self.running:
            try:
                if not self.command_queue.empty():
                    command = self.command_queue.get(block=False)
                    # Just print the command instead of sending it to serial
                    print(f"Command sent (simulated): {command}")
                    self.command_queue.task_done()
                time.sleep(0.01)  # Small delay to prevent CPU hogging
            except Exception as e:
                print(f"Error processing command (simulated): {e}")
                time.sleep(0.1)  # Longer delay if there's an error


class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("800x600")
        
        # Arduino handler
        self.arduino = ArduinoHandler()
        
        # Create GUI components
        self.create_widgets()
        
        # Set initial state
        self.current_mode = "Manual"  # Start in manual mode
        self.update_mode_ui()
    
    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection Settings", padding="10")
        conn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.port_entry = ttk.Entry(conn_frame, width=15)
        self.port_entry.grid(row=0, column=1, padx=5)
        self.port_entry.insert(0, "COM3")  # Default port
        
        ttk.Label(conn_frame, text="Baud Rate:").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.baud_entry = ttk.Entry(conn_frame, width=10)
        self.baud_entry.grid(row=0, column=3, padx=5)
        self.baud_entry.insert(0, "9600")  # Default baud rate
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=10)
        
        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=0, column=5, padx=10)
        
        # Control frame
        control_frame = ttk.LabelFrame(main_frame, text="Robot Controls", padding="10")
        control_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Mode toggle
        mode_frame = ttk.Frame(control_frame)
        mode_frame.pack(fill=tk.X, pady=10)
        
        self.mode_btn = ttk.Button(mode_frame, text="Switch to Automatic Mode", command=self.toggle_mode)
        self.mode_btn.pack(side=tk.LEFT, padx=10)
        
        self.mode_label = ttk.Label(mode_frame, text="Current Mode: Manual", font=("Arial", 12, "bold"))
        self.mode_label.pack(side=tk.LEFT, padx=20)
        
        # Create a frame for all the controls
        controls_container = ttk.Frame(control_frame)
        controls_container.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Left frame for steering
        left_frame = ttk.Frame(controls_container)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        # Steering control
        steering_frame = ttk.LabelFrame(left_frame, text="Steering Control", padding="10")
        steering_frame.pack(fill=tk.X, pady=10)
        
        self.steering_slider = ttk.Scale(steering_frame, from_=-100, to=100, orient=tk.HORIZONTAL, 
                                        length=300, command=self.on_steering_change)
        self.steering_slider.pack(pady=10)
        self.steering_slider.set(0)
        
        self.steering_value = ttk.Label(steering_frame, text="Value: 0")
        self.steering_value.pack(pady=5)
        
        # Direction control
        direction_frame = ttk.LabelFrame(left_frame, text="Direction Control", padding="10")
        direction_frame.pack(fill=tk.X, pady=10)
        
        self.forward_btn = ttk.Button(direction_frame, text="Forward", command=lambda: self.send_direction("F"))
        self.forward_btn.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.X, expand=True)
        
        self.reverse_btn = ttk.Button(direction_frame, text="Reverse", command=lambda: self.send_direction("R"))
        self.reverse_btn.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.X, expand=True)
        
        # Right frame for acceleration and emergency
        right_frame = ttk.Frame(controls_container)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10)
        
        # Acceleration control
        accel_frame = ttk.LabelFrame(right_frame, text="Acceleration Control", padding="10")
        accel_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.accel_slider = ttk.Scale(accel_frame, from_=0, to=100, orient=tk.VERTICAL, 
                                     length=200, command=self.on_accel_change)
        self.accel_slider.pack(side=tk.LEFT, padx=20, pady=10)
        self.accel_slider.set(0)
        
        self.accel_value = ttk.Label(accel_frame, text="Value: 0")
        self.accel_value.pack(side=tk.LEFT, padx=10)
        
        # Emergency and F command
        emergency_frame = ttk.LabelFrame(right_frame, text="Emergency Controls", padding="10")
        emergency_frame.pack(fill=tk.X, pady=10)
        
        self.emergency_btn = ttk.Button(emergency_frame, text="EMERGENCY STOP", 
                                       command=self.emergency_stop, style="Emergency.TButton")
        self.emergency_btn.pack(fill=tk.X, pady=5)
        
        self.f_command_btn = ttk.Button(emergency_frame, text="Send F Command", command=self.send_f_command)
        self.f_command_btn.pack(fill=tk.X, pady=5)
        
        # Create a custom style for the emergency button
        self.root.style = ttk.Style()
        self.root.style.configure("Emergency.TButton", foreground="white", background="red", 
                                 font=("Arial", 12, "bold"))
    
    def toggle_connection(self):
        if not self.arduino.connected:
            port = self.port_entry.get()
            try:
                baud_rate = int(self.baud_entry.get())
            except ValueError:
                messagebox.showerror("Error", "Baud rate must be a number")
                return
            
            if self.arduino.connect(port, baud_rate):
                self.status_label.config(text="Status: Connected", foreground="green")
                self.connect_btn.config(text="Disconnect")
            else:
                messagebox.showerror("Connection Error", f"Failed to connect to {port}")
        else:
            self.arduino.disconnect()
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
    
    def toggle_mode(self):
        if not self.arduino.connected:
            messagebox.showwarning("Warning", "Connect to Arduino first")
            return
        
        if self.current_mode == "Manual":
            # Switching to Automatic
            self.current_mode = "Automatic"
            self.arduino.send_command("A")
            self.arduino.send_command("V 0")
        else:
            # Switching to Manual
            self.current_mode = "Manual"
            self.arduino.send_command("M")
            # Reset controls
            self.steering_slider.set(0)
            self.accel_slider.set(0)
        
        self.update_mode_ui()
    
    def update_mode_ui(self):
        if self.current_mode == "Automatic":
            self.mode_label.config(text="Current Mode: Automatic")
            self.mode_btn.config(text="Switch to Manual Mode")
            self.enable_controls(True)
        else:
            self.mode_label.config(text="Current Mode: Manual")
            self.mode_btn.config(text="Switch to Automatic Mode")
            self.enable_controls(False)
    
    def enable_controls(self, enable):
        state = "normal" if enable else "disabled"
        self.steering_slider.config(state=state)
        self.accel_slider.config(state=state)
        self.forward_btn.config(state=state)
        self.reverse_btn.config(state=state)
        self.emergency_btn.config(state=state)
        self.f_command_btn.config(state=state)
    
    def on_steering_change(self, value):
        value = int(float(value))
        self.steering_value.config(text=f"Value: {value}")
        if self.arduino.connected and self.current_mode == "Automatic":
            self.arduino.send_command(f"O {value}")
    
    def on_accel_change(self, value):
        value = int(float(value))
        self.accel_value.config(text=f"Value: {value}")
        if self.arduino.connected and self.current_mode == "Automatic":
            self.arduino.send_command(f"V {value}")
    
    def send_direction(self, direction):
        if self.arduino.connected and self.current_mode == "Automatic":
            self.arduino.send_command(f"D {direction}")
    
    def emergency_stop(self):
        if self.arduino.connected and self.current_mode == "Automatic":
            self.arduino.send_command("V 0")
            # Toggle f_command button state
            current_state = str(self.f_command_btn.cget("state"))
            new_state = "disabled" if current_state == "normal" else "normal"
            self.f_command_btn.config(state=new_state)
    
    def send_f_command(self):
        if self.arduino.connected and self.current_mode == "Automatic":
            self.arduino.send_command("F")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.mainloop()