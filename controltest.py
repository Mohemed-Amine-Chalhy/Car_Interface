import tkinter as tk
from tkinter import ttk, messagebox
import serial
import threading
import queue
import time

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("800x600")
        
        # Serial communication setup
        self.serial_port = None
        self.is_connected = False
        self.command_queue = queue.Queue()
        self.connect_thread = None
        self.send_thread = None
        self.is_running = True
        
        # Application state
        self.is_auto_mode = False
        self.is_emergency_stopped = False
        
        self.create_ui()
        self.start_command_processing()
        
    def create_ui(self):
        # Create main frames
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.pack(fill=tk.BOTH, expand=True)
        
        left_frame = ttk.Frame(control_frame, padding=10, borderwidth=2, relief="groove")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        right_frame = ttk.Frame(control_frame, padding=10, borderwidth=2, relief="groove")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Serial connection controls
        connection_frame = ttk.LabelFrame(top_frame, text="Serial Connection", padding=10)
        connection_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_entry = ttk.Entry(connection_frame, width=15)
        self.port_entry.insert(0, "COM3")  # Default port
        self.port_entry.grid(row=0, column=1, padx=5, pady=5)
        
        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)
        
        self.connection_status = ttk.Label(connection_frame, text="Status: Disconnected", foreground="red")
        self.connection_status.grid(row=0, column=3, padx=5, pady=5)
        
        # Mode toggle
        mode_frame = ttk.LabelFrame(left_frame, text="Drive Mode", padding=10)
        mode_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.mode_button = ttk.Button(mode_frame, text="MANUAL MODE", 
                                     command=self.toggle_mode, width=20)
        self.mode_button.pack(padx=5, pady=5)
        
        # Direction control
        direction_frame = ttk.LabelFrame(left_frame, text="Direction Control", padding=10)
        direction_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.forward_button = ttk.Button(direction_frame, text="Forward (D F)", 
                                        command=lambda: self.send_command('D F'), state=tk.DISABLED)
        self.forward_button.pack(fill=tk.X, padx=5, pady=5)
        
        self.reverse_button = ttk.Button(direction_frame, text="Reverse (D R)", 
                                        command=lambda: self.send_command('D R'), state=tk.DISABLED)
        self.reverse_button.pack(fill=tk.X, padx=5, pady=5)
        
        # Emergency stop
        emergency_frame = ttk.LabelFrame(left_frame, text="Emergency Controls", padding=10)
        emergency_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.estop_button = ttk.Button(emergency_frame, text="EMERGENCY STOP", 
                                      command=self.emergency_stop, state=tk.DISABLED)
        self.estop_button.configure(style="Emergency.TButton")
        self.estop_button.pack(fill=tk.X, padx=5, pady=5)
        
        self.f_command_button = ttk.Button(emergency_frame, text="Send F Command", 
                                          command=lambda: self.send_command('F'), state=tk.DISABLED)
        self.f_command_button.pack(fill=tk.X, padx=5, pady=5)
        
        # Steering control
        steering_frame = ttk.LabelFrame(right_frame, text="Steering Control", padding=10)
        steering_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.steering_value = tk.IntVar()
        self.steering_value.set(0)
        
        ttk.Label(steering_frame, text="Left").pack(side=tk.LEFT)
        self.steering_slider = ttk.Scale(steering_frame, from_=-100, to=100, orient=tk.HORIZONTAL,
                                        variable=self.steering_value, length=300, 
                                        command=self.on_steering_change, state=tk.DISABLED)
        self.steering_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(steering_frame, text="Right").pack(side=tk.LEFT)
        
        # Acceleration control
        accel_frame = ttk.LabelFrame(right_frame, text="Acceleration Control", padding=10)
        accel_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.accel_value = tk.IntVar()
        self.accel_value.set(0)
        
        ttk.Label(accel_frame, text="Max").pack(side=tk.TOP)
        self.accel_slider = ttk.Scale(accel_frame, from_=100, to=0, orient=tk.VERTICAL,
                                     variable=self.accel_value, length=300, 
                                     command=self.on_accel_change, state=tk.DISABLED)
        self.accel_slider.pack(side=tk.TOP, fill=tk.Y, expand=True, pady=5)
        ttk.Label(accel_frame, text="Zero").pack(side=tk.TOP)
        
        # Status bar
        self.status_bar = ttk.Label(self.root, text="System ready. Not connected to robot.", 
                                   relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Create custom style for emergency button
        style = ttk.Style()
        style.configure("Emergency.TButton", foreground="white", background="red",
                      font=('Helvetica', 12, 'bold'))
    
    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_entry.get()
            self.connect_to_port(port)
        else:
            self.disconnect()
    
    def connect_to_port(self, port):
        def connect_task():
            try:
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                self.root.after(0, self.connection_established)
            except Exception as e:
                # Capture the exception 'e' immediately in the lambda
                self.root.after(0, lambda e=e: self.connection_failed(str(e)))

        
        self.connect_button.config(state=tk.DISABLED)
        self.status_bar.config(text=f"Connecting to {port}...")
        
        self.connect_thread = threading.Thread(target=connect_task)
        self.connect_thread.daemon = True
        self.connect_thread.start()
    
    def connection_established(self):
        self.is_connected = True
        self.connection_status.config(text="Status: Connected", foreground="green")
        self.connect_button.config(text="Disconnect", state=tk.NORMAL)
        self.status_bar.config(text=f"Connected to robot on {self.port_entry.get()}")
        
        # When connected, enable the mode button but keep controls disabled
        self.mode_button.config(state=tk.NORMAL)
    
    def connection_failed(self, error_msg):
        self.connect_button.config(state=tk.NORMAL)
        self.status_bar.config(text=f"Connection failed: {error_msg}")
        messagebox.showerror("Connection Error", f"Failed to connect: {error_msg}")
    
    def disconnect(self):
        try:
            # First, ensure we're in manual mode to stop the robot
            if self.is_auto_mode:
                self.toggle_mode()
                
            # Send emergency stop command before disconnecting
            self.send_command('V 0')
            time.sleep(0.2)  # Brief pause to allow command to send
            
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
            self.is_connected = False
            self.connection_status.config(text="Status: Disconnected", foreground="red")
            self.connect_button.config(text="Connect")
            self.status_bar.config(text="Disconnected from robot")
            
            # Disable all controls
            self.mode_button.config(state=tk.DISABLED)
            self.disable_driving_controls()
            
        except Exception as e:
            self.status_bar.config(text=f"Error disconnecting: {str(e)}")
    
    def toggle_mode(self):
        if not self.is_connected:
            return
            
        self.is_auto_mode = not self.is_auto_mode
        
        if self.is_auto_mode:
            # Switching to automatic
            self.mode_button.config(text="AUTOMATIC MODE")
            self.status_bar.config(text="Switched to AUTOMATIC mode")
            self.send_command('A')
            self.send_command('V 0')  # Initialize velocity to 0
            self.enable_driving_controls()
        else:
            # Switching to manual
            self.mode_button.config(text="MANUAL MODE")
            self.status_bar.config(text="Switched to MANUAL mode")
            self.send_command('M')
            self.disable_driving_controls()
            # Reset sliders to 0
            self.accel_value.set(0)
            self.steering_value.set(0)
    
    def enable_driving_controls(self):
        self.steering_slider.config(state=tk.NORMAL)
        self.accel_slider.config(state=tk.NORMAL)
        self.forward_button.config(state=tk.NORMAL)
        self.reverse_button.config(state=tk.NORMAL)
        self.estop_button.config(state=tk.NORMAL)
        self.f_command_button.config(state=tk.NORMAL)
    
    def disable_driving_controls(self):
        self.steering_slider.config(state=tk.DISABLED)
        self.accel_slider.config(state=tk.DISABLED)
        self.forward_button.config(state=tk.DISABLED)
        self.reverse_button.config(state=tk.DISABLED)
        self.estop_button.config(state=tk.DISABLED)
        self.f_command_button.config(state=tk.DISABLED)
    
    def emergency_stop(self):
        self.send_command('V 0')
        self.accel_value.set(0)
        self.is_emergency_stopped = not self.is_emergency_stopped
        
        # Toggle F command button state
        if self.is_emergency_stopped:
            self.f_command_button.config(state=tk.DISABLED)
            self.status_bar.config(text="EMERGENCY STOP activated")
        else:
            self.f_command_button.config(state=tk.NORMAL)
            self.status_bar.config(text="EMERGENCY STOP released")
    
    def on_steering_change(self, value):
        # Only send periodic updates to avoid flooding the serial port
        self.root.after_cancel(self.send_steering_command) if hasattr(self, 'send_steering_command') else None
        self.send_steering_command = self.root.after(100, lambda: self.send_command(f'O {int(float(value))}'))
    
    def on_accel_change(self, value):
        # Only send periodic updates to avoid flooding the serial port
        self.root.after_cancel(self.send_accel_command) if hasattr(self, 'send_accel_command') else None
        self.send_accel_command = self.root.after(100, lambda: self.send_command(f'V {int(float(value))}'))
    
    def send_command(self, command):
        if not self.is_connected:
            self.status_bar.config(text=f"Not connected. Cannot send: {command}")
            return
            
        self.command_queue.put(command)
        self.status_bar.config(text=f"Sent: {command}")
    
    def start_command_processing(self):
        def process_commands():
            while self.is_running:
                try:
                    if not self.command_queue.empty() and self.is_connected:
                        command = self.command_queue.get(timeout=0.1)
                        try:
                            if self.serial_port and self.serial_port.is_open:
                                self.serial_port.write(f"{command}\n".encode())
                        except Exception as e:
                            # Use after() to update UI from the main thread
                            self.root.after(0, lambda e=e: self.status_bar.config(
                                text=f"Error sending command: {str(e)}"))
                    time.sleep(0.01)  # Small sleep to reduce CPU usage
                except queue.Empty:
                    pass
                except Exception as e:
                    print(f"Command processing error: {e}")
        
        self.send_thread = threading.Thread(target=process_commands)
        self.send_thread.daemon = True
        self.send_thread.start()
    
    def on_closing(self):
        # Clean shutdown procedure
        self.is_running = False
        if self.is_connected:
            try:
                # Send emergency stop before closing
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write("V 0\n".encode())
                    time.sleep(0.2)  # Brief pause to allow command to send
                    self.serial_port.close()
            except:
                pass
        
        # Wait for threads to finish
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        if self.connect_thread and self.connect_thread.is_alive():
            self.connect_thread.join(timeout=1.0)
            
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()