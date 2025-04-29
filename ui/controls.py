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

        ttk.Label(self, text="Connection", style="Title.TLabel").pack(pady=10)
        self.connect_button = ttk.Button(self, text="Connect", command=self.main_window.toggle_connection)
        self.connect_button.pack(fill="x", padx=20, pady=5)
        self.connection_label = ttk.Label(self, text="Disconnected", style="Dark.TLabel")
        self.connection_label.pack(fill="x", padx=20, pady=5)
        
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
         # Keep track of the frein à main state
        self.frein_a_main_button = ttk.Button(self, text="Activate Frein",
                                                command=self.toggle_frein_a_main)
        self.frein_a_main_button.pack(pady=5, padx=20, fill="x")

        self.emergency_button = ttk.Button(self, text="EMERGENCY STOP", command=self.emergency_stop)
        self.emergency_button.pack(fill="x", padx=20, pady=20)
        self.emergency_button.configure(style="Dark.TButton")

        #  Proximity alerts
        proximity_frame = ttk.Frame(self, style="Dark.TFrame")
        proximity_frame.pack(fill="x", pady=10)
        self.proximity_label = ttk.Label(proximity_frame, text="No obstacles detected", style="Dark.TLabel")
        self.proximity_label.pack(side="left", padx=5)
        
        
    def f(self):
            print(self.auto_stop_var.get())
    def set_control_mode(self, auto_mode):
        import serial
        import time

        serial_port = RobotState().sp32port  # Adjust to your serial port
        baud_rate = 115200
        timeout = 1

        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
            time.sleep(2)  # Wait for device reset

            self.main_window.robot_state.is_auto_mode = auto_mode
            if auto_mode:
                self.auto_button.state(["selected"])
                self.manual_button.state(["!selected"])
                self.mode_label.config(text="Platform Control")
                print("Control mode set to Platform. Sending 'A' command.")
                ser.write(b'A\n')  # Send 'A' for auto mode
            else:
                self.manual_button.state(["selected"])
                self.auto_button.state(["!selected"])
                self.mode_label.config(text="Manual Control")
                print("Control mode set to Manual. Sending 'M' command.")
                ser.write(b'M\n')  # Send 'M' for manual mode

            # Optional: Read response
            while ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Received response: {response}")

        except serial.SerialException as e:
            print(f"Error communicating with serial port {serial_port}: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print("Serial port closed.")

    def on_steering_change(self, value):
        if not self.main_window.is_connected or not self.main_window.robot_state.is_auto_mode:
            return

        angle = float(value)
        print(angle)
        self.steering_angle = angle
        self.main_window.robot_state.steering_angle = angle  # Update robot state
        
    def set_steering(self, value):
        

        serial_port = RobotState().sp32port  # Adjust to your serial port
        baud_rate = 115200
        timeout = 1

        if not self.main_window.is_connected or not self.main_window.robot_state.is_auto_mode:
            return

        angle = float(value)
        self.steering_angle = angle
        self.main_window.robot_state.steering_angle = angle
        self.steering_label.config(text=f"{angle:.0f}%")  # Update label

        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
            time.sleep(2)  # Wait for device reset

            command = f"O {int(angle)}\n"  # Using 'O' and integer value
            print(f"Sending steering command: {command.strip()}")
            ser.write(command.encode())

            # Optional: Read response
            while ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Received response: {response}")

        except serial.SerialException as e:
            print(f"Error communicating with serial port {serial_port}: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print("Serial port closed.")



    def set_speed(self, event):
        import serial
        import time

        serial_port = RobotState().sp32port  # Adjust to your serial port
        baud_rate = 115200
        timeout = 1

        if not self.main_window.is_connected or not self.main_window.robot_state.is_auto_mode:
            return

        self.speed = int(self.speed_slider.get())
        self.main_window.robot_state.speed = self.speed  # Update robot state
        self.speed_label.config(text=f"{self.speed}%")

        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
            time.sleep(2)  # Wait for device reset

            command = f"V {self.speed}\n"
            print(f"Sending speed command: {command.strip()}")
            ser.write(command.encode())

            # Optional: Read response
            while ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Received response: {response}")

        except serial.SerialException as e:
            print(f"Error communicating with serial port {serial_port}: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print("Serial port closed.")

    def emergency_stop(self):
        if not self.main_window.is_connected:
            return
        self.speed = 0
        self.steering_angle = 0
        self.main_window.robot_state.speed = 0
        self.main_window.robot_state.steering_angle = 0
        self.speed_label.config(text=f"{self.speed}%")
        self.steering_canvas.coords(self.steering_line, 100, 100, 100, 25)
        self.toggle_frein_a_main()



    def toggle_frein_a_main(self):
 

        serial_port = RobotState().sp32port  # Adjust to your serial port
        baud_rate = 115200
        timeout = 1

        try:
            ser = serial.Serial(serial_port, baud_rate, timeout=timeout)
            time.sleep(2)  # Wait for device reset

            self.frein_a_main_active = not self.frein_a_main_active
            if self.frein_a_main_active:
                self.frein_a_main_button.config(text="Deactivate Frein à Main")
                print("Frein à main activated! Sending 'HF' command.")
                ser.write(b'F\n')  # Send activation command
            else:
                self.frein_a_main_button.config(text="Activate Frein à Main")
                print("Frein à main deactivated! Sending 'Z' command.")
                ser.write(b'Z\n')  # Send deactivation command

            # Optional: Read response
            while ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"Received response: {response}")

        except serial.SerialException as e:
            print(f"Error communicating with serial port {serial_port}: {e}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print("Serial port closed.")