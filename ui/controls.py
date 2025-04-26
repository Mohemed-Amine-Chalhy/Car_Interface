import tkinter as tk
from tkinter import ttk
import math

class ControlsPanel(ttk.Frame):
    def __init__(self, parent, main_window):
        super().__init__(parent, style="Dark.TFrame")
        self.main_window = main_window
        self.speed = 0
        self.steering_angle = 0
        self.steering_active = False

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

        ttk.Label(self, text="Steering", style="Title.TLabel").pack(pady=10)
        self.steering_canvas = tk.Canvas(self, width=200, height=200, bg="#333333",
                                         highlightbackground="#333333")
        self.steering_canvas.pack(padx=20, pady=5)
        self.steering_wheel = self.steering_canvas.create_oval(25, 25, 175, 175, outline="#00BFFF", width=3)
        self.steering_line = self.steering_canvas.create_line(100, 100, 100, 25, fill="#00BFFF", width=3)
        self.steering_canvas.bind("<Button-1>", self.start_steering)
        self.steering_canvas.bind("<B1-Motion>", self.steer)
        self.steering_canvas.bind("<ButtonRelease-1>", self.stop_steering)

        ttk.Label(self, text="Speed Control", style="Title.TLabel").pack(pady=10)
        speed_frame = ttk.Frame(self, style="Dark.TFrame")
        speed_frame.pack(fill="x", padx=20, pady=5)

        self.speed_slider = ttk.Scale(speed_frame, from_=-30, to=30, orient="vertical",
                                     command=self.set_speed, length=200)
        self.speed_slider.pack(side="left", fill="x", expand=True, pady=5)
        self.speed_label = ttk.Label(speed_frame, text="0 km/h", style="Speed.TLabel")
        self.speed_label.pack(side="right", padx=5, pady=5)

        self.emergency_button = ttk.Button(self, text="EMERGENCY STOP", command=self.emergency_stop)
        self.emergency_button.pack(fill="x", padx=20, pady=20)
        self.emergency_button.configure(style="Dark.TButton")

        #  Proximity alerts
        proximity_frame = ttk.Frame(self, style="Dark.TFrame")
        proximity_frame.pack(fill="x", pady=10)
        self.proximity_label = ttk.Label(proximity_frame, text="No obstacles detected", style="Dark.TLabel")
        self.proximity_label.pack(side="left", padx=5)

    def set_control_mode(self, auto_mode):
        self.main_window.robot_state.is_auto_mode = auto_mode
        if auto_mode:
            self.auto_button.state(["selected"])
            self.manual_button.state(["!selected"])
            self.mode_label.config(text="Platform Control")
        else:
            self.manual_button.state(["selected"])
            self.auto_button.state(["!selected"])
            self.mode_label.config(text="Manual Control")

    def start_steering(self, event):
        if not self.main_window.is_connected or not self.main_window.robot_state.is_auto_mode:
            return
        self.steering_active = True

    def steer(self, event):
        if not self.main_window.is_connected or not self.main_window.robot_state.is_auto_mode or not self.steering_active:
            return

        x, y = event.x - 100, event.y - 100
        angle = math.degrees(math.atan2(y, x)) - 90
        angle = max(-90, min(90, angle))
        self.steering_angle = angle
        self.main_window.robot_state.steering_angle = angle  # Update robot state

        rad_angle = math.radians(angle)
        end_x = 100 + 75 * math.sin(rad_angle)
        end_y = 100 - 75 * math.cos(rad_angle)
        self.steering_canvas.coords(self.steering_line, 100, 100, end_x, end_y)

    def stop_steering(self, event):
        self.steering_active = False

    def set_speed(self, event):
        if not self.main_window.is_connected or not self.main_window.robot_state.is_auto_mode:
            return
        self.speed = int(self.speed_slider.get())
        self.main_window.robot_state.speed = self.speed  # Update robot state
        self.speed_label.config(text=f"{self.speed} km/h")

    def emergency_stop(self):
        if not self.main_window.is_connected:
            return
        self.speed = 0
        self.steering_angle = 0
        self.main_window.robot_state.speed = 0
        self.main_window.robot_state.steering_angle = 0
        self.speed_label.config(text=f"{self.speed} km/h")
        self.steering_canvas.coords(self.steering_line, 100, 100, 100, 25)
