import tkinter as tk
from tkinter import ttk
import threading
import time 
from PIL import Image, ImageTk
import cv2 
import sys
from pathlib import Path

# Get the path of the current script (main_window.py)
current_file_path = Path(__file__).resolve()

# Get the directory containing the current script (the 'ui' folder)
ui_dir = current_file_path.parent

# Get the parent directory of 'ui', which is 'car_interface'
car_interface_dir = ui_dir.parent

# Add the 'car_interface' directory to sys.path
sys.path.append(str(car_interface_dir))


from core.robot_state import RobotState
from ui.controls import ControlsPanel
from core.rplidar_interface import RPLidarHandler
from core.camera_handler import CameraHandler
from ui.visualizations import LidarView, CameraView





#from core.robot_state import RobotState




class MainWindow(ttk.Frame):
    def __init__(self,root):
        super().__init__(root)
        self.robot_state = RobotState()
        self.lidar_handler = RPLidarHandler()
        self.camera_handler = CameraHandler(camera_index=0)
        self.root = root
        self.root.title("Car Control Platform")
        self.root.geometry("1200x800")
        self.root.configure(bg="#222222")
        
        self.robot_state = RobotState()
        self.lidar_handler = RPLidarHandler(port='COM5')
        self.camera_handler = CameraHandler(camera_index=1)
        
        self.is_connected = False
        self.camera_active = False
        
        # Main Container
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Left Panel (visualizations)
        self.left_panel = ttk.Frame(self.main_container, style="Dark.TFrame")
        self.left_panel.pack(side='left',fill="both", expand=True, padx=5, pady=5)
        
        # Righ panel (control)
        self.right_panel = ttk.Frame(self.main_container, style="Dark.TFrame")
        self.right_panel.pack(side='right',fill='both', padx=5, pady=5)
        
        #self.setup_styles()
        self.setup_visualizations()
        self.setup_controls()
        self.setup_sensor_integration()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_ui()
        

    def setup_visualizations(self):
        # Create frames to hold the visualization canvases
        top_panel = ttk.Frame(self.left_panel, style="Dark.TFrame")
        top_panel.pack(fill="both", expand=True, padx=10, pady=10)

        self.camera_frame = ttk.Frame(top_panel, style="Dark.TFrame")
        self.camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        self.lidar_frame = ttk.Frame(top_panel, style="Dark.TFrame")
        self.lidar_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        # Initialize visualization views (the actual canvases will be created upon connection)
        self.camera_view = None
        self.lidar_view = None

    def setup_controls(self):
        self.controls_panel = ControlsPanel(self.right_panel, self)
        self.controls_panel.pack(fill="both", expand=True)
        # Initialize connection status in the controls panel
        self.controls_panel.connection_label.config(text="Disconnected")
        self.controls_panel.mode_label.config(text="Manual")

    def setup_sensor_integration(self):
        self.is_connected = False
        
    def toggle_connection(self):
        self.is_connected = not self.is_connected
        if self.is_connected:
            if self.lidar_handler.connect():
                self.lidar_handler.start_scanning()
                if self.camera_handler.start_capture():
                    self.controls_panel.connect_button.config(text='Disconnect')
                    self.controls_panel.connection_label.config(text = 'Connected')
                    self.camera_active = True
                    # Now instantiate the visualization panels as they depend on connection
                    self.camera_view = CameraView(self.left_panel)
                    self.camera_view.pack(side="left", fill="both", expand=True, padx=10, pady=10)
                    self.lidar_view = LidarView(self.left_panel)
                    self.lidar_view.pack(side="left", fill="both", expand=True, padx=10, pady=10)
                else :
                    self.lidar_handler.stop_scanning()
                    self.lidar_handler.disconnect()
                    self.is_connected = False
                    self.controls_panel.connection_label.config(text = 'Camera Connect Failed')
                    self.controls_panel.connect_button.config(text="Connect")
            else:
                self.is_connected = False
                self.controls_panel.connection_label.config(text="LIDAR Connect Failed")
                self.controls_panel.connect_button.config(text="Connect")
        else:
            self.lidar_handler.stop_scanning()
            self.lidar_handler.disconnect()
            self.camera_handler.stop_capture()
            self.controls_panel.connect_button.config(text="Connect")
            self.controls_panel.connect_button.config(text = 'Disconnected')
            self.camera_active = False
            
            # Destroy visualization panels on disconnect
            
            if self.camera_view:
                self.camera_view.destroy()
                self.camera_view = None
            if self.lidar_view:
                self.lidar_view.destroy()
                self.lidar_view = None
            self.robot_state.lidar_data = [] # Clear lidar data
            
            
    def update_ui(self):
        #print("update_ui called")
        print(f"Is connected: {self.is_connected}, Camera active: {self.camera_active}")
        if self.is_connected:
            lidar_data = self.lidar_handler.get_scan_data()
            #print(f"LIDAR data: {lidar_data}")
            if lidar_data:
                self.robot_state.lidar_data = lidar_data
                if self.lidar_view:
                    self.lidar_view.update_display(self.robot_state.lidar_data)

            print("Trying to get camera frame")
            if self.camera_active and self.camera_handler.cap is not None and self.camera_view:
                frame = self.camera_handler.get_frame()
                if frame is not None:
                    print("Got a camera frame")
                    self.camera_view.update_frame(frame)
                else:
                    print("No camera frame")

            # Update status labels
            self.controls_panel.speed_label.config(text=f"{int(self.robot_state.speed)} km/h")
            if any(dist < 200 for _, dist in self.robot_state.lidar_data):
                self.controls_panel.proximity_label.config(text="WARNING: Obstacle detected!", style="Alert.TLabel")
            else:
                self.controls_panel.proximity_label.config(text="No obstacles detected", style="Dark.TLabel")

            self.controls_panel.mode_label.config(text="Platform Control" if self.robot_state.is_auto_mode else "Manual Control")

        self.root.after(30, self.update_ui) # Schedule the next call at the end
    
    def on_closing(self):
        self.lidar_handler.stop_scanning()
        self.lidar_handler.disconnect()
        self.camera_handler.stop_capture()
        self.root.destroy()