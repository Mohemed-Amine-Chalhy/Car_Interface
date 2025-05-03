import tkinter as tk
from tkinter import ttk
import threading
import time
from PIL import Image, ImageTk
import cv2
import sys
from pathlib import Path

from core.robot_state import RobotState
from ui.controls import ControlsPanel
from core.rplidar_interface import RPLidarHandler
from core.camera_handler import CameraHandler
from ui.visualizations import LidarView, CameraView

class MainWindow(ttk.Frame):
    def __init__(self, root):
        super().__init__(root)
        self.robot_state = RobotState()
        self.root = root
        self.root.title("Car Control Platform")
        self.root.geometry("1350x800")
        self.root.configure(bg="#222222")
       
        # Initialize device handlers with default values but don't connect yet
        self.lidar_handler = RPLidarHandler(port='COM6')
        self.camera_handler = CameraHandler(camera_index=0)
       
        # Track component connection status separately
        self.lidar_connected = False
        self.camera_connected = False
        self.control_enabled = False  # Track if control is enabled independently
       
        # Main Container
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill='both', expand=True, padx=10, pady=10)
       
        # Left Panel (visualizations)
        self.left_panel = ttk.Frame(self.main_container, style="Dark.TFrame")
        self.left_panel.pack(side='left', fill="both", expand=True, padx=5, pady=5)
       
        # Right panel (control)
        self.right_panel = ttk.Frame(self.main_container, style="Dark.TFrame")
        self.right_panel.pack(side='right', fill='both', padx=5, pady=5)
       
        self.setup_visualizations()
        self.setup_controls()
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_ui()
       
    def setup_visualizations(self):
        # Create frames to hold the visualization canvases
        self.top_panel = ttk.Frame(self.left_panel, style="Dark.TFrame")
        self.top_panel.pack(fill="both", expand=True, padx=10, pady=10)

        self.camera_frame = ttk.Frame(self.top_panel, style="Dark.TFrame")
        self.camera_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        self.lidar_frame = ttk.Frame(self.top_panel, style="Dark.TFrame")
        self.lidar_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        # Initialize visualization views (will be created when respective devices connect)
        self.camera_view = None
        self.lidar_view = None

    def setup_controls(self):
        self.controls_panel = ControlsPanel(self.right_panel, self)
        self.controls_panel.pack(fill="both", expand=True)
        
        # Initialize connection status in the controls panel
        self.controls_panel.lidar_status_label.config(text="LIDAR: Disconnected")
        self.controls_panel.camera_status_label.config(text="Camera: Disconnected")
        self.controls_panel.control_status_label.config(text="Control: Disabled")
        self.controls_panel.mode_label.config(text="Manual")

    def toggle_lidar_connection(self):
        """Toggle the LIDAR connection status"""
        if not self.lidar_connected:
            # Try to connect to LIDAR
            if self.lidar_handler.connect():
                self.lidar_handler.start_scanning()
                self.lidar_connected = True
                self.controls_panel.lidar_status_label.config(text="LIDAR: Connected")
                self.controls_panel.toggle_lidar_button.config(text="Disconnect LIDAR")
                
                # Create LIDAR view if not exists
                if not self.lidar_view:
                    self.lidar_view = LidarView(self.lidar_frame)
                    self.lidar_view.pack(fill="both", expand=True)
            else:
                self.controls_panel.lidar_status_label.config(text="LIDAR: Connection Failed")
        else:
            # Disconnect LIDAR
            self.lidar_handler.stop_scanning()
            self.lidar_handler.disconnect()
            self.lidar_connected = False
            self.controls_panel.lidar_status_label.config(text="LIDAR: Disconnected")
            self.controls_panel.toggle_lidar_button.config(text="Connect LIDAR")
            
            # Destroy LIDAR view
            if self.lidar_view:
                self.lidar_view.destroy()
                self.lidar_view = None
            self.robot_state.lidar_data = []  # Clear LIDAR data

    def toggle_camera_connection(self):
        """Toggle the camera connection status"""
        if not self.camera_connected:
            # Try to connect to camera
            if self.camera_handler.start_capture():
                self.camera_connected = True
                self.controls_panel.camera_status_label.config(text="Camera: Connected")
                self.controls_panel.toggle_camera_button.config(text="Disconnect Camera")
                
                # Create camera view if not exists
                if not self.camera_view:
                    self.camera_view = CameraView(self.camera_frame)
                    self.camera_view.pack(fill="both", expand=True)
            else:
                self.controls_panel.camera_status_label.config(text="Camera: Connection Failed")
        else:
            # Disconnect camera
            self.camera_handler.stop_capture()
            self.camera_connected = False
            self.controls_panel.camera_status_label.config(text="Camera: Disconnected")
            self.controls_panel.toggle_camera_button.config(text="Connect Camera")
            
            # Destroy camera view
            if self.camera_view:
                self.camera_view.destroy()
                self.camera_view = None

    def toggle_control_status(self):
        """Toggle the control status independently"""
        self.control_enabled = not self.control_enabled
        
        if self.control_enabled:
            self.controls_panel.control_status_label.config(text="Control: Enabled")
            self.controls_panel.toggle_control_button.config(text="Disable Control")
        else:
            self.controls_panel.control_status_label.config(text="Control: Disabled")
            self.controls_panel.toggle_control_button.config(text="Enable Control")

    def update_ui(self):
        # Update LIDAR visualization if connected
        if self.lidar_connected:
            lidar_data = self.lidar_handler.get_scan_data()
            if lidar_data:
                self.robot_state.lidar_data = lidar_data
                if self.lidar_view:
                    self.lidar_view.update_display(self.robot_state.lidar_data)

        # Update camera visualization if connected
        if self.camera_connected and self.camera_handler.cap is not None and self.camera_view:
            frame = self.camera_handler.get_frame()
            if frame is not None:
                self.camera_view.update_frame(frame)

        # Update status labels
        if self.control_enabled:
            self.controls_panel.speed_label.config(text=f"{int(self.robot_state.speed)}%")
            if self.lidar_connected and any(dist < 200 for _, dist in self.robot_state.lidar_data):
                self.controls_panel.proximity_label.config(text="WARNING: Obstacle detected!", style="Alert.TLabel")
            else:
                self.controls_panel.proximity_label.config(text="No obstacles detected", style="Dark.TLabel")

        self.controls_panel.mode_label.config(text="Platform Control" if self.robot_state.is_auto_mode else "Manual Control")

        # Schedule the next update
        self.root.after(30, self.update_ui)
   
    def on_closing(self):
        """Clean up before closing the application"""
        if self.lidar_connected:
            self.lidar_handler.stop_scanning()
            self.lidar_handler.disconnect()
        
        if self.camera_connected:
            self.camera_handler.stop_capture()
            
        self.root.destroy()