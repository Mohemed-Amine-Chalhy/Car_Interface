import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import math
import cv2

class CameraView(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, style="Dark.TFrame")
        ttk.Label(self, text="Camera Feed", style="Title.TLabel").pack(pady=5)
        self.canvas = tk.Canvas(self, bg="black", width=640, height=360)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.image_tk = None
        
    def update_frame(self, frame):
        if frame is not None:
            try:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                img = img.resize((640, 360), Image.LANCZOS)
                self.image_tk = ImageTk.PhotoImage(image=img)
                self.canvas.create_image(0, 0, anchor="nw", image=self.image_tk)
                self.canvas.image = self.image_tk  # Keep a reference
            except Exception as e:
                print(f"Error updating camera feed: {e}")
                self.canvas.delete("all")
                self.canvas.config(bg="black")
        else:
            self.canvas.delete("all")
            self.canvas.config(bg="black")
            
    
import tkinter as tk
from tkinter import ttk
import math

class LidarView(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, style="Dark.TFrame")
        ttk.Label(self, text="LIDAR Visualization", style="Title.TLabel").pack(pady=5)
        self.canvas = tk.Canvas(self, bg="black", width=300, height=300)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self.point_size = 2  # Pre-defined point size
        self.max_distance = 2000 # Pre-defined max distance
        self.scale_factor = 300 / self.max_distance

    def update_display(self, lidar_data):
        self.canvas.delete("lidar")
        if not lidar_data:
            return

        center_x = 150
        center_y = 150
        points = [] #list to hold the points

        for angle_deg, distance_mm in lidar_data:
            if distance_mm > self.max_distance:
                continue  # Skip points outside the display range

            angle_rad = math.radians(angle_deg)
            scaled_distance = distance_mm * self.scale_factor
            end_x = center_x + scaled_distance * math.cos(angle_rad)
            end_y = center_y - scaled_distance * math.sin(angle_rad)

            color = "#00FF00"  # Default color
            if distance_mm < 500:
                color = "#FF0000"
            elif distance_mm < 1500:
                color = "#FFFF00"

            points.append((end_x, end_y, color))

        #create points after the loop
        for x, y, color in points:
             self.canvas.create_oval(x - self.point_size, y - self.point_size,
                                        x + self.point_size, y + self.point_size,
                                        fill=color, outline=color, tags="lidar")
