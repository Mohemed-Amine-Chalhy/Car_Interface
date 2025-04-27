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


class LidarView(ttk.Frame):
    def __init__(self, parent, point_size=2, max_distance=5000, **kwargs):
        super().__init__(parent, style="Dark.TFrame", **kwargs)
        self.point_size = point_size
        self.max_distance = max_distance
        self.scale_factor = 300 / self.max_distance
        self.center_x = 150
        self.center_y = 150
        self._create_widgets()

    def _create_widgets(self):
        ttk.Label(self, text="LIDAR Visualization", style="Title.TLabel").pack(pady=5)
        self.canvas = tk.Canvas(self, bg="black", width=300, height=300)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self._draw_safe_zone() # Initialize safe zone

    def _draw_safe_zone(self, inner_radius_scaled=50, outer_radius_scaled=150, color="#444"):
        """Draws visual cues for safe zones."""
        self.canvas.create_oval(self.center_x - inner_radius_scaled,
                                self.center_y - inner_radius_scaled,
                                self.center_x + inner_radius_scaled,
                                self.center_y + inner_radius_scaled,
                                outline=color, dash=(2, 2), tags="safe_zone")
        self.canvas.create_oval(self.center_x - outer_radius_scaled,
                                self.center_y - outer_radius_scaled,
                                self.center_x + outer_radius_scaled,
                                self.center_y + outer_radius_scaled,
                                outline=color, dash=(2, 2), tags="safe_zone")

    def update_display(self, lidar_data):
        self.canvas.delete("lidar_point") # Use a specific tag for points
        if not lidar_data:
            return

        points_to_draw = []

        for angle_deg, distance_mm in lidar_data:
            if distance_mm > self.max_distance or distance_mm < 0:
                continue  # Skip out-of-range or invalid data

            angle_rad = math.radians(angle_deg)
            scaled_distance = distance_mm * self.scale_factor
            end_x = self.center_x + scaled_distance * math.cos(angle_rad)
            end_y = self.center_y - scaled_distance * math.sin(angle_rad)

            color = "#00FF00"  # Default green
            if distance_mm < 500:
                color = "#FF0000"  # Red for very close
            elif distance_mm < 1500:
                color = "#FFFF00"  # Yellow for close

            points_to_draw.append((end_x, end_y, color))

        for x, y, color in points_to_draw:
            self.canvas.create_oval(x - self.point_size, y - self.point_size,
                                    x + self.point_size, y + self.point_size,
                                    fill=color, outline=color, tags="lidar_point")
