import tkinter as tk
from tkinter import ttk
import math

class CarLidarView(ttk.Frame):
    def __init__(self, parent, point_size=3, max_distance=100, **kwargs):
        super().__init__(parent, style="Dark.TFrame", **kwargs)
        self.point_size = point_size
        self.max_distance = max_distance
        self.scale_factor = 280 / self.max_distance  # Adjust scale for the limited view
        self.center_x = 150
        self.center_y = 280  # Center towards the bottom for a forward view
        self._create_widgets()

    def _create_widgets(self):
        ttk.Label(self, text="Car LIDAR (Forward View)", style="Title.TLabel").pack(pady=5)
        self.canvas = tk.Canvas(self, bg="black", width=300, height=300)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)
        self._draw_fov()

    def _draw_fov(self, angle_start=60, angle_end=120, radius=150, color="#555"):
        """Draws the field of view arc."""
        start_rad = math.radians(angle_start - 90) # Adjust for canvas orientation
        end_rad = math.radians(angle_end - 90)     # Adjust for canvas orientation
        self.canvas.create_arc(self.center_x - radius, self.center_y - radius,
                               self.center_x + radius, self.center_y + radius,
                               start=math.degrees(start_rad), extent=math.degrees(end_rad - start_rad),
                               style=tk.ARC, outline=color, dash=(3, 3), tags="fov")

    def update_display(self, lidar_data):
        self.canvas.delete("lidar_point")
        if not lidar_data:
            return

        points_to_draw = []

        for angle_deg, distance_mm in lidar_data:
            # Assuming data is already filtered to 60-120 degrees
            if 0 <= distance_mm <= self.max_distance:
                angle_rad = math.radians(angle_deg - 90) # Adjust for canvas orientation
                scaled_distance = distance_mm * self.scale_factor
                end_x = self.center_x + scaled_distance * math.cos(angle_rad)
                end_y = self.center_y - scaled_distance * math.sin(angle_rad)

                color = "#00FF00"  # Default green
                if distance_mm < 10:
                    color = "#FF0000"  # Red for very close
                elif distance_mm < 30:
                    color = "#FFFF00"  # Yellow for close

                points_to_draw.append((end_x, end_y, color))

        for x, y, color in points_to_draw:
            self.canvas.create_oval(x - self.point_size, y - self.point_size,
                                    x + self_point_size, y + self.point_size,
                                    fill=color, outline=color, tags="lidar_point")

if __name__ == '__main__':
    import tkinter as tk
    import random

    def update_lidar_data():
        # Simulate car LIDAR data (60 to 120 degrees)
        new_data = []
        for angle in range(60, 121, 2):
            distance = random.randint(5, 100)
            if random.random() < 0.8: # Simulate some detections
                new_data.append((angle, distance))
        car_lidar_view.update_display(new_data)
        root.after(100, update_lidar_data)

    root = tk.Tk()
    root.title("Car LIDAR Visualization")
    root.style = ttk.Style()
    root.style.theme_use('clam')

    car_lidar_view = CarLidarView(root)
    car_lidar_view.pack(fill="both", expand=True)

    root.after(500, update_lidar_data)

    root.mainloop()