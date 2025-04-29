import sys
import time
import threading
import math
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk
from collections import deque
from rplidar import RPLidar

# Configuration
PORT_NAME = 'COM5'  # Adjust according to your port
VEHICLE_WIDTH = 42  # Vehicle width in cm
OBSTACLE_THRESHOLD = 200  # Max distance to consider an obstacle (2m = 200cm)
VIEW_ANGLE = 90  # 180° angle centered (-90° to +90°)
WINDOW_SIZE = (800, 600)
MAX_DISTANCE = 500  # Maximum distance to display (in cm)
HISTORY_SIZE = 5  # Number of frames to preserve for stability

# Colors
WHITE = "#FFFFFF"
BLACK = "#000000"
RED = "#FF0000"
GREEN = "#00FF00"
BLUE = "#0000FF"
YELLOW = "#FFFF00"
DARK_GRAY = "#1E1E1E"
LIGHT_GRAY = "#646464"

class LidarProcessor:
    def __init__(self):
        self.lidar = None
        self.running = False
        self.scan_data = {}
        self.obstacle_points = []
        self.closest_distance = float('inf')
        self.closest_angle = 0
        self.history = deque(maxlen=HISTORY_SIZE)

        # Calculate vehicle passage zone
        self.angle_width = math.degrees(math.atan((VEHICLE_WIDTH/2) / 100))  # Half-angle in degrees

        # Mutex to protect data access
        self.data_lock = threading.Lock()

    def connect(self):
        try:
            self.lidar = RPLidar(PORT_NAME)
            info = self.lidar.get_info()
            print(f"LIDAR connected: {info}")
            health = self.lidar.get_health()
            print(f"LIDAR status: {health}")
            return True
        except Exception as e:
            print(f"Error connecting to LIDAR: {e}")
            return False

    def start(self):
        if self.connect():
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_thread)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            return True
        return False

    def stop(self):
        self.running = False
        if hasattr(self, 'scan_thread') and self.scan_thread:
            self.scan_thread.join(timeout=1.0)
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()

    def _scan_thread(self):
        try:
            self.lidar.start_motor()
            time.sleep(1)  # Wait for motor to reach speed

            for scan in self.lidar.iter_scans():
                if not self.running:
                    break

                # Process scan data
                scan_data = {}
                for _, angle, distance in scan:
                    # Convert to cm
                    distance = distance / 10
                    # Normalize angle between -180 and 180
                    norm_angle = angle
                    if norm_angle > 180:
                        norm_angle -= 360
                    scan_data[norm_angle] = distance

                # Identify obstacles
                obstacle_points = []
                min_distance = float('inf')
                min_angle = 0

                for angle, distance in scan_data.items():
                    # Check if point is in area of interest (in front of vehicle)
                    if -VIEW_ANGLE <= angle <= VIEW_ANGLE:
                        # Check if distance is less than threshold
                        if distance <= OBSTACLE_THRESHOLD:
                            # Calculate lateral position relative to central axis
                            lateral_position = distance * math.tan(math.radians(angle))

                            # Check if obstacle is in vehicle path
                            if abs(lateral_position) <= (VEHICLE_WIDTH / 2):
                                obstacle_points.append((angle, distance, True))  # In path

                                # Update minimum distance
                                if distance < min_distance:
                                    min_distance = distance
                                    min_angle = angle
                            else:
                                obstacle_points.append((angle, distance, False))  # Out of path

                # Add to history for stability
                with self.data_lock:
                    self.scan_data = scan_data
                    self.obstacle_points = obstacle_points
                    if min_distance < float('inf'):
                        self.closest_distance = min_distance
                        self.closest_angle = min_angle
                    self.history.append((scan_data, obstacle_points, min_distance, min_angle))

        except Exception as e:
            print(f"Error in scan thread: {e}")
        finally:
            if self.running:
                self.stop()

    def get_stable_data(self):
        """Returns stabilized data using history"""
        with self.data_lock:
            if not self.history:
                return {}, [], float('inf'), 0

            # Average of latest measurements for closest distance
            closest_distances = [h[2] for h in self.history if h[2] < float('inf')]
            if closest_distances:
                avg_closest = sum(closest_distances) / len(closest_distances)
            else:
                avg_closest = float('inf')

            # Use most recent data for display
            return self.scan_data, self.obstacle_points, avg_closest, self.closest_angle

class LidarVisualizer(ttk.Frame):
    def __init__(self, master=None, lidar_processor=None):
        super().__init__(master)
        self.master = master
        self.lidar_processor = lidar_processor
        self.running = False

        if self.master:
            self.master.title("RPLidar - Obstacle Detection")
            self.master.geometry(f"{WINDOW_SIZE[0]}x{WINDOW_SIZE[1]}")
            self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.canvas = tk.Canvas(self, width=WINDOW_SIZE[0], height=WINDOW_SIZE[1], bg=BLACK)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Define fonts
        self.font = tkfont.Font(family="Arial", size=14, weight="bold")
        self.small_font = tkfont.Font(family="Arial", size=10)

        # Center of screen (vehicle position)
        self.center_x = WINDOW_SIZE[0] // 2
        self.center_y = WINDOW_SIZE[1] - 50

        # Scale factor for display
        self.scale = min(WINDOW_SIZE) / (MAX_DISTANCE * 1.5)

        # Text elements
        self.distance_text = self.canvas.create_text(10, 10, anchor=tk.NW,
                                                    text="Distance: -- cm",
                                                    fill=WHITE, font=self.font)
        self.warning_text = self.canvas.create_text(10, 50, anchor=tk.NW,
                                                   text="", fill=RED, font=self.font)
        self.vehicle_info = self.canvas.create_text(10, WINDOW_SIZE[1] - 30, anchor=tk.NW,
                                                   text=f"Width: {VEHICLE_WIDTH} cm",
                                                   fill=WHITE, font=self.small_font)

        # Create update loop
        self.update_id = None

    def start(self):
        self.running = True
        self.update_display()

    def stop(self):
        self.running = False
        if self.update_id:
            self.after_cancel(self.update_id)
        if self.master:
            self.master.destroy()

    def on_closing(self):
        self.running = False
        if self.master:
            self.master.quit()

    def update_display(self):
        if not self.running:
            return

        # Get stabilized data
        scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()

        # Clear canvas
        self.canvas.delete("scan", "obstacle", "closest", "reference")

        # Draw reference elements
        self.draw_reference_elements()

        # Draw scan points
        for angle, distance in scan_data.items():
            if -VIEW_ANGLE <= angle <= VIEW_ANGLE and distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill=WHITE, outline=WHITE, tags="scan")

        # Draw obstacles
        for angle, distance, in_path in obstacle_points:
            x, y = self._polar_to_cartesian(angle, distance)
            color = RED if in_path else YELLOW
            self.canvas.create_oval(x-4, y-4, x+4, y+4, fill=color, outline=color, tags="obstacle")

        # Display closest distance
        if closest_distance < float('inf'):
            # Draw line to closest obstacle
            x, y = self._polar_to_cartesian(closest_angle, closest_distance)
            self.canvas.create_line(self.center_x, self.center_y, x, y, fill=GREEN, width=2, tags="closest")

            # Update distance text
            self.canvas.itemconfig(self.distance_text, text=f"Distance: {closest_distance:.2f} cm")

            # Display warning if necessary
            if closest_distance < 50:  # Alert if less than 50 cm
                self.canvas.itemconfig(self.warning_text, text="BRAKING RECOMMENDED!")
            else:
                self.canvas.itemconfig(self.warning_text, text="")
        else:
            self.canvas.itemconfig(self.distance_text, text="Distance: -- cm")
            self.canvas.itemconfig(self.warning_text, text="")

        # Schedule next update
        self.update_id = self.after(33, self.update_display)  # ~30 FPS

    def draw_reference_elements(self):
        # Draw distance reference circles
        for r in range(100, MAX_DISTANCE + 1, 100):
            radius = int(r * self.scale)
            self.canvas.create_oval(
                self.center_x - radius, self.center_y - radius,
                self.center_x + radius, self.center_y + radius,
                outline=DARK_GRAY, tags="reference"
            )
            # Distance label
            self.canvas.create_text(
                self.center_x + 5, self.center_y - radius - 10,
                text=f"{r} cm", fill=LIGHT_GRAY, anchor=tk.W,
                font=self.small_font, tags="reference"
            )

        # Draw reference angles
        for angle in range(-VIEW_ANGLE, VIEW_ANGLE + 1, 30):
            rads = math.radians(angle)
            x = self.center_x + int(MAX_DISTANCE * self.scale * math.sin(rads))
            y = self.center_y - int(MAX_DISTANCE * self.scale * math.cos(rads))
            self.canvas.create_line(
                self.center_x, self.center_y, x, y,
                fill=DARK_GRAY, tags="reference"
            )

            # Angle label
            if angle != 0:
                self.canvas.create_text(
                    x + 5, y + 5, text=f"{angle}°",
                    fill=LIGHT_GRAY, anchor=tk.NW,
                    font=self.small_font, tags="reference"
                )

        # Draw vehicle (simple rectangle)
        half_width = int((VEHICLE_WIDTH / 2) * self.scale)
        self.canvas.create_rectangle(
            self.center_x - half_width, self.center_y - 10,
            self.center_x + half_width, self.center_y + 10,
            fill=BLUE, outline=BLUE, tags="reference"
        )

        # Draw vehicle path as two parallel lines
        half_width = int((VEHICLE_WIDTH / 2) * self.scale)

        # Left parallel line
        self.canvas.create_line(
            self.center_x - half_width, self.center_y,
            self.center_x - half_width, self.center_y - int(MAX_DISTANCE * self.scale),
            fill=BLUE, width=1, tags="reference"
        )

        # Right parallel line
        self.canvas.create_line(
            self.center_x + half_width, self.center_y,
            self.center_x + half_width, self.center_y - int(MAX_DISTANCE * self.scale),
            fill=BLUE, width=1, tags="reference"
        )

    def _polar_to_cartesian(self, angle, distance):
        """Converts polar coordinates to cartesian coordinates for display"""
        rads = math.radians(angle)
        x = self.center_x + int(distance * self.scale * math.sin(rads))
        y = self.center_y - int(distance * self.scale * math.cos(rads))
        return x, y


def main():
    try:
        # Initialize LIDAR processor
        lidar_processor = LidarProcessor()
        if not lidar_processor.start():
            print("Unable to start LIDAR. Check connection.")
            return

        # Initialize Tkinter root
        root = tk.Tk()
        app = LidarVisualizer(root, lidar_processor)
        app.pack(fill=tk.BOTH, expand=True)
        app.start()
        root.mainloop()

        # Clean up after mainloop
        lidar_processor.stop()

    except Exception as e:
        print(f"Error in main program: {e}")
    finally:
        # Ensure LIDAR is stopped if an error occurred before visualizer started
        if 'lidar_processor' in locals() and lidar_processor.running:
            lidar_processor.stop()
        sys.exit()

if __name__ == "__main__":
    main()