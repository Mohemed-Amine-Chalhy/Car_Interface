import sys
import time
import threading
import math
import tkinter as tk
from tkinter import font as tkfont, ttk
from collections import deque
from rplidar import RPLidar

# Default Configuration
DEFAULT_PORT_NAME = 'COM6'  # Default port
VEHICLE_WIDTH = 42  # Vehicle width in cm
OBSTACLE_THRESHOLD = 200  # Max distance to consider an obstacle (2m = 200cm)
VIEW_ANGLE = 90  # 180° angle centered around 90° (0° to 180°)
WINDOW_SIZE = (800, 700)  # Increased height for controls
MAX_DISTANCE = 2000  # Maximum distance to display (in cm)
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
    def __init__(self, port_name=DEFAULT_PORT_NAME):
        self.port_name = port_name
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
            self.lidar = RPLidar(self.port_name)
            info = self.lidar.get_info()
            print(f"LIDAR connected on {self.port_name}: {info}")
            health = self.lidar.get_health()
            print(f"LIDAR status: {health}")
            return True
        except Exception as e:
            print(f"Error connecting to LIDAR on {self.port_name}: {e}")
            return False

    def start(self):
        if not self.running:
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
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print(f"LIDAR disconnected from {self.port_name}")
            except Exception as e:
                print(f"Error stopping LIDAR: {e}")

    def _scan_thread(self):
        try:
            if self.lidar:
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

                        # Shift angle so that 0° is the front of the vehicle
                        adjusted_angle = angle - 90
                        # Normalize angle between -180 and 180
                        if adjusted_angle > 180:
                            adjusted_angle -= 360
                        elif adjusted_angle < -180:
                            adjusted_angle += 360

                        scan_data[adjusted_angle] = distance

                    # Identify obstacles
                    obstacle_points = []
                    min_distance = float('inf')
                    min_angle = 0

                    for angle, distance in scan_data.items():
                        # Check if point is in area of interest (in front of vehicle)
                        if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2:
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

class LidarVisualizer:
    def __init__(self, master, lidar_processor):
        self.master = master
        self.lidar_processor = lidar_processor
        self.running = False
        self.lidar_assist_enabled = tk.BooleanVar(value=True) # Default to enabled

        master.title("RPLidar - Obstacle Detection")
        master.geometry(f"{WINDOW_SIZE[0]}x{WINDOW_SIZE[1]}")
        master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # UI Controls Frame
        self.controls_frame = ttk.LabelFrame(master, text="LIDAR Control")
        self.controls_frame.pack(pady=10, padx=10, fill=tk.X)

        ttk.Label(self.controls_frame, text="LIDAR Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_entry = ttk.Entry(self.controls_frame, width=15)
        self.port_entry.insert(0, DEFAULT_PORT_NAME)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        self.activate_button = ttk.Button(self.controls_frame, text="Activate LIDAR", command=self.start_lidar)
        self.activate_button.grid(row=1, column=0, padx=5, pady=5, sticky=tk.W+tk.E)

        self.deactivate_button = ttk.Button(self.controls_frame, text="Deactivate LIDAR", command=self.stop_lidar, state=tk.DISABLED)
        self.deactivate_button.grid(row=1, column=1, padx=5, pady=5, sticky=tk.W+tk.E)

        self.assist_checkbox = ttk.Checkbutton(self.controls_frame, text="Lidar Assist", variable=self.lidar_assist_enabled)
        self.assist_checkbox.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky=tk.W)

        # Canvas for Lidar Display
        self.canvas = tk.Canvas(master, width=WINDOW_SIZE[0], height=WINDOW_SIZE[1] - 100, bg=BLACK)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Define fonts
        self.font = tkfont.Font(family="Arial", size=14, weight="bold")
        self.small_font = tkfont.Font(family="Arial", size=10)

        # Center of screen (vehicle position)
        self.center_x = WINDOW_SIZE[0] // 2
        self.center_y = (WINDOW_SIZE[1] - 100) - 50

        # Scale factor for display
        self.scale = min(WINDOW_SIZE[0], WINDOW_SIZE[1] - 100) / (MAX_DISTANCE * 1.5)

        # Text elements
        self.distance_text = self.canvas.create_text(10, 10, anchor=tk.NW,
                                                    text="Distance: -- m",
                                                    fill=WHITE, font=self.font)
        self.warning_text = self.canvas.create_text(10, 50, anchor=tk.NW,
                                                   text="", fill=RED, font=self.font)
        self.vehicle_info = self.canvas.create_text(10, (WINDOW_SIZE[1] - 100) - 30, anchor=tk.NW,
                                                   text=f"Width: {VEHICLE_WIDTH} cm",
                                                   fill=WHITE, font=self.small_font)

        # Create update loop
        self.update_id = None

    def start_lidar(self):
        port = self.port_entry.get()
        if port:
            self.lidar_processor.port_name = port
            if self.lidar_processor.start():
                self.running = True
                self.activate_button.config(state=tk.DISABLED)
                self.deactivate_button.config(state=tk.NORMAL)
                self.update_display()
            else:
                tk.messagebox.showerror("Error", f"Could not connect to LIDAR on port {port}")
        else:
            tk.messagebox.showerror("Error", "Please enter a LIDAR port.")

    def stop_lidar(self):
        if self.running:
            self.running = False
            self.lidar_processor.stop()
            self.activate_button.config(state=tk.NORMAL)
            self.deactivate_button.config(state=tk.DISABLED)
            if self.update_id:
                self.master.after_cancel(self.update_id)
            self.canvas.delete("scan", "obstacle", "closest")
            self.canvas.itemconfig(self.distance_text, text="Distance: -- cm")
            self.canvas.itemconfig(self.warning_text, text="")

    def on_closing(self):
        self.running = False
        self.lidar_processor.stop()
        self.master.quit()
        self.master.destroy()

    def update_display(self):
        if not self.running:
            return

        # Get stabilized data
        scan_data, obstacle_points, closest_distance, closest_angle = self.lidar_processor.get_stable_data()

        # Clear canvas
        self.canvas.delete("scan", "obstacle", "closest")

        # Draw reference elements
        self.draw_reference_elements()

        # Draw scan points
        for angle, distance in scan_data.items():
            if -VIEW_ANGLE/2 <= angle <= VIEW_ANGLE/2 and distance <= MAX_DISTANCE:
                x, y = self._polar_to_cartesian(angle, distance)
                self.canvas.create_oval(x-2, y-2, x+2, y+2, fill=WHITE, outline=WHITE, tags="scan")

        # Draw obstacles based on 'Lidar Assist' checkbox
        if self.lidar_assist_enabled.get():
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
            if closest_distance < 50 and self.lidar_assist_enabled.get():  # Only warn if assist is on
                self.canvas.itemconfig(self.warning_text, text="BRAKING RECOMMENDED!")
            else:
                self.canvas.itemconfig(self.warning_text, text="")
        else:
            self.canvas.itemconfig(self.distance_text, text="Distance: -- cm")
            self.canvas.itemconfig(self.warning_text, text="")

        # Schedule next update
        self.update_id = self.master.after(33, self.update_display)  # ~30 FPS

    def draw_reference_elements(self):
        # Clear previous reference elements
        self.canvas.delete("reference")

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
                text=f"{r* 0.01} m", fill=LIGHT_GRAY, anchor=tk.W,
                font=self.small_font, tags="reference"
            )

        # Draw reference angles
        half_view = VIEW_ANGLE/2
        for angle in range(int(-half_view), int(half_view) + 1, 30):
            rads = math.radians(angle)
            x = self.center_x + int(MAX_DISTANCE * self.scale * math.sin(rads))
            y = self.center_y - int(MAX_DISTANCE * self.scale * math.cos(rads))
            self.canvas.create_line(
                self.center_x, self.center_y, x, y,
                fill=DARK_GRAY, tags="reference"
            )

            # Angle label with adjusted values to show that 0° is front (90° in LIDAR's reference)
            if angle != 0:
                lidar_angle = angle + 90  # Convert back to LIDAR's original angle
                self.canvas.create_text(
                    x + 5, y + 5, text=f"{lidar_angle}°",
                    fill=LIGHT_GRAY, anchor=tk.NW,
                    font=self.small_font, tags="reference"
                )
            else:
                # Mark front (90° in LIDAR's reference)
                self.canvas.create_text(
                    x, y - 15, text="90° (FRONT)",
                    fill=GREEN, anchor=tk.S,
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
        # Initialize LIDAR processor with default port
        lidar_processor = LidarProcessor()

        # Initialize Tkinter root window
        root = tk.Tk()

        # Initialize visualizer, passing the root window
        visualizer = LidarVisualizer(root, lidar_processor)

        # Start the Tkinter main loop
        root.mainloop()

    except Exception as e:
        print(f"Error in main program: {e}")
    finally:
        # Clean up
        if 'lidar_processor' in locals():
            lidar_processor.stop()
        if 'root' in locals():
            root.destroy()
        sys.exit()

if __name__ == "__main__":
    main()