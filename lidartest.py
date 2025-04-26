import serial
from rplidar import RPLidar
import numpy as np
import cv2
import math
import time

class RPLidarHandler:
    def __init__(self, port='COM5', baudrate=115200, timeout=3):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
    
    def connect(self):
        try:
            self.lidar = RPLidar(self.port, self.baudrate, timeout=self.timeout)
            self.lidar.start_motor()
            info = self.lidar.get_info()
            print(f"RPLIDAR Info: {info}")
            health = self.lidar.get_health()
            print(f"RPLIDAR Health: {health}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to RPLIDAR on {self.port}: {e}")
            return False
        except Exception as e:
            print(f"An unexpected error occurred during RPLIDAR connection: {e}")
            return False
    
    def disconnect(self):
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("RPLIDAR disconnected.")
            except Exception as e:
                print(f"Error during disconnection: {e}")
    
    def get_scan_data(self):
        if self.lidar:
            try:
                # Get just one scan instead of getting stuck in an iterator
                scan = next(self.lidar.iter_scans(scan_type='express'))
                points = []
                for quality, angle, distance in scan:
                    points.append((angle, distance / 1000.0))  # distance in meters
                return points
            except serial.SerialException as e:
                print(f"Serial error while reading RPLIDAR data: {e}")
            except Exception as e:
                print(f"Error reading RPLIDAR data: {e}")
        return []

def draw_radar_grid(frame, center, size, max_distance):
    # Draw concentric circles
    for d in range(1, int(max_distance) + 1):
        radius = int((d / max_distance) * (size / 2))
        cv2.circle(frame, center, radius, (30, 30, 30), 1)
        # Add distance label
        cv2.putText(frame, f"{d}m", 
                   (center[0] + radius - 20, center[1] - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    # Draw radial lines (every 30 degrees)
    for angle in range(0, 360, 30):
        radian = math.radians(angle)
        x = int(center[0] + (size/2) * math.cos(radian))
        y = int(center[1] + (size/2) * math.sin(radian))
        cv2.line(frame, center, (x, y), (30, 30, 30), 1)
        
        # Add angle label at the edge
        label_x = int(center[0] + (size/2 - 30) * math.cos(radian))
        label_y = int(center[1] + (size/2 - 30) * math.sin(radian))
        cv2.putText(frame, f"{angle}°", (label_x-10, label_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

def main():
    lidar = RPLidarHandler(port='COM5')
    
    if lidar.connect():
        try:
            print("Starting visualization...")
            # Create a black image
            img_size = 800
            center = (img_size // 2, img_size // 2)
            max_distance = 8  # meters
            
            # Create window with trackbars
            cv2.namedWindow("RPLIDAR Visualization")
            
            running = True
            while running:
                # Create a fresh frame
                frame = np.zeros((img_size, img_size, 3), dtype=np.uint8)
                
                # Draw radar grid
                draw_radar_grid(frame, center, img_size, max_distance)
                
                # Get scan data
                scan = lidar.get_scan_data()
                
                if scan:
                    # Draw scan points
                    for angle, distance in scan:
                        if distance <= 0.0 or distance > max_distance:
                            continue
                        
                        # Scale distance to pixels
                        r = (distance / max_distance) * (img_size / 2)
                        theta = math.radians(angle)
                        x = int(center[0] + r * math.cos(theta))
                        y = int(center[1] + r * math.sin(theta))
                        
                        # Draw point with color based on distance
                        # Close = red, far = green
                        intensity = int(255 * (distance / max_distance))
                        color = (0, intensity, 255-intensity)
                        cv2.circle(frame, (x, y), 2, color, -1)
                
                # Draw center point and axes
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.line(frame, (center[0], 0), (center[0], img_size), (50, 50, 50), 1)
                cv2.line(frame, (0, center[1]), (img_size, center[1]), (50, 50, 50), 1)
                
                # Add info text
                cv2.putText(frame, "RPLIDAR Visualization", (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.putText(frame, "Press ESC to exit", (10, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
                
                # Show points count
                if scan:
                    cv2.putText(frame, f"Points: {len(scan)}", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
                
                # Show the frame
                cv2.imshow("RPLIDAR Visualization", frame)
                
                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC key
                    running = False
                
                # Add a small delay to reduce CPU usage
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("Operation interrupted by user.")
        
        finally:
            print("Stopping visualization...")
            lidar.disconnect()
            cv2.destroyAllWindows()
    
    else:
        print("Failed to connect to RPLIDAR.")

if __name__ == "__main__":
    main()