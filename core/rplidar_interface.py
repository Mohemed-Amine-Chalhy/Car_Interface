import serial
from rplidar import RPLidar
import time

class RPLidarHandler:
    def __init__(self, port='COM5', baudrate=115200, timeout=3):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.scan_generator = None
        self.is_scanning = False

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
                self.stop_scanning()
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                self.lidar = None
                print("RPLIDAR disconnected.")
            except Exception as e:
                print(f"Error during RPLIDAR disconnection: {e}")

    def start_scanning(self):
        if self.lidar and not self.is_scanning:
            try:
                # Initialize the iterator for scans
                self.scan_generator = self.lidar.iter_scans(scan_type='normal')
                self.is_scanning = True
                print("RPLIDAR scanning started.")
            except Exception as e:
                print(f"Error starting scan: {e}")
                self.is_scanning = False

    def stop_scanning(self):
        if self.lidar and self.is_scanning:
            try:
                self.scan_generator = None
                self.is_scanning = False
                print("RPLIDAR scanning stopped.")
            except Exception as e:
                print(f"Error stopping scan: {e}")

    def get_scan_data(self):
        if self.lidar and self.is_scanning and self.scan_generator:
            try:
                # Get just one scan from the iterator
                scan = next(self.scan_generator)
                scan_data = []
                for quality, angle, distance in scan:
                    if distance > 0 and 60<angle<120:  # Ignore zero values
                        scan_data.append((angle, distance))  # Angle in degrees, distance in cm
                return scan_data
            except StopIteration:
                print("Scan iteration complete, restarting...")
                self.start_scanning()  # Restart the scan
                return []
            except serial.SerialException as e:
                print(f"Serial error while reading RPLIDAR data: {e}")
                return []
            except Exception as e:
                print(f"Error reading RPLIDAR data: {e}")
                return []
        return []

    def __del__(self):
        self.disconnect()