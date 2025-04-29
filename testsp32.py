import serial
import time

# Adjust the port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' or '/dev/ttyACM0' on Linux)
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # Wait for ESP32 to reset after opening serial

# Send a command (for example, "S" to tighten the belt)
ser.write(b'S\n')

# Optional: Read response from ESP32
while ser.in_waiting:
    response = ser.readline().decode().strip()
    print(response)

ser.close()
