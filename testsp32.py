import serial
import time

# Open serial connection
ser = serial.Serial('COM6', 115200, timeout=2)

# Clear buffers and wait for ESP32 to stabilize
time.sleep(3)
ser.reset_input_buffer()

# First set to manual mode
print("Setting to manual mode...")

ser.write(b'M\n')

ser.write(b'A\n')
ser.write(b'V100\n')
time.sleep(50)
print("Response:", ser.read(ser.in_waiting).decode().strip())


#ser.close()