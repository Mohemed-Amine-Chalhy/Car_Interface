import serial
import time

# Set the correct port for your Arduino (usually something like COM3 on Windows or /dev/ttyUSB0 on Linux)
arduino_port = 'COM6'  # Replace with your Arduino port (check Device Manager on Windows or dmesg on Linux)
baud_rate = 115200  # Match the baud rate in the Arduino code

# Create a serial object
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

# Function to send a command to Arduino
def send_command(command):
    ser.write(command.encode())  # Send the command as bytes
    time.sleep(1)  # Wait for Arduino to process the command
    response = ser.readline().decode('utf-8').strip()  # Read and decode the response
    print(f"Response from Arduino: {response}")

# Loop to interact with the user
while True:
    print("Enter command ('S' - Serrer, 'D' - Desserrer, 'A' - Arret d'urgence):")
    command = input()
    if command in ['S', 'D', 'A']:
        send_command(command)
    else:
        print("Invalid command. Please enter 'S', 'D', or 'A'.")
