import serial
import time

class ESP32Controller:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self._connect()

    def _connect(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for ESP32 to reset
            print(f"Connected to ESP32 on {self.port}")
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            self.serial_connection = None

    def _send_command(self, command):
        if self.serial_connection and self.serial_connection.is_open:
            try:
                command_bytes = (command + '\n').encode()
                self.serial_connection.write(command_bytes)
                print(f"Sent: {command}")
                self._read_response()
                return True
            except serial.SerialException as e:
                print(f"Error sending command '{command}': {e}")
                self._reconnect()
                return False
        else:
            print("Serial connection not active. Attempting to reconnect...")
            self._reconnect()
            return False

    def _read_response(self):
        while self.serial_connection and self.serial_connection.in_waiting:
            try:
                response = self.serial_connection.readline().decode().strip()
                print(f"Received: {response}")
            except serial.SerialException as e:
                print(f"Error reading response: {e}")
                self._reconnect()
                break
            except UnicodeDecodeError:
                print("Received non-text data.")
                break

    def _reconnect(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed.")
            self.serial_connection = None
        self._connect()

    def set_automatic(self):
        return self._send_command('A')

    def set_manual(self):
        return self._send_command('M')

    def set_speed(self, speed):
        if 0 <= speed <= 100:
            return self._send_command(f'V{int(speed)}')
        else:
            print("Speed value must be between 0 and 100.")
            return False

    def forward(self):
        return self._send_command('D')

    def backward(self):
        return self._send_command('R')

    def activate_brake(self):
        return self._send_command('F')

    def deactivate_brake(self):
        return self._send_command('Z')

    def close(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    # Replace 'COM5' with the actual serial port of your ESP32
    esp32 = ESP32Controller('COM5')

    try:
        esp32.set_automatic()
        time.sleep(1)
        esp32.set_speed(50)
        time.sleep(1)
        esp32.forward()
        time.sleep(2)
        esp32.backward()
        time.sleep(2)
        esp32.activate_brake()
        time.sleep(1)
        esp32.deactivate_brake()
        time.sleep(1)
        esp32.set_manual()

    finally:
        esp32.close()