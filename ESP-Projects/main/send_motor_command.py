import serial # type: ignore
from time import sleep

COM_Port = '/dev/tty.debug-console'  # Change this to your actual COM port
BAUD_Rate = 115200    # Set the baud rate

# Open the serial port
# Replace 'COM1' with your port name (e.g., '/dev/ttyUSB0' on Linux)
try:
    ser = serial.Serial(COM_Port, baudrate=BAUD_Rate, timeout=1)
    print(f"Opened port: {ser.name}")

    on_off_data = bytearray([0x00, 0x01])
    # full speed forward for top left motor, neutral speed for all others

    while (True):
        ser.write(on_off_data[0])
        print(on_off_data[0])
        sleep(2)  # Send data every 100ms
        ser.write(on_off_data[1])
        print(on_off_data[1])
        break
        # data = ser.read(16)
        # if len(data) > 0:
        #     print(f"Received: {data}")
        # else:
        #     print("No data received")

except serial.SerialException as e:
    print(f"Error opening or communicating with serial port: {e}")

finally:
    # Close the port if it was opened
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")