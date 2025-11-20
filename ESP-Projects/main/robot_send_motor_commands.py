import serial # type: ignore
from time import sleep

COM_Port = 'COM12'  # Change this to your actual COM port
BAUD_Rate = 115200    # Set the baud rate

# Open the serial port
# Replace 'COM1' with your port name (e.g., '/dev/ttyUSB0' on Linux)
try:
    ser = serial.Serial(COM_Port, baudrate=BAUD_Rate, timeout=1) 
    print(f"Opened port: {ser.name}")

    motor_data = bytearray([0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, 0x0, 0xFF, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F])  # Example motor command data
    # full speed forward for top left motor, neutral speed for all others

    while (True):
        ser.write(motor_data)
        sleep(0.1)  # Send data every 100ms
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