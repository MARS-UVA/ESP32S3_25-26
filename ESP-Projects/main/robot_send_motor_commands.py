import serial 
from time import sleep
from array import array

COM_Port = 'COM3'  # Change this to your actual COM port
BAUD_Rate = 115200    # Set the baud rate

# Open the serial port
# Replace 'COM1' with your port name (e.g., '/dev/ttyUSB0' on Linux)
try:
    ser = serial.Serial(COM_Port, baudrate=BAUD_Rate, timeout=1) 
    print(f"Opened port: {ser.name}")

    motor_data = bytearray([0xFF, 0x7F, 0x9F, 0x7F, 0x03, 0x7F, 0xFE])  # Example motor command data
    # full speed forward for top left motor, neutral speed for all others
    i = 5
    while (True):
        #i = (i + 1) % 250
        #motor_data[4] = i
        #ser.write(motor_data)
        #sleep(0.01)  # Send data every 100ms
        
        data = ser.read(12)
        if len(data) == 12:
            floats = array('f', data[4:])
            print(f"Received: {data[0]} {data[1]} {data[2]} {data[3]} {floats[0]} {floats[1]}")
        else:
            print("No data received")

except serial.SerialException as e:
    print(f"Error opening or communicating with serial port: {e}")

finally:
    # Close the port if it was opened
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
