import serial # type: ignore
from time import sleep
import struct

COM_Port = 'COM3'  # Change this to your actual COM port
BAUD_Rate = 115200    # Set the baud rate

# Open the serial port
# Replace 'COM1' with your port name (e.g., '/dev/ttyUSB0' on Linux)
try:
    ser = serial.Serial(COM_Port, baudrate=BAUD_Rate, timeout=1) 
    print(f"Opened port: {ser.name}")
        
    motor_data = bytearray([0xFF, 0x0, 0x70, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F])  # Example motor command data
    # first byte: start byte (0xFF)
    # second byte: header byte (just 0x0)
    # the next 6 bytes set neutral speed for all 6 motors and 2 actuators

    # writing and reading serial packets sequentially will cause the motor to stutter, so make sure to do only do one at a time!
    while (True):
        # ser.write(motor_data) # Send motor command
        
        print(ser.in_waiting)
        header = ser.read(4) # Read header bytes (blocking), expecting 0x02
        print(f"Debug header: {header}")
        if header == b'\xFF\x02\x00\x00': # Check if the header matches the expected value
            print(f"Read header: {header}")
            feedback = list(struct.iter_unpack("6i",ser.read(24))) # tuple of: fl, fr, bl, br, ldrum, rdrum, la, ra, potentiometer
            feedback = [i[0] for i in feedback]
            print(f"Header: {header}, Feedback: {feedback}")
            #print(f"Potentiometer reading: {feedback[8]}")
        else:
            print("Timeout: No data received")


except serial.SerialException as e:
    print(f"Error opening or communicating with serial port: {e}")

finally:
    # Close the port if it was opened
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")
