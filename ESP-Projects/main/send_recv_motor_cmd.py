import serial
import time

# Verify this port! On Mac, it's often /dev/tty.usbmodem... or /dev/tty.usbserial...
COM_Port = '/dev/tty.usbserial-0001' 
BAUD_Rate = 115200

try:
    ser = serial.Serial(COM_Port, BAUD_Rate, timeout=0.5)
    print(f"--- ATTEMPTING TO READ FROM {COM_Port} ---")
    
    while True:
        waiting = ser.in_waiting
        if waiting > 0:
            raw = ser.read(waiting)
            print(f"[{time.strftime('%H:%M:%S')}] Received {len(raw)} bytes: {raw.hex(' ')}")
        else:
            print(f"Buffer Empty... (Check ESP32 TX Pin)", end='\r')
            time.sleep(0.1)

except Exception as e:
    print(f"\nError: {e}")