from time import sleep
import sys
import socket

server_addr = sys.argv[1]
server_port = sys.argv[2]

#udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#udp_socket.bind((server_addr, int(server_port)))

print(f"UDP socket bound to port {server_port}, ready to send motor commands.")

try:
    motor_data = bytearray([0xFF, 0x0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F]) # Example motor command data
    # first byte: start byte (0xFF)
    # second byte: header byte (just 0x0)
    # the next 6 bytes set neutral speed for all 6 motors and 2 actuators

    #data, addr = udp_socket.recvfrom(1024) # You must receive something first to get the client's address
    while (True):
        udp_socket.sendto(motor_data, addr)
        sleep(0.05)
        # Send data every 100ms
        # data = ser.read(16)
        # if len(data) > 0:
        #     print(f"Received: {data}")
        # else:
        #     print("No data received")

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Close the socket if it was opened
    if 'udp_socket' in locals():
        udp_socket.close()
        print("UDP socket closed.")