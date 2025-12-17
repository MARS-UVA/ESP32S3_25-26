import socket

def send_buffer(ip: str, port: int):
    # The exact buffer you specified
    buf = bytearray([0xFF, 0x00, 0x9F, 0x9F, 0x9F, 0x9F, 0x9F])

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Send it
    sock.sendto(buf, (ip, port))
    print(f"Sent {buf} to {ip}:{port}")

if __name__ == "__main__":
    # Example usage
    send_buffer("192.168.0.103", 25000)
