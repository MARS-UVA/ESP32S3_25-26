import socket

def listen_udp(ip: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))

    print(f"Listening on {ip}:{port}... (press Ctrl+C to stop)")

    try:
        while True:
            data, addr = sock.recvfrom(2048)  # buffer size
            print(f"Received {len(data)} bytes from {addr}: {data.hex(' ')}")
    except KeyboardInterrupt:
        print("\nExiting listener...")
    finally:
        sock.close()

if __name__ == "__main__":
    listen_udp("0.0.0.0", 25000)
