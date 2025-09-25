import socket

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to localhost on port 43897
server_address = ('0.0.0.0', 43897)
sock.bind(server_address)

print(f"Listening on {server_address}...")

try:
    while True:
        # Receive data (up to 1024 bytes)
        data, addr = sock.recvfrom(10)
        print(f"Received {len(data)} bytes from {addr}: {data}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    sock.close()