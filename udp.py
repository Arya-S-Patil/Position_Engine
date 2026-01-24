import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
sock.bind(('', 5004))  # Port must match what you set on the EVB

print("Listening for broadcast UDP packets...")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"From {addr}: {data.decode(errors='ignore')}")

