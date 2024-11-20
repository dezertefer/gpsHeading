import socket
import json

# Server address and port
UDP_IP = "127.0.0.1"  # Adjust if the server is on another machine
UDP_PORT = 9090        # Same port as the server

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto("connect".encode('utf-8'), (UDP_IP, UDP_PORT))
print("Sent 'connect' to the server.")

print(f"Listening for data on {UDP_IP}:{UDP_PORT}...")

def receive_data():
    while True:
        # Receive data from the server
        data, address = sock.recvfrom(1024)  # No need to bind explicitly

        # Decode and load the received JSON data
        try:
            #json_data = json.loads(data.decode('utf-8'))
            print(data)
            #print(json.dumps(json_data, indent=4))
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON data: {e}")

if __name__ == "__main__":
    receive_data()
