import socket
import json

# Constants
UDP_IP = "127.0.0.1"  # Server IP address
UDP_PORT = 9090       # UDP port to listen on

# Create UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((UDP_IP, UDP_PORT))

print(f"Listening for data on {UDP_IP}:{UDP_PORT}...")

def handle_received_data(data):
    """Parse and process the received data."""
    try:
        # Attempt to load JSON data
        parsed_data = json.loads(data)
        
        # Process the data (here we just print it, but you can process it further)
        print("Received data:")
        for key, value in parsed_data.items():
            print(f"{key}: {value}")
    
    except json.JSONDecodeError:
        print("Failed to decode JSON data.")
    except Exception as e:
        print(f"Error handling received data: {e}")

while True:
    try:
        # Receive data from the client
        data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        
        print(f"Received message from {addr}:")
        
        # Handle the received data
        handle_received_data(data.decode('utf-8'))
    
    except Exception as e:
        print(f"Error receiving data: {e}")
