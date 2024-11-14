import socket
import threading

# Server configuration
server_ip = '0.0.0.0'  # Listen on all available network interfaces
server_port = 13370    # Port to listen on

def handle_client_connection(client_socket, client_address):
    """Handle a single client connection and display raw data."""
    print(f"Connection established with {client_address}")

    try:
        while True:
            # Receive data from the client in chunks
            data = client_socket.recv(4096).decode('utf-8', errors='ignore')
            if not data:
                # If data is empty, client has disconnected
                print(f"Connection closed by {client_address}")
                break

            # Print raw data as received from the client
            print(f"Raw data from {client_address}: {data}")

    except Exception as e:
        print(f"Error with client {client_address}: {e}")
    finally:
        # Ensure the client socket is closed
        client_socket.close()
        print(f"Closed connection with {client_address}")

def start_server():
    """Initialize and start the server to listen for incoming client connections."""
    # Create a TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((server_ip, server_port))
    server_socket.listen(5)
    print(f"Server listening on {server_ip}:{server_port}")

    try:
        while True:
            # Accept a new client connection
            client_socket, client_address = server_socket.accept()
            print(f"New connection from {client_address}")

            # Handle client connection in a new thread
            client_thread = threading.Thread(
                target=handle_client_connection, args=(client_socket, client_address)
            )
            client_thread.start()
    except KeyboardInterrupt:
        print("Server shutting down...")
    finally:
        server_socket.close()

if __name__ == "__main__":
    start_server()
