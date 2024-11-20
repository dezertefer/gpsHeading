import socket

def start_tcp_client(host='127.0.0.1', port=13370):
    """Start a TCP client that connects to the server and listens for data."""
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # Connect to the server
        client_socket.connect((host, port))
        print(f"Connected to server at {host}:{port}...")

        while True:
            try:
                # Receive data from the server
                data = client_socket.recv(1024)
                if not data:
                    break  # If no data, the server closed the connection

                # Print the received JSON data
                #print("Received data:")
                print(data)
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
    except Exception as e:
        print(f"Error connecting to server: {e}")
    finally:
        # Close the connection
        client_socket.close()
        print("Connection closed.")

# Call the function to start the client
start_tcp_client()
