import serial
import socket
import threading

# List to store client connections
clients = []

def broadcast_data(data):
    """Broadcast raw serial data to all connected clients."""
    for client in clients[:]:  # Use a copy of the list to avoid modification during iteration
        try:
            client.sendall(data)
        except Exception as e:
            print(f"Error sending data to client: {e}")
            clients.remove(client)
            client.close()

def handle_client(client_socket):
    """Handle client connection."""
    try:
        clients.append(client_socket)
        while True:
            # Keep the connection open
            threading.Event().wait(1)
    except Exception as e:
        print(f"Client connection error: {e}")
    finally:
        clients.remove(client_socket)
        client_socket.close()

def start_tcp_server(host='0.0.0.0', port=13370):
    """Start a TCP server that handles multiple clients."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"TCP Server listening on {host}:{port}...")

    while True:
        try:
            client_socket, _ = server_socket.accept()
            # Start a new thread to handle the client
            client_thread = threading.Thread(target=handle_client, args=(client_socket,), daemon=True)
            client_thread.start()
        except Exception as e:
            print(f"Error accepting new client: {e}")

def read_serial_data(serial_port_imu='/dev/ttyAMA1', serial_port_gps='/dev/ttyS0', baudrate_imu=4800, baudrate_gps=115200):
    """Read raw data from IMU and GPS serial ports and broadcast to clients."""
    try:
        # Open both serial ports
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu, \
             serial.Serial(serial_port_gps, baudrate_gps, timeout=1) as ser_gps:

            print(f"Listening on IMU serial port {serial_port_imu} at {baudrate_imu} baud...")
            print(f"Listening on GPS serial port {serial_port_gps} at {baudrate_gps} baud...")

            while True:
                # Read and broadcast IMU data
                if ser_imu.in_waiting > 0:
                    raw_imu_data = ser_imu.read(ser_imu.in_waiting or 1)
                    broadcast_data(raw_imu_data)

                # Read and broadcast GPS data
                if ser_gps.in_waiting > 0:
                    raw_gps_data = ser_gps.read(ser_gps.in_waiting or 1)
                    broadcast_data(raw_gps_data)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    # Start the TCP server in a separate thread
    server_thread = threading.Thread(target=start_tcp_server, daemon=True)
    server_thread.start()

    # Start reading serial data in the main thread
    read_serial_data()