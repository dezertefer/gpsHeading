import serial
import socket
import threading

# UDP server configuration
UDP_IP = "127.0.0.1"  # Listen on localhost
UDP_PORT = 9090       # UDP port to listen on

# List to store client addresses
clients = []

# Create a UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((UDP_IP, UDP_PORT))

def broadcast_raw_data(raw_data):
    """Broadcast raw serial data to all connected clients."""
    print(f"Broadcasting raw data: {raw_data}")  # Debugging output
    for client in clients:
        udp_socket.sendto(raw_data, client)
        print(f"Sent data to {client}")

def start_udp_server():
    """Handle client connections via UDP."""
    global clients
    while True:
        message, address = udp_socket.recvfrom(1024)
        if message.decode('utf-8') == "connect" and address not in clients:
            clients.append(address)
            print(f"Client {address} connected")

def read_serial_data(serial_port_imu='/dev/ttyAMA1', serial_port_gps='/dev/ttyS0', baudrate_imu=4800, baudrate_gps=115200):
    """Read raw data from IMU and GPS serial ports and broadcast it."""
    try:
        # Open both serial ports
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu, \
             serial.Serial(serial_port_gps, baudrate_gps, timeout=1) as ser_gps:

            print(f"Listening on IMU serial port {serial_port_imu} at {baudrate_imu} baud...")
            print(f"Listening on GPS serial port {serial_port_gps} at {baudrate_gps} baud...")

            while True:
                # Read raw IMU data
                if ser_imu.in_waiting > 0:
                    raw_data_imu = ser_imu.readline()
                    print(f"Received raw IMU data: {raw_data_imu}")
                    broadcast_raw_data(raw_data_imu)

                # Read raw GPS data
                if ser_gps.in_waiting > 0:
                    raw_data_gps = ser_gps.read(ser_gps.in_waiting)
                    print(f"Received raw GPS data: {raw_data_gps}")
                    broadcast_raw_data(raw_data_gps)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    # Start the UDP server in a separate thread
    server_thread = threading.Thread(target=start_udp_server, daemon=True)
    server_thread.start()

    # Start reading raw serial data in the main thread
    read_serial_data()