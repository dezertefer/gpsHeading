import serial
import socket
import json
import threading

# Constants for carrier solution
FLAGS_CARR_SOLN_NONE = 0     # No carrier phase range solution
FLAGS_CARR_SOLN_FLOAT = 8    # Float solution
FLAGS_CARR_SOLN_FIXED = 16   # Fixed solution
FLAGS_CARR_SOLN_MASK = 0b00011000  # Mask for bits 3-4

UBX_HEADER = b'\xb5\x62'  # UBX message header for u-blox GPS

# Load configuration from config.json
try:
    with open("config.json", "r") as config_file:
        config = json.load(config_file)
except FileNotFoundError:
    print("config.json not found. Using default configuration.")
    config = {
        "heading_offset": 0
    }

# Configure data buffer with default values for GPS only
data_buffer = {
    "Timestamp": None,
    "Latitude": None,
    "Longitude": None,
    "Altitude": None,
    "Heading": None,
    "Antenna_Distance": None,
    "RTK_Fix_Quality": None
}

# List to store client connections
clients = []
original_heading = None

# Lock for synchronizing access to data_buffer
buffer_lock = threading.Lock()

def apply_offset_to_heading():
    """Apply heading offset to the original heading."""
    global original_heading
    if original_heading is not None:
        adjusted_heading = (original_heading + config.get("heading_offset", 0)) % 360.0
        
    # Round and format each relevant field to seven decimal places as a string
    fields_to_round_seven = ["Latitude", "Longitude"]

    for field in fields_to_round_seven:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.7f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")
    
    fields_to_round_one = ["Altitude", "Heading", "Antenna_Distance"]
                   
    for field in fields_to_round_one:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")
        

def broadcast_data():
    """Broadcast GPS data to all connected clients."""
    with buffer_lock:
        # Apply heading offset
        apply_offset_to_heading()

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
        
        print(f"Broadcasting data: {json_data}")

        # Broadcast to all clients
        for client in clients[:]:  # Use a copy of the list to avoid modification during iteration
            try:
                client.sendall(json_data.encode('utf-8'))
                print(f"Sent data to client: {client.getpeername()}")
            except Exception as e:
                print(f"Error sending data to client {client.getpeername()}: {e}")
                clients.remove(client)
                client.close()

def handle_client(client_socket, client_address):
    """Handle client connection."""
    try:
        print(f"New client connected: {client_address}")
        clients.append(client_socket)
        while True:
            # Keep the connection open
            threading.Event().wait(1)
    except Exception as e:
        print(f"Client connection error ({client_address}): {e}")
    finally:
        print(f"Client disconnected: {client_address}")
        with buffer_lock:
            if client_socket in clients:
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
            client_socket, client_address = server_socket.accept()
            # Start a new thread to handle the client
            client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address), daemon=True)
            client_thread.start()
        except Exception as e:
            print(f"Error accepting new client: {e}")

def parse_ubx_message(ubx_message):
    """Parse relevant UBX message details and store in buffer."""
    global original_heading
    msg_class = ubx_message[2]
    msg_id = ubx_message[3]
    payload = ubx_message[6:-2]  # Exclude header and checksum

    if msg_class == 0x01 and msg_id == 0x07:  # NavPVT (Position, Velocity, and Time solution)
        parse_ubx_navpvt(payload)
    elif msg_class == 0x01 and msg_id == 0x3C:  # NavRELPOSNED (Relative Positioning Information)
        parse_ubx_navrelposned(payload)

    # Broadcast updated GPS data to clients
    broadcast_data()

def parse_ubx_navpvt(payload):
    """Parse and store information from a UBX NavPVT message."""
    with buffer_lock:
        data_buffer["Timestamp"] = int.from_bytes(payload[0:4], byteorder='little')
        data_buffer["Latitude"] = int.from_bytes(payload[28:32], byteorder='little', signed=True) * 1e-7
        data_buffer["Longitude"] = int.from_bytes(payload[24:28], byteorder='little', signed=True) * 1e-7
        data_buffer["Altitude"] = int.from_bytes(payload[32:36], byteorder='little', signed=True) * 0.001  # in meters

def parse_ubx_navrelposned(payload):
    """Parse and store information from a UBX NavRELPOSNED message."""
    global original_heading
    with buffer_lock:
        data_buffer['Antenna_Distance'] = int.from_bytes(payload[20:24], 'little') * 0.01
        original_heading = int.from_bytes(payload[24:28], byteorder='little', signed=True) * 1e-5  # in degrees

        flags = int.from_bytes(payload[36:40], byteorder='little')
        carr_soln_status = flags & FLAGS_CARR_SOLN_MASK  # Extract carrier solution bits

        # Determine RTK fix quality
        if carr_soln_status == FLAGS_CARR_SOLN_NONE:
            data_buffer["RTK_Fix_Quality"] = "None"
        elif carr_soln_status == FLAGS_CARR_SOLN_FLOAT:
            data_buffer["RTK_Fix_Quality"] = "Float"
        elif carr_soln_status == FLAGS_CARR_SOLN_FIXED:
            data_buffer["RTK_Fix_Quality"] = "Fixed"
        else:
            data_buffer["RTK_Fix_Quality"] = "Unknown"

def read_serial_data(serial_port_gps='/dev/ttyS0', baudrate_gps=115200):
    """Read data from the GPS serial port and update the data buffer."""
    try:
        # Open GPS serial port
        with serial.Serial(serial_port_gps, baudrate_gps, timeout=1) as ser_gps:
            print(f"Listening on GPS serial port {serial_port_gps} at {baudrate_gps} baud...")

            buffer_gps = b''

            while True:
                # Read GPS data (UBX protocol)
                raw_data_gps = ser_gps.read(ser_gps.in_waiting or 1)
                if raw_data_gps:
                    buffer_gps += raw_data_gps
                    while len(buffer_gps) >= 6:  # Minimum UBX message size
                        if buffer_gps.startswith(UBX_HEADER):
                            msg_class = buffer_gps[2]
                            msg_id = buffer_gps[3]
                            length = int.from_bytes(buffer_gps[4:6], byteorder='little')
                            total_length = 6 + length + 2  # Header + payload + checksum

                            if len(buffer_gps) >= total_length:
                                ubx_message = buffer_gps[:total_length]
                                buffer_gps = buffer_gps[total_length:]
                                parse_ubx_message(ubx_message)
                            else:
                                break
                        else:
                            buffer_gps = buffer_gps[1:]  # Shift buffer if header not found
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    # Start the TCP server in a separate thread
    server_thread = threading.Thread(target=start_tcp_server, daemon=True)
    server_thread.start()

    # Start reading serial data in the main thread
    read_serial_data()
