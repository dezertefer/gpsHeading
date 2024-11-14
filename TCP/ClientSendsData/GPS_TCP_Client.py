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
except (FileNotFoundError, json.JSONDecodeError):
    print("Error loading config.json. Using default configuration.")
    config = {
        "heading_offset": 0
    }

# Configure data buffer with default values
data_buffer = {
    "Timestamp": None,
    "Latitude": None,
    "Longitude": None,
    "Altitude": None,
    "Heading": None,
    "Antenna_Distance": None,
    "RTK_Fix_Quality": None
}

# Server configuration
server_ip = '127.0.0.1'  # Replace with the remote server's IP
server_port = 13370      # Replace with the remote server's port

client_socket = None
original_heading = None
socket_lock = threading.Lock()

# Lock for synchronizing access to data_buffer
buffer_lock = threading.Lock()

def init_socket():
    """Initialize and connect the client socket to the server."""
    global client_socket
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port))
        print("Connection established with server.")
    except Exception as e:
        print(f"Error connecting to server: {e}")
        client_socket = None

def apply_offset_and_sign(data):
    """Apply offset to GPS heading and optional negation to Roll, Pitch, and Yaw."""
    global original_heading
    # Apply offset to heading based on original_heading
    if original_heading is not None:
        adjusted_heading = (original_heading + config.get("heading_offset", 0)) % 360.0
        data["Heading"] = adjusted_heading
    else:
        data["Heading"] = None

    # Apply optional negation for IMU fields

    # Round and format each relevant field to seven decimal places as a string
    fields_to_round_seven = ["Latitude", "Longitude"]

    for field in fields_to_round_seven:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.7f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")
    
    fields_to_round_one = ["Altitude", "Heading", "Antenna_Distance", "Heading"]
                   
    for field in fields_to_round_one:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")



def send_data_to_server(data_buffer):
    """Send data to the server, keeping the connection open as long as possible."""
    global client_socket
    apply_offset_and_sign(data_buffer)
    # Prepare JSON data, excluding fields that are None
    json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)

    with socket_lock:
        if client_socket is None:
            init_socket()  # Attempt to connect if not already connected

        if client_socket:
            try:
                client_socket.sendall(json_data.encode('utf-8'))
                print(f"Sent data to server: {json_data}")
            except (BrokenPipeError, ConnectionResetError) as e:
                print(f"Connection lost. Attempting to reconnect: {e}")
                client_socket.close()
                client_socket = None  # Reset the socket to trigger reconnection next time
        else:
            print("No connection to server. Data not sent.")

def read_serial_data(serial_port_gps='/dev/ttyS0', baudrate_gps=115200):
    """Read data from GPS serial port and update the data buffer."""
    try:
        with serial.Serial(serial_port_gps, baudrate_gps, timeout=1) as ser_gps:
            print(f"Listening on GPS serial port {serial_port_gps} at {baudrate_gps} baud...")
            buffer_gps = b''

            while True:
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

                # Attempt to send data to the server periodically or after updates
                send_data_to_server(data_buffer)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

def parse_ubx_message(ubx_message):
    """Parse relevant UBX message details and store in buffer."""
    msg_class = ubx_message[2]
    msg_id = ubx_message[3]
    payload = ubx_message[6:-2]  # Exclude header and checksum

    if msg_class == 0x01 and msg_id == 0x07:  # NavPVT (Position, Velocity, and Time solution)
        parse_ubx_navpvt(payload)
    elif msg_class == 0x01 and msg_id == 0x3C:  # NavRELPOSNED (Relative Positioning Information)
        parse_ubx_navrelposned(payload)

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

if __name__ == "__main__":
    # Start reading GPS serial data in the main thread
    read_serial_data()