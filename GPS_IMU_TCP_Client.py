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
        "heading_offset": 0,
        "negate_roll": False,
        "negate_pitch": False,
        "negate_yaw": False
    }

# Configure data buffer with default values
data_buffer = {
    "Timestamp": None,
    "Latitude": None,
    "Longitude": None,
    "Altitude": None,
    "Heading": None,
    "Antenna_Distance": None,
    "RTK_Fix_Quality": None,
    "IMU_Heading": None,
    "IMU_Pitch": None,
    "IMU_Roll": None,
    "IMU_Temperature": None
}

# Original values for offsets
original_heading = None
original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None

# Lock for synchronizing access to data_buffer and original_heading
buffer_lock = threading.Lock()

# Server configuration
server_ip = '127.0.0.1'  # Replace with the remote server's IP
server_port = 13370           # Replace with the remote server's port

def apply_offset_and_sign(data):
    """Apply offset to GPS heading and optional negation to Roll, Pitch, and Yaw."""
    global original_heading, original_imu_pitch, original_imu_roll, original_imu_heading

    # Apply offset to heading based on original_heading
    if original_heading is not None:
        adjusted_heading = (original_heading + config.get("heading_offset", 0)) % 360.0
        data["Heading"] = adjusted_heading
    else:
        data["Heading"] = None

    # Apply optional negation for IMU fields
    data["IMU_Roll"] = -original_imu_roll if config.get("negate_roll", False) and original_imu_roll is not None else original_imu_roll
    data["IMU_Pitch"] = -original_imu_pitch if config.get("negate_pitch", False) and original_imu_pitch is not None else original_imu_pitch
    data["IMU_Heading"] = -original_imu_heading if config.get("negate_yaw", False) and original_imu_heading is not None else original_imu_heading

        # Round and format each relevant field to seven decimal places as a string
    fields_to_round_seven = ["Latitude", "Longitude"]

    for field in fields_to_round_seven:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.7f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")
    
    fields_to_round_one = ["Altitude", "Heading", "Antenna_Distance", 
                       "IMU_Heading", "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]
                   
    for field in fields_to_round_one:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")

def send_data_to_server():
    """Send combined IMU and GPS data to the remote server."""
    with buffer_lock:
        # Apply offset and optional negations to fields
        apply_offset_and_sign(data_buffer)

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)

        # Attempt to connect and send data to the server
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((server_ip, server_port))
                client_socket.sendall(json_data.encode('utf-8'))
                print(f"Sent data to server: {json_data}")
        except Exception as e:
            print(f"Failed to send data to server: {e}")

def read_serial_data(serial_port_imu='/dev/ttyAMA1', serial_port_gps='/dev/ttyS0', baudrate_imu=4800, baudrate_gps=115200):
    """Read data from both IMU and GPS serial ports and update the data buffer."""
    try:
        # Open both serial ports
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu, \
             serial.Serial(serial_port_gps, baudrate_gps, timeout=1) as ser_gps:

            print(f"Listening on IMU serial port {serial_port_imu} at {baudrate_imu} baud...")
            print(f"Listening on GPS serial port {serial_port_gps} at {baudrate_gps} baud...")

            buffer_gps = b''

            while True:
                # Read IMU data
                if ser_imu.in_waiting > 0:
                    raw_message_imu = ser_imu.readline().decode('utf-8', errors='ignore').strip()
                    print("Received IMU message:", raw_message_imu)

                    # Parse the IMU message and store data
                    imu_data = parse_message(raw_message_imu)
                    if imu_data:
                        with buffer_lock:
                            data_buffer["IMU_Heading"] = imu_data['Heading']
                            data_buffer["IMU_Pitch"] = imu_data['Pitch']
                            data_buffer["IMU_Roll"] = imu_data['Roll']
                            data_buffer["IMU_Temperature"] = imu_data['Temperature']

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

                # Attempt to send data to the server periodically or after updates
                send_data_to_server()

    except serial.SerialException as e:
        print(f"Serial error: {e}")

# Parsing functions (parse_message, parse_ubx_message, parse_ubx_navpvt, parse_ubx_navrelposned) remain the same as in your original code.

if __name__ == "__main__":
    # Start reading serial data in the main thread
    read_serial_data()
