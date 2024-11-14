import serial
import socket
import json
import threading

# Constants for carrier solution
FLAGS_CARR_SOLN_NONE = 0     # No carrier phase range solution
FLAGS_CARR_SOLN_FLOAT = 8    # Float solution
FLAGS_CARR_SOLN_FIXED = 16   # Fixed solution
FLAGS_CARR_SOLN_MASK = 0b00011000  # Mask for bits 3-4

# Load configuration from config.json
UDP_IP = "127.0.0.1"      # IP address of the server (adjust accordingly)
UDP_PORT = 9090           # UDP port to send data to

try:
    with open("config.json", "r") as config_file:
        config = json.load(config_file)
except FileNotFoundError:
    print("config.json not found. Using default configuration.")
    config = {
        "heading_offset": 0,
        "negate_roll": False,
        "negate_pitch": False,
        "negate_yaw": False
    }
except json.JSONDecodeError:
    print("Error decoding config.json. Using default configuration.")
    config = {
        "heading_offset": 0,
        "negate_roll": False,
        "negate_pitch": False,
        "negate_yaw": False
    }

# Configure data buffer with default values for IMU only
data_buffer = {
    "IMU_Heading": None,
    "IMU_Pitch": None,
    "IMU_Roll": None,
    "IMU_Temperature": None
}

# Lock for synchronizing access to data_buffer and original_heading
buffer_lock = threading.Lock()

original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None

# Create UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def apply_offset_and_sign(data):
    """Apply offset to IMU heading and optional negation to Roll, Pitch, and Yaw."""
    global original_imu_pitch, original_imu_roll, original_imu_heading

    # Apply optional negation for IMU fields
    data["IMU_Roll"] = -original_imu_roll if config.get("negate_roll", False) and original_imu_roll is not None else original_imu_roll
    data["IMU_Pitch"] = -original_imu_pitch if config.get("negate_pitch", False) and original_imu_pitch is not None else original_imu_pitch
    data["IMU_Heading"] = -original_imu_heading if config.get("negate_yaw", False) and original_imu_heading is not None else original_imu_heading

    # Round and format each relevant field to seven decimal places as a string
    fields_to_round_one = ["IMU_Heading", "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]

    for field in fields_to_round_one:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")
    


def broadcast_data():
    """Broadcast IMU data to the server."""
    with buffer_lock:
        # Apply offset and optional negations to all available IMU fields
        apply_offset_and_sign(data_buffer)

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
        
        print(f"Broadcasting data: {json_data}")  # Debugging line to check data being broadcasted

        # Send data to the server
        udp_socket.sendto(json_data.encode('utf-8'), (UDP_IP, UDP_PORT))
        print(f"Sent data to server at {UDP_IP}:{UDP_PORT}")

def parse_message(message):
    """Parse the IMU message and convert it to a dictionary."""
    data = {
        'Heading': None,
        'Pitch': None,
        'Roll': None,
        'Temperature': None
    }

    global original_imu_pitch, original_imu_roll, original_imu_heading

    try:
        # Remove the starting '$' and ending '*checksum' if present
        if message.startswith('$') and '*' in message:
            message = message[1:message.index('*')]

        # Extract and convert each component
        if 'C' in message:
            c_index = message.find('C')
            p_index = message.find('P', c_index)
            if p_index != -1:
                heading_str = message[c_index+1:p_index]
                try:
                    original_imu_heading = float(heading_str)
                except ValueError:
                    print(f"Error parsing heading value: {heading_str}")
        if 'P' in message:
            p_index = message.find('P')
            r_index = message.find('R', p_index)
            if r_index != -1:
                pitch_str = message[p_index+1:r_index]
                try:
                    original_imu_pitch = float(pitch_str)
                except ValueError:
                    print(f"Error parsing pitch value: {pitch_str}")
        if 'R' in message:
            r_index = message.find('R')
            t_index = message.find('T', r_index)
            if t_index != -1:
                roll_str = message[r_index+1:t_index]
                try:
                    original_imu_roll = float(roll_str)
                except ValueError:
                    print(f"Error parsing roll value: {roll_str}")
        if 'T' in message:
            t_index = message.find('T')
            temp_str = message[t_index+1:]
            try:
                data['Temperature'] = float(temp_str)
            except ValueError:
                print(f"Error parsing temperature value: {temp_str}")

        return data  # Return parsed data as a dictionary

    except Exception as e:
        print(f"Failed to parse message: {e}")
        return None


def read_serial_data(serial_port_imu='/dev/ttyAMA1', baudrate_imu=4800):
    """Read data from the IMU serial port and update the data buffer."""
    try:
        # Open the IMU serial port
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu:

            print(f"Listening on IMU serial port {serial_port_imu} at {baudrate_imu} baud...")

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

                # Attempt to broadcast data periodically or after updates
                broadcast_data()

    except serial.SerialException as e:
        print(f"Serial error: {e}")

# Start reading data and broadcasting to the server
read_serial_data()
