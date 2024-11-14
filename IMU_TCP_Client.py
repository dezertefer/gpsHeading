import serial
import socket
import json
import threading

# Load configuration from config.json
try:
    with open("config.json", "r") as config_file:
        config = json.load(config_file)
except (FileNotFoundError, json.JSONDecodeError):
    print("Error loading config.json. Using default configuration.")
    config = {
        "negate_roll": False,
        "negate_pitch": False,
        "negate_yaw": False
    }

# Configure data buffer with IMU-only fields
data_buffer = {
    "IMU_Heading": None,
    "IMU_Pitch": None,
    "IMU_Roll": None,
    "IMU_Temperature": None
}

# Original values for offsets
original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None

# Lock for synchronizing access to data_buffer
buffer_lock = threading.Lock()

# Server configuration
server_ip = '127.0.0.1'  # Replace with the remote server's IP
server_port = 13370      # Replace with the remote server's port

def apply_sign_adjustments(data):
    """Apply optional negations to IMU fields based on configuration."""
    global original_imu_pitch, original_imu_roll, original_imu_heading

    # Apply optional negation for IMU fields
    data["IMU_Roll"] = -original_imu_roll if config.get("negate_roll", False) and original_imu_roll is not None else original_imu_roll
    data["IMU_Pitch"] = -original_imu_pitch if config.get("negate_pitch", False) and original_imu_pitch is not None else original_imu_pitch
    data["IMU_Heading"] = -original_imu_heading if config.get("negate_yaw", False) and original_imu_heading is not None else original_imu_heading

    # Format each field to one decimal place as a string
    fields_to_round = ["IMU_Heading", "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]
    for field in fields_to_round:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")

def send_data_to_server():
    """Send combined IMU data to the remote server, keeping the connection open."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((server_ip, server_port))
            print("Connection established with server.")

            while True:
                with buffer_lock:
                    apply_offset_and_sign(data_buffer)

                    json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
                    client_socket.sendall(json_data.encode('utf-8'))
                    print(f"Sent data to server: {json_data}")

                # Send data at intervals (e.g., every second)
                time.sleep(1)

    except Exception as e:
        print(f"Error in connection or sending data: {e}")

def read_serial_data(serial_port_imu='/dev/ttyAMA1', baudrate_imu=4800):
    """Read data from the IMU serial port and update the data buffer."""
    try:
        # Open IMU serial port
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu:
            print(f"Listening on IMU serial port {serial_port_imu} at {baudrate_imu} baud...")

            while True:
                # Read IMU data
                if ser_imu.in_waiting > 0:
                    raw_message_imu = ser_imu.readline().decode('utf-8', errors='ignore').strip()
                    print("Received IMU message:", raw_message_imu)

                    # Parse the IMU message and store data
                    imu_data = parse_imu_message(raw_message_imu)
                    if imu_data:
                        with buffer_lock:
                            data_buffer["IMU_Heading"] = imu_data['Heading']
                            data_buffer["IMU_Pitch"] = imu_data['Pitch']
                            data_buffer["IMU_Roll"] = imu_data['Roll']
                            data_buffer["IMU_Temperature"] = imu_data['Temperature']

                # Send data to the server periodically or after updates
                send_data_to_server()

    except serial.SerialException as e:
        print(f"Serial error: {e}")

def parse_imu_message(message):
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

if __name__ == "__main__":
    # Start reading serial data in the main thread
    read_serial_data()
