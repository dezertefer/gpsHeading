import serial
import socket
import json
import threading

# Load configuration from config.json
try:
    with open("config.json", "r") as config_file:
        config = json.load(config_file)
except (FileNotFoundError, json.JSONDecodeError):
    print("config.json not found or unreadable. Using default configuration.")
    config = {
        "negate_roll": False,
        "negate_pitch": False,
        "negate_yaw": False
    }

# Server information
UDP_IP = "127.0.0.1"  # Target IP address
UDP_PORT = 9090       # Target UDP port

# Data buffer for IMU fields
data_buffer = {
    "IMU_Heading": None,
    "IMU_Pitch": None,
    "IMU_Roll": None,
    "IMU_Temperature": None
}

# Original IMU values (for optional negation)
original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None

# Lock for synchronizing data access
buffer_lock = threading.Lock()

# Configure UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def apply_config_to_imu_data():
    """Apply optional negation for IMU fields based on configuration."""
    with buffer_lock:
        # Adjust each field based on config options
        data_buffer["IMU_Roll"] = -original_imu_roll if config.get("negate_roll", False) and original_imu_roll is not None else original_imu_roll
        data_buffer["IMU_Pitch"] = -original_imu_pitch if config.get("negate_pitch", False) and original_imu_pitch is not None else original_imu_pitch
        data_buffer["IMU_Heading"] = -original_imu_heading if config.get("negate_yaw", False) and original_imu_heading is not None else original_imu_heading
    fields_to_round_one = ["IMU_Heading", "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]
                   
    for field in fields_to_round_one:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")
              
def send_imu_data():
    """Send IMU data over UDP."""
    with buffer_lock:
        # Filter out None values and prepare JSON
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
    udp_socket.sendto(json_data.encode('utf-8'), (UDP_IP, UDP_PORT))
    print(f"Sent IMU data to {UDP_IP}:{UDP_PORT}: {json_data}")

def parse_imu_message(message):
    """Parse IMU message and store data in the buffer."""
    global original_imu_heading, original_imu_pitch, original_imu_roll

    # Clean up and split message
    if message.startswith('$') and '*' in message:
        message = message[1:message.index('*')]

    # Update IMU values based on parsed components
    try:
        if 'C' in message:
            heading_str = message.split('C')[1].split('P')[0]
            original_imu_heading = float(heading_str)
        if 'P' in message:
            pitch_str = message.split('P')[1].split('R')[0]
            original_imu_pitch = float(pitch_str)
        if 'R' in message:
            roll_str = message.split('R')[1].split('T')[0]
            original_imu_roll = float(roll_str)
        if 'T' in message:
            temp_str = message.split('T')[1]
            data_buffer["IMU_Temperature"] = float(temp_str)
    except ValueError as e:
        print(f"Error parsing IMU data: {e}")

def read_imu_data(serial_port='/dev/ttyAMA1', baudrate=4800):
    """Read data from IMU serial port."""
    try:
        with serial.Serial(serial_port, baudrate, timeout=1) as ser:
            print(f"Listening on IMU serial port {serial_port} at {baudrate} baud...")
            while True:
                if ser.in_waiting > 0:
                    raw_message = ser.readline().decode('utf-8', errors='ignore').strip()
                    print("Received IMU message:", raw_message)
                    parse_imu_message(raw_message)
                    apply_config_to_imu_data()
                    send_imu_data()
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    # Start reading IMU data in the main thread
    read_imu_data()
