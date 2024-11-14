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

clients = [] 
# Original IMU values (for optional negation)
original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None

# Lock for synchronizing data access
buffer_lock = threading.Lock()

# Configure UDP socket
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((UDP_IP, UDP_PORT))
#udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def apply_config_to_imu_data(data):
    """Apply optional negation for IMU fields based on configuration."""
    global original_imu_roll, original_imu_pitch, original_imu_heading
    
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
        # Apply offset and optional negations to all available fields
        #apply_offset_and_sign(data_buffer)

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
        
        print(f"Sent data: {json_data}")  # Debugging line to check data being broadcasted

        # Broadcast to all clients
        for client in clients:
            udp_socket.sendto(json_data.encode('utf-8'), client)
            print(f"Broadcasting data to {client}: {json_data}")

def start_udp_server():
    global clients
    while True:
        message, address = udp_socket.recvfrom(1024)
        if message.decode('utf-8') == "connect" and address not in clients:
            clients.append(address)
            print(f"Client {address} connected")

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
                    apply_config_to_imu_data(data_buffer)
                    send_imu_data()
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    # Start reading IMU data in the main thread
    server_thread = threading.Thread(target=start_udp_server, daemon=True)
    server_thread.start()

    # Start reading serial data in the main thread
    read_imu_data()
