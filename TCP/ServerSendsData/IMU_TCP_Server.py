import serial
import socket
import json
import threading

# Load configuration from config.json
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

# Configure data buffer with default values
data_buffer = {
    "IMU_Heading": None,
    "IMU_Pitch": None,
    "IMU_Roll": None,
    "IMU_Temperature": None
}

# List to store client connections
clients = []
original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None

# Lock for synchronizing access to data_buffer
buffer_lock = threading.Lock()

def apply_offset_and_sign(data):
    """Apply optional negation to Roll, Pitch, and Yaw, and round data."""
    global original_imu_pitch, original_imu_roll, original_imu_heading

    # Apply optional negation for IMU fields
    data["IMU_Roll"] = -original_imu_roll if config.get("negate_roll", False) and original_imu_roll is not None else original_imu_roll
    data["IMU_Pitch"] = -original_imu_pitch if config.get("negate_pitch", False) and original_imu_pitch is not None else original_imu_pitch
    data["IMU_Heading"] = -original_imu_heading if config.get("negate_yaw", False) and original_imu_heading is not None else original_imu_heading

    # Round each relevant field to seven decimal places
    fields_to_round = ["IMU_Heading", "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]

    for field in fields_to_round:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.7f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")

def broadcast_data():
    """Broadcast GPS data to all connected clients."""
    with buffer_lock:
        # Apply heading offset
        apply_offset_and_sign(data_buffer)

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
        
        print(f"Broadcasting data: {json_data}")

        # Broadcast to all clients
        for client in clients[:]:  # Use a copy of the list to avoid modification during iteration
            try:
                client.sendall(json_data.encode('utf-8'))
                print(f"Sent data to client.")
            except (OSError, ConnectionResetError) as e:
                # Handle client disconnection without calling getpeername
                print("Error sending data to client. Removing client from list.")
                clients.remove(client)
                client.close()
            except Exception as e:
                # Catch any other exceptions that may arise
                print("Unexpected error with client. Removing client from list.")
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

def parse_message(message):
    """Parse the IMU message and convert it to a dictionary."""
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
                original_imu_heading = float(heading_str)
        if 'P' in message:
            p_index = message.find('P')
            r_index = message.find('R', p_index)
            if r_index != -1:
                pitch_str = message[p_index+1:r_index]
                original_imu_pitch = float(pitch_str)
        if 'R' in message:
            r_index = message.find('R')
            t_index = message.find('T', r_index)
            if t_index != -1:
                roll_str = message[r_index+1:t_index]
                original_imu_roll = float(roll_str)
        if 'T' in message:
            t_index = message.find('T')
            temp_str = message[t_index+1:]
            data_buffer['IMU_Temperature'] = float(temp_str)

        return True  # Successfully parsed

    except ValueError as e:
        print(f"Failed to parse message: {e}")
        return False

def read_imu_data(serial_port_imu='/dev/ttyAMA1', baudrate_imu=4800):
    """Read data from the IMU serial port and update the data buffer."""
    try:
        # Open the serial port for IMU
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu:
            print(f"Listening on IMU serial port {serial_port_imu} at {baudrate_imu} baud...")

            while True:
                if ser_imu.in_waiting > 0:
                    raw_message_imu = ser_imu.readline().decode('utf-8', errors='ignore').strip()
                    print("Received IMU message:", raw_message_imu)

                    # Parse the IMU message and store data
                    if parse_message(raw_message_imu):
                        with buffer_lock:
                            # Update the data buffer
                            data_buffer["IMU_Heading"] = original_imu_heading
                            data_buffer["IMU_Pitch"] = original_imu_pitch
                            data_buffer["IMU_Roll"] = original_imu_roll

                    # Broadcast data periodically or after updates
                    broadcast_data()

    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    # Start the TCP server in a separate thread
    server_thread = threading.Thread(target=start_tcp_server, daemon=True)
    server_thread.start()

    # Start reading IMU data in the main thread
    read_imu_data()
