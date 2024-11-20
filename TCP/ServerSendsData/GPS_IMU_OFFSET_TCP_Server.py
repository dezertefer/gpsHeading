import serial
import socket
import json
import threading
import time

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
    config = {}
except json.JSONDecodeError:
    print("Error decoding config.json. Using default configuration.")
    config = {}

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

# IMU offset value
imu_heading_offset = 0.0

# List to store client connections
clients = []
# Original heading values for calculations
original_heading = None
original_imu_heading = None

# Lock for synchronizing access to data_buffer and offsets
buffer_lock = threading.Lock()

def apply_offset_and_sign(data):
    """Apply offset to GPS heading and optional negation to IMU fields."""
    global imu_heading_offset, original_imu_heading

    # Apply offset to IMU heading
    if original_imu_heading is not None:
        adjusted_heading = (original_imu_heading + imu_heading_offset) % 360.0
        data["IMU_Heading"] = f"{adjusted_heading:.1f}"
    else:
        data["IMU_Heading"] = None

    # Round other relevant fields to one decimal place
    fields_to_round = ["Latitude", "Longitude", "Altitude", "Heading", 
                       "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]

    for field in fields_to_round:
        if data[field] is not None:
            try:
                data[field] = f"{float(data[field]):.1f}"
            except ValueError:
                print(f"Warning: Field {field} could not be converted to float for rounding.")

def broadcast_data():
    """Broadcast combined IMU and GPS data to all connected clients."""
    with buffer_lock:
        apply_offset_and_sign(data_buffer)

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)

        print(f"Broadcasting data: {json_data}")

        for client in clients[:]:  # Use a copy to avoid issues while iterating
            try:
                client.sendall(json_data.encode('utf-8'))
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
        client_socket, client_address = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket, client_address), daemon=True)
        client_thread.start()

def adjust_imu_heading_offset():
    """Periodically adjust IMU heading offset to match GPS heading."""
    global imu_heading_offset, original_imu_heading, original_heading

    while True:
        time.sleep(30)  # Wait for 30 seconds
        with buffer_lock:
            if original_imu_heading is not None and original_heading is not None:
                difference = (original_heading - (original_imu_heading + imu_heading_offset)) % 360.0
                if difference > 180.0:  # Normalize to [-180, 180]
                    difference -= 360.0

                # Update offset if difference exceeds 0.1 degrees
                if abs(difference) > 0.1:
                    imu_heading_offset += difference
                    imu_heading_offset %= 360.0  # Keep within 0-360
                    print(f"Updated IMU heading offset: {imu_heading_offset:.1f} degrees")

def read_serial_data(serial_port_imu='/dev/ttyAMA1', serial_port_gps='/dev/ttyS0', baudrate_imu=4800, baudrate_gps=115200):
    """Read data from both IMU and GPS serial ports and update the data buffer."""
    try:
        with serial.Serial(serial_port_imu, baudrate_imu, timeout=1) as ser_imu, \
             serial.Serial(serial_port_gps, baudrate_gps, timeout=1) as ser_gps:

            buffer_gps = b''

            while True:
                if ser_imu.in_waiting > 0:
                    raw_message_imu = ser_imu.readline().decode('utf-8', errors='ignore').strip()
                    print("Received IMU message:", raw_message_imu)

                    imu_data = parse_message(raw_message_imu)
                    if imu_data:
                        with buffer_lock:
                            data_buffer["IMU_Pitch"] = imu_data['Pitch']
                            data_buffer["IMU_Roll"] = imu_data['Roll']
                            data_buffer["IMU_Temperature"] = imu_data['Temperature']

                raw_data_gps = ser_gps.read(ser_gps.in_waiting or 1)
                if raw_data_gps:
                    buffer_gps += raw_data_gps
                    while len(buffer_gps) >= 6:
                        if buffer_gps.startswith(UBX_HEADER):
                            length = int.from_bytes(buffer_gps[4:6], byteorder='little')
                            total_length = 6 + length + 2
                            if len(buffer_gps) >= total_length:
                                ubx_message = buffer_gps[:total_length]
                                buffer_gps = buffer_gps[total_length:]
                                parse_ubx_message(ubx_message)
                            else:
                                break
                        else:
                            buffer_gps = buffer_gps[1:]

                broadcast_data()
    except serial.SerialException as e:
        print(f"Serial error: {e}")

def parse_ubx_message(ubx_message):
    global original_heading
    msg_class = ubx_message[2]
    msg_id = ubx_message[3]
    payload = ubx_message[6:-2]

    if msg_class == 0x01 and msg_id == 0x07:  # NavPVT
        parse_ubx_navpvt(payload)
    elif msg_class == 0x01 and msg_id == 0x3C:  # NavRELPOSNED
        parse_ubx_navrelposned(payload)

def parse_ubx_navpvt(payload):
    with buffer_lock:
        data_buffer["Timestamp"] = int.from_bytes(payload[0:4], byteorder='little')
        data_buffer["Latitude"] = int.from_bytes(payload[28:32], byteorder='little', signed=True) * 1e-7
        data_buffer["Longitude"] = int.from_bytes(payload[24:28], byteorder='little', signed=True) * 1e-7
        data_buffer["Altitude"] = int.from_bytes(payload[32:36], byteorder='little', signed=True) * 0.001

def parse_ubx_navrelposned(payload):
    global original_heading
    with buffer_lock:
        original_heading = int.from_bytes(payload[24:28], byteorder='little', signed=True) * 1e-5
        data_buffer['RTK_Fix_Quality'] = "Fixed"

if __name__ == "__main__":
    server_thread = threading.Thread(target=start_tcp_server, daemon=True)
    server_thread.start()

    offset_thread = threading.Thread(target=adjust_imu_heading_offset, daemon=True)
    offset_thread.start()

    read_serial_data()
