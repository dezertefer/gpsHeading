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

# List to store client connections
clients = []
# original_heading is an internal variable, not stored in data_buffer
original_heading = None
original_imu_pitch = None
original_imu_roll = None
original_imu_heading = None
imu_heading_offset = 0.0

# Lock for synchronizing access to data_buffer and original_heading
buffer_lock = threading.Lock()

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

    if original_imu_heading is not None:
        adjusted_imu_heading = (original_imu_heading + imu_heading_offset) % 360.0
        data["IMU_Heading"] = f"{adjusted_imu_heading:.1f}"
    else:
        data["IMU_Heading"] = None

    # Round and format each relevant field to seven decimal places as a string
    fields_to_round_seven = ["Latitude", "Longitude", "Altitude", "Heading", "Antenna_Distance", 
                       "IMU_Heading", "IMU_Pitch", "IMU_Roll", "IMU_Temperature"]

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



def broadcast_data():
    """Broadcast combined IMU and GPS data to all connected clients."""
    with buffer_lock:
        # Apply offset and optional negations to all available fields
        apply_offset_and_sign(data_buffer)

        # Prepare JSON data, excluding fields that are None
        json_data = json.dumps({k: v for k, v in data_buffer.items() if v is not None}, indent=4)
        
        print(f"Broadcasting data: {json_data}")  # Debugging line to check data being broadcasted

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

def parse_message(message):
    """Parse the IMU message and convert it to a dictionary."""
    data = {
        'Heading': None,
        'Pitch': None,
        'Roll': None,
        'Temperature': None
    }

    global original_imu_pitch
    global original_imu_roll
    global original_imu_heading
    
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

def adjust_imu_heading_offset():
    """Periodically adjust IMU heading offset to match GPS heading."""
    global imu_heading_offset, original_imu_heading, original_heading

    while True:
        time.sleep(1)  # Adjust every 30 seconds
        with buffer_lock:
            if original_imu_heading is not None and original_heading is not None:
                # Normalize headings (if necessary)
                gps_heading = original_heading % 360.0
                imu_heading = original_imu_heading % 360.0

                #print(f"Original GPS Heading: {gps_heading}")
                #print(f"Original IMU Heading: {imu_heading}")

                # Compute the difference and normalize to [-180, 180]
                difference = (gps_heading - imu_heading) % 360.0
                if difference > 180.0:
                    difference -= 360.0

               # print(f"Normalized difference: {difference}")

                # Avoid floating-point inaccuracies
                difference = round(difference, 7)

                # Update offset if the difference exceeds a threshold
                if abs(difference) > 0.1:
                    imu_heading_offset = difference
                    print(f"Updated IMU heading offset: {imu_heading_offset:.1f} degrees")
                else:
                    print(f"No significant adjustment needed. Difference: {difference:.7f}")



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
                    #print("Received IMU message:", raw_message_imu)

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

                # Attempt to broadcast data periodically or after updates
                broadcast_data()

    except serial.SerialException as e:
        print(f"Serial error: {e}")

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

    # Broadcast combined data to clients
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
            rtk_fix_quality = "None"
        elif carr_soln_status == FLAGS_CARR_SOLN_FLOAT:
            rtk_fix_quality = "Float"
        elif carr_soln_status == FLAGS_CARR_SOLN_FIXED:
            rtk_fix_quality = "Fixed"
        else:
            rtk_fix_quality = "Unknown"

        data_buffer["RTK_Fix_Quality"] = "N/A"

if __name__ == "__main__":
    # Start the TCP server in a separate thread
    server_thread = threading.Thread(target=start_tcp_server, daemon=True)
    server_thread.start()

    offset_thread = threading.Thread(target=adjust_imu_heading_offset, daemon=True)
    offset_thread.start()
    
    # Start reading serial data in the main thread
    read_serial_data()
