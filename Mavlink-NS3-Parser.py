import zmq
from pymavlink import mavutil
import threading
import time
import os
import socket  # Added missing import

# ZMQ setup
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5555")
subscriber.setsockopt(zmq.SUBSCRIBE, b"")

# Setup UDP sockets for QGroundControl instances
qgc_socket_1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
qgc_socket_2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
qgc_socket_3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
qgc_address_1 = ('127.0.0.1', 14550)  # QGroundControl port for drone 1
qgc_address_2 = ('127.0.0.1', 14560)  # QGroundControl port for drone 2
qgc_address_3 = ('127.0.0.1', 14570)  # QGroundControl port for drone 3

print("Mavlink Parser Connected to ZMQ tcp://localhost:5555")
print(f"Forwarding messages to QGroundControl instances:")
print(f"Drone 1: {qgc_address_1}")
print(f"Drone 2: {qgc_address_2}")
print(f"Drone 3: {qgc_address_3}")

# MAVLink parser
mav = mavutil.mavlink.MAVLink(None)

# Drone connection strings (SITL endpoints)
CONNECTION_STRINGS = [
    'udp:127.0.0.1:14553',   # Drone 1 (System ID = 1)
    'udp:127.0.0.1:14563',   # Drone 2 (System ID = 2)
    'udp:127.0.0.1:14573'    # Drone 3 (System ID = 3)
]

# Dictionary to store live connections
drone_connections = {}

# Lock for thread-safe access to connections
connection_lock = threading.Lock()

# Thread function to handle a single drone connection
def drone_thread(drone_id, conn_str):
    try:
        print(f"[+] Connecting to Drone {drone_id} at {conn_str}...")
        conn = mavutil.mavlink_connection(conn_str)
        conn.wait_heartbeat()
        print(f"Connected to Drone {drone_id}")

        with connection_lock:
            drone_connections[drone_id] = conn

        while True:
            # Optional: monitor heartbeats, or keep connection alive
            time.sleep(1)

    except Exception as e:
        print(f"Drone {drone_id} failed: {e}")
        with connection_lock:
            drone_connections.pop(drone_id, None)

# Start a thread for each drone connection
for i, conn_str in enumerate(CONNECTION_STRINGS, start=1):
    t = threading.Thread(target=drone_thread, args=(i, conn_str), daemon=True)
    t.start()

print("Waiting for MAVLink Commands...")
while True:
    try:
        raw_data = subscriber.recv(zmq.NOBLOCK)
    except zmq.Again:
        time.sleep(0.01)
        continue

    try:
        # Parse the message for logging and other processing
        msgs = mav.parse_buffer(raw_data)
        
        # Forward the raw data to appropriate QGroundControl based on target system
        if msgs and len(msgs) > 0:
            # Get target system from the first message
            target_system = msgs[0].target_system if hasattr(msgs[0], 'target_system') else None
            
            if target_system == 1:
                qgc_socket_1.sendto(raw_data, qgc_address_1)
            elif target_system == 2:
                qgc_socket_2.sendto(raw_data, qgc_address_2)
            elif target_system == 3:
                qgc_socket_3.sendto(raw_data, qgc_address_3)
            else:
                # If no target system, forward to all QGC instances
                qgc_socket_1.sendto(raw_data, qgc_address_1)
                qgc_socket_2.sendto(raw_data, qgc_address_2)
                qgc_socket_3.sendto(raw_data, qgc_address_3)
                
    except Exception as e:
        print(f"[X] Failed to parse MAVLink buffer: {e}")
        continue

    if not msgs:
        continue

    for msg in msgs:
        print(f"Received: {msg}")
        
        try:
            # Send the commands to drone 
            # Command ID and will be :
                # mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION for flight termination  
                # mavlink.MAVLINK_MSG_ID_ARM_DISARM  for Force disarm 
                # mavlink.MAV_CMD_DO_CHANGE_SPEED for changing drone speed
                # mavutil.mavlink.MAV_CMD_DO_SET_HOME for setting home position 
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
                drone_id = int(msg.target_system) 
                with connection_lock:
                    conn = drone_connections.get(drone_id)
                if conn:
                    try:
                        conn.mav.command_long_send(
                            msg.target_system,
                            msg.target_component,
                            msg.command,
                            msg.confirmation,
                            msg.param1,
                            msg.param2,
                            msg.param3,
                            msg.param4,
                            msg.param5,
                            msg.param6,
                            msg.param7
                        )
                        print(f"Sent command to Drone {drone_id}")
                    except Exception as e:
                        print(f"Failed to send command to Drone {drone_id}: {e}")
                else:
                    print(f"Connection for Drone {drone_id} not found.")

            # Send the Set mode commands to drone
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                drone_id = int(msg.target_system)
                with connection_lock:
                    conn = drone_connections.get(drone_id)
                if conn:
                    try:
                        conn.mav.set_mode_send(
                            msg.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            msg.custom_mode
                        )
                        print(f"Set mode for Drone {drone_id}")
                    except Exception as e:
                        print(f"Failed to send set mode to Drone {drone_id}: {e}")
                else:
                    print(f"Connection for Drone {drone_id} not found.")

            # This is for the attack to inject a MISSION_ITEM into the drone's mission plan
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM and msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                drone_id = int(msg.target_system)                                                                                                                                                                                  
                lat = float(msg.x)
                lon = float(msg.y)
                alt = float(msg.z)                                                                                                                                                                  

                mission_line = f"{drone_id},{lat:.7f},{lon:.7f},{alt:.1f}"
                filename = f"mission/mission-drone-{drone_id}.pln"

                os.makedirs(os.path.dirname(filename), exist_ok=True)

                try:
                    with open(filename, "a") as f:
                        f.write(mission_line + "\n")
                    print(f"Appended Mission Item To {filename}: {mission_line}")
                except Exception as e:
                    print(f"[X] Failed to write mission item for Drone {drone_id}: {e}")
                    
        except Exception as e:
            print(f"[X] Error handling MAVLink message: {e}")
