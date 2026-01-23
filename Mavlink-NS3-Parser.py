import zmq
from pymavlink import mavutil
import threading
import time
import os
import socket
import json

# ZMQ setup
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5555")
subscriber.setsockopt(zmq.SUBSCRIBE, b"")

# Load configuration
with open("drones_config.json") as f:
    config_file = json.load(f)

drones_cfg = config_file["Drones_config"]

# Setup UDP sockets for QGroundControl instances
qgc_sockets = {}
qgc_addresses = {}

for d in drones_cfg:
    sid = d["id"]
    port = d["qgroundcontrol_port"]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    qgc_sockets[sid] = s
    qgc_addresses[sid] = ('127.0.0.1', port)

print("Mavlink Parser Connected to ZMQ tcp://localhost:5555")
print(f"Forwarding messages to QGroundControl instances:")
for d in drones_cfg:
    print(f"  Drone {d['id']} -> QGC port {d['qgroundcontrol_port']}")

# MAVLink parser
mav = mavutil.mavlink.MAVLink(None)

# Drone connection strings (SITL endpoints)
CONNECTION_STRINGS = [d["mavlink_parser_connection"] for d in drones_cfg]

# Dictionary to store live connections
drone_connections = {}

# Lock for thread-safe access to connections
connection_lock = threading.Lock()

# Thread function to handle a single drone connection
def drone_thread(drone_id, conn_str):
    try:
        print(f"Connecting to Drone {drone_id} at {conn_str}...")
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
for d in drones_cfg:
    t = threading.Thread(target=drone_thread, args=(d["id"], d["mavlink_parser_connection"]), daemon=True)
    t.start()

print("\nWaiting for MAVLink Commands...")
while True:
    try:
        raw_data = subscriber.recv(zmq.NOBLOCK)
    except zmq.Again:
        time.sleep(0.01)
        continue

    try:
        # Parse the message for logging and other processing
        msgs = mav.parse_buffer(raw_data)
        
        # Forward the raw data to appropriate QGroundControl based on source system ID
        if msgs and len(msgs) > 0:
            # Get source system from the first message
            source_system = msgs[0].get_srcSystem()
            
            # Forward based on source system ID
            if source_system in qgc_addresses:
                qgc_sockets[source_system].sendto(raw_data, qgc_addresses[source_system])
            else:
                # If unknown source system, forward to all QGC instances
                for sid in qgc_sockets:
                    qgc_sockets[sid].sendto(raw_data, qgc_addresses[sid])
                
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
            # mavlink.MAVLINK_MSG_ID_ARM_DISARM for Force disarm 
            # mavlink.MAV_CMD_DO_CHANGE_SPEED for changing drone speed
            # mavutil.mavlink.MAV_CMD_DO_SET_HOME for setting home position 
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
                drone_id = int(msg.target_system)
                if drone_id in drone_connections:
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
                else:
                    print(f"Drone {drone_id} not in active connections")

            # Send the Set mode commands to drone
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                drone_id = int(msg.target_system)
                if drone_id in drone_connections:
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
                else:
                    print(f"Drone {drone_id} not in active connections")

            # This is for the attack to inject a MISSION_ITEM into the drone's mission plan
            if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM and msg.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                mavlink_system_id = int(msg.target_system)  # This is 1, 2, 3 from ns-3
                drone_id = mavlink_system_id
                #drone_id = int(msg.target_system)                                                                                                                                                                                  
                lat = float(msg.x)
                lon = float(msg.y)
                alt = float(msg.z)                                                                                                                                                                  

                mission_line = f"{drone_id},{lat:.7f},{lon:.7f},{alt:.1f}"
                filename = f"mission/mission-drone-{drone_id}.pln"

                # Create directory if it doesn't exist
                os.makedirs(os.path.dirname(filename), exist_ok=True)
                
                # Create file if it doesn't exist
                if not os.path.exists(filename):
                    try:
                        with open(filename, "w") as f:
                            f.write("")  # Create empty file
                        print(f"Created new mission file: {filename}")
                    except Exception as e:
                        print(f"Failed to create mission file for Drone {drone_id}: {e}")

                try:
                    with open(filename, "a") as f:
                        f.write(mission_line + "\n")
                    print(f"Appended Mission Item To {filename}: {mission_line}")
                except Exception as e:
                    print(f"Failed to write mission item for Drone {drone_id}: {e}")
                    
        except Exception as e:
            print(f"Error handling MAVLink message: {e}")
