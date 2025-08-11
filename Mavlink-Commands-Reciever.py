import zmq
from pymavlink import mavutil
import time
import os

# ZMQ setup
context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5555")
subscriber.setsockopt(zmq.SUBSCRIBE, b"")

print("Mission logger connected to ZMQ tcp://localhost:5555")

# MAVLink parser
mav = mavutil.mavlink.MAVLink(None)

try:
    print("Waiting for MAVLink MISSION_ITEM commands...")
    while True:
        try:
            raw_data = subscriber.recv(zmq.NOBLOCK)
            msgs = mav.parse_buffer(raw_data)
            if not msgs:
                continue

            for msg in msgs:
                if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM :
                    try:
                        drone_id = int(msg.target_system) + 1
                        lat = float(msg.x)
                        lon = float(msg.y)
                        alt = float(msg.z)

                        mission_line = f"{drone_id},{lat:.7f},{lon:.7f},{alt:.1f}"
                        filename = f"mission/mission-drone-{drone_id}.pln"
                        

                        # Append to file on new line
                        with open(filename, "a") as f:
                            f.write(mission_line + "\n")

                        print(f"Appended to {filename}: {mission_line}")
                    except Exception as e:
                        print(f"Invalid MISSION_ITEM message: {e}")

        except zmq.Again:
            time.sleep(0.1)
        except Exception as e:
            print(f"Error processing MAVLink message: {e}")

except KeyboardInterrupt:
    print("\nLogging stopped by user.")
finally:
    subscriber.close()
    context.term()