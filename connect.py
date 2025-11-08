# Main drone mission orchestration script
# Connects to multiple drones, manages mission execution, and handles return/landing logic

from dronekit import connect, VehicleMode
from mission_commands import MoveToWaypoint, Sleep, ReturnHome, Land
from data_logger import DataLogger
from pymavlink import mavutil
import threading
import time
import subprocess
import zmq
import os
import queue
import json 


# ZMQ Setup for position publishing
context = zmq.Context()
position_publisher = context.socket(zmq.PUB)
position_publisher.bind("tcp://*:5556")


with open("drones_config.json") as f:
    config_file = json.load(f)

drones = config_file["Drones_config"]
CONNECTION_STRINGS_Dronekit = [d["dronekit_connection"] for d in drones]
CONNECTION_STRINGS_Mavlink = [d["mavlink_connection"] for d in drones]



# This was used for hardcoded connections before config file approach
# Connection IP and port for each drone running in Ardupilot for DroneKit
#CONNECTION_STRINGS_Dronekit = [
 #   'udp:127.0.0.1:14551',   # Drone 1
  #  'udp:127.0.0.1:14561',   # Drone 2
   # 'udp:127.0.0.1:14571'    # Drone 3
#]

# Connection IP and port for each drone running in Ardupilot to forward Mavlink packets (GPS_RAW_INT)
#CONNECTION_STRINGS_Mavlink = [
 #   'udp:127.0.0.1:14552',   # Drone 1
  #  'udp:127.0.0.1:14562',   # Drone 2
   # 'udp:127.0.0.1:14572'    # Drone 3
#]



# Connect to drones via MAVLink
mav_connections = []
for conn_str in CONNECTION_STRINGS_Mavlink:
    master = mavutil.mavlink_connection(conn_str)
    master.wait_heartbeat()
    mav_connections.append(master)
    print(f"Connected to MAVLink on {conn_str}")



def publish_drone_mavlink(vehicles):
    """
    Continuously capture GPS_RAW_INT and SYS_STATUS and HEARTBEAT messages from each drone
    and send raw MAVLink packets via ZMQ.

    Message format for ZMQ:
      - First byte: message type (0 = GPS_RAW_INT, 1 = SYS_STATUS, 2 = HEARTBEAT)
      - Second byte: drone index (0, 1, 2 for Drone 1..3)
      - Remaining bytes: raw MAVLink message bytes
    """
    while True:
        # Iterate over all MAVLink connections (one per drone)
        for drone_id, master in enumerate(mav_connections):
            if master is None:
                continue
                
            # Try to receive GPS_RAW_INT message (non-blocking)
            gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
            if gps_msg:
                # Get the raw MAVLink-encoded byte buffer for the message
                raw_bytes = gps_msg.get_msgbuf()
                # Prepend message type (0 for GPS) and drone ID
                payload = bytes([0, drone_id]) + raw_bytes
                # Send the payload over ZMQ PUB socket
                position_publisher.send(payload)
               
                
            # Try to receive SYS_STATUS message (SYS_STATUS , non-blocking)
            SYS_STATUS = master.recv_match(type='SYS_STATUS', blocking=False)
            if SYS_STATUS:
                # Get the raw MAVLink-encoded byte buffer for the message
                raw_bytes = SYS_STATUS.get_msgbuf()
                # Prepend message type (1 for battery) and drone ID
                payload = bytes([1, drone_id]) + raw_bytes
                # Send the payload over ZMQ PUB socket
                position_publisher.send(payload)
                
            # NEW: Try to receive HEARTBEAT message (non-blocking)
            heartbeat_msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if heartbeat_msg:
                # Get the raw MAVLink-encoded byte buffer for the message
                raw_bytes = heartbeat_msg.get_msgbuf()
                # Prepend message type (2 for heartbeat) and drone ID
                payload = bytes([2, drone_id]) + raw_bytes
                # Send the payload over ZMQ PUB socket
                position_publisher.send(payload)
                # NEW: Log heartbeat for debugging
                print(f"Drone {drone_id+1}: Heartbeat forwarded")
                
        # Small delay to avoid overwhelming CPU
        time.sleep(0.05)


class DynamicMissionController:
    """Controller for executing queued commands for a single drone."""
    def __init__(self, vehicle, drone_id):
        self.vehicle = vehicle
        self.drone_id = drone_id
        self.command_queue = queue.Queue()
        self.active = True
        self.current_command = None
        self.logger = None
        self.last_command_pop_time = None  # Timestamp of last command pop

        # Drone 1 publishes mission-complete when Land finishes
        self.complete_pub = None
        if self.drone_id == 1:
            self.complete_pub = context.socket(zmq.PUB)
            self.complete_pub.bind("tcp://*:5560")
    
    def notify_command_popped(self):
        """Update timestamp when command is taken from queue."""
        self.last_command_pop_time = time.time()

    def add_command(self, command):
        """Add command to execution queue."""
        self.command_queue.put(command)

    def prepare_drone(self, target_altitude=10):
        """Arm drone and takeoff to initial altitude."""
        print(f"Drone {self.drone_id}: Preparing for mission...")
        self._set_mode("GUIDED")
        while not self.vehicle.is_armable:
            print(f"Drone {self.drone_id}: Waiting to become armable...")
            time.sleep(1)
        self._arm()
        self._takeoff(target_altitude)

    def execute_mission(self):
        """Main mission loop: execute commands from queue after takeoff."""
        self.prepare_drone()
        self.logger = DataLogger(self.vehicle, self.drone_id)
        self.logger.start()
        print(f"Drone {self.drone_id}: Starting mission execution")

        while self.active:
            try:
                # If no current command, get next from queue
                if self.current_command is None:
                    if self.command_queue.empty():
                        time.sleep(1)
                        continue
                    self.current_command = self.command_queue.get()
                    self.notify_command_popped()
                    self.current_command.begin()

                # Update command if supported
                if hasattr(self.current_command, 'update'):
                    self.current_command.update()

                # Check if command is done
                if self.current_command.is_done():
                    cmd = self.current_command
                    print(f"Drone {self.drone_id}: {type(cmd).__name__} completed")
                    # If this was Drone 1’s Land, signal mission complete
                    if self.complete_pub and isinstance(cmd, Land):
                        print("Drone 1: Land done — publishing MISSION_COMPLETE")
                        self.complete_pub.send_string("MISSION_COMPLETE")
                    self.current_command = None
                else:
                    time.sleep(0.1)

            except Exception as e:
                print(f"Drone {self.drone_id}: Mission loop error: {e}")
                self.current_command = None

        # Stop logging when mission ends
        if self.logger:
            self.logger.stop()
        print(f"Drone {self.drone_id}: Mission controller exiting")

    def stop(self):
        """Stop mission execution loop."""
        self.active = False

    def _set_mode(self, mode):
        """Set drone mode."""
        print(f"Drone {self.drone_id}: Setting mode to {mode}")
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            time.sleep(0.5)

    def _arm(self):
        """Arm the drone."""
        print(f"Drone {self.drone_id}: Arming...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(0.5)

    def _takeoff(self, target_altitude):
        """Takeoff to target altitude with timeout safety."""
        print(f"Drone {self.drone_id}: Taking off to {target_altitude}m")
        self.vehicle.simple_takeoff(target_altitude)
        start_time = time.time()
        timeout = 30
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                break
            if time.time() - start_time > timeout:
                break
            time.sleep(0.5)


class DroneCommander:
    """Main orchestrator for all drones and mission logic."""
    def __init__(self):
        self.vehicles = []
        self.controllers = []           # Keep refs to all controllers
        self.threads = []
        self.watchdog_threads = []
        self.watchdog_stop_events = []
        self.ns3_process = None

        # Listen for Drone 1’s mission-complete
        self.complete_sub = context.socket(zmq.SUB)
        self.complete_sub.connect("tcp://localhost:5560")
        self.complete_sub.setsockopt_string(zmq.SUBSCRIBE, "")

    def connect_single_drone(self, index, conn_str, connected_vehicles):
        """Connect to a single drone and wait until armable."""
        try:
            vehicle = connect(conn_str, wait_ready=True, heartbeat_timeout=60)
            print(f"Connected to drone {index+1}, waiting for armable state...")
            while not vehicle.is_armable:
                print(f"Drone {index+1}: Waiting to become armable...")
                time.sleep(1)
            connected_vehicles[index] = vehicle
            print(f"Drone {index+1} connected and armable at {conn_str}")
        except Exception as e:
            print(f"Failed to connect to drone {index+1} ({conn_str}): {e}")

    def connect_drones(self):
        """Connect to all drones in parallel."""
        print("Connecting to drones in parallel...")
        connected = [None]*len(CONNECTION_STRINGS_Dronekit)
        threads = []
        for i, cs in enumerate(CONNECTION_STRINGS_Dronekit):
            t = threading.Thread(target=self.connect_single_drone,
                                 args=(i, cs, connected))
            t.start(); threads.append(t)
        for t in threads: t.join()

        self.vehicles = [v for v in connected if v]
        print(f"Total drones connected: {len(self.vehicles)}")

        # Start position publishing thread
        if self.vehicles:
            t = threading.Thread(target=publish_drone_mavlink,
                                 args=(self.vehicles,), daemon=True)
            t.start()
            print("Started drone position publishing thread")

    def start_ns3(self):
        """Start NS-3 simulation in a new xterm window."""
        print("Starting NS-3 simulation in xterm...")

        # get number of drones from the JSON you already loaded at top
        drones_count = len(drones)   # number to pass to ns-3

        ns3_bin = "/home/boda/Desktop/ns-3-dev/build/scratch/NS3-Multi-Drone/ns3.44-NS3-Multi-Drone-default"  # Change this based on your ns3 executalbe script path

        self.ns3_process = subprocess.Popen([
            "xterm", "-hold", "-e",
            ns3_bin, f"--n={drones_count}"  # Since the environment is scalable, the number of drones should be specified as a parameter in NS-3.
        ])

        print(f"NS-3 PID {self.ns3_process.pid}, started with --n={drones_count}")


    def watchdog(self, drone_id, controller, stop_event):
        """Monitor mission file and queue new waypoints for the drone."""
        filename = f"mission/mission-drone-{drone_id}.pln"
        last_size = 0
        timeout_triggered = False
        # Create file if it doesn't exist
        if not os.path.exists(filename):
            open(filename, 'a').close()
        while not stop_event.is_set():
            try:
                if not os.path.exists(filename):
                    time.sleep(1)
                    continue
                size = os.path.getsize(filename)
                if size > last_size:
                    with open(filename) as f:
                        f.seek(last_size)
                        for line in f:
                            parts = line.strip().split(',')
                            if len(parts) == 4 and parts[0] == str(drone_id):
                                try:
                                    lat, lon, alt = map(float, parts[1:])
                                    cmd = MoveToWaypoint(lat, lon, alt, controller.vehicle)
                                    controller.add_command(cmd)
                                    timeout_triggered = False
                                except ValueError:
                                    pass
                        last_size = size

                # If no command popped for 60s, queue RTL+LAND
                now = time.time()
                last_pop = controller.last_command_pop_time
                if last_pop is not None and not timeout_triggered and (now - last_pop) > 60:
                    controller.add_command(ReturnHome(controller.vehicle))
                    controller.add_command(Land(controller.vehicle))
                    timeout_triggered = True

                time.sleep(0.5)
            except Exception:
                time.sleep(1)

    def _handle_mission_complete(self):
        """When Drone 1 lands, queue RTL+LAND on drones 2 & 3."""
        while True:
            try:
                msg = self.complete_sub.recv_string(zmq.NOBLOCK)
                if msg == "MISSION_COMPLETE":
                    print("Commander: Received MISSION_COMPLETE → ordering drones 2 & 3 to RTL+LAND")
                    for ctrl in self.controllers[1:]:
                        ctrl.add_command(ReturnHome(ctrl.vehicle))
                        ctrl.add_command(Land(ctrl.vehicle))
            except zmq.Again:
                time.sleep(0.5)
            except Exception as e:
                print(f"Mission complete handler error: {e}")
                time.sleep(1)

    def start_missions(self):
        """Start NS-3, then start mission and watchdog threads for all drones."""
        if not self.vehicles:
            print("No drones connected. Abort.")
            return

        print("Starting NS-3 simulation before missions…")
        self.start_ns3()
        time.sleep(2)

        # Start listener for Drone 1’s completion
        threading.Thread(target=self._handle_mission_complete,
                         daemon=True).start()

        print("Starting drone missions…")
        for idx, vehicle in enumerate(self.vehicles):
            drone_id = idx+1
            ctrl = DynamicMissionController(vehicle, drone_id)
            self.controllers.append(ctrl)

            # Start watchdog on mission file FIRST
            ev = threading.Event()
            wd = threading.Thread(
                target=self.watchdog,
                args=(drone_id, ctrl, ev),
                daemon=True
            )
            wd.start()
            self.watchdog_threads.append(wd)
            self.watchdog_stop_events.append(ev)

            # Then start mission execution thread
            t = threading.Thread(target=ctrl.execute_mission, daemon=True)
            t.start()
            self.threads.append(t)

            time.sleep(0.5)

        # Wait for all mission threads to exit
        for t in self.threads:
            t.join()

        print("All mission threads have exited.")

    def cleanup(self):
        """Cleanup all resources and safely close drones and ZMQ."""
        print("Cleaning up…")
        # Stop filesystem watchdogs
        for ev in self.watchdog_stop_events:
            ev.set()

        # Stop all controllers
        for ctrl in self.controllers:
            ctrl.stop()

        # Terminate NS-3
        if self.ns3_process and self.ns3_process.poll() is None:
            print("Terminating NS-3…")
            self.ns3_process.terminate()
            self.ns3_process.wait()

        # Close vehicles
        for v in self.vehicles:
            try:
                if v.armed:
                    v.mode = VehicleMode("LAND")
                    time.sleep(1)
                v.close()
            except Exception as e:
                print(f"Error closing vehicle: {e}")

        # Teardown ZMQ
        try:
            position_publisher.close()
            self.complete_sub.close()
            context.term()
        except Exception as e:
            print(f"Error closing ZMQ: {e}")


if __name__ == "__main__":
    # Entry point: connect drones, start missions, cleanup on exit
    commander = DroneCommander()
    try:
        commander.connect_drones()
        print(">> Waiting 5s before starting mission…")
        time.sleep(5)
        commander.start_missions()
    finally:
        commander.cleanup()
