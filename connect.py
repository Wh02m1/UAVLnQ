from dronekit import connect, VehicleMode
from mission import MissionController
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
import shlex  # Added for proper parameter splitting

# ZMQ Setup for position publishing
context = zmq.Context()
position_publisher = context.socket(zmq.PUB)
position_publisher.bind("tcp://*:5556")


with open("drones_config.json") as f:
    config_file = json.load(f)

drones = config_file["Drones_config"]
CONNECTION_STRINGS_Dronekit = [d["dronekit_connection"] for d in drones]
CONNECTION_STRINGS_Mavlink = [d["mavlink_connection"] for d in drones]

#ns3_bin = "/home/boda/Desktop/ns-3-dev/build/scratch/NS3-Multi-Drone/ns3.44-NS3-Multi-Drone-default"
#ns3_bin = "/home/boda/Desktop/ns-3-dev/build/scratch/drone-mesh/ns3.44-drone_mesh-default"

# Get NS-3 configuration from JSON
try:
    ns3_config = config_file["NS3_config"]
    ns3_bin = ns3_config["ns3_bin"]          # required in JSON
    ns3_parameters = ns3_config["parameters"]  # required in JSON
except KeyError as e:
    raise RuntimeError(f"Missing NS3_config field in drones_config.json: {e}")

# Connect to drones via MAVLink
mav_connections = []
for conn_str in CONNECTION_STRINGS_Mavlink:
    master = mavutil.mavlink_connection(conn_str)
    master.wait_heartbeat()
    mav_connections.append(master)
    print(f"Connected to MAVLink on {conn_str}")


def publish_drone_mavlink():
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
                
            # Try to receive HEARTBEAT message (non-blocking)
            heartbeat_msg = master.recv_match(type='HEARTBEAT', blocking=False)
            if heartbeat_msg:
                # Get the raw MAVLink-encoded byte buffer for the message
                raw_bytes = heartbeat_msg.get_msgbuf()
                # Prepend message type (2 for heartbeat) and drone ID
                payload = bytes([2, drone_id]) + raw_bytes
                # Send the payload over ZMQ PUB socket
                position_publisher.send(payload)
                
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
                    # If this was Drone 1's Land, signal mission complete
                    if self.complete_pub and isinstance(cmd, Land):
                        print("Drone 1: Land done - publishing MISSION_COMPLETE")
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
    def __init__(self):
        self.vehicles = []   # List of connected drone objects
        self.controllers = []  # Dynamic mission controllers
        self.threads = []    # Mission execution threads
        self.watchdog_threads = []
        self.watchdog_stop_events = []
        self.ns3_process = None

        # Listen for Drone 1's mission-complete
        self.complete_sub = context.socket(zmq.SUB)
        self.complete_sub.connect("tcp://localhost:5560")
        self.complete_sub.setsockopt_string(zmq.SUBSCRIBE, "")

    def connect_single_drone(self, index, conn_str, connected_vehicles):
        try:
            vehicle = connect(conn_str, wait_ready=True, heartbeat_timeout=60)
            print(f"Connected to drone {index + 1}, waiting for armable state...")
            while not vehicle.is_armable:
                print(f"Drone {index + 1}: Waiting to become armable...")
                time.sleep(1)
            connected_vehicles[index] = vehicle
            print(f"Drone {index + 1} connected and armable at {conn_str}")
        except Exception as e:
            print(f"Failed to connect to drone {index + 1} ({conn_str}): {e}")

    def connect_drones(self, batch_size=5):
        """Connect to all drones in batches to avoid resource exhaustion."""
        print(f"Connecting to drones in batches of {batch_size}...")
        connected = [None] * len(CONNECTION_STRINGS_Dronekit)
        
        # Process drones in batches
        for batch_start in range(0, len(CONNECTION_STRINGS_Dronekit), batch_size):
            batch_end = min(batch_start + batch_size, len(CONNECTION_STRINGS_Dronekit))
            print(f"Connecting batch: drones {batch_start+1} to {batch_end}")
            
            threads = []
            for i in range(batch_start, batch_end):
                cs = CONNECTION_STRINGS_Dronekit[i]
                t = threading.Thread(target=self.connect_single_drone,
                                    args=(i, cs, connected))
                t.start()
                threads.append(t)
            
            # Wait for current batch to complete
            for t in threads:
                t.join()
            
            print(f"Batch {batch_start//batch_size + 1} completed")
            time.sleep(1)  # Brief pause between batches
        
        # Transfer connected vehicles to self.vehicles
        self.vehicles = [v for v in connected if v is not None]
        print(f"Successfully connected to {len(self.vehicles)} drones")

    def start_ns3(self):
        """Launch NS-3 simulation inside an xterm window"""
        print("Starting NS-3 simulation in xterm...")

        # Get drone count from the JSON
        drones_count = len(drones)

        # Use NS-3 configuration from JSON
        print(f"NS-3 binary: {ns3_bin}")
        print(f"NS-3 parameters: {ns3_parameters}")
        
        # Split parameters string into individual arguments using shlex
        param_list = shlex.split(ns3_parameters)
        print(f"NS-3 parameter list: {param_list}")
        
        self.ns3_process = subprocess.Popen(
            ["xterm", "-hold", "-e", ns3_bin] + param_list
        )

        print(f"NS-3 PID {self.ns3_process.pid}, started with parameters: {ns3_parameters}")

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
        """When Drone 1 lands, queue RTL+LAND on other drones."""
        while True:
            try:
                msg = self.complete_sub.recv_string(zmq.NOBLOCK)
                if msg == "MISSION_COMPLETE":
                    print("Commander: Received MISSION_COMPLETE -> ordering other drones to RTL+LAND")
                    for ctrl in self.controllers[1:]:
                        ctrl.add_command(ReturnHome(ctrl.vehicle))
                        ctrl.add_command(Land(ctrl.vehicle))
            except zmq.Again:
                time.sleep(0.5)
            except Exception as e:
                print(f"Mission complete handler error: {e}")
                time.sleep(1)

    def start_missions(self):
        """Execute missions on all drones concurrently"""
        if not self.vehicles:
            print("No drones connected. Aborting missions.")
            return

        print("\nStarting NS-3 simulation before missions...")
        self.start_ns3()
        time.sleep(2)  # Give NS-3 a moment to start

        # Start MAVLink publishing thread
        print("Starting MAVLink publisher...")
        mavlink_thread = threading.Thread(
            target=publish_drone_mavlink,
            daemon=True
        )
        mavlink_thread.start()

        # Start listener for Drone 1's completion
        threading.Thread(target=self._handle_mission_complete,
                         daemon=True).start()

        print("\nStarting drone missions...")
        for idx, vehicle in enumerate(self.vehicles):
            drone_id = idx + 1
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

        print("All missions completed or stopped.")

    def cleanup(self):
        """Clean up resources when missions are complete"""
        print("Cleaning up resources...")

        # Stop filesystem watchdogs
        for ev in self.watchdog_stop_events:
            ev.set()

        # Stop all controllers
        for ctrl in self.controllers:
            ctrl.stop()

        # Terminate NS-3 simulation if still running
        if self.ns3_process and self.ns3_process.poll() is None:
            print("Terminating NS-3 simulation...")
            self.ns3_process.terminate()
            self.ns3_process.wait()

        # Close all vehicles
        for vehicle in self.vehicles:
            try:
                if vehicle.armed:
                    vehicle.mode = VehicleMode("LAND")
                    time.sleep(1)
                vehicle.close()
            except Exception as e:
                print(f"Error closing vehicle: {e}")
        
        # Close ZMQ connections
        try:
            position_publisher.close()
            self.complete_sub.close()
            context.term()
            print("ZMQ connections closed")
        except Exception as e:
            print(f"Error closing ZMQ: {e}")


if __name__ == "__main__":
    commander = DroneCommander()
    try:
        commander.connect_drones()
        print(">> Waiting 5 seconds before starting missions...")
        time.sleep(5)
        commander.start_missions()
    finally:
        commander.cleanup()
