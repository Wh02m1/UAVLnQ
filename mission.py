import time
from dronekit import Vehicle, VehicleMode
from mission_commands import MoveToWaypoint, Sleep, ReturnHome, Land
from data_logger import DataLogger
import subprocess

class MissionController:
    """Executes mission sequences for a single drone"""
    def __init__(self, vehicle, drone_id):
        self.vehicle = vehicle
        self.drone_id = drone_id
        self.commands = []
        self.current_index = 0
        self.current_command = None

    def add_command(self, command):
        """Add command to mission sequence"""
        self.commands.append(command)

    def prepare_drone(self, target_altitude=10):
        """Arm drone and takeoff to initial altitude"""
        print(f"Drone {self.drone_id}: Preparing for mission...")

        self._set_mode("GUIDED")

        # Wait until vehicle is armable
        while not self.vehicle.is_armable:
            print(f"Drone {self.drone_id}: Waiting to become armable...")
            time.sleep(1)

        self._arm()

        self._takeoff(target_altitude)

    def execute_mission(self):
        # Prepare drone before starting mission
        
        self.prepare_drone()
        
        # Start telemetry logging
        logger = DataLogger(self.vehicle, self.drone_id)
        logger.start()

        while self.current_index < len(self.commands):
            cmd = self.commands[self.current_index]
            if self.current_command is None:
                # Start command
                if hasattr(cmd, 'begin'):
                    cmd.begin()
                self.current_command = cmd

            # Update command if method exists
            if hasattr(self.current_command, 'update'):
                self.current_command.update()

            # Check if command is done
            if self.current_command.is_done():
                print(f"[Drone {self.drone_id}] Command {self.current_index + 1} done.")
                self.current_index += 1
                self.current_command = None
            else:
                # Sleep a bit to avoid busy waiting, adjust as needed
                time.sleep(0.5)
        
        # Stop telemetry logging
        logger.stop()
        print(f"[Drone {self.drone_id}] Mission complete!")

    def _set_mode(self, mode):
        """Set vehicle flight mode"""
        print(f"Drone {self.drone_id}: Setting mode to {mode}")
        self.vehicle.mode = VehicleMode(mode)
        while self.vehicle.mode.name != mode:
            time.sleep(0.5)

    def _arm(self):
        """Arm vehicle motors"""
        print(f"Drone {self.drone_id}: Arming...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(0.5)

    def _takeoff(self, target_altitude):
        """Execute takeoff to target altitude"""
        print(f"Drone {self.drone_id}: Taking off to {target_altitude} meters")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                break
            time.sleep(0.5)

    def _land_and_disarm(self):
        """Land vehicle"""
        print(f"Drone {self.drone_id}: Landing and disarming...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            time.sleep(0.5)
