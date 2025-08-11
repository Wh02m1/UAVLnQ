
import time
import math
import queue
from dronekit import VehicleMode
from pymavlink import mavutil
from dronekit import LocationGlobalRelative


COMMAND_QUEUES = None


class Command:
    """Base command interface"""
    def begin(self):
        raise NotImplementedError

    def is_done(self):
        raise NotImplementedError

class MoveToWaypoint(Command):
    """Fly to local NED position"""
    def __init__(self, east, north, up, vehicle, tolerance=0.5, debug=False):
        self.vehicle = vehicle
        self.east = east      # Target East position (meters)
        self.north = north    # Target North position (meters)
        self.up = up          # Target Up position (meters)
        self.tolerance = tolerance  # Position tolerance
        self.debug = debug
        self._started = False

    def begin(self):
        """Send initial position command"""
        if self.debug:
            print(f"[MoveToWaypoint] Sending target: E={self.east}, N={self.north}, U={self.up}")
        self._send_position()
        self._started = True

    def update(self):
        """Resend position command (helps maintain accuracy)"""
        self._send_position()
        if self.debug:
            print(f"[MoveToWaypoint] Position command resent.")

    def _send_position(self):
        """Send MAVLink position command (NED coordinates)"""
        down = -self.up  # Convert up to down
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # Time boot ms (not used)
            0, 0,    # Target system, component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame
            0b0000111111111000,  # Position control mask
            self.north, self.east, down,  # Position (NED)
            0, 0, 0,  # Velocity (not used)
            0, 0, 0,  # Acceleration (not used)
            0, 0      # Yaw, yaw rate (not used)
        )
        self.vehicle.send_mavlink(msg)

    def is_done(self):
        """Check if reached target position"""
        current = self.vehicle.location.local_frame
        # Calculate 3D distance to target
        dist = math.sqrt(
            (current.north - self.north)**2 +
            (current.east - self.east)**2 +
            (current.down + self.up)**2  # Note: down is negative
        )
        if self.debug:
            print(f"[MoveToWaypoint] Distance: {dist:.2f}m")
        return dist < self.tolerance

    def execute(self):
        """Execute the move to waypoint command"""
        self.begin()
        while not self.is_done():
            self.update()
            time.sleep(0.5)
        if self.debug:
            print(f"[MoveToWaypoint] Arrived at waypoint.")

class Sleep(Command):
    """Wait for specified duration"""
    def __init__(self, sleep_time):
        self.sleep_time = sleep_time
        self.start_time = None

    def begin(self):
        """Record start time"""
        self.start_time = time.time()

    def is_done(self):
        """Check if sleep duration elapsed"""
        return (time.time() - self.start_time) >= self.sleep_time

class ReturnHome(Command):
    """Return to launch location"""
    def __init__(self, vehicle, debug=False, timeout=120):
        self.vehicle = vehicle
        self.debug = debug
        self.timeout = timeout  # Max operation time
        self.start_time = None

    def begin(self):
        """Activate RTL mode"""
        if self.debug:
            print("Returning to launch (RTL)")
        self.vehicle.mode = VehicleMode("RTL")
        self.start_time = time.time()

    def is_done(self):
        """Check if landed or timeout"""
        elapsed = time.time() - self.start_time
        altitude = self.vehicle.location.global_relative_frame.alt
        if self.debug:
            print(f"[ReturnHome] Alt: {altitude:.2f}m, Time: {elapsed:.1f}s")
        return altitude < 1.0 or elapsed > self.timeout

class Land(Command):
    """Land at current position"""
    def __init__(self, vehicle, debug=False, timeout=120):
        self.vehicle = vehicle
        self.debug = debug
        self.timeout = timeout
        self.start_time = None

    def begin(self):
        """Activate LAND mode"""
        if self.debug:
            print("Landing")
        self.vehicle.mode = VehicleMode("LAND")
        self.start_time = time.time()

    def is_done(self):
        """Check if landed or timeout"""
        elapsed = time.time() - self.start_time
        altitude = self.vehicle.location.global_relative_frame.alt
        if self.debug:
            print(f"[Land] Alt: {altitude:.2f}m, Time: {elapsed:.1f}s")
        # Check if altitude is very low or timeout
        return altitude < 0.5 or elapsed > self.timeout
    
class CheckCommandQueue(Command):
    """Check for new commands in the queue throughout the mission"""
    def __init__(self, drone_id, check_interval=0.5):
        self.drone_id = drone_id
        self.check_interval = check_interval
        self.queue = COMMAND_QUEUES[drone_id]
        self.shutdown = False  # Added shutdown flag

    def begin(self):
        pass

    def is_done(self):
        """Now returns shutdown status"""
        return self.shutdown

    def update(self):
        if self.shutdown:  # Early exit if shutting down
            return
            
        try:
            while not self.queue.empty():
                cmd_type, data = self.queue.get_nowait()
                if cmd_type == 'move':
                    lat, lon, alt = data
                    print(f"Drone {self.drone_id+1}: Executing new move command")
                    
                    move_cmd = MoveToGlobalWaypoint(lat, lon, alt, self.vehicle)
                    move_cmd.begin()
                    
                    # Add shutdown check in inner loop
                    while not move_cmd.is_done() and not self.shutdown:
                        move_cmd.update()
                        time.sleep(0.1)
                        
                    if self.shutdown:
                        return
                        
                    print(f"Drone {self.drone_id+1}: Move command completed")
                
                self.queue.task_done()
        except queue.Empty:
            pass
        
        time.sleep(self.check_interval)

class MoveToGlobalWaypoint(Command):
    """Fly to global position (latitude/longitude)"""
    def __init__(self, lat, lon, alt, vehicle, tolerance=1.0):
        self.vehicle = vehicle
        self.target = LocationGlobalRelative(lat, lon, alt)
        self.tolerance = tolerance
        self._sent = False

    def begin(self):
        """Send initial position command"""
        self.vehicle.simple_goto(self.target)
        self._sent = True

    def update(self):
        """Resend position command periodically (optional)"""
        # Resend every 5 seconds to ensure command sticks
        if time.time() % 5 < 0.1:
            self.vehicle.simple_goto(self.target)

    def is_done(self):
        """Check if reached target position"""
        current = self.vehicle.location.global_relative_frame
        dist = current.get_distance(self.target)
        alt_diff = abs(current.alt - self.target.alt)
        return dist < self.tolerance and alt_diff < self.tolerance