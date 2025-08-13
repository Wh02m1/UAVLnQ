import csv
from datetime import datetime
import threading
import time
from dronekit import Vehicle
# log physical data of the drone
class DataLogger:
    """Records drone telemetry to CSV files"""
    def __init__(self, vehicle: Vehicle, drone_id: int):
        self.vehicle = vehicle
        self.drone_id = drone_id
        self.running = False
        self.log_thread = None
        self.log_interval = 0.5  # Log twice per second
        self.filename = f"logs/drone_{drone_id}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        # Initialize CSV file
        with open(self.filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'lat', 'lon', 'alt',
                'velocity_x', 'velocity_y', 'velocity_z',
                'ground_speed', 'air_speed', 'heading',
                'roll', 'pitch', 'yaw',
                'battery_voltage', 'battery_current', 'battery_level',
                'mode', 'armed', 'system_status',
                'satellites_visible', 'fix_type'
            ])
    
    def start(self):  # Start logging thread
        self.running = True
        self.log_thread = threading.Thread(target=self._log_loop)
        self.log_thread.start()
        print(f"Drone {self.drone_id}: Logging started -> {self.filename}")

    def stop(self): # Stop logging thread
        self.running = False
        if self.log_thread:
            self.log_thread.join()
        print(f"Drone {self.drone_id}: Logging stopped")

    def _log_loop(self):
        while self.running:
            try:
                # Collect telemetry data
                timestamp = datetime.now().isoformat()
                loc = self.vehicle.location.global_relative_frame
                vel = self.vehicle.velocity
                attitude = self.vehicle.attitude
                battery = self.vehicle.battery
                gps = self.vehicle.gps_0

                # Write to CSV
                with open(self.filename, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        timestamp,
                        loc.lat, loc.lon, loc.alt,
                        vel[0] if vel else 0,
                        vel[1] if vel else 0,
                        vel[2] if vel else 0,
                        self.vehicle.groundspeed,
                        self.vehicle.airspeed,
                        self.vehicle.heading,
                        attitude.roll, attitude.pitch, attitude.yaw,
                        battery.voltage, battery.current, battery.level,
                        self.vehicle.mode.name,
                        self.vehicle.armed,
                        self.vehicle.system_status.state,
                        gps.satellites_visible,
                        gps.fix_type
                    ])
                
                time.sleep(self.log_interval)
                
            except Exception as e:
                print(f"Logging error (Drone {self.drone_id}): {str(e)}")
                break
