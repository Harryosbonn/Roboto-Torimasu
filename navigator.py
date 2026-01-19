"""
Navigator - GPS + Lidar hybrid navigation for person-following robot

Modes:
  - GPS_FOLLOWING: Use GPS bearing when target is far (>5m)
  - LIDAR_TRACKING: Use Lidar to track target when close (<5m)
  - STOPPED: Target is very close (<1m), hold position
"""

import math
import time
import threading
import serial

from gps_driver import GPSDriver
from lidar_driver import LidarDriver
from beacon_receiver import BeaconReceiver
from rssi_monitor import RSSIMonitor

class Navigator:
    def __init__(self, arduino_port='/dev/ttyACM0'):
        # Sensors
        self.robot_gps = GPSDriver(port='/dev/ttyAMA0', baudrate=115200)
        self.lidar = LidarDriver(port='/dev/ttyUSB0', baudrate=230400)
        self.beacon = BeaconReceiver(port=5005)
        self.rssi_monitor = RSSIMonitor()
        
        # Arduino for motor control
        self.arduino = serial.Serial(arduino_port, 115200, timeout=0.1)
        
        # State
        self.mode = "STOPPED"
        self.last_mode = "STOPPED"
        self.target_bearing = 0
        self.target_distance = 0
        self.current_rssi = 0
        self.running = False
        
        # Tuning Parameters
        self.GPS_SWITCH_DIST = 5.0      # Switch to Lidar below this (meters)
        self.STOP_DIST = 1.0            # Stop when this close (meters)
        self.RESUME_DIST = 1.5          # Resume when target moves away
        self.MAX_SPEED = 30             # Max speed percentage
        self.STEER_GAIN = 0.5           # How aggressively to steer
        
    def start(self):
        self.robot_gps.start()
        self.lidar.start()
        self.beacon.start()
        
        self.running = True
        self.thread = threading.Thread(target=self._nav_loop)
        self.thread.daemon = True
        self.thread.start()
        print("Navigator started")
        
    def _nav_loop(self):
        while self.running:
            try:
                # Get positions
                robot_pos = self.robot_gps.get_data()
                beacon_pos = self.beacon.get_position()
                lidar_scan = self.lidar.get_scan()
                
                # Check if beacon data is valid and fresh
                if beacon_pos and beacon_pos['age'] < 3.0:
                    # Get RSSI
                    self.current_rssi = self.rssi_monitor.get_rssi(beacon_pos['ip']) if beacon_pos['ip'] else -100
                    
                    if beacon_pos['valid'] and robot_pos['fix'] > 0:
                        # OUTDOOR MODE: GPS Valid
                        gps_dist = self._haversine(
                            robot_pos['lat'], robot_pos['lon'],
                            beacon_pos['lat'], beacon_pos['lon']
                        )
                        gps_bearing = self._bearing(
                            robot_pos['lat'], robot_pos['lon'],
                            beacon_pos['lat'], beacon_pos['lon']
                        )
                        
                        self.target_distance = gps_dist
                        self.target_bearing = gps_bearing
                        
                        if gps_dist < self.STOP_DIST:
                            self.mode = "STOPPED"
                        elif gps_dist < self.GPS_SWITCH_DIST:
                            self.mode = "LIDAR_TRACKING"
                        else:
                            self.mode = "GPS_FOLLOWING"
                    else:
                        # INDOOR MODE: Use RSSI for distance estimation
                        # Map RSSI to approximate distance
                        if self.current_rssi > -45:
                            self.target_distance = 1.0 # Very Close
                        elif self.current_rssi > -60:
                            self.target_distance = 3.5 # Medium
                        else:
                            self.target_distance = 7.0 # Far
                            
                        # Bearing is unknown via RSSI, relies on Lidar or search
                        if self.current_rssi > -40:
                            self.mode = "STOPPED"
                        else:
                            self.mode = "LIDAR_TRACKING" # Default to Lidar for bearing
                            
                else:
                    self.mode = "STOPPED" # Lost beacon
                
                # Execute mode
                if self.mode == "STOPPED":
                    self._send_command(90, 1500)
                    
                elif self.mode == "GPS_FOLLOWING":
                    steer, throttle = self._gps_control()
                    self._send_command(steer, throttle)
                    
                elif self.mode == "LIDAR_TRACKING":
                    steer, throttle = self._lidar_control(lidar_scan)
                    self._send_command(steer, throttle)
                    
            except Exception as e:
                print(f"Nav error: {e}")
                self._send_command(90, 1500)  # Safe stop
                
            time.sleep(0.1)  # 10Hz control loop
            
    def _gps_control(self):
        """Generate steering/throttle from GPS bearing"""
        # Assume robot heading is 0 (north) for now
        # TODO: Add magnetometer for true heading
        robot_heading = 0
        
        # Error = target bearing - robot heading
        error = self.target_bearing - robot_heading
        
        # Normalize to -180 to 180
        while error > 180: error -= 360
        while error < -180: error += 360
        
        # Steering: 90 is center, 45-135 range
        steer = 90 + int(error * self.STEER_GAIN)
        steer = max(45, min(135, steer))
        
        # Speed proportional to distance
        speed = min(self.MAX_SPEED, self.target_distance * 10)
        throttle = 1500 + int(speed * 2)  # Convert to PWM
        
        return steer, throttle
        
    def _lidar_control(self, scan):
        """Use Lidar to find and track nearest large object (person)"""
        if not scan:
            return 90, 1500
            
        # Find cluster of points (person's legs/body)
        # Simple approach: find the closest point in front arc
        front_points = [(a, d) for a, d in scan if -60 < a < 60 and d > 100]
        
        if not front_points:
            return 90, 1500  # No target, stop
            
        # Find closest
        closest = min(front_points, key=lambda p: p[1])
        angle, dist = closest
        
        # Steering toward target
        steer = 90 + int(angle * 0.5)
        steer = max(45, min(135, steer))
        
        # Speed based on distance (dist is in mm)
        dist_m = dist / 1000.0
        if dist_m < self.STOP_DIST:
            throttle = 1500
        else:
            speed = min(self.MAX_SPEED, dist_m * 15)
            throttle = 1500 + int(speed * 2)
            
        return steer, throttle
        
    def _send_command(self, steer, throttle):
        """Send command to Arduino"""
        msg = f"<{steer},{throttle}>"
        self.arduino.write(msg.encode())
        
    def _haversine(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points in meters"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
        
    def _bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from point 1 to point 2 in degrees"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        
        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
        
        bearing = math.atan2(x, y)
        return math.degrees(bearing)
        
    def stop(self):
        self.running = False
        self._send_command(90, 1500)
        self.robot_gps.stop()
        self.lidar.stop()
        self.beacon.stop()
        

# Test mode
if __name__ == "__main__":
    nav = Navigator()
    nav.start()
    
    print("Navigator running... (Ctrl+C to stop)")
    try:
        while True:
            print(f"Mode: {nav.mode} | Dist: {nav.target_distance:.1f}m | Bearing: {nav.target_bearing:.0f}°")
            time.sleep(1)
    except KeyboardInterrupt:
        nav.stop()
        print("\nStopped.")
