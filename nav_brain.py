"""
NavigationBrain - Pure logic for person-following behavior.
Accepts sensor data and returns motor commands.
"""

import math

class NavigationBrain:
    def __init__(self):
        # State
        self.mode = "STOPPED"
        self.target_bearing = 0
        self.target_distance = 0
        self.target_source = "NONE"

        # Tracking State
        self.last_known_bearing = 0
        self.last_target_time = 0

        # Tuning Parameters
        self.GPS_SWITCH_DIST = 5.0      # Switch to Lidar below this (meters)
        self.STOP_DIST = 1.0            # Stop when this close (meters)
        self.MAX_SPEED = 30             # Max speed percentage
        self.STEER_GAIN = 0.5           # How aggressively to steer

    def compute(self, robot_pos, beacon_pos, lidar_scan, current_rssi=-100, vision_data=None):
        """
        Compute steer and throttle based on latest sensor data.
        Returns: (steer_val, throttle_val, mode_string)
        """
        # Reset Target State
        self.target_source = "NONE"
        self.target_bearing = 0
        self.target_distance = 0
        
        # 1. Determine Target Properties
        import time
        now = time.time()

        has_vision = False
        if vision_data and (now - vision_data.get('ts', 0) < 1.0):
             has_vision = True

        has_gps = False
        if beacon_pos and beacon_pos.get('valid') and robot_pos.get('fix', 0) > 0:
             # Check if beacon data is fresh enough (e.g. < 5s old)
             if beacon_pos.get('age', 99) < 5.0:
                 has_gps = True

        has_rssi = current_rssi > -90 # Simple threshold

        
        
        if has_vision:
            # VISION PRIORITY: If we see a person, track them!
            self.target_bearing = vision_data['bearing']
            self.target_source = "VISION"
            
            # SENSOR FUSION: Use Lidar to validate distance
            # Find Lidar points within small angle of visual bearing
            lidar_dist = self._get_lidar_distance_at_angle(lidar_scan, self.target_bearing, tolerance=10.0)
            
            if lidar_dist:
                # GREAT SUCCESS: We have visual confirmation AND laser precision
                self.target_distance = lidar_dist
                self.target_source = "VISION+LIDAR"
            elif has_rssi:
                 # ESTIMATION: Visual bearing is good, but no Lidar hit (too low? too high?)
                 self.target_distance = 10 ** ((-59 - current_rssi) / 25.0)
            else:
                 self.target_distance = 1.0 # Default/Safe distance if unknown

        elif has_gps:
            # GPS FUSION: Use GPS as primary source
            self.target_distance = self._haversine(
                robot_pos['lat'], robot_pos['lon'],
                beacon_pos['lat'], beacon_pos['lon']
            )
            self.target_bearing = self._bearing(
                robot_pos['lat'], robot_pos['lon'],
                beacon_pos['lat'], beacon_pos['lon']
            )
            self.target_source = "GPS"
        elif has_rssi:
            # RSSI FALLBACK: Estimate distance from signal
            # Standard Formula: dist = 10 ^ ((TxPower - RSSI) / (10 * n))
            # Tuned for Phone Bluetooth: 
            # - Ref Power (at 1m) = -59 dBm (was -35 which is too high)
            # - N (Env Factor) = 2.5 (Indoor/Mixed)
            # Example: -59 -> 1.0m, -35 -> 0.1m, -84 -> 10m
            self.target_distance = 10 ** ((-59 - current_rssi) / 25.0)
            self.target_bearing = 0 # Direction unknown without GPS/Lidar
            self.target_source = "RSSI"
        else:
            self.mode = "STOPPED"
            return 90, 1500, "NO_SIGNAL"


        # 2. Select Navigation Mode
        if self.target_distance < self.STOP_DIST:
             self.mode = "STOPPED"
        elif has_vision:
             self.mode = "VISION_TRACKING"
        elif self.target_distance < self.GPS_SWITCH_DIST:
             self.mode = "LIDAR_TRACKING" # Handover to Lidar for precision
        elif has_gps:
             self.mode = "GPS_FOLLOWING"
        else:
             # RSSI only and far away? Try Lidar anyway to find object
             self.mode = "LIDAR_TRACKING"

        # 3. Execute Control Logic
        if self.mode == "STOPPED":
             return 90, 1500, self.mode
             
        elif self.mode == "GPS_FOLLOWING":
             # Simple bearing-based steering
             error = self.target_bearing # Assuming 0 heading
             while error > 180: error -= 360
             while error < -180: error += 360
             
             steer = 90 + int(error * self.STEER_GAIN)
             steer = max(60, min(120, steer))
             
             speed = min(self.MAX_SPEED, self.target_distance * 8)
             throttle = 1500 + int(speed * 2)
             return steer, throttle, self.mode
             
        elif self.mode == "VISION_TRACKING":
             # Vision gives us a good bearing
             steer = 90 + int(self.target_bearing * self.STEER_GAIN * 1.5) # Boost gain a bit?
             steer = max(60, min(120, steer))
             
             # Use Distance for throttle
             # Slow down as we get closer (target_distance is in meters)
             speed = min(self.MAX_SPEED, self.target_distance * 10)
             throttle = 1500 + int(speed * 2)
             
             return steer, throttle, self.mode

        elif self.mode == "LIDAR_TRACKING":
             if not lidar_scan:
                 return 90, 1500, "LIDAR_LOST"
                 
             # FUSION: Filter points based on Estimated Target Distance
             # Widen tolerance to 3000mm (3m) to handle noisy/saturated RSSI
             # If RSSI says 0.1m (saturated), we still want to find the user at 1.5m
             target_mm = self.target_distance * 1000
             tolerance_mm = 3000 
             
             candidates = []
             for a, d in lidar_scan:
                 if -60 < a < 60 and d > 150: # In front, ignore self/noise
                     if abs(d - target_mm) < tolerance_mm:
                         candidates.append((a, d))
                         
             if candidates:
                 # Found matching objects!
                 
                 # TRACKING STRATEGY: Prioritize Distance Match, then Angle
                 # 1. Find object closest to Expected Distance (RSSI)
                 # 2. Break ties with Stickiness/Center (Angle)
                 
                 import time
                 now = time.time()
                 if now - self.last_target_time < 2.0:
                     bias_angle = self.last_known_bearing
                 else:
                     bias_angle = 0
                 
                 # Tuples compare element by element.
                 # Primary Key: Abs Diff from Target Distance (RSSI)
                 # Secondary Key: Abs Diff from Bias Angle
                 # This ensures even if RSSI is 0.1m, we pick the object at 1.5m over the wall at 3m.
                 closest = min(candidates, key=lambda p: (abs(p[1] - target_mm), abs(p[0] - bias_angle)))
                 angle, dist_mm = closest
                 
                 # Update State
                 self.last_known_bearing = angle
                 self.last_target_time = now
                 
                 # Update Target with precise Lidar data
                 self.target_bearing = angle
                 self.target_distance = dist_mm / 1000.0
                 self.target_source = "LIDAR"
                 
                 # Steer towards it
                 steer = 90 + int(angle * 0.6)
                 steer = max(60, min(120, steer))
                 
                 speed = min(self.MAX_SPEED, self.target_distance * 12)
                 throttle = 1500 + int(speed * 2)
                 return steer, throttle, self.mode
             else:
                 return 90, 1500, "LIDAR_SEARCHING"

        return 90, 1500, self.mode

    def _haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def _bearing(self, lat1, lon1, lat2, lon2):
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dlam = math.radians(lon2 - lon1)
        x = math.sin(dlam) * math.cos(phi2)
        y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlam)
        return math.degrees(math.atan2(x, y))

    def _get_lidar_distance_at_angle(self, scan, target_angle, tolerance=5.0):
        """
        Finds the closest lidar point within +/- tolerance of target_angle.
        Returns distance in meters, or None.
        """
        if not scan:
            return None
        
        matches = []
        for angle, dist_mm in scan:
            # Check angle difference (handle 360 wrap if needed, but here simple diff usually works for -180 to 180)
            diff = abs(angle - target_angle)
            if diff > 180: diff = abs(diff - 360)
            
            if diff < tolerance:
                # Valid candidate
                # Filter noise: Must be valid distance (>0.1m and < 5.0m)
                if 100 < dist_mm < 5000:
                    matches.append(dist_mm)
        
        if matches:
            # Return minimum distance (closest object in that direction blocks view)
            return min(matches) / 1000.0
        
        return None
