"""
NavigationBrain - Vision+Lidar Person-Following with Smooth Homing.
Revamped for reliable target tracking with persistence and search behavior.
"""

import math
import time

class NavigationBrain:
    def __init__(self):
        # State
        self.mode = "STOPPED"
        self.target_bearing = 0
        self.target_distance = 0
        self.target_source = "NONE"

        # Tracking State
        self.last_known_bearing = 0
        self.last_vision_time = 0
        self.target_lost_time = 0
        self.search_direction = 1  # 1 = right, -1 = left

        # Tuning Parameters
        self.STOP_DIST = 0.5            # Stop when this close (meters)
        self.APPROACH_DIST = 2.0        # Start slowing down here
        self.MAX_SPEED = 40             # Max speed percentage
        self.MIN_SPEED = 15             # Minimum speed to prevent stalling
        self.STEER_KP = 1.5             # Proportional gain for steering
        self.STEER_DEADBAND = 3.0       # Degrees - ignore small errors
        self.COAST_TIMEOUT = 2.0        # Seconds to coast after losing vision
        self.SEARCH_TIMEOUT = 5.0       # Seconds to search before stopping
        self.LIDAR_TOLERANCE = 10.0     # Degrees tolerance for lidar matching

    def compute(self, robot_pos, beacon_pos, lidar_scan, current_rssi=-100, vision_data=None):
        """
        Compute steer and throttle based on vision + lidar.
        Returns: (steer_val, throttle_val, mode_string)
        """
        now = time.time()
        
        # Reset Target State
        self.target_source = "NONE"
        self.target_bearing = 0
        self.target_distance = 0
        
        # Check for vision detection
        has_vision = False
        if vision_data and (now - vision_data.get('ts', 0) < 0.5):
            has_vision = True
            self.last_vision_time = now
            self.last_known_bearing = vision_data['bearing']
            # Remember which side target was last seen
            self.search_direction = 1 if vision_data['bearing'] > 0 else -1
        
        # --- STATE MACHINE ---
        
        if has_vision:
            # ACTIVE TRACKING: Vision is working
            self.target_lost_time = 0
            return self._track_target(vision_data, lidar_scan)
            
        else:
            # VISION LOST
            time_since_vision = now - self.last_vision_time
            
            if time_since_vision < self.COAST_TIMEOUT:
                # COAST: Continue toward last known position
                return self._coast_mode(lidar_scan)
                
            elif time_since_vision < self.SEARCH_TIMEOUT:
                # SEARCH: Spin toward last known direction
                return self._search_mode()
                
            else:
                # STOP: Given up
                self.mode = "STOPPED"
                return 90, 1500, "STOPPED"

    def _track_target(self, vision_data, lidar_scan):
        """Active tracking with vision lock."""
        self.target_bearing = vision_data['bearing']
        self.target_source = "VISION"
        
        # Get distance from Lidar
        lidar_dist = self._get_lidar_distance_at_angle(
            lidar_scan, self.target_bearing, tolerance=self.LIDAR_TOLERANCE
        )
        
        if lidar_dist:
            self.target_distance = lidar_dist
            self.target_source = "VISION+LIDAR"
        else:
            # Estimate distance based on bbox size if available
            bbox = vision_data.get('bbox')
            if bbox:
                _, _, w, h = bbox
                # Larger bbox = closer object
                self.target_distance = max(0.5, 500.0 / max(w, h))
            else:
                self.target_distance = 1.5  # Default
        
        # Check stop condition
        if self.target_distance < self.STOP_DIST:
            self.mode = "ARRIVED"
            return 90, 1500, "ARRIVED"
        
        # Compute steering with P-controller and deadband
        steer = self._compute_steering(self.target_bearing)
        
        # Compute throttle with distance-based scaling
        throttle = self._compute_throttle(self.target_distance)
        
        self.mode = "TRACKING"
        return steer, throttle, self.mode

    def _coast_mode(self, lidar_scan):
        """Coast toward last known bearing."""
        self.target_bearing = self.last_known_bearing
        self.target_source = "MEMORY"
        
        # Check lidar for obstacle
        lidar_dist = self._get_lidar_distance_at_angle(
            lidar_scan, self.target_bearing, tolerance=self.LIDAR_TOLERANCE
        )
        self.target_distance = lidar_dist if lidar_dist else 2.0
        
        if self.target_distance < self.STOP_DIST:
            self.mode = "ARRIVED"
            return 90, 1500, "ARRIVED"
        
        steer = self._compute_steering(self.target_bearing)
        throttle = self._compute_throttle(self.target_distance * 0.7)  # Slower when coasting
        
        self.mode = "COASTING"
        return steer, throttle, self.mode

    def _search_mode(self):
        """Spin to search for target."""
        self.target_source = "SEARCHING"
        self.target_distance = 0
        
        # Slow spin in the direction target was last seen
        spin_offset = 25 * self.search_direction
        steer = 90 + spin_offset
        steer = max(50, min(130, steer))
        
        self.mode = "SEARCHING"
        return steer, 1500, self.mode  # No forward motion while searching

    def _compute_steering(self, bearing):
        """P-controller with deadband for smooth steering."""
        # Apply deadband
        if abs(bearing) < self.STEER_DEADBAND:
            return 90  # Centered
        
        # P-controller
        steer_output = 90 + int(bearing * self.STEER_KP)
        
        # Clamp to safe range
        return max(50, min(130, steer_output))

    def _compute_throttle(self, distance):
        """Distance-based throttle for smooth approach."""
        if distance < self.STOP_DIST:
            return 1500  # Stop
        
        if distance > self.APPROACH_DIST:
            # Far away - max speed
            speed = self.MAX_SPEED
        else:
            # Proportional slowdown as we get closer
            ratio = (distance - self.STOP_DIST) / (self.APPROACH_DIST - self.STOP_DIST)
            speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * ratio
        
        # Convert speed percentage to PWM (1500 = stop, 1700 = full forward)
        pwm = 1500 + int(speed * 2)
        return min(1700, pwm)

    def _get_lidar_distance_at_angle(self, scan, target_angle, tolerance=10.0):
        """
        Find closest lidar point within tolerance of target_angle.
        Uses median filtering for noise reduction.
        Returns distance in meters, or None.
        """
        if not scan:
            return None
        
        matches = []
        for angle, dist_mm in scan:
            diff = abs(angle - target_angle)
            if diff > 180:
                diff = abs(diff - 360)
            
            if diff < tolerance:
                # Filter noise: valid range 0.2m to 5.0m
                if 200 < dist_mm < 5000:
                    matches.append(dist_mm)
        
        if matches:
            # Use median to filter outliers
            matches.sort()
            median_idx = len(matches) // 2
            return matches[median_idx] / 1000.0
        
        return None
