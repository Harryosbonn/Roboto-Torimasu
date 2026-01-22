"""
Autonomous Follow Me + Obstacle Avoidance with GUI

Standalone program that combines:
- Vision-based person tracking (green object detection)
- Lidar-based obstacle avoidance (360° safety zones)
- Real-time GUI showing camera feed and lidar visualization

Run: python autonomous_follower.py
Press 'q' to quit, 'p' to pause/resume motors
"""

import time
import math
import threading
import serial
import cv2
import numpy as np

from vision_driver import VisionDriver
from lidar_driver import LidarDriver


class ObstacleAvoidance:
    """
    Analyzes Lidar scan data for obstacles and provides safety recommendations.
    
    Zones (degrees, 0 = front):
    - FRONT: -30 to +30 (critical)
    - LEFT:  -90 to -30
    - RIGHT: +30 to +90
    """
    
    # Zone definitions (angle ranges in degrees)
    FRONT_ZONE = (-30, 30)
    LEFT_ZONE = (-90, -30)
    RIGHT_ZONE = (30, 90)
    
    # Distance thresholds (meters)
    STOP_DISTANCE = 0.25     # Emergency stop (reduced - was too sensitive)
    SLOW_DISTANCE = 0.8      # Start slowing down
    EVADE_DISTANCE = 0.5     # Trigger evasion
    MIN_VALID_DISTANCE = 0.2 # Ignore readings closer than this (robot body/ground)
    
    def __init__(self):
        self.last_analysis = None
    
    def analyze(self, lidar_scan):
        """
        Analyze lidar scan for obstacles.
        
        Returns:
            dict with keys:
                - front_clear: bool
                - left_clear: bool  
                - right_clear: bool
                - front_dist: float (meters, or None)
                - nearest_obstacle: (angle, dist_m) or None
                - action: "CLEAR" | "SLOW" | "STOP" | "EVADE_LEFT" | "EVADE_RIGHT"
        """
        if not lidar_scan:
            return {
                "front_clear": True,
                "left_clear": True,
                "right_clear": True,
                "front_dist": None,
                "nearest_obstacle": None,
                "action": "CLEAR"
            }
        
        front_dists = []
        left_dists = []
        right_dists = []
        nearest = None
        nearest_dist = float('inf')
        
        for angle, dist_mm in lidar_scan:
            # Filter invalid readings (ignore very close - likely robot body/ground)
            if dist_mm < (self.MIN_VALID_DISTANCE * 1000) or dist_mm > 8000:
                continue
                
            dist_m = dist_mm / 1000.0
            
            # Normalize angle to -180 to +180
            while angle > 180:
                angle -= 360
            while angle < -180:
                angle += 360
            
            # Track nearest obstacle
            if dist_m < nearest_dist:
                nearest_dist = dist_m
                nearest = (angle, dist_m)
            
            # Categorize by zone
            if self.FRONT_ZONE[0] <= angle <= self.FRONT_ZONE[1]:
                front_dists.append(dist_m)
            elif self.LEFT_ZONE[0] <= angle <= self.LEFT_ZONE[1]:
                left_dists.append(dist_m)
            elif self.RIGHT_ZONE[0] <= angle <= self.RIGHT_ZONE[1]:
                right_dists.append(dist_m)
        
        # Calculate minimum distances per zone
        front_min = min(front_dists) if front_dists else float('inf')
        left_min = min(left_dists) if left_dists else float('inf')
        right_min = min(right_dists) if right_dists else float('inf')
        
        # Determine clearance
        front_clear = front_min > self.SLOW_DISTANCE
        left_clear = left_min > self.EVADE_DISTANCE
        right_clear = right_min > self.EVADE_DISTANCE
        
        # Determine action
        if front_min < self.STOP_DISTANCE:
            action = "STOP"
        elif front_min < self.EVADE_DISTANCE:
            # Need to evade - pick the clearer side
            if left_clear and not right_clear:
                action = "EVADE_LEFT"
            elif right_clear and not left_clear:
                action = "EVADE_RIGHT"
            elif left_min > right_min:
                action = "EVADE_LEFT"
            else:
                action = "EVADE_RIGHT"
        elif front_min < self.SLOW_DISTANCE:
            action = "SLOW"
        else:
            action = "CLEAR"
        
        result = {
            "front_clear": front_clear,
            "left_clear": left_clear,
            "right_clear": right_clear,
            "front_dist": front_min if front_dists else None,
            "nearest_obstacle": nearest,
            "action": action
        }
        
        self.last_analysis = result
        return result


class FollowerBrain:
    """
    State machine for person-following with obstacle avoidance integration.
    
    States:
    - SEARCHING: No target, spinning to find
    - TRACKING: Target visible, driving toward
    - APPROACHING: Target close, slowing down
    - ARRIVED: Target very close, stopped
    - EMERGENCY: Obstacle too close, emergency stop
    """
    
    # Distance thresholds (meters)
    ARRIVED_DISTANCE = 0.3   # Stop when very close (reduced from 0.5)
    APPROACH_DISTANCE = 1.2  # Start slowing
    
    MAX_SPEED = 70       # Sane base speed
    APPROACH_SPEED = 40  # Slower approach
    MIN_SPEED = 30       # Minimum moving
    
    # Speed multiplier range (adjusted via GUI)
    SPEED_MIN = 0.3
    SPEED_MAX = 1.0
    SPEED_STEP = 0.1
    
    # Steering settings
    STEER_KP = 1.5
    STEER_KD = 0.3            # Derivative gain for damping
    STEER_DEADBAND = 3.0      # degrees
    
    # Timing
    VISION_TIMEOUT = 0.5      # seconds - vision data considered stale
    PERSISTENCE_TIME = 0.5    # seconds - continue toward last position before searching
    SEARCH_SPIN_SPEED = 20    # steering offset for searching
    
    def __init__(self):
        self.state = "SEARCHING"
        self.target_bearing = 0
        self.target_distance = None
        self.last_known_bearing = 0
        self.prev_bearing_error = 0     # For D-term
        self.last_vision_time = 0       # For persistence
        self.search_direction = 1       # 1 = right, -1 = left
        self.speed_multiplier = 0.5     # Start at 50% speed
    
    def adjust_speed(self, delta):
        """Adjust speed multiplier by delta (+/- SPEED_STEP)."""
        self.speed_multiplier = max(0.1, 
                                     min(self.SPEED_MAX, 
                                         self.speed_multiplier + delta))
        
    def compute(self, vision_data, obstacle_data, lidar_scan):
        """
        Compute motor commands based on vision and obstacle data.
        
        Returns:
            (steer, throttle_pwm, state_string)
        """
        now = time.time()
        
        # --- Check for emergency stop ---
        if obstacle_data and obstacle_data["action"] == "STOP":
            self.state = "EMERGENCY"
            return 90, 1500, "EMERGENCY"
        
        # --- Check vision status ---
        has_vision = False
        if vision_data:
            age = now - vision_data.get("ts", 0)
            if age < self.VISION_TIMEOUT:
                has_vision = True
                self.target_bearing = vision_data["bearing"]
                self.last_known_bearing = self.target_bearing
                self.last_vision_time = now
                self.search_direction = 1 if self.target_bearing > 0 else -1
        
        # --- State machine ---
        if not has_vision:
            # Check for persistence mode - continue toward last known position briefly
            time_since_vision = now - self.last_vision_time
            if time_since_vision < self.PERSISTENCE_TIME and self.last_vision_time > 0:
                # Persistence mode - keep going toward last known bearing
                self.state = "PERSISTING"
                steer = self._compute_steering(self.last_known_bearing)
                return steer, 1560, "PERSISTING"  # Slow crawl
            
            # Lost target - search
            self.state = "SEARCHING"
            spin_offset = self.SEARCH_SPIN_SPEED * self.search_direction
            steer = 90 + spin_offset
            return max(50, min(130, steer)), 1500, "SEARCHING"
        
        # We have vision - get distance
        self.target_distance = self._get_target_distance(vision_data, lidar_scan)
        
        # Check arrival
        if self.target_distance and self.target_distance < self.ARRIVED_DISTANCE:
            self.state = "ARRIVED"
            return 90, 1500, "ARRIVED"
        
        # --- Compute steering ---
        steer = self._compute_steering(self.target_bearing)
        
        # --- Handle obstacle evasion while tracking ---
        if obstacle_data:
            action = obstacle_data["action"]
            front_dist = obstacle_data.get("front_dist")
            
            # Proportional evasion - scale offset based on proximity
            if front_dist and front_dist > 0.1:
                evade_offset = min(40, int(20 / front_dist))  # Closer = sharper turn
            else:
                evade_offset = 20  # Default
            
            if action == "EVADE_LEFT":
                steer = max(50, steer - evade_offset)
            elif action == "EVADE_RIGHT":
                steer = min(130, steer + evade_offset)
        
        # --- Compute throttle ---
        if self.target_distance and self.target_distance < self.APPROACH_DISTANCE:
            self.state = "APPROACHING"
            speed = self.APPROACH_SPEED
        else:
            self.state = "TRACKING"
            speed = self.MAX_SPEED
        
        # Apply obstacle slowdown
        if obstacle_data and obstacle_data["action"] == "SLOW":
            speed = min(speed, self.MIN_SPEED)
        
        # Apply speed multiplier (from GUI control)
        # speed is 0-100 based on MAX_SPEED setting (e.g. 70)
        target_speed = speed * self.speed_multiplier
        
        # Non-linear mapping:
        # If speed > 0, we must output at least MIN_PWM to move.
        MIN_PWM = 1560  # Lowered from 1620 to allow slower crawl
        MAX_PWM = 1750  # Lowered from 1950 to cap top speed
        
        if target_speed > 0:
            # Normalize speed (0-100) to PWM range (1620-1950)
            # We assume '100' is the theoretical max input speed
            pct = min(1.0, target_speed / 100.0) 
            pwm_range = MAX_PWM - MIN_PWM
            throttle_pwm = MIN_PWM + int(pct * pwm_range)
        else:
            throttle_pwm = 1500
            
        return steer, throttle_pwm, self.state
    
    def _compute_steering(self, bearing):
        """PD-controller with deadband for smooth steering."""
        if abs(bearing) < self.STEER_DEADBAND:
            self.prev_bearing_error = 0
            return 90
        
        # P-term
        p_term = bearing * self.STEER_KP
        
        # D-term (damping to reduce oscillation)
        d_term = (bearing - self.prev_bearing_error) * self.STEER_KD
        self.prev_bearing_error = bearing
        
        steer = 90 + int(p_term + d_term)
        return max(50, min(130, steer))
    
    def _get_target_distance(self, vision_data, lidar_scan):
        """Get distance to target from lidar or estimate from bbox."""
        if lidar_scan and vision_data:
            target_angle = vision_data["bearing"]
            
            matches = []
            for angle, dist_mm in lidar_scan:
                while angle > 180:
                    angle -= 360
                
                diff = abs(angle - target_angle)
                if diff > 180:
                    diff = abs(diff - 360)
                
                if diff < 15:
                    if 200 < dist_mm < 5000:
                        matches.append(dist_mm)
            
            if matches:
                matches.sort()
                return matches[len(matches) // 2] / 1000.0
        
        if vision_data and "bbox" in vision_data:
            _, _, w, h = vision_data["bbox"]
            return max(0.5, 400.0 / max(w, h))
        
        return None


class FollowerGUI:
    """
    OpenCV-based GUI showing camera feed and lidar visualization.
    """
    
    # Window sizes
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    LIDAR_SIZE = 300  # Square
    
    # Lidar display settings
    MAX_DISPLAY_DIST = 3.0  # meters
    
    def __init__(self):
        self.paused = False
        
    def render(self, vision_frame, lidar_scan, obstacle_data, brain_state, vision_data):
        """
        Render combined GUI frame.
        
        Returns:
            Combined OpenCV frame (BGR)
        """
        # Create lidar visualization
        lidar_frame = self._render_lidar(lidar_scan, obstacle_data, vision_data)
        
        # Get camera frame from vision driver
        if vision_frame is None:
            camera_frame = np.zeros((self.CAMERA_HEIGHT, self.CAMERA_WIDTH, 3), dtype=np.uint8)
            cv2.putText(camera_frame, "NO CAMERA", (60, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            camera_frame = vision_frame.copy()
        
        # Resize camera to match lidar height
        camera_display = cv2.resize(camera_frame, (self.LIDAR_SIZE, self.LIDAR_SIZE))
        
        # Add state overlay to camera frame
        self._add_state_overlay(camera_display, brain_state, obstacle_data, vision_data)
        
        # Combine horizontally: camera on left, lidar on right
        combined = np.hstack([camera_display, lidar_frame])
        
        # Add header bar
        header = np.zeros((40, combined.shape[1], 3), dtype=np.uint8)
        header[:] = (40, 40, 40)
        
        title = "AUTONOMOUS FOLLOWER"
        if self.paused:
            title += " [PAUSED]"
        cv2.putText(header, title, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        controls = "Q:Quit P:Pause +/-:Speed"
        cv2.putText(header, controls, (combined.shape[1] - 240, 28), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        
        final = np.vstack([header, combined])
        
        return final
    
    def _render_lidar(self, lidar_scan, obstacle_data, vision_data):
        """Render lidar as top-down polar view."""
        size = self.LIDAR_SIZE
        center = size // 2
        
        # Dark background
        frame = np.zeros((size, size, 3), dtype=np.uint8)
        frame[:] = (30, 30, 30)
        
        # Draw range rings
        for r in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
            radius = int((r / self.MAX_DISPLAY_DIST) * (size // 2 - 10))
            color = (60, 60, 60)
            cv2.circle(frame, (center, center), radius, color, 1)
        
        # Draw danger zone (red arc in front)
        self._draw_zone_arc(frame, center, -30, 30, 0.4, (0, 0, 80))
        
        # Draw warning zone
        self._draw_zone_arc(frame, center, -30, 30, 1.0, (0, 50, 80))
        
        # Draw robot (triangle pointing up)
        robot_pts = np.array([
            [center, center - 15],
            [center - 10, center + 10],
            [center + 10, center + 10]
        ], np.int32)
        cv2.fillPoly(frame, [robot_pts], (0, 200, 200))
        
        # Draw lidar points
        if lidar_scan:
            for angle, dist_mm in lidar_scan:
                if dist_mm < 100 or dist_mm > 8000:
                    continue
                
                dist_m = dist_mm / 1000.0
                if dist_m > self.MAX_DISPLAY_DIST:
                    continue
                
                # Convert to screen coordinates (0° = up)
                rad = math.radians(angle - 90)
                scale = (size // 2 - 10) / self.MAX_DISPLAY_DIST
                x = center + int(dist_m * scale * math.cos(rad))
                y = center + int(dist_m * scale * math.sin(rad))
                
                # Color based on distance
                if dist_m < 0.4:
                    color = (0, 0, 255)  # Red
                elif dist_m < 1.0:
                    color = (0, 165, 255)  # Orange
                else:
                    color = (255, 255, 0)  # Cyan
                
                cv2.circle(frame, (x, y), 2, color, -1)
        
        # Draw target bearing line if tracking
        if vision_data and vision_data.get("bearing") is not None:
            bearing = vision_data["bearing"]
            rad = math.radians(bearing - 90)
            line_len = size // 2 - 20
            x2 = center + int(line_len * math.cos(rad))
            y2 = center + int(line_len * math.sin(rad))
            cv2.line(frame, (center, center), (x2, y2), (0, 255, 0), 2)
        
        # Labels
        cv2.putText(frame, "FRONT", (center - 25, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        cv2.putText(frame, "LEFT", (5, center), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        cv2.putText(frame, "RIGHT", (size - 45, center), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        
        # Obstacle action indicator
        if obstacle_data:
            action = obstacle_data["action"]
            if action == "STOP":
                color = (0, 0, 255)
            elif action in ["EVADE_LEFT", "EVADE_RIGHT"]:
                color = (0, 165, 255)
            elif action == "SLOW":
                color = (0, 255, 255)
            else:
                color = (0, 255, 0)
            
            cv2.putText(frame, action, (10, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return frame
    
    def _draw_zone_arc(self, frame, center, start_angle, end_angle, max_dist, color):
        """Draw a filled arc zone."""
        size = self.LIDAR_SIZE
        radius = int((max_dist / self.MAX_DISPLAY_DIST) * (size // 2 - 10))
        
        # OpenCV ellipse uses different angle convention
        cv2.ellipse(frame, (center, center), (radius, radius), 
                    -90, start_angle, end_angle, color, -1)
    
    def _add_state_overlay(self, frame, brain_state, obstacle_data, vision_data):
        """Add state information overlay to camera frame."""
        # State badge
        state = brain_state.state
        if state == "TRACKING":
            color = (0, 255, 0)
        elif state == "APPROACHING":
            color = (0, 255, 255)
        elif state == "ARRIVED":
            color = (255, 200, 0)
        elif state == "EMERGENCY":
            color = (0, 0, 255)
        else:
            color = (128, 128, 128)
        
        cv2.rectangle(frame, (5, 5), (120, 35), color, -1)
        cv2.putText(frame, state, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Distance info
        if brain_state.target_distance:
            dist_text = f"Dist: {brain_state.target_distance:.2f}m"
            cv2.putText(frame, dist_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Bearing info
        if vision_data:
            bearing_text = f"Bearing: {brain_state.target_bearing:.1f}deg"
            cv2.putText(frame, bearing_text, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Speed info (bottom of frame)
        speed_pct = int(brain_state.speed_multiplier * 100)
        speed_text = f"Speed: {speed_pct}%"
        cv2.putText(frame, speed_text, (10, frame.shape[0] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


class AutonomousFollower:
    """
    Main controller that ties everything together.
    """
    
    ARDUINO_PORT = "/dev/ttyACM0"
    ARDUINO_BAUD = 115200
    CONTROL_HZ = 10
    
    def __init__(self, headless=False):
        self.running = False
        self.headless = headless
        
        # Components
        self.vision = VisionDriver(camera_index=0, width=320, height=240)
        self.lidar = LidarDriver(port="/dev/ttyUSB0", baudrate=230400)
        self.obstacle_avoidance = ObstacleAvoidance()
        self.brain = FollowerBrain()
        self.gui = FollowerGUI() if not headless else None
        
        # Arduino connection
        self.arduino = None
        self.last_command_time = 0
        self.last_command_time = 0
        self.command_rate_limit = 0.05
        self.last_arduino_rx_time = time.time()
        
    def start(self):
        """Initialize all components and start the control loop."""
        print("=" * 50)
        print("AUTONOMOUS FOLLOWER - Starting...")
        print("=" * 50)
        
        # Connect Arduino
        # Connect Arduino
        ports_to_try = [self.ARDUINO_PORT, "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM2"]
        self.arduino = None
        
        for port in ports_to_try:
            try:
                print(f"ARDUINO: Attempting to connect on {port}...")
                ser = serial.Serial(port, self.ARDUINO_BAUD, timeout=0.1)
                
                # Verify it's not the Lidar (Lidar usually doesn't respond to ASCII reset command nicely, 
                # or we can rely on manual user config. For now, just checking if it opens is a start,
                # but let's try to be smart - the Lidar is usually USB0, Arduino ACMx on Linux usually.
                # If we open the Lidar port by mistake, it might confuse the Lidar driver.)
                
                # Better approach: Just try opening. If it works, we assume it's the right one for now 
                # unless we have a specific handshake (which we don't really have time to build fully).
                # However, the Lidar driver catches its own port.
                
                self.arduino = ser
                self.ARDUINO_PORT = port
                print(f"✓ Arduino: Connected on {port}")
                time.sleep(2)
                break
            except Exception as e:
                # print(f"  Failed on {port}: {e}")
                pass
        
        if self.arduino is None:
            print(f"✗ Arduino: Failed to connect. Checked: {ports_to_try}")
        
        # Start vision
        self.vision.start()
        print("✓ Vision: Started (Green object detection)")
        
        # Start lidar
        self.lidar.start()
        if self.lidar.connected:
            print("✓ Lidar: Connected and scanning")
        else:
            print("✗ Lidar: Not connected")
        
        if not self.headless:
            print("=" * 50)
            print("GUI starting... Press 'q' to quit, 'p' to pause")
            print("=" * 50)
        else:
            print("=" * 50)
            print("HEADLESS MODE - Running without GUI")
            print("=" * 50)
        
        self.running = True
        self._control_loop()
    
    def _control_loop(self):
        """Main control loop with GUI."""
        try:
            while self.running:
                loop_start = time.time()
                
                # Get sensor data
                vision_data = self.vision.get_latest_detection()
                lidar_scan = self.lidar.get_scan()
                
                # Get raw camera frame for GUI
                with self.vision.frame_lock:
                    camera_frame = self.vision.current_frame
                
                # Analyze obstacles
                obstacle_data = self.obstacle_avoidance.analyze(lidar_scan)
                
                # Compute control
                paused = False
                if self.gui:
                    paused = self.gui.paused
                
                if not paused:
                    steer, throttle, state = self.brain.compute(
                        vision_data, obstacle_data, lidar_scan
                    )
                    self._send_command(steer, throttle)
                else:
                    self._send_command(90, 1500)
                
                # Render GUI if active
                if self.gui:
                    gui_frame = self.gui.render(
                        camera_frame, lidar_scan, obstacle_data, self.brain, vision_data
                    )
                    cv2.imshow("Autonomous Follower", gui_frame)
                    
                    # Handle keyboard input
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('p'):
                        self.gui.paused = not self.gui.paused
                        if self.gui.paused:
                            print("[PAUSED] Motors stopped")
                        else:
                            print("[RESUMED]")
                    elif key == ord('+') or key == ord('='):
                        self.brain.adjust_speed(self.brain.SPEED_STEP)
                        print(f"[SPEED] {int(self.brain.speed_multiplier * 100)}%")
                    elif key == ord('-') or key == ord('_'):
                        self.brain.adjust_speed(-self.brain.SPEED_STEP)
                        print(f"[SPEED] {int(self.brain.speed_multiplier * 100)}%")
                
                # Rate limit
                elapsed = time.time() - loop_start
                sleep_time = (1.0 / self.CONTROL_HZ) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n[!] Interrupted by user")
        finally:
            self.stop()
    
    def _send_command(self, steer, throttle):
        """Send command to Arduino with rate limiting."""
        now = time.time()
        if now - self.last_command_time < self.command_rate_limit:
            return
        
        if self.arduino and self.arduino.is_open:
            try:
                # Arduino expects format: <steer,throttle>
                cmd = f"<{steer},{throttle}>"
                self.arduino.write(cmd.encode())
                self.last_command_time = now
                
                # Consume debug lines manually
                while self.arduino.in_waiting > 0:
                    self.last_arduino_rx_time = time.time()
                    line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if line.startswith("DEBUG") or line.startswith("INIT"):
                            print(f"[ARDUINO] {line}")
                            
                # Health check 
                if time.time() - self.last_arduino_rx_time > 3.0:
                     # Only print every 3s to avoid spam
                     if time.time() % 3.0 < 0.1: 
                         print("[ALERT] Arduino connection silent! (Possible IMU hang/Hardware freeze)")
                    
            except Exception as e:
                print(f"[!] Arduino write error: {e}")
    
    def stop(self):
        """Stop all components safely."""
        print("\n" + "=" * 50)
        print("SHUTTING DOWN...")
        print("=" * 50)
        
        self.running = False
        
        # Close GUI
        if self.gui:
            cv2.destroyAllWindows()
        
        # Stop motors first
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(b"<90,1500>")
                time.sleep(0.1)
                self.arduino.close()
                print("✓ Arduino: Stopped and disconnected")
            except:
                pass
        
        # Stop sensors
        self.vision.stop()
        print("✓ Vision: Stopped")
        
        self.lidar.stop()
        print("✓ Lidar: Stopped")
        
        print("=" * 50)
        print("Shutdown complete.")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    args = parser.parse_args()

    follower = AutonomousFollower(headless=args.headless)
    follower.start()
