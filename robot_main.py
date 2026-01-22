import serial
import time
import math
import threading
import pynmea2
import socket
from lidar_driver import LidarDriver

# --- CONFIGURATION ---
ARDUINO_PORT = '/dev/ttyACM0'  # or /dev/ttyUSB0
GPS_PORT = '/dev/serial0'      # Pi Built-in UART
LIDAR_PORT = '/dev/ttyUSB1'    # YDLiDAR
BAUD_RATE = 115200

# Waypoint / User Target (Updated via Wifi/Telemetry)
# Default Global Var (Safety: Start at 0,0)
target_lat = 0.0
target_lon = 0.0
target_updated = 0

# Robot State
current_lat = 0.0
current_lon = 0.0
current_heading = 0.0 # Degrees, 0=North

# Control Params
Kp_STEER = 2.0  # Proportional Gain
BASE_SPEED = 1600 # PWM (1500 + 100)
STOP_SPEED = 1500
MAX_TURN = 45 # Deg from center

# UDP Config for User Beacon
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

class ArduinoLink:
    """Robust Arduino connection with rate limiting and health monitoring."""
    
    PORTS_TO_TRY = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    COMMAND_RATE_LIMIT = 0.05  # 50ms between commands (20Hz max)
    HEALTH_TIMEOUT = 3.0       # seconds without response triggers alert
    
    def __init__(self, port=None):
        self.ser = None
        self.last_command_time = 0
        self.last_rx_time = time.time()
        self.connected = False
        
        ports = [port] if port else self.PORTS_TO_TRY
        
        for p in ports:
            try:
                print(f"Arduino: Trying {p}...")
                self.ser = serial.Serial(p, BAUD_RATE, timeout=0.1)
                time.sleep(2)  # Wait for Arduino reset
                self.connected = True
                print(f"✓ Arduino: Connected on {p}")
                break
            except Exception as e:
                continue
        
        if not self.connected:
            print(f"✗ Arduino: Failed to connect on any port")

    def send_command(self, steer_angle, throttle_pwm):
        """Send command with rate limiting and buffer management."""
        now = time.time()
        
        # Rate limit to prevent serial buffer overflow
        if now - self.last_command_time < self.COMMAND_RATE_LIMIT:
            return False
        
        if self.ser and self.ser.is_open:
            try:
                # Consume any pending debug output first
                while self.ser.in_waiting > 0:
                    self.last_rx_time = time.time()
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line and (line.startswith("DEBUG") or line.startswith("INIT")):
                        print(f"[ARDUINO] {line}")
                
                # Send command
                msg = f"<{int(steer_angle)},{int(throttle_pwm)}>"
                self.ser.write(msg.encode())
                self.last_command_time = now
                
                # Health check
                if now - self.last_rx_time > self.HEALTH_TIMEOUT:
                    if int(now) % 3 == 0:  # Print warning every ~3s
                        print("[ALERT] Arduino silent - possible freeze or disconnect!")
                
                return True
            except Exception as e:
                print(f"[!] Arduino write error: {e}")
                return False
        return False
    
    def close(self):
        """Safely close the connection."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"<90,1500>")  # Stop motors
                time.sleep(0.1)
                self.ser.close()
            except:
                pass

class GPSModule:
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, 9600, timeout=1)
        self.running = True

    def run(self):
        global current_lat, current_lon, current_heading
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8')
                if line.startswith('$GPRMC') or line.startswith('$GNGGA'):
                    msg = pynmea2.parse(line)
                    if hasattr(msg, 'latitude'):
                        current_lat = msg.latitude
                        current_lon = msg.longitude
                        # current_heading = msg.true_course if hasattr(msg, 'true_course') else 0
                        # Note: GPS heading only works when moving.
            except Exception:
                pass

def get_bearing(lat1, lon1, lat2, lon2):
    # Calculate bearing from p1 to p2
    dLon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    brng = math.atan2(y, x)
    return math.degrees(brng) # -180 to 180

def get_distance(lat1, lon1, lat2, lon2):
    # Haversine
    R = 6371000 # Meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(dlam/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def udp_listener():
    global target_lat, target_lon, target_updated
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for User GPS on UDP {UDP_PORT}...")
    while True:
        data, addr = sock.recvfrom(1024)
        try:
            # Expected Fmt: "LAT,LON"
            parts = data.decode().split(',')
            target_lat = float(parts[0])
            target_lon = float(parts[1])
            target_updated = time.time()
            print(f"Target Updated: {target_lat}, {target_lon}")
        except:
            pass

def calculate_avoidance(scan_data):
    """
    Simple Reactive Avoidance
    Returns: Steering Offset (-45 to 45)
    """
    if not scan_data:
        return 0
    
    left_score = 0
    right_score = 0
    
    # Check Frontal Cone (-45 to 45 deg)
    # Range limit: 1000mm (1m)
    for angle, dist in scan_data:
        if dist > 0 and dist < 1000:
            if -45 < angle < 0:
                left_score += 1
            elif 0 <= angle < 45:
                right_score += 1
                
    # If blocked on Left, Turn Right (Positive Offset)
    # If blocked on Right, Turn Left (Negative Offset)
    
    turn_cmd = 0
    if left_score > 5 or right_score > 5:
        if left_score > right_score:
            turn_cmd = 30 # Turn Right
        else:
            turn_cmd = -30 # Turn Left
            
    # Emergency Stop Check handled by Speed Logic? 
    # Or return a "STOP" flag. For now just Steering.
    return turn_cmd

# --- MAIN CONTROL LOOP ---
CONTROL_HZ = 10  # 10Hz main loop rate

def main():
    print("=" * 50)
    print("ROBOT MAIN - GPS Following Mode (Headless)")
    print("=" * 50)
    
    # Init Hardware (auto-detect Arduino port)
    arduino = ArduinoLink()
    lidar = LidarDriver(LIDAR_PORT)
    lidar.start()
    print(f"✓ Lidar: Started on {LIDAR_PORT}")
    
    # Start GPS Thread
    gps = GPSModule(GPS_PORT)
    gps_thread = threading.Thread(target=gps.run)
    gps_thread.daemon = True
    gps_thread.start()
    print(f"✓ GPS: Thread started on {GPS_PORT}")
    
    # Start Telemetry Listener (User Position)
    udp_thread = threading.Thread(target=udp_listener)
    udp_thread.daemon = True
    udp_thread.start()
    print(f"✓ UDP: Listening on port {UDP_PORT}")
    
    print("=" * 50)
    print("Running... Press Ctrl+C to stop")
    print("=" * 50)

    try:
        while True:
            loop_start = time.time()
            
            # 1. Check Safety / Target Freshness
            if time.time() - target_updated > 5:
                # Lost signal from user -> STOP
                arduino.send_command(90, 1500)
                # Rate-limited sleep
                elapsed = time.time() - loop_start
                if elapsed < (1.0 / CONTROL_HZ):
                    time.sleep((1.0 / CONTROL_HZ) - elapsed)
                continue

            # 2. Navigation Logic
            dist = get_distance(current_lat, current_lon, target_lat, target_lon)
            bearing = get_bearing(current_lat, current_lon, target_lat, target_lon)
            
            # Simple GPS Heading correction
            # Note: Requires calibration/compass for true reliability
            heading_error = bearing - current_heading
            # Normalize to -180 to 180
            heading_error = (heading_error + 180) % 360 - 180
            
            # 3. Obstacle Avoidance (Lidar Bubble)
            scan = lidar.get_scan()
            avoidance_turn = calculate_avoidance(scan)
            
            # 4. Mix
            # Add Avoidance Offset to Heading Error Steering
            # Priority to Avoidance: if avoidance_turn is large, it dominates
            steering_cmd = 90 + (heading_error * Kp_STEER) + avoidance_turn
            steering_cmd = max(45, min(135, steering_cmd)) # Clamp
            
            throttle_cmd = STOP_SPEED
            if dist > 3.0:
                throttle_cmd = BASE_SPEED # Go
            elif dist > 1.5:
                throttle_cmd = 1550 # Slow
            else:
                throttle_cmd = 1500 # Stop (Arrived)
                
            # Stop if very close obstacle in BOTH directions (Blind Alley)?
            # Implemented in Arduino Firmware as Ultrasonic interrupt, 
            # Can also add Lidar Stop here if needed.

            arduino.send_command(steering_cmd, throttle_cmd)
            
            # Rate limit main loop
            elapsed = time.time() - loop_start
            sleep_time = (1.0 / CONTROL_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
    finally:
        print("\n" + "=" * 50)
        print("SHUTTING DOWN...")
        arduino.close()
        print("✓ Arduino: Stopped")
        lidar.stop()
        print("✓ Lidar: Stopped")
        print("=" * 50)
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
