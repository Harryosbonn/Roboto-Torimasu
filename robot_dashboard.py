import os
import time
import json
import threading
import serial
from flask import Flask, render_template, request, jsonify, Response

# Import our drivers
from lidar_driver import LidarDriver
from gps_driver import GPSDriver

from rssi_monitor import RSSIMonitor

from nav_brain import NavigationBrain
from vision_driver import VisionDriver

app = Flask(__name__)

# --- BEACON STATE ---
# ... (lines 17-27)
# --- BEACON STATE ---
beacon_data = {
    "lat": 0,
    "lon": 0,
    "accuracy": 999,
    "rssi": 0,
    "valid": False,
    "last_update": 0
}
beacon_lock = threading.Lock()
rssi_monitor = RSSIMonitor()

# --- HARDWARE STATE ---
class RobotState:
    def __init__(self):
        self.arduino_port = '/dev/ttyACM0'
        self.arduino_baud = 115200
        self.ser = None
        self.lidar = LidarDriver(port='/dev/ttyUSB0', baudrate=230400)
        self.gps = GPSDriver(port='/dev/ttyAMA0', baudrate=115200)
        self.vision = VisionDriver()
        self.nav = NavigationBrain()
        
        # State
        self.auto_nav = False
        self.nav_status = "OFF"
        self.manual_steer = 90
        self.manual_throttle = 1500
        self.last_cmd_time = 0
        self.ser_lock = threading.Lock()
        
        # Telemetry
        self.telemetry = {
            "pitch": 0,
            "roll": 0,
            "dist": 999,
            "tipped": 0,
            "blocked": 0,
            "timeout": 0,
            "steer": 90,
            "throttle": 1500,
            "gps": {"lat": 0, "lon": 0, "sats": 0, "fix": 0},
            "nav_mode": "OFF"
        }
        
        self.connect_arduino()
        self.lidar.start()
        self.gps.start()
        self.vision.start()
        
        # Monitor Thread
        self.thread = threading.Thread(target=self._update_loop)
        self.thread.daemon = True
        self.thread.start()

    def connect_arduino(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.arduino_baud, timeout=0.1)
            print(f"SUCCESS: Dashboard linked to Arduino on {self.arduino_port}")
            print("Waiting for Arduino to initialize after reset...")
            time.sleep(2)  # Wait for Arduino to reset and init after opening serial
        except Exception as e:
            print(f"CRITICAL ERROR: Arduino Link Failed on {self.arduino_port}: {e}")
            # Try alternative port
            try:
                self.arduino_port = '/dev/ttyUSB1' # Common alternative
                self.ser = serial.Serial(self.arduino_port, self.arduino_baud, timeout=0.1)
                print(f"SUCCESS: Dashboard linked to Arduino on {self.arduino_port}")
            except:
                print("FATAL: All Arduino port attempts failed.")

        # Start dedicated serial read thread
        if self.ser:
            self.serial_thread = threading.Thread(target=self._serial_read_loop)
            self.serial_thread.daemon = True
            self.serial_thread.start()

    def _serial_read_loop(self):
        """Continuously read Arduino serial in a dedicated thread."""
        serial_buffer = ""
        print("SERIAL THREAD: Started")
        while True:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    serial_buffer += chunk
                    
                    # Process complete lines
                    while '\n' in serial_buffer:
                        line, serial_buffer = serial_buffer.split('\n', 1)
                        line = line.strip()
                        if "DEBUG |" in line:
                            parts = line.split('|')
                            for p in parts:
                                if ':' in p:
                                    try:
                                        key, val = p.split(':', 1)
                                        k = key.strip().lower()
                                        v = val.strip()
                                        if k in self.telemetry:
                                            self.telemetry[k] = float(v)
                                    except (ValueError, TypeError):
                                        continue
                    
                    # Prevent buffer from growing too large
                    if len(serial_buffer) > 1000:
                        serial_buffer = serial_buffer[-500:]
            except Exception as e:
                print(f"Serial Read Error: {e}")
                serial_buffer = ""
            time.sleep(0.01)

    def _update_loop(self):
        last_nav_time = 0
        while True:
            # 1. Update GPS
            gps_data = self.gps.get_data()
            if gps_data:
                self.telemetry["gps"] = gps_data
            
            # 2. Update Lidar (driver returns mm, we display meters)
            scan = self.lidar.get_scan()
            if scan:
                # Filter valid readings: > 50mm and < 10000mm (10 meters), ignore noise
                valid_dists = [d for a, d in scan if 50 < d < 10000]
                if valid_dists:
                    self.telemetry["lidar_dist"] = min(valid_dists) / 1000.0  # Convert mm to m
            
            # 3. Auto-Navigation & Tracking Logic
            now = time.time()
            if now - last_nav_time > 0.1:
                last_nav_time = now
                
                # Active RSSI polling (even if no beacon POSTs)
                # This ensures we have Bluetooth signal strength
                if rssi_monitor:
                    freshest_rssi = rssi_monitor.get_rssi()
                    if freshest_rssi:
                        with beacon_lock:
                            beacon_data["rssi"] = freshest_rssi
                            beacon_data["last_update"] = time.time()  # active update

                with beacon_lock:
                    b_data = dict(beacon_data)
                    # Use actual age relative to now
                    b_data["age"] = time.time() - beacon_data.get("last_update", 0)

                # DEBUG RSSI
                rssi_val = b_data.get("rssi", -100)
                # print(f"DEBUG: RSSI passed to nav: {rssi_val}")

                # ALWAYS compute tracking target (for UI)
                vision_data = self.vision.get_latest_detection()
                
                # Inject lidar distance into vision for overlay
                if vision_data:
                    # Lidar angle 0 = front, vision bearing 0 = center
                    # Use wider tolerance (25 deg) for better matching
                    lidar_dist = self.nav._get_lidar_distance_at_angle(scan, vision_data.get('bearing', 0), tolerance=25.0)
                    self.vision.set_lidar_distance(lidar_dist)
                    # DEBUG
                    if lidar_dist:
                        print(f"FUSION: Vision@{vision_data.get('bearing', 0):.1f}deg -> Lidar={lidar_dist:.2f}m")
                
                steer, throttle, mode = self.nav.compute(
                    self.telemetry["gps"],
                    b_data,
                    scan,
                    rssi_val,
                    vision_data
                )

                # Only drive if auto-nav enabled
                if self.auto_nav:
                    self.send_command(steer, throttle, is_auto=True)
                    self.nav_status = mode
                else:
                    self.nav_status = "OFF"

            # Store fused target info for UI
            self.telemetry["tracking_target"] = {
                "bearing": self.nav.target_bearing,
                "distance": self.nav.target_distance,
                "source": self.nav.target_source
            }
            # print(f"DEBUG: Target: {self.telemetry['tracking_target']}")
            
            time.sleep(0.01)

    def send_command(self, steer, throttle, is_auto=False):
        if not is_auto:
            self.manual_steer = steer
            self.manual_throttle = throttle
        
        # Send command directly - no rate limiting (handled by heartbeat)
        if self.ser:
            with self.ser_lock:
                msg = f"<{steer},{throttle}>\n"
                self.ser.write(msg.encode())
                # DEBUG: Show what's being sent
                print(f"SERIAL TX: {msg.strip()}")

robot = RobotState()

# --- WEB ROUTES ---
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/beacon')
def beacon_page():
    return render_template('beacon.html')

@app.route('/api/telemetry')
def get_telemetry():
    data = dict(robot.telemetry)
    data["scan"] = robot.lidar.get_scan()
    # Include beacon info
    with beacon_lock:
        data["beacon"] = dict(beacon_data)
        data["beacon"]["age"] = time.time() - beacon_data["last_update"]
    return jsonify(data)

@app.route('/api/beacon', methods=['POST'])
def receive_beacon():
    global beacon_data
    data = request.json
    client_ip = request.remote_addr
    
    # Try to get RSSI for this client
    rssi = rssi_monitor.get_rssi(client_ip)
    print(f"DEBUG: RSSI from monitor: {rssi}")
    
    with beacon_lock:
        beacon_data["lat"] = data.get("lat", 0)
        beacon_data["lon"] = data.get("lon", 0)
        beacon_data["accuracy"] = data.get("accuracy", 999)
        beacon_data["rssi"] = rssi if rssi else 0
        beacon_data["valid"] = data.get("valid", False)
        beacon_data["last_update"] = time.time()
    return jsonify({"status": "ok", "rssi": rssi if rssi else 0})

def generate_frames():
    while True:
        frame_bytes = robot.vision.get_frame_jpeg()
        if frame_bytes:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/control', methods=['POST'])
def control():
    data = request.json
    steer = data.get('steer', 90)
    throttle = data.get('throttle', 1500)
    # Only allow manual override if auto_nav is off
    print(f"CONTROL: steer={steer}, throttle={throttle}, auto_nav={robot.auto_nav}", flush=True)
    if not robot.auto_nav:
        robot.send_command(steer, throttle)
    return jsonify({"status": "ok"})

@app.route('/api/nav/start', methods=['POST'])
def nav_start():
    robot.auto_nav = True
    return jsonify({"status": "ok"})

@app.route('/api/nav/stop', methods=['POST'])
def nav_stop():
    robot.auto_nav = False
    robot.send_command(90, 1500) # Safety stop
    return jsonify({"status": "ok"})

if __name__ == '__main__':
    os.makedirs('templates', exist_ok=True)
    # Use HTTPS for phone GPS access (browsers require secure origins)
    ssl_context = ('cert.pem', 'key.pem')
    print("Dashboard running on https://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, ssl_context=ssl_context)
