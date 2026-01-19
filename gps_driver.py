import serial
import threading
import time

class GPSDriver:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.latitude = 0.0
        self.longitude = 0.0
        self.num_sats = 0
        self.fix_quality = 0
        self.running = False
        self.connected = False
        self._lock = threading.Lock()
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.connected = True
        except Exception as e:
            print(f"GPS Connection Error: {e}")
            self.connected = False

    def start(self):
        if not self.connected:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                    parts = line.split(',')
                    if len(parts) > 9:
                        with self._lock:
                            # Parse Latitude
                            if parts[2]:
                                lat_raw = float(parts[2])
                                lat_deg = int(lat_raw / 100)
                                lat_min = lat_raw - (lat_deg * 100)
                                self.latitude = lat_deg + (lat_min / 60)
                                if parts[3] == 'S': self.latitude *= -1
                                
                            # Parse Longitude
                            if parts[4]:
                                lon_raw = float(parts[4])
                                lon_deg = int(lon_raw / 100)
                                lon_min = lon_raw - (lon_deg * 100)
                                self.longitude = lon_deg + (lon_min / 60)
                                if parts[5] == 'W': self.longitude *= -1
                                
                            self.fix_quality = int(parts[6]) if parts[6] else 0
                            self.num_sats = int(parts[7]) if parts[7] else 0
            except:
                pass
            time.sleep(0.1)

    def get_data(self):
        with self._lock:
            return {
                "lat": self.latitude,
                "lon": self.longitude,
                "sats": self.num_sats,
                "fix": self.fix_quality
            }

    def stop(self):
        self.running = False
        if self.connected:
            self.ser.close()
