"""
Beacon Receiver - Receives UDP packets from the ESP32 GPS beacon
and provides the latest beacon position to the navigator.
"""

import socket
import json
import threading
import time

class BeaconReceiver:
    def __init__(self, port=5005):
        self.port = port
        self.running = False
        self.latest_data = None
        self.last_update = 0
        self._lock = threading.Lock()
        
    def start(self):
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(1.0)
        
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()
        print(f"Beacon receiver listening on port {self.port}")
        
    def _run(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                packet = json.loads(data.decode())
                
                if packet.get('type') == 'beacon':
                    with self._lock:
                        packet['ip'] = addr[0]
                        self.latest_data = packet
                        self.last_update = time.time()
                    # print(f"Beacon: {packet}")
                    
            except socket.timeout:
                pass
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"Beacon error: {e}")
                
    def get_position(self):
        """Returns (lat, lon, valid, age_seconds) or None if no data"""
        with self._lock:
            if self.latest_data is None:
                return None
            age = time.time() - self.last_update
            return {
                'lat': self.latest_data.get('lat', 0),
                'lon': self.latest_data.get('lon', 0),
                'alt': self.latest_data.get('alt', 0),
                'sats': self.latest_data.get('sats', 0),
                'ip': self.latest_data.get('ip', None),
                'valid': self.latest_data.get('valid', False),
                'age': age
            }
    
    def stop(self):
        self.running = False
        self.sock.close()


# Test mode
if __name__ == "__main__":
    receiver = BeaconReceiver()
    receiver.start()
    
    print("Waiting for beacon packets... (Ctrl+C to stop)")
    try:
        while True:
            pos = receiver.get_position()
            if pos:
                print(f"Beacon: {pos['lat']:.6f}, {pos['lon']:.6f} | Sats: {pos['sats']} | Age: {pos['age']:.1f}s")
            else:
                print("No beacon data yet...")
            time.sleep(1)
    except KeyboardInterrupt:
        receiver.stop()
        print("\nStopped.")
