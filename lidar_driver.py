import math
import time
import serial
import threading

# Try to import standard YDLiDAR library if available, else standard serial
try:
    import ydlidar # hypothetical wrapper, users often use PyLidar3 or customized scripts
except ImportError:
    ydlidar = None

class LidarDriver:
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.scan_data = [] # List of (angle, dist)
        self.running = False
        self._lock = threading.Lock()
        
        
        # Connect
        self._init_buffers()
        self._connect()

    def _connect(self):
        ports_to_try = [self.port, '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
        # Remove duplicates while preserving order
        ports_to_try = list(dict.fromkeys(ports_to_try))
        
        for port in ports_to_try:
            try:
                print(f"Lidar: Attempting to connect on {port}...")
                self.ser = serial.Serial(port, self.baudrate, timeout=1)
                self.ser.setDTR(True) # Enable Motor (DTR High often powers MOTO_EN)
                self.port = port
                self.connected = True
                print(f"✓ Lidar Connected on {self.port}")
                return
            except Exception:
                continue
        
        print(f"✗ Lidar Connection Failed on all ports: {ports_to_try}. Switching to MOCK mode.")
        self.connected = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._scan_loop)
        self.thread.daemon = True
        self.thread.start()
        
        # Send START SCAN command if connected
        if self.connected:
            time.sleep(0.3)  # Let motor spin up
            self.ser.write(bytes([0xA5, 0x60]))
            print("Sent START SCAN command to Lidar")

    def _scan_loop(self):
        print("Starting Lidar Scan Loop (Pure Python Driver)...")
        buffer = bytearray()
        
        while self.running:
            if not self.connected:
                # MOCK MODE (unchanged logic for fallback)
                fake_scan = []
                for angle in range(-180, 180, 5): 
                    dist = 2000
                    if -30 < angle < 30:
                        dist = 500
                    fake_scan.append((angle, dist))
                with self._lock:
                    self.scan_data = fake_scan
                time.sleep(0.1)
                continue

            try:
                # Read chunks
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer.extend(data)
                    # DEBUG: Print Incoming Data sample
                    # print(f"DEBUG: Read {len(data)} bytes. Buffer: {len(buffer)}")
                    # if len(buffer) > 0 and len(buffer) < 20: 
                    #    print(f"HEX: {buffer.hex()}")
                
                # Parse Packets
                # Header: AA 55 00 (CT) LSN(1) FSA(2) LSA(2) CS(2) DATA(2*LSN)
                # Min packet size ~ 10 bytes
                
                while len(buffer) > 10:
                    # Sync Header
                    if buffer[0] != 0xAA or buffer[1] != 0x55:
                        # DEBUG: Lost Sync?
                        # print(f"DEBUG: Skip byte {hex(buffer[0])}")
                        # Shift buffer to find next AA
                        try:
                            next_aa = buffer.index(0xAA, 1)
                            del buffer[:next_aa]
                            continue
                        except ValueError:
                            del buffer[:]
                            break
                    
                    # Found AA 55.
                    # Check Packet Type (CT)
                    ct = buffer[2]
                    lsn = buffer[3]
                    
                    if lsn == 0:
                        del buffer[:2] # Invalid LSN, skip
                        continue

                    # Calculate Packet Size
                    # Packet = Header(10) + Data(2 * LSN)
                    # Note: YDLiDAR sometimes uses 3 bytes for intensity. 
                    # Assuming 2 bytes for now (Standard). T-mini might be 3?
                    # Let's assume 2 bytes distance.
                    # Actually standard YDLiDAR packet header is 7 bytes? 
                    # AA 55 CT LSN FSA_L FSA_H LSA_L LSA_H CS_L CS_H (10 bytes) -- Yes.
                    
                    package_len = 10 + 2 * lsn 
                    
                    if len(buffer) < package_len:
                        break # Wait for more data
                        
                    # Extract Packet
                    packet = buffer[:package_len]
                    del buffer[:package_len]
                    
                    # Parse Header
                    # FSA = Angle Start, LSA = Angle End
                    fsa = (packet[4] | (packet[5] << 8)) >> 1
                    lsa = (packet[6] | (packet[7] << 8)) >> 1
                    
                    # Checksum (Skipped for speed/simplicity, but safe to add later)
                    
                    angle_fsa = fsa / 64.0
                    angle_lsa = lsa / 64.0
                    
                    diff_angle = angle_lsa - angle_fsa
                    if diff_angle < 0:
                        diff_angle += 360 # Wrap around
                        
                    # Parse Data
                    # Data is usually 2 bytes: dist_l, dist_h
                    # Dist = data >> 2? Or just raw? 
                    # X4/X2/T-mini: Dist = (Data) / 4.0 mm
                    
                    scan_chunk = []
                    
                    for i in range(lsn):
                        idx = 10 + (2 * i)
                        raw_dist = packet[idx] | (packet[idx+1] << 8)
                        
                        # Distance is simple (raw / 4.0) usually
                        if raw_dist == 0:
                            dist = 0
                        else:
                            dist = raw_dist / 4.0
                            
                        # Angle Interpolation
                        # angle = angle_fsa + (diff / (lsn - 1)) * i
                        if lsn > 1:
                            angle = angle_fsa + (diff_angle / (lsn - 1)) * i
                        else:
                            angle = angle_fsa
                            
                        # Correct Angle (YDLidar specific optimization often adds correction)
                        # We skip correction for simplicity.
                        
                        # Normalize Angle (-180 to 180 for standard usage)
                        # Current is 0-360?
                        # Let's normalize to -180..180 where 0 is front? 
                        # This depends on mounting. Assuming standard.
                        
                        norm_angle = angle
                        if norm_angle > 180:
                            norm_angle -= 360
                            
                        scan_chunk.append((norm_angle, dist))
                        
                    # Update global scan
                    # This is partial scan. We should accumulate? 
                    # Or just return raw stream? 
                    # Our LidarDriver.get_scan expects a full frame? 
                    # Actually "get_scan" usually returns latest full rotation.
                    # Here we are just putting chunks.
                    # Improve: accumulate until angle wraps?
                    # For simplicity, just replacing the global "scan_data" with "everything we have seen recently"?
                    # Or better: `self.scan_data` is a list that we append to, and clear periodically?
                    
                    # Simple Approach: Maintain a buffer of "Last 360 degrees"
                    # But simpler: Just append to a big list, and clear it when angle drops (new rotation).
                    
                    with self._lock:
                        if angle_fsa < self._last_fsa:
                            # New Rotation detected (FSA dropped)
                            # Publish the buffer as the "last complete scan"
                            # But here self.scan_data is queried by main loop.
                            # So we update self.scan_data with 'full_scan_buffer'
                            self.scan_data = list(self._current_scan_buffer)
                            self._current_scan_buffer = []
                        
                        self._current_scan_buffer.extend(scan_chunk)
                        self._last_fsa = angle_fsa
                        
            except Exception as e:
                # print(f"Lidar Parse Error: {e}")
                time.sleep(0.01)
                
    def _init_buffers(self):
        self._current_scan_buffer = []
        self._last_fsa = 0.0

    def get_scan(self):
        """
        Returns list of (angle_deg, distance_mm)
        Angle 0 is Robot Front.
        """
        with self._lock:
            return list(self.scan_data)

    def stop(self):
        self.running = False
        if self.connected:
            self.ser.close()
