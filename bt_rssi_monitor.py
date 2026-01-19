"""
Bluetooth RSSI Monitor - Tracks signal strength of a target phone via BLE scanning.
Uses the Pi's built-in Bluetooth adapter.
"""

import subprocess
import threading
import time
import re


class BluetoothRSSIMonitor:
    def __init__(self, target_name="vivo Y36 5G", target_mac="74:33:57:43:18:7E"):
        """
        Initialize the Bluetooth RSSI monitor.
        
        Args:
            target_name: Bluetooth name of the target phone
            target_mac: MAC address (more reliable if known)
        """
        self.target_name = target_name.lower()
        self.target_mac = target_mac.upper() if target_mac else None
        self.current_rssi = None
        self.last_seen = 0
        self.running = False
        self._lock = threading.Lock()
        
    def start(self):
        """Start background BLE scanning thread."""
        self.running = True
        self.thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.thread.start()
        print(f"Bluetooth RSSI Monitor started. Looking for: {self.target_name}")
        
    def stop(self):
        """Stop the scanning thread."""
        self.running = False
        
    def _scan_loop(self):
        """Continuously scan for target device and update RSSI."""
        while self.running:
            try:
                # Method 1: Direct RSSI query (works if device was recently seen)
                if self.target_mac:
                    rssi = self._get_direct_rssi()
                    if rssi is not None:
                        self._update_rssi(rssi)
                        time.sleep(1)
                        continue
                
                # Method 2: Inquiry scan to find device
                self._run_inquiry()
                
            except Exception as e:
                print(f"BT scan error: {e}")
                
            time.sleep(2)  # Scan interval
    
    def _get_direct_rssi(self):
        """Get RSSI using hcitool rssi (requires recent connection)."""
        try:
            result = subprocess.run(
                ['sudo', 'hcitool', 'rssi', self.target_mac],
                capture_output=True,
                text=True,
                timeout=3
            )
            # Output: "RSSI return value: -45"
            match = re.search(r'RSSI return value:\s*(-?\d+)', result.stdout)
            if match:
                return int(match.group(1))
        except:
            pass
        return None
    
    def _run_inquiry(self):
        """Run Bluetooth inquiry to find devices with RSSI."""
        try:
            # Use hcitool inq with RSSI
            result = subprocess.run(
                ['sudo', 'hcitool', 'inq', '--flush'],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            # Check if target MAC is in results
            for line in result.stdout.split('\n'):
                if self.target_mac and self.target_mac in line.upper():
                    # Device found! Now try to get actual RSSI
                    rssi = self._get_direct_rssi()
                    if rssi is not None:
                        self._update_rssi(rssi)
                        return
                    # If direct RSSI fails, use a default "found" value
                    self._update_rssi(-50)  # Assume medium signal
                    return
                    
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            print(f"Inquiry error: {e}")
    
    def _update_rssi(self, rssi):
        """Thread-safe RSSI update."""
        with self._lock:
            # hcitool rssi returns values around 0 (+/- 10)
            # We convert to WiFi-like scale for consistency (-30 to -90 dBm)
            # Bluetooth Classic: 0 = optimal, positive = too strong, negative = weaker
            # Map: +10 -> -25, 0 -> -35, -10 -> -55, -20 -> -75
            wifi_like_rssi = -35 - (rssi * -2)
            wifi_like_rssi = max(-90, min(-25, wifi_like_rssi))  # Clamp
            
            self.current_rssi = wifi_like_rssi
            self.raw_rssi = rssi  # Keep original for debugging
            self.last_seen = time.time()
            
    def get_rssi(self):
        """
        Get the current RSSI value for the target phone.
        
        Returns:
            int: RSSI in dBm (WiFi-like scale), or None if phone not found recently
        """
        with self._lock:
            # Consider stale if not seen in 10 seconds
            if time.time() - self.last_seen > 10:
                return None
            return self.current_rssi
    
    def get_distance_estimate(self):
        """
        Estimate distance based on RSSI (rough approximation).
        
        Returns:
            float: Estimated distance in meters, or None
        """
        rssi = self.get_rssi()
        if rssi is None:
            return None
        
        # Simple path loss model: distance = 10 ^ ((TxPower - RSSI) / (10 * n))
        # TxPower ~= -35 dBm at 1m for Bluetooth
        # n = 2-4 for indoor environments
        tx_power = -35
        n = 2.5
        distance = 10 ** ((tx_power - rssi) / (10 * n))
        return round(distance, 2)
    
    def get_status(self):
        """Get detailed status info."""
        with self._lock:
            age = time.time() - self.last_seen if self.last_seen > 0 else None
            return {
                'rssi': self.current_rssi,
                'target_name': self.target_name,
                'target_mac': self.target_mac,
                'last_seen_age': age,
                'found': age is not None and age < 10
            }


# Test mode
if __name__ == "__main__":
    monitor = BluetoothRSSIMonitor(target_name="vivo Y36 5G")
    monitor.start()
    
    print("Scanning for phone... (Ctrl+C to stop)")
    print("Make sure phone Bluetooth is ON and discoverable!")
    print()
    
    try:
        while True:
            status = monitor.get_status()
            if status['found']:
                print(f"📱 Found! RSSI: {status['rssi']} dBm | MAC: {status['target_mac']} | Age: {status['last_seen_age']:.1f}s")
            else:
                print(f"🔍 Searching for '{status['target_name']}'...")
            time.sleep(2)
    except KeyboardInterrupt:
        monitor.stop()
        print("\nStopped.")
