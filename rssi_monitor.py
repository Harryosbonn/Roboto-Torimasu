"""
RSSI Monitor - Tracks signal strength of the beacon phone.
Uses Bluetooth RSSI as primary method (works independently of WiFi mode).
Falls back to WiFi RSSI if Bluetooth unavailable.
"""

import subprocess
import re
import os

# Import Bluetooth RSSI monitor
try:
    from bt_rssi_monitor import BluetoothRSSIMonitor
    BT_AVAILABLE = True
except ImportError:
    BT_AVAILABLE = False
    print("Warning: bt_rssi_monitor not available, using WiFi only")

class RSSIMonitor:
    def __init__(self, interface='wlan0', phone_name="vivo Y36 5G"):
        self.interface = interface
        self.bt_monitor = None
        
        # Initialize Bluetooth RSSI if available
        if BT_AVAILABLE:
            try:
                self.bt_monitor = BluetoothRSSIMonitor(target_name=phone_name)
                self.bt_monitor.start()
                print(f"Bluetooth RSSI Monitor initialized for: {phone_name}")
            except Exception as e:
                print(f"Bluetooth RSSI init failed: {e}")

    def get_mac_from_ip(self, ip_address):
        """Finds the MAC address for a given IP in the ARP table."""
        try:
            # Run arp command
            output = subprocess.check_output(['arp', '-n', ip_address], stderr=subprocess.STDOUT).decode()
            # Match MAC address pattern
            match = re.search(r'([0-9a-fA-F]{2}[:-]){5}([0-9a-fA-F]{2})', output)
            if match:
                return match.group(0)
        except Exception as e:
            print(f"ARP look up failed for {ip_address}: {e}")
        return None

    def get_rssi(self, ip_address=None):
        """Gets the RSSI for the beacon phone. Tries Bluetooth first, then WiFi."""
        
        # Try Bluetooth RSSI first (works regardless of WiFi mode)
        if self.bt_monitor:
            bt_rssi = self.bt_monitor.get_rssi()
            if bt_rssi is not None:
                return bt_rssi
        
        # Fall back to WiFi RSSI (only works when Pi is hotspot)
        try:
            # Use station dump for maximum compatibility
            output = subprocess.check_output(['sudo', 'iw', 'dev', self.interface, 'station', 'dump'], stderr=subprocess.STDOUT).decode()
            # Split by station blocks
            stations = [s for s in output.split('Station ') if s.strip()]
            
            # If only one station is connected, it's almost certainly the phone beacon
            if len(stations) == 1:
                sig_match = re.search(r'signal:\s+(-?\d+)\s+dBm', stations[0])
                if sig_match:
                    return int(sig_match.group(1))
            
            # If multiple, try to match by MAC from IP
            if ip_address:
                mac = self.get_mac_from_ip(ip_address)
                if mac:
                    for station in stations:
                        if mac.lower() in station.lower():
                            sig_match = re.search(r'signal:\s+(-?\d+)\s+dBm', station)
                            if sig_match:
                                return int(sig_match.group(1))
        except Exception as e:
            pass  # WiFi RSSI not available (not in hotspot mode)
        
        return None

if __name__ == "__main__":
    # Test with the known phone IP
    monitor = RSSIMonitor()
    target_ip = '10.86.157.251'
    print(f"Monitoring RSSI for {target_ip}...")
    rssi = monitor.get_rssi(target_ip)
    if rssi:
        print(f"Current RSSI: {rssi} dBm")
    else:
        print("Could not retrieve RSSI. Ensure the Pi is the Hotspot.")
