from rssi_monitor import RSSIMonitor
import time

monitor = RSSIMonitor()
ip = '10.86.157.251'
print(f"Testing RSSI for {ip}...")
for _ in range(3):
    rssi = monitor.get_rssi(ip)
    print(f"RSSI: {rssi}")
    time.sleep(1)
