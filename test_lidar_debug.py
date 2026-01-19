import time
from lidar_driver import LidarDriver

def main():
    print("--- Lidar DEBUG Script ---")
    print("Attempting to connect to YDLiDAR T-mini Plus...")
    
    lidar = LidarDriver(port='/dev/ttyUSB0', baudrate=115200) 
    
    # Wait a moment for motor to spin up
    time.sleep(1)
    
    if lidar.connected:
        print(">> Hardware connected. Starting Lidar...")
        lidar.start()
        time.sleep(2) # Give it time to spin up and start sending
        
        # Read some raw bytes to see what's coming
        print("Reading raw bytes for 5 seconds...")
        start_time = time.time()
        total_bytes = 0
        
        while time.time() - start_time < 5:
            if lidar.ser.in_waiting:
                data = lidar.ser.read(min(100, lidar.ser.in_waiting))
                total_bytes += len(data)
                print(f"Received {len(data)} bytes. Sample hex: {data[:20].hex()}")
            time.sleep(0.1)
        
        print(f"\nTotal bytes received in 5 seconds: {total_bytes}")
        
        if total_bytes == 0:
            print("\n>> ISSUE: No data received even after START command!")
            print("Possible causes:")
            print("  1. Lidar motor not spinning (check power/DTR)")
            print("  2. Wrong baud rate (currently 230400)")
            print("  3. Wrong USB port")
        else:
            print("\n>> Data is flowing! Fetching processed scans...")
            
            for i in range(5):
                scan = lidar.get_scan()
                print(f"Frame {i+1}: {len(scan)} points")
                time.sleep(0.5)
            
            lidar.stop()
    else:
        print(">> WARNING: Could not connect")
    
    print("Done.")

if __name__ == "__main__":
    main()
