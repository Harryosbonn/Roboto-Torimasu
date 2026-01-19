import time
from lidar_driver import LidarDriver

def main():
    print("--- Lidar Test Script ---")
    print("Attempting to connect to YDLiDAR T-mini Plus...")
    
    # Initialize Driver (Try USB0 or USB1 if default fails)
    # You might need to change '/dev/ttyUSB1' to '/dev/ttyUSB0' depending on what else is plugged in.
    lidar = LidarDriver(port='/dev/ttyUSB0') 
    
    lidar.start()
    
    # Allow some time for spin-up
    time.sleep(2)
    
    if lidar.connected:
        print(">> SUCCESS: Hardware Lidar connected!")
    else:
        print(">> WARNING: Running in MOCK mode (Hardware not found).")
    
    print("Reading 10 frames...")
    try:
        for i in range(10):
            scan = lidar.get_scan()
            print(f"Frame {i+1}: {len(scan)} points detected.")
            
            # Print a sample point (e.g., straight ahead 0 degrees)
            # Find closest point to 0 deg
            front_dist = "N/A"
            for angle, dist in scan:
                if -5 < angle < 5:
                    front_dist = f"{dist} mm"
                    break
            
            print(f"  -> Front Distance: {front_dist}")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping Lidar...")
        lidar.stop()
        print("Done.")

if __name__ == "__main__":
    main()
