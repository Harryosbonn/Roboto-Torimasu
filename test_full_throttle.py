import serial
import time

def run_full_throttle():
    port = '/dev/ttyACM0'
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Connected to {port}")
        time.sleep(2) # Wait for reset
        
        print("Sending FULL THROTTLE (2000) for 3 seconds...")
        start_time = time.time()
        while time.time() - start_time < 3:
            ser.write(b"<90,2000>")
            time.sleep(0.05) # 20Hz
            
        print("Stopping...")
        ser.write(b"<90,1500>")
        time.sleep(0.5)
        ser.close()
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    run_full_throttle()
