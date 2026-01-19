import serial
import time

def main():
    port = '/dev/ttyACM0'
    baud = 115200
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2) # Wait for reset
        
        print("SAFETY CHECK: Wheels must be OFF THE GROUND.")
        print("Testing SLOW FORWARD (1550) for 2 seconds...")
        ser.write(b"<90,1550>")
        time.sleep(2)
        
        print("Stopping...")
        ser.write(b"<90,1500>")
        time.sleep(1)
        
        print("Testing SLOW REVERSE (1450) for 2 seconds...")
        ser.write(b"<90,1450>")
        time.sleep(2)
        
        print("Stopping...")
        ser.write(b"<90,1500>")
        time.sleep(1)
        
        ser.close()
        print("Throttle test complete.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
