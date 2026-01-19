import serial
import time

def main():
    port = '/dev/ttyACM0'
    baud = 115200
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2) # Wait for reset
        print("Moving steering to 60 (Left)...")
        ser.write(b"<60,1500>")
        time.sleep(1)
        print("Moving steering to 120 (Right)...")
        ser.write(b"<120,1500>")
        time.sleep(1)
        print("Centering steering (90)...")
        ser.write(b"<90,1500>")
        time.sleep(1)
        ser.close()
        print("Steering test complete.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
