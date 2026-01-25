import serial
import time

def test_l298n():
    port = '/dev/ttyACM0'
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        print(f"Connected to {port}")
        time.sleep(2)
        # Ensure stop
        ser.write(b"<90,1500>")
        time.sleep(0.5)
        print("Testing forward (1800)...")
        ser.write(b"<90,1800>")
        time.sleep(3)
        print("Testing reverse (1200)...")
        ser.write(b"<90,1200>")
        time.sleep(3)
        ser.write(b"<90,1500>")
        time.sleep(0.5)
        ser.close()
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    test_l298n()
