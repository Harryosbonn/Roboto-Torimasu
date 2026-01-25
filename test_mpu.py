import serial
import time

def test_mpu():
    port = '/dev/ttyACM0'
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        print(f"Connected to {port}")
        time.sleep(2)
        # Flush any existing data
        ser.read_all()
        # Request data by sending a neutral command repeatedly to keep Arduino alive
        ser.write(b"<90,1500>")
        start = time.time()
        while time.time() - start < 5:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('DEBUG'):
                    print(line)
            time.sleep(0.1)
        ser.close()
        print("Done.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    test_mpu()
