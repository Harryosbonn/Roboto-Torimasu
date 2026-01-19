import serial
import time

def test_gps(port='/dev/serial0', baud=9600):
    print(f"Opening {port} at {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        start_time = time.time()
        while time.time() - start_time < 10:
            line = ser.readline()
            if line:
                print(f"Data: {line.decode('ascii', errors='replace').strip()}")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    import sys
    p = sys.argv[1] if len(sys.argv) > 1 else '/dev/serial0'
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    test_gps(port=p, baud=b)
