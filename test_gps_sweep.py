import serial
import time

def sweep_gps(port='/dev/ttyAMA0'):
    bauds = [9600, 4800, 19200, 38400, 57600, 115200]
    for baud in bauds:
        print(f"Testing {baud}...")
        try:
            ser = serial.Serial(port, baud, timeout=1)
            time.sleep(1)
            data = ser.read(100)
            if data:
                text = data.decode('ascii', errors='replace')
                print(f"  Received: {text[:50]}...")
                if '$G' in text or 'GPRMC' in text:
                    print(f"  >> SUCCESS at {baud}!")
                    ser.close()
                    return baud
            ser.close()
        except:
            pass
    print("Sweep complete. No clear NMEA data found.")
    return None

if __name__ == "__main__":
    sweep_gps()
