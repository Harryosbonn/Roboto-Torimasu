import serial
import time

def check_gps_health(port='/dev/ttyAMA0', baud=9600):
    print(f"Opening {port} at {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        start_time = time.time()
        print("Listening for 10 seconds. Wiggle the wires now to check for stability...")
        
        while time.time() - start_time < 10:
            if ser.in_waiting:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$'):
                    print(f"[RECV] {line}")
                    if "GNGGA" in line or "GPGGA" in line:
                        parts = line.split(',')
                        if len(parts) > 6:
                            fix_quality = parts[6]
                            sats = parts[7]
                            print(f" >>> STATS: Fix={fix_quality}, Sats={sats}")
            else:
                # If nothing in waiting, might be disconnected
                pass
                
        ser.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # The user earlier mentioned 115200 was working or at least tested. 
    # But Neo-6M is usually 9600 by default. 
    # Let's try 9600 first as it's the standard for these modules.
    check_gps_health(baud=9600)
