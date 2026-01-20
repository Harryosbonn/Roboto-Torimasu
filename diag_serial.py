import serial
import time

def test_serial():
    ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']
    
    for port in ports:
        print(f"Trying {port}...")
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"Connected to {port}!")
            
            # Reset Arduino (DTR toggle)
            ser.setDTR(False)
            time.sleep(0.1)
            ser.setDTR(True)
            print("Asserted DTR to reset...")
            
            start = time.time()
            data_count = 0
            
            while time.time() - start < 10:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"RX: {line}")
                        data_count += 1
                        if "DEBUG" in line:
                             print(">> VALID TELEMETRY DETECTED")
                time.sleep(0.01)
            
            print(f"Finished. Received {data_count} lines.")
            ser.close()
            if data_count > 0:
                return # Found it
                
        except serial.SerialException as e:
            print(f"Failed to open {port}: {e}")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    test_serial()
