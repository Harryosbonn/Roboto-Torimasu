import time
import serial

def main():
    print("--- YDLiDAR Command Test ---")
    
    port = '/dev/ttyUSB0'
    baudrate = 230400
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        ser.setDTR(True)
        print(f"Connected to {port}")
        
        time.sleep(0.5)
        
        # Try different start commands
        print("\n1. Sending START SCAN command (0xA5 0x60)...")
        ser.write(bytes([0xA5, 0x60]))
        time.sleep(1)
        
        # Read response
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"   Response: {data.hex()}")
        else:
            print("   No response")
        
        # Try another variant
        print("\n2. Trying alternative baud rates...")
        ser.close()
        
        for baud in [230400, 115200, 128000, 153600]:
            print(f"\n   Testing {baud} baud...")
            try:
                ser = serial.Serial(port, baud, timeout=1)
                ser.setDTR(True)
                time.sleep(0.3)
                
                # Send start command
                ser.write(bytes([0xA5, 0x60]))
                time.sleep(0.5)
                
                if ser.in_waiting:
                    data = ser.read(min(100, ser.in_waiting))
                    print(f"   >> GOT DATA at {baud}: {data[:20].hex()}")
                    break
                else:
                    print(f"   No data at {baud}")
                    
                ser.close()
            except Exception as e:
                print(f"   Error at {baud}: {e}")
        
        print("\nDone.")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
