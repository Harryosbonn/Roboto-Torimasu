#!/usr/bin/env python3
"""
Test script to find ESC reverse threshold.
"""
import serial
import time

def test_reverse():
    port = '/dev/ttyACM0'
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 115200, timeout=1)
    
    # Wait for Arduino reset
    print("Waiting for Arduino reset...")
    time.sleep(2)
    
    # Flush any startup messages
    while ser.in_waiting:
        ser.readline()
    
    print("\n=== Reverse Threshold Test ===")
    print("Testing reverse PWM values to find where motor starts moving...\n")
    
    # Test reverse values from 1400 down to 1200
    test_values = [1400, 1375, 1350, 1325, 1300, 1275, 1250]
    
    for pwm in test_values:
        cmd = f"<90,{pwm}>\n"
        print(f"Testing PWM={pwm}...", end=" ", flush=True)
        ser.write(cmd.encode())
        time.sleep(1.5)
        
        # Read response
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if 'Throttle' in line:
                print(f"(Arduino received)")
                break
        else:
            print("(sent)")
        
        input("Press Enter for next value (or Ctrl+C to stop)...")
    
    # Stop
    ser.write(b"<90,1500>\n")
    print("\nStopped. Test complete.")
    ser.close()

if __name__ == "__main__":
    test_reverse()
