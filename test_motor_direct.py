#!/usr/bin/env python3
"""
Test script to verify Arduino motor control.
Sends steering and throttle commands directly via serial.
"""
import serial
import time

def test_motor():
    port = '/dev/ttyACM0'
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 115200, timeout=1)
    
    # Wait for Arduino reset
    print("Waiting for Arduino reset...")
    time.sleep(2)
    
    # Flush any startup messages
    while ser.in_waiting:
        print(f"RX: {ser.readline().decode('utf-8', errors='ignore').strip()}")
    
    print("\n=== Motor Control Test ===")
    
    # Test commands: (steer, throttle, description)
    tests = [
        (90, 1500, "CENTER / STOP"),
        (60, 1500, "LEFT / STOP"),
        (120, 1500, "RIGHT / STOP"),
        (90, 1600, "CENTER / FORWARD SLOW"),
        (90, 1500, "CENTER / STOP"),
    ]
    
    for steer, throttle, desc in tests:
        cmd = f"<{steer},{throttle}>\n"
        print(f"\nSending: {cmd.strip()} ({desc})")
        ser.write(cmd.encode())
        time.sleep(0.5)
        
        # Read any response
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  RX: {line}")
        
        # Wait for user to observe
        time.sleep(1.5)
    
    print("\n=== Test Complete ===")
    print("Did you see the servo move? (Check physically)")
    ser.close()

if __name__ == "__main__":
    test_motor()
