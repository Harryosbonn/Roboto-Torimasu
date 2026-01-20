#!/usr/bin/env python3
"""Quick ESC reverse test - sends reverse command and reads what Arduino receives."""
import serial
import time

port = '/dev/ttyACM0'
print(f"Connecting to {port}...")
ser = serial.Serial(port, 115200, timeout=1)

print("Waiting for Arduino reset...")
time.sleep(2)

# Flush
while ser.in_waiting:
    print(f"INIT: {ser.readline().decode('utf-8', errors='ignore').strip()}")

print("\n=== Testing Reverse Command ===")

# Send reverse command
pwm = 1300  # Reverse
cmd = f"<90,{pwm}>\n"
print(f"Sending: {cmd.strip()}")
ser.write(cmd.encode())

# Read responses for 3 seconds
start = time.time()
while time.time() - start < 3:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if 'Throttle' in line:
            print(f"RX: {line}")
    time.sleep(0.1)

# Stop
ser.write(b"<90,1500>\n")
print("\nStopped.")
ser.close()
