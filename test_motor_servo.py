"""
Motor and Servo Test Diagnostic

Tests:
1. Arduino connectivity
2. Sending servo/motor commands
3. Verifying Arduino response
"""

import serial
import time
import sys

def drain_serial(ser, timeout=0.5):
    """Read all available serial data with error handling."""
    lines = []
    start = time.time()
    while time.time() - start < timeout:
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    lines.append(line)
                    print(f"    RX: {line}")
        except OSError:
            break
        time.sleep(0.02)
    return lines

def test_motor_servo():
    port = '/dev/ttyACM0'
    
    print("=" * 50)
    print("MOTOR/SERVO DIAGNOSTIC TEST")
    print("=" * 50)
    
    # Try to connect
    print(f"\n[1] Attempting to connect to {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"    ✓ Connection established on {port}")
    except Exception as e:
        print(f"    ✗ Connection FAILED: {e}")
        print("\n    >> Check if Arduino is plugged in")
        print("    >> Check if port is correct (try /dev/ttyACM1 or /dev/ttyUSB0)")
        return
    
    # Wait for Arduino to boot
    print("\n[2] Waiting for Arduino to boot (3 seconds)...")
    time.sleep(3)
    
    # Read startup messages
    print("\n[3] Reading Arduino boot messages...")
    boot_messages = drain_serial(ser, 2)
    
    if not boot_messages:
        print("    ✗ NO RESPONSE from Arduino!")
        print("\n    >> Possible causes:")
        print("       - Arduino firmware not uploaded")
        print("       - Arduino stuck/frozen in previous state")
        print("       - Wrong port (Lidar is also on USB)")
        print("       - Power issue")
        print("\n    >> Try: Unplug and replug Arduino, or reset button")
        ser.close()
        return
    
    ready_found = any("READY" in msg or "INIT" in msg for msg in boot_messages)
    if ready_found:
        print("    ✓ Arduino is responsive!")
    else:
        print("    ? Arduino responded but 'READY' not seen")
    
    # Test servo movement
    print("\n[4] Testing SERVO movement (left -> center -> right)...")
    servo_tests = [(60, "LEFT"), (90, "CENTER"), (120, "RIGHT"), (90, "CENTER")]
    
    for steer, name in servo_tests:
        cmd = f"<{steer},1500>"
        ser.write(cmd.encode())
        print(f"    Sending: {cmd} -> Servo {name}")
        time.sleep(0.5)
        
        # Read any response
        drain_serial(ser, 0.2)
    
    time.sleep(0.5)
    
    # Test motor (ESC) - Use slight throttle
    print("\n[5] Testing MOTOR (ESC) - Forward pulse...")
    print("    WARNING: Motor will try to spin briefly!")
    time.sleep(1)
    
    # Send forward throttle (above neutral 1500)
    # According to firmware, 1650-1850 should move
    motor_tests = [
        (1500, "NEUTRAL"),
        (1600, "SLIGHT FORWARD"),
        (1700, "MEDIUM FORWARD"),
        (1500, "NEUTRAL"),
    ]
    
    for throttle, name in motor_tests:
        cmd = f"<90,{throttle}>"
        ser.write(cmd.encode())
        print(f"    Sending: {cmd} -> Motor {name}")
        time.sleep(0.7)
        
        # Read responses
        drain_serial(ser, 0.2)
    
    # Final stop
    print("\n[6] Stopping motors...")
    ser.write(b"<90,1500>")
    time.sleep(0.3)
    
    # Read final debug
    print("\n[7] Final DEBUG output:")
    drain_serial(ser, 2)
    
    ser.close()
    print("\n" + "=" * 50)
    print("TEST COMPLETE")
    print("=" * 50)
    print("\nDid you see/hear the servo move? Did the motor try to spin?")
    print("If NO:")
    print("  - Check servo wiring to pin D9")
    print("  - Check ESC wiring to pin D10")
    print("  - Check ESC power (battery connected?)")
    print("  - ESC may need calibration")

if __name__ == "__main__":
    test_motor_servo()
