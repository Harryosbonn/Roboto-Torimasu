#!/usr/bin/env python3
import serial
import time
import sys

# Configuration
ARDUINO_PORT = '/dev/ttyACM0'  # Might need to change to /dev/ttyUSB0
BAUD_RATE = 115200

def main():
    print("--- Arduino Actuator Test ---")
    print(f"Connecting to Arduino on {ARDUINO_PORT}...")
    
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino reset
        print("Connected!")
    except Exception as e:
        print(f"Error: {e}")
        print("\nTIP: Run 'ls /dev/tty*' to find your Arduino port.")
        sys.exit(1)

    print("\nSAFETY NOTICE: Keep wheels off the ground!")
    print("Commands:")
    print("  s <angle> : Set steering (45 to 135, 90 is center)")
    print("  t <pwm>   : Set throttle (1500=Stop, 1600=Slow Forward, 1400=Slow Reverse)")
    print("  stop      : Emergency STOP")
    print("  q         : Quit")

    try:
        while True:
            cmd_input = input("\nEnter command: ").strip().lower()
            
            if cmd_input == 'q':
                break
            elif cmd_input == 'stop':
                ser.write(b"<90,1500>")
                print("EMERGENCY STOP SENT")
            elif cmd_input.startswith('s '):
                try:
                    angle = int(cmd_input.split()[1])
                    msg = f"<{angle},1500>"
                    ser.write(msg.encode())
                    print(f"Sent steering: {angle}")
                except:
                    print("Invalid angle")
            elif cmd_input.startswith('t '):
                try:
                    pwm = int(cmd_input.split()[1])
                    msg = f"<90,{pwm}>"
                    ser.write(msg.encode())
                    print(f"Sent throttle: {pwm}")
                except:
                    print("Invalid PWM")
            else:
                print("Unknown command format")

    except KeyboardInterrupt:
        pass
    finally:
        # Final safety stop
        ser.write(b"<90,1500>")
        ser.close()
        print("\nDone.")

if __name__ == '__main__':
    main()
