# Implementation Plan - Personal Following Robot

## Goal Description
Build a robust software and hardware integration for a personal following robot that follows a user via GPS, avoids obstacles using Lidar, and detects falls using an IMU. The system uses a Raspberry Pi 5 for high-level logic and an Arduino Mega for low-level motor control and safety checks.

## User Review Required
> [!IMPORTANT]
> **Safety First**: This system controls physical motors. Always test with wheels off the ground first. The "Fall Detection" actively stops motors, which is a critical safety feature. All code provided should be treated as prototype/experimental.

## Proposed Changes

### Documentation
#### [NEW] [wiring_guide.md](file:///home/mono/.gemini/antigravity/brain/670e44b3-4703-4201-b6a7-d17a9bb9017b/wiring_guide.md)
A comprehensive guide detailing pinouts and connections between the Raspberry Pi, Arduino, Sensors, and Actuators.

### Microcontroller Firmware (Arduino Mega)
#### [NEW] `firmware/robot_controller.ino`
- **Responsibilities**:
    - Receive `[STEER, SPEED]` commands from Raspberry Pi via Serial (USB).
    - Read MPU6050 IMU for pitch/roll (Fall Detection).
    - Read Ultrasonic sensors for emergency braking (< 30cm).
    - Control Servo (Steering) and ESC (Throttle).
    - Activate Buzzer on alarm conditions.
    - Failsafe: Stop motors if Serial connection is lost for > 1 second.

### High-Level Control (Raspberry Pi 5)
#### [NEW] `src/main.py`
- Entry point for the robot control software.
- Initializes modules (Lidar, GPS, Serial Link).
- Runs the main control loop (10-20Hz).

#### [NEW] `src/lidar_driver.py`
- Interfaces with YDLiDAR T-mini Plus.
- Returns an obstacle map or nearest obstacle vectors.

#### [NEW] `src/gps_driver.py`
- Interfaces with Neo-6M GPS module.
- Parses NMEA sentences to get current Lat/Lon.
- Calculates Distance and Bearing to `Target_Lat/Lon`.

#### [NEW] `src/navigation.py`
- **Logic**:
    - **Obstacle Avoidance**: Uses a potential field or simple reactive method (steer away from close points).
    - **Following**: Calculates heading error towards target.
    - **Fusion**: Combines "Avoidance Heading" and "Target Heading" to produce final steering command.

## Verification Plan

### Automated Tests
- **Unit Tests**: Test vector math (distance/bearing calculations) in Python.
- **Mock Tests**: Simulate Lidar data strings and check avoidance logic outputs using a test script.

### Manual Verification
1.  **Wiring Check**: Double-check all connections against `wiring_guide.md`.
2.  **IMU Alarm Test**: Tilt the robot >45 degrees manually and verify the Buzzer sounds and Motors disable.
3.  **Motor Lock**: Verify wheels do not spin when the Pi is not sending commands.
4.  **Sensor Readout**: Run a diagnostic script on Pi to print Lidar and GPS values to the terminal.
