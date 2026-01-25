# Walkthrough: Personal Following Robot

## 1. Hardware Setup
Refer to [wiring_guide.md](wiring_guide.md) for detailed connections.
**Safety Check**: Ensure the robot is raised on blocks (wheels off the ground) before applying power.

## 2. Firmware Installation (Arduino Mega)
1.  Open `robot_firmware.ino` in the Arduino IDE.
2.  Select **Tools > Board > Arduino Mega 2560**.
3.  Select the correct Port.
4.  **Verify & Upload**.
5.  **Test**: Open Serial Monitor (115200 baud). You should see `READY`.
    -   Tilt the IMU > 45 degrees. The Buzzer should sound.
    -   Reset the board to clear the alarm.

## 3. Raspberry Pi Setup
1.  Ensure the Pi is connected to the Internet.
2.  Install dependencies:
    ```bash
    sudo apt-get update
    sudo apt-get install python3-pip
    pip3 install -r requirements.txt
    ```
3.  Connect the Lidar and Arduino via USB.
4.  Find the ports:
    -   Arduino is usually `/dev/ttyACM0` or `/dev/ttyUSB0`.
    -   Lidar is usually `/dev/ttyUSB0` or `/dev/ttyUSB1`.
5.  Edit `robot_main.py`:
    -   Update `ARDUINO_PORT` to match your system.

## 4. Running the Robot
1.  **Start the Controller**:
    ```bash
    python3 robot_main.py
    ```
2.  **User Beacon (Simulation)**:
    -   The robot waits for a UDP packet containing Target GPS.
    -   You can simulate this from another PC/Phone on the same WiFi:
    ```bash
    # Run this on your laptop (Python)
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(b"37.7749,-122.4194", ("ROBOT_IP", 5005))
    ```
3.  **Field Test**:
    -   Place robot in open field.
    -   Start script.
    -   Walk away with your Beacon (Phone/ESP32 sending UDP).
    -   The robot should turn and follow.

## Troubleshooting & Limitations
### GPS Heading
The Neo-6M GPS only knows "Heading" (Course over Ground) **when moving**. If the robot stops, it may lose its sense of direction.
-   **Fix**: Add a Compass (HMC5883L) to the I2C bus and update the code to use magnetic heading.

### Lidar
The provided code contains a placeholder for Lidar data. You must install the `ydlidar` driver or SDK and pass the scan array to the `calculate_avoidance` function in `robot_main.py`.

### Motor Power
If the Pi reboots when motors start ("Brownout"), your power supply is insufficient. Use a separate battery for motors or a better UBEC for the Pi.
    
### Troubleshooting
#### Camera Not Detected
- The script now tries indices 0-9. If you see "No working camera found!", verify the USB connection or try `ls /dev/video*`.

#### Arduino Port
- The script now auto-scans ports (`/dev/ttyACM0`, `ACM1`, `USB0`, etc.). 
- If it still fails, check `ls /dev/tty*` to see if the device is recognized at all.

#### IMU/Arduino Issues
- If you see `[ALERT] Arduino connection silent!`, the microcontroller is likely hung.
- **Action**: Press the physical RESET button on the Arduino or power-cycle the robot.

#### Camera "Device Busy" or Failure
- If the camera fails even if plugged in, check for background services holding the lock.
- Run: `sudo systemctl stop robot-follower`
- Process locks can be checked with `fuser /dev/video*`.
- The logs may also show `INIT: IMU NOT FOUND!` if the I2C bus is locked up.

## 5. Critical Fixes Log (Jan 25 2026)
During the initial deployment, we encountered and fixed the following hardware/software integration issues:

### A. Arduino Freeze (MPU6050)
- **Problem**: The Arduino would hang at startup (`INIT: Initializing I2C...`) because the MPU sensor was interfering with the I2C bus.
- **Fix**: Flashed a custom firmware that **bypasses the MPU check**. The robot now runs without tilt-detection safety, but it runs.

### B. Servo Direction
- **Problem**: The wheels turned *away* from the target.
- **Fix**: Inverted the steering logic in `autonomous_follower.py`.

### C. Motor Deadband
- **Problem**: Motors would not spin at low speeds (Power Starvation / High Internal Resistance).
- **Fix**: Increased `MIN_PWM` to **1630** (from 1600). This provides a stronger initial kick.
- **Warning**: If the battery is low, this higher current draw can cause **Brownouts** (Servo twitching, USB disconnects). **Keep battery charged!**
