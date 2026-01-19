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
