# Hardware Connection Guide

## Power Distribution System
**Critical**: Proper power distribution is essential to prevent "brownouts" (restarts) of the Raspberry Pi or Arduino when the motors kick in.

### 1. Battery (2S Li-Po 7.4V)
- **Connector**: XT60 or T-Plug (Deans).
- **Splitter**: You need to split the battery voltage to two paths:
    1.  **To ESC** (Electronic Speed Controller) -> Powers the DC Motor.
    2.  **To UBEC** (Voltage Regulator) -> Powers the Raspberry Pi 5.

### 2. Servo & Motor Power
- **DC Motor**: Connect to the **Output A** (or B) terminals of the **L298N Motor Driver**.
- **Servo (20kg)**:
    -   **Signal (Orange/White)**: To Arduino **Pin 9**.
    -   **Power (+/-)**: **DO NOT** connect to Arduino 5V. The Servo draws too much current.
    -   *Option A*: If your ESC has a built-in BEC (3 wires to receiver), use that 5V/GND.
    -   *Option B (Recommended)*: Share the **YSIDO UBEC** 5V output, but be careful not to starve the Pi.

### 3. Logic Power
- **Raspberry Pi 5**: Power via **YSIDO UBEC (5V 6A)**. Connect UBEC Output to Pi GPIO **Pin 2 (5V)** and **Pin 6 (GND)**.
- **Arduino Mega**: Connect via **USB Cable** to Raspberry Pi 5. This handles both Power and Data.

---

## Signal Connections (Pinout)

### A. Arduino Mega Connections
| Component | Pin Name | Arduino Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Steering Servo** | Signal (PWM) | **D9** |  |
| **L298N ENA**    | PWM Speed   | **D10** | Reuses old ESC pin |
| **L298N IN1**    | Direction 1 | **D7**  | New |
| **L298N IN2**    | Direction 2 | **D8**  | New |
| **IMU (MPU6050)** | SDA | **D20** (SDA) |  |
| | SCL | **D21** (SCL) |  |
| | VCC | 5V |  |
| | GND | GND |  |
| **Buzzer** | Signal (+) | **D11** | Connect (-) to GND |
| **Ultrasonic** | Trig | **D12** | Front Facing |
| | Echo | **D13** |  |

### B. Raspberry Pi 5 Connections
| Component | Connection Type | Port/Pin | Notes |
| :--- | :--- | :--- | :--- |
| **YDLiDAR T-mini** | USB | **USB 3.0 (Blue)** | High bandwidth required |
| **Arduino Mega** | USB | **USB 2.0/3.0** | Serial Communication |
| **GPS (Neo-6M)** | UART TX | **GPIO 15 (RXD)** | GPS TX -> Pi RX |
| | UART RX | **GPIO 14 (TXD)** | GPS RX -> Pi TX |
| | VCC | 3.3V (Pin 1) | **Start with 3.3V**. If no fix, check 5V tollerance |
| | GND | GND (Pin 9) | |

> [!WARNING]
> **GPS Voltage Level**: The Neo-6M module VCC is often 5V, but the TX/RX logic might be 3.3V compatible. Raspberry Pi GPIO is **Strictly 3.3V**. sending 5V to a Pi GPIO will destroy it. Verify your GPS module's logic level. If it's 5V logic, use a **Logic Level Converter** or connect the GPS to the Arduino instead (Arduino Mega is 5V tolerant) and pass data to Pi, or use a USB GPS dongle. For this guide, we will assume a safe 3.3V connection or USB-TTL adapter for the Pi.

---

## Detailed Power & Actuator Subsystem

For troubleshooting motor and steering issues, refer to this detailed power distribution:

```mermaid
graph TD
    subgraph "Power Source"
        BAT[2S LiPo Battery 7.4V] --- SPLIT{Power Splitter / T-Connector}
    end

    subgraph "Drive System"
        SPLIT ===|7.4V Thick Wires| L298N[L298N Motor Driver]
        L298N ---|Output A/B| M[DC Brushed Motor]
        L298N ---|IN1/IN2| MEGA_DIR[Arduino Mega D7, D8]
        L298N ---|ENA| MEGA_D10[Arduino Mega Pin D10]
    end

    subgraph "Logic & Steering Power"
        SPLIT ===|7.4V| UBEC[YSIDO UBEC 5V 6A]
        UBEC ---|5V & GND| PI[Raspberry Pi 5 GPIO]
        UBEC ---|5V & GND| MEGA_VCC[Arduino Mega 5V/GND]
        
        UBEC -.->|5V Power Tap| SERVO[Steering Servo]
        MEGA_D9[Arduino Mega Pin D9] --->|Signal Only| SERVO
    end

    style BAT fill:#f96,stroke:#333,stroke-width:2px
    style L298N fill:#69f,stroke:#333,stroke-width:2px
    style UBEC fill:#6f6,stroke:#333,stroke-width:2px
```

### Critical Actuator Checks:
1.  **L298N Power**: Ensure 7.4V (VCC) and GND are connected to the battery.
2.  **Jumpers**: Ensure the ENA jumper is REMOVED if using Pin 10 for PWM, or keep it if always full speed (not recommended).
3.  **Servo Power**: The servo red/black wires should go to the 5V UBEC output, not the Arduino's 5V pin.
4.  **Signal Ground**: Ensure the ESC/Servo share a common ground with the Arduino.

---

## Diagram Overview

```mermaid
    Batt[2S LiPo Battery] -->|7.4V| Split{Power Splitter}
    Split -->|7.4V| L298N[L298N Driver]
    Split -->|7.4V| UBEC[YSIDO UBEC 6A]
    
    L298N -->|Output A/B| Motor[DC Motor]
    
    UBEC -->|5V Regulated| Pi[Raspberry Pi 5]
    UBEC -->|5V Regulated| Servo[Steering Servo]
    
    Pi ===|USB Cable| Mega[Arduino Mega]
    Pi ===|USB| Lidar[YDLiDAR]
    
    Mega -->|PWM D9| Servo
    Mega -->|PWM D10| ENA[L298N ENA]
    Mega -->|Digital D7/D8| IN[L298N IN1/IN2]
    Mega -->|I2C| IMU[MPU6050]
    Mega -->|Digital| Buzzer
    Mega -->|Digital| Sonic[Ultrasonic Sensor]
    
    GPS[Neo-6M GPS] -->|UART| Pi
```
