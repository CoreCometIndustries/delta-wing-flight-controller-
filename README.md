# Delta Wing Flight Controller (STM32)

[cite_start]This firmware is a high-performance flight stabilization system designed for the STM32F103C8T6 (Blue Pill), specifically tailored for Delta Wing and Flying Wing RC aircraft[cite: 1].

## 🚀 Features

* [cite_start]**Real-Time Stability:** Implements a fixed 500 Hz control loop with a 2000 µs period for ultra-low latency response[cite: 1, 11].
* [cite_start]**Sensor Fusion:** Employs a Complementary Filter with $\alpha = 0.98$ to fuse high-speed gyroscope integration with accelerometer tilt data, effectively eliminating orientation drift[cite: 9, 11, 75, 76].
* [cite_start]**iBUS Integration:** Supports 14-channel FlySky iBUS protocol via USART2 with built-in failsafe logic that holds the aircraft level upon signal loss[cite: 1, 12, 14, 51].
* [cite_start]**Advanced Mixing:** Built-in mixing logic for mirrored/opposing servo mounting, ensuring correct physical deflection for both pitch and roll[cite: 91, 93, 98].
* [cite_start]**Clean Telemetry:** Outputs 20 Hz serial telemetry (115200 baud) compatible with COMET FC Ground Station for monitoring angles, gyro rates, and PWM outputs[cite: 102, 104, 107].

## 🛠 Hardware Mapping

The system utilizes the following pin configuration as defined in the source code and wiring diagram:

| Component | Pin | Interface |
| :--- | :--- | :--- |
| **MPU-6050 (IMU)** | **PB7 (SDA), PB6 (SCL)** | [cite_start]I2C (400 kHz) [cite: 1, 54] |
| **iBUS Receiver** | **PA3 (RX), PA2 (TX)** | [cite_start]USART2 [cite: 1, 13] |
| **Left Elevon Servo** | **PB1** | [cite_start]PWM Output [cite: 1, 15] |
| **Right Elevon Servo**| **PB0** | [cite_start]PWM Output [cite: 1, 15] |
| **Rudder Servo** | **PA7** | [cite_start]PWM Output [cite: 1, 15] |
| **Power** | **3.3V / GND** | Dedicated rails for MPU-6050 and sensors |

## ⚙️ PID Parameters (Rate Mode)

[cite_start]The controller is pre-configured with the following PID gains[cite: 3]:

* [cite_start]**Pitch:** $K_P = 0.80$, $K_I = 0.30$, $K_D = 0.04$ [cite: 4]
* [cite_start]**Roll:** $K_P = 0.75$, $K_I = 0.28$, $K_D = 0.04$ [cite: 5]
* [cite_start]**Yaw:** $K_P = 1.00$, $K_I = 0.20$, $K_D = 0.00$ [cite: 6]

## 📝 Setup & Calibration

1. [cite_start]**Initial Calibration:** Upon power-up, the system averages 2000 samples to calibrate the gyroscope[cite: 39, 40]. [cite_start]The aircraft must remain perfectly still during this process[cite: 39].
2. **Servo Mixing Check:** Confirm physical movement. [cite_start]Nose-up tilt should cause both elevons to rise[cite: 94]. [cite_start]If movement is reversed, adjust the `pitchCorr` or `rollCorr` signs in the mixing block[cite: 95, 97].
3. [cite_start]**Failsafe:** The system is pre-loaded with failsafe values that center all control surfaces (1500 µs) except for throttle[cite: 14, 56].
