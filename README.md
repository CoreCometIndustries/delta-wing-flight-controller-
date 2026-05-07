# Delta Wing Flight Controller (STM32)

This firmware is a high-performance flight stabilization system designed for the STM32F103C8T6 (Blue Pill), specifically tailored for Delta Wing and Flying Wing RC aircraft.

## 🚀 Features

* **Real-Time Stability:** Implements a fixed 500 Hz control loop with a 2000 µs period for ultra-low latency response.
* **Sensor Fusion:** Employs a Complementary Filter with α = 0.98 to fuse high-speed gyroscope integration with accelerometer tilt data, effectively eliminating orientation drift.
* **iBUS Integration:** Supports 14-channel FlySky iBUS protocol via USART2 with built-in failsafe logic that holds the aircraft level upon signal loss.
* **Advanced Mixing:** Built-in mixing logic for mirrored/opposing servo mounting, ensuring correct physical deflection for both pitch and roll.
* **Clean Telemetry:** Outputs 20 Hz serial telemetry (115200 baud) compatible with COMET FC Ground Station for monitoring angles, gyro rates, and PWM outputs.

## 🛠 Hardware Mapping

The system utilizes the following pin configuration as defined in the firmware:

| Component | Pin | Interface |
| :--- | :--- | :--- |
| **MPU-6050 (IMU)** | **PB7 (SDA), PB6 (SCL)** | I2C (400 kHz) |
| **iBUS Receiver** | **PA3 (RX), PA2 (TX)** | USART2 |
| **Left Elevon Servo** | **PB1** | PWM Output |
| **Right Elevon Servo**| **PB0** | PWM Output |
| **Rudder Servo** | **PA7** | PWM Output |

## ⚙️ PID Parameters (Rate Mode)

The controller is pre-configured with the following default PID gains:

* **Pitch:** $K_P = 0.80$, $K_I = 0.30$, $K_D = 0.04$
* **Roll:** $K_P = 0.75$, $K_I = 0.28$, $K_D = 0.04$
* **Yaw:** $K_P = 1.00$, $K_I = 0.20$, $K_D = 0.00$

## 📝 Setup & Calibration

1. **Initial Calibration:** Upon power-up, the system averages 2000 samples to calibrate the gyroscope. The aircraft must remain perfectly still during this process.
2. **Servo Mixing Check:** Confirm physical movement. Nose-up tilt should cause both elevons to rise. If movement is reversed, adjust the `pitchCorr` or `rollCorr` signs in the mixing block.
3. **Failsafe:** The system is pre-loaded with failsafe values that center all control surfaces (1500 µs) except for throttle.
