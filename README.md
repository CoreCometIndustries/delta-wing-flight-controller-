# Delta Wing Flight Controller (STM32 Blue Pill)

An Arduino-based flight controller designed for Delta Wing/Flying Wing aircraft (e.g., F-35 style RC models) using the STM32F103C8T6.

## Hardware Specifications
* [cite_start]**Microcontroller:** STM32F103C8T6 (Blue Pill).
* [cite_start]**IMU:** MPU-6050 (Gyro + Accelerometer)[cite: 1, 54].
* [cite_start]**Receiver Protocol:** FlySky iBUS (14 Channels)[cite: 1, 12, 50].
* [cite_start]**Control Loop:** 500 Hz (2000 µs period)[cite: 2, 58].

## Wiring Diagram
* [cite_start]**MPU-6050 I2C:** SDA to PB7, SCL to PB6[cite: 1, 53].
* [cite_start]**iBUS Input:** RX to PA3, TX to PA2 (USART2)[cite: 1, 13].
* **Servos:**
    * [cite_start]Left Elevon: PB1.
    * [cite_start]Right Elevon: PB0.
    * [cite_start]Rudder: PA7.
* **Power:** Sensors are powered via the 3.3V and GND rails of the Blue Pill.

## PID Control Parameters
The controller uses a rate-based PID loop with the following default gains:
* [cite_start]**Pitch:** $K_P = 0.80$, $K_I = 0.30$, $K_D = 0.04$[cite: 3, 4].
* [cite_start]**Roll:** $K_P = 0.75$, $K_I = 0.28$, $K_D = 0.04$[cite: 4, 5].
* [cite_start]**Yaw:** $K_P = 1.00$, $K_I = 0.20$, $K_D = 0.00$[cite: 6].

## Features
* [cite_start]**Complementary Filter:** Fuses gyro and accelerometer data to eliminate pitch/roll drift[cite: 10, 68].
* [cite_start]**Elevon Mixing:** Handles mirrored/opposing servo mounting logic automatically[cite: 91, 93].
* [cite_start]**Failsafe:** Defaults to level flight if iBUS signal is lost[cite: 14].
