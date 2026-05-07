// =============================================================================
//  DELTA WING FLIGHT CONTROLLER
//  Target:   STM32F103C8T6 (Blue Pill) via Arduino IDE (STM32duino core)
//  Aircraft: Delta wing / Flying wing (e.g. F-35 style RC model)
//
//  Hardware:
//    IMU  : MPU-6050  SDA→PB7  SCL→PB6  (I2C)
//    iBUS : FlySky receiver  RX→PA3  TX→PA2  (USART2)
//    Servos: PB1=Left elevon  PB0=Right elevon  PA7=Rudder
//
//  Channels (iBUS, 1-indexed):
//    CH1 = Roll (Aileron)
//    CH2 = Pitch (Elevator)
//    CH4 = Yaw   (Rudder)
//    CH3 = Throttle (direct to ESC – NOT used here)
//
//  Control loop target: ~500 Hz  (2 000 µs period)
// =============================================================================

#include <Wire.h>
#include <Servo.h>

// ---------------------------------------------------------------------------
//  PIN DEFINITIONS
// ---------------------------------------------------------------------------
#define PIN_SERVO_LEFT   PB1   // Left  elevon
#define PIN_SERVO_RIGHT  PB0   // Right elevon
#define PIN_SERVO_RUDDER PA7   // Rudder

// ---------------------------------------------------------------------------
//  MPU-6050 REGISTER MAP (only what we need)
// ---------------------------------------------------------------------------
#define MPU_ADDR         0x68
#define MPU_PWR_MGMT_1   0x6B
#define MPU_SMPLRT_DIV   0x19
#define MPU_CONFIG       0x1A  // DLPF config
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H  0x43

// ---------------------------------------------------------------------------
//  FLIGHT PARAMETERS  – tune these in flight
// ---------------------------------------------------------------------------

// --- PID gains (rate mode) ---
float KP_PITCH = 0.80f;
float KI_PITCH = 0.30f;
float KD_PITCH = 0.04f;

float KP_ROLL  = 0.75f;
float KI_ROLL  = 0.28f;
float KD_ROLL  = 0.04f;

float KP_YAW   = 1.00f;
float KI_YAW   = 0.20f;
float KD_YAW   = 0.00f;

// --- RC stick → desired rate scaling (µs deviation → deg/s) ---
//     e.g. 500 µs stick throw → 150 deg/s max rate
#define RC_RATE_SCALE    0.70f   // deg/s per µs of stick deflection

// --- Deadband around RC stick centre (µs) ---
#define RC_DEADBAND      15

// --- Integral windup limit (µs equivalent) ---
#define I_LIMIT          200.0f

// --- PID output clamp (µs) ---
#define PID_CLAMP        400.0f

// --- Low-pass filter coefficient for gyro (0=frozen, 1=no filter) ---
//     0.20 gives a good balance at 500 Hz
#define GYRO_LPF_ALPHA   0.20f

// --- Gyro scale factor for ±500 °/s full-scale range ---
//     Raw LSB → deg/s  = raw / 65.5
#define GYRO_SENS        65.5f

// --- Accelerometer scale factor for ±2 g full-scale range ---
//     Raw LSB → g  = raw / 16384.0
#define ACCEL_SENS       16384.0f

// --- Complementary filter coefficient ---
//     Blends gyro integration (fast) with accelerometer angle (drift-free).
//     0.98 = trust gyro 98% each loop, let accel correct 2%.
//     Lower value = faster accel correction but more vibration noise.
//     Range: 0.95 (aggressive correction) – 0.99 (very slow correction)
#define CF_ALPHA         0.98f

// --- Servo neutral and travel limits (µs) ---
#define SERVO_MID        1500
#define SERVO_MIN        1100
#define SERVO_MAX        1900

// --- Control loop period (µs) ---
#define LOOP_PERIOD_US   2000   // 500 Hz

// ---------------------------------------------------------------------------
//  iBUS PARSER
//  Reads the 32-byte FlySky iBUS packet from a HardwareSerial port.
//  Packet: 0x20 0x40 [CH1_L CH1_H ... CH14_L CH14_H] [SUM_L SUM_H]
// ---------------------------------------------------------------------------
#define IBUS_CHANNELS    14
#define IBUS_PACKET_LEN  32

HardwareSerial ibusSerial(PA3, PA2);   // USART2 RX=PA3 TX=PA2

uint16_t rcChannel[IBUS_CHANNELS];    // Decoded channel values (µs)
uint8_t  ibusBuf[IBUS_PACKET_LEN];
uint8_t  ibusIdx = 0;

// --- Failsafe defaults (aircraft holds level) ---
const uint16_t RC_FAILSAFE[4] = {1500, 1500, 1000, 1500};

// ---------------------------------------------------------------------------
//  SERVO OBJECTS
// ---------------------------------------------------------------------------
Servo servoLeft;
Servo servoRight;
Servo servoRudder;

// ---------------------------------------------------------------------------
//  GYRO STATE
// ---------------------------------------------------------------------------
float gyroRoll  = 0, gyroPitch = 0, gyroYaw = 0;   // Filtered rate (deg/s)
float gyroOffX  = 0, gyroOffY  = 0, gyroOffZ = 0;  // Calibration offsets

// Attitude angles — maintained by complementary filter (gyro + accel fusion)
// Roll and Pitch return to correct value when level; Yaw is gyro-only (no mag)
float angleRoll  = 0.0f;
float anglePitch = 0.0f;
float angleYaw   = 0.0f;   // Yaw drifts — no magnetometer available

// ---------------------------------------------------------------------------
//  PID STATE
// ---------------------------------------------------------------------------
struct PID {
  float kp, ki, kd;
  float prevError;
  float integral;

  PID(float p, float i, float d)
    : kp(p), ki(i), kd(d), prevError(0), integral(0) {}

  float compute(float setpoint, float measured, float dt) {
    float error     = setpoint - measured;
    integral       += error * dt;
    // Windup clamp
    integral        = constrain(integral, -I_LIMIT, I_LIMIT);
    float derivative = (error - prevError) / dt;
    prevError       = error;
    float out = kp * error + ki * integral + kd * derivative;
    return constrain(out, -PID_CLAMP, PID_CLAMP);
  }

  void reset() { prevError = 0; integral = 0; }
};

PID pidPitch(KP_PITCH, KI_PITCH, KD_PITCH);
PID pidRoll (KP_ROLL,  KI_ROLL,  KD_ROLL );
PID pidYaw  (KP_YAW,   KI_YAW,   KD_YAW  );

// ---------------------------------------------------------------------------
//  TIMING
// ---------------------------------------------------------------------------
uint32_t lastLoopTime = 0;

// =============================================================================
//  HELPER FUNCTIONS
// =============================================================================

// --- I2C write to MPU register ---
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// --- Initialise MPU-6050 ---
void initMPU() {
  // Wake the MPU, use PLL with X-gyro as clock source (more stable)
  mpuWrite(MPU_PWR_MGMT_1, 0x01);
  delay(100);

  // Sample-rate divider: 0 → full gyro output rate (1 kHz with DLPF)
  mpuWrite(MPU_SMPLRT_DIV, 0x00);

  // DLPF: bandwidth ~42 Hz (register value 3) – reduces high-freq noise
  mpuWrite(MPU_CONFIG, 0x03);

  // Gyro full-scale: ±500 °/s (0x08)
  mpuWrite(MPU_GYRO_CONFIG, 0x08);

  delay(20);
}

// --- Read accelerometer + gyro in one I2C burst (14 bytes from 0x3B) ---
//     Register map: AX AY AZ TEMP GX GY GZ  (each 2 bytes, big-endian)
void readRawIMU(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);   // start at 0x3B
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();        // skip temperature (2 bytes)
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// --- Kept for backward compatibility (calibration still uses gyro only) ---
void readRawGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
  int16_t ax, ay, az;
  readRawIMU(ax, ay, az, gx, gy, gz);
}

// --- Gyro calibration: average 2000 samples at rest ---
void calibrateGyro() {
  Serial.println("Calibrating gyro – keep aircraft still...");
  long sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 2000;
  int16_t gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    readRawGyro(gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(1);
  }
  gyroOffX = (float)sumX / samples;
  gyroOffY = (float)sumY / samples;
  gyroOffZ = (float)sumZ / samples;
  Serial.println("Gyro calibration complete.");
}

// --- Apply deadband: return 0 if within ±deadband of 1500 ---
int16_t applyDeadband(int16_t val, int16_t deadband) {
  int16_t dev = val - SERVO_MID;
  if (abs(dev) <= deadband) return SERVO_MID;
  // Shrink towards centre so there is no jump at deadband edge
  return SERVO_MID + (dev > 0 ? dev - deadband : dev + deadband);
}

// --- iBUS parser tick – call every loop iteration ---
void parseIBUS() {
  while (ibusSerial.available()) {
    uint8_t b = ibusSerial.read();

    // Detect packet start: 0x20 at index 0
    if (ibusIdx == 0 && b != 0x20) continue;
    // Second byte must be 0x40
    if (ibusIdx == 1 && b != 0x40) { ibusIdx = 0; continue; }

    ibusBuf[ibusIdx++] = b;

    if (ibusIdx == IBUS_PACKET_LEN) {
      ibusIdx = 0;

      // Verify checksum: sum of first 30 bytes, stored little-endian in [30-31]
      uint16_t checksum = 0xFFFF;
      for (int i = 0; i < 30; i++) checksum -= ibusBuf[i];
      uint16_t rxSum = ibusBuf[30] | (ibusBuf[31] << 8);

      if (checksum == rxSum) {
        // Decode 14 channels (2 bytes each, starting at byte 2)
        for (int ch = 0; ch < IBUS_CHANNELS; ch++) {
          rcChannel[ch] = ibusBuf[2 + ch * 2] | (ibusBuf[3 + ch * 2] << 8);
        }
      }
      // Bad checksum → keep previous values (natural failsafe hold)
    }
  }
}

// --- Clamp servo output to safe range ---
int16_t clampServo(int16_t val) {
  return constrain(val, SERVO_MIN, SERVO_MAX);
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Delta Wing Flight Controller ===");

  // ---- I2C on PB6 (SCL) / PB7 (SDA) at 400 kHz ----
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(400000);

  // ---- MPU-6050 ----
  initMPU();
  calibrateGyro();

  // ---- iBUS serial ----
  ibusSerial.begin(115200);

  // Pre-load failsafe channel values
  for (int i = 0; i < 4; i++) rcChannel[i] = RC_FAILSAFE[i];

  // ---- Servos ----
  servoLeft.attach(PIN_SERVO_LEFT,   SERVO_MIN, SERVO_MAX);
  servoRight.attach(PIN_SERVO_RIGHT, SERVO_MIN, SERVO_MAX);
  servoRudder.attach(PIN_SERVO_RUDDER, SERVO_MIN, SERVO_MAX);

  // Centre servos on boot
  servoLeft.writeMicroseconds(SERVO_MID);
  servoRight.writeMicroseconds(SERVO_MID);
  servoRudder.writeMicroseconds(SERVO_MID);

  delay(1000);
  Serial.println("Flight controller ready.");
  lastLoopTime = micros();
}

// =============================================================================
//  MAIN LOOP
// =============================================================================
void loop() {
  // ---- Enforce fixed loop period ----
  uint32_t now = micros();
  uint32_t elapsed = now - lastLoopTime;
  if (elapsed < LOOP_PERIOD_US) return;   // Not time yet

  float dt = (float)elapsed * 1e-6f;      // Loop time in seconds
  lastLoopTime = now;

  // ----------------------------------------------------------------
  //  1. READ IMU — accelerometer + gyro in one burst
  // ----------------------------------------------------------------
  int16_t rawAx0, rawAy0, rawAz0, rawGx, rawGy, rawGz;
  readRawIMU(rawAx0, rawAy0, rawAz0, rawGx, rawGy, rawGz);

  // Convert gyro raw → deg/s, subtract calibration offsets
  float rateRoll  = ((float)rawGx - gyroOffX) / GYRO_SENS;
  float ratePitch = ((float)rawGy - gyroOffY) / GYRO_SENS;
  float rateYaw   = ((float)rawGz - gyroOffZ) / GYRO_SENS;

  // First-order low-pass filter on gyro rates: removes high-freq vibration noise
  gyroRoll  = GYRO_LPF_ALPHA * rateRoll  + (1.0f - GYRO_LPF_ALPHA) * gyroRoll;
  gyroPitch = GYRO_LPF_ALPHA * ratePitch + (1.0f - GYRO_LPF_ALPHA) * gyroPitch;
  gyroYaw   = GYRO_LPF_ALPHA * rateYaw   + (1.0f - GYRO_LPF_ALPHA) * gyroYaw;

  // ----------------------------------------------------------------
  //  COMPLEMENTARY FILTER — fuse gyro + accelerometer
  //
  //  Why this fixes drift:
  //    Pure gyro integration: angle += rate * dt
  //      → fast and smooth but tiny offset errors accumulate → drift
  //    Pure accelerometer:    angle = atan2(ax, az)
  //      → always correct when still but very noisy during motion
  //
  //  Complementary filter blends both:
  //    angle = CF_ALPHA * (angle + gyro*dt) + (1-CF_ALPHA) * accel_angle
  //
  //    The gyro part tracks fast motion accurately.
  //    The accel part slowly pulls the angle back to the true level
  //    position, cancelling any drift that has built up.
  //    Result: when you return the aircraft to level, roll and pitch
  //    will return to 0° within about 1-2 seconds.
  // ----------------------------------------------------------------

  // Use the accelerometer values already read at the top of the loop
  // (rawAx0, rawAy0, rawAz0 from readRawIMU above — no extra I2C transaction needed)
  float ax = (float)rawAx0 / ACCEL_SENS;
  float ay = (float)rawAy0 / ACCEL_SENS;
  float az = (float)rawAz0 / ACCEL_SENS;

  // Calculate absolute tilt angles from accelerometer
  // atan2 gives angle in radians → convert to degrees
  // These are noisy but drift-free when the aircraft is near level
  float accelRoll  =  atan2f(ay, az) * (180.0f / M_PI);
  float accelPitch = -atan2f(ax, sqrtf(ay*ay + az*az)) * (180.0f / M_PI);

  // Complementary filter: trust gyro for fast moves, accel corrects slow drift
  angleRoll  = CF_ALPHA * (angleRoll  + gyroRoll  * dt) + (1.0f - CF_ALPHA) * accelRoll;
  anglePitch = CF_ALPHA * (anglePitch + gyroPitch * dt) + (1.0f - CF_ALPHA) * accelPitch;

  // Yaw: gyro-only (accelerometer cannot sense yaw rotation)
  // No drift correction available without a magnetometer
  angleYaw  += gyroYaw * dt;
  if (angleYaw >  360.0f) angleYaw -= 360.0f;
  if (angleYaw <    0.0f) angleYaw += 360.0f;

  // ----------------------------------------------------------------
  //  2. PARSE iBUS (non-blocking, processes all waiting bytes)
  // ----------------------------------------------------------------
  parseIBUS();

  // ----------------------------------------------------------------
  //  3. DECODE RC STICKS → desired rotation rates
  //     Channel values are 1000–2000 µs; centre = 1500 µs.
  //     Apply deadband first, then scale to deg/s.
  // ----------------------------------------------------------------
  // FlySky FS-i6S channel mapping (Mode 2, default):
  //   CH1 (index 0) = Right stick Left/Right → ROLL
  //   CH2 (index 1) = Right stick Up/Down    → PITCH (inverted on FS-i6S, negated below)
  //   CH4 (index 3) = Left  stick Left/Right → YAW
  int16_t rcRollRaw  = applyDeadband((int16_t)rcChannel[0], RC_DEADBAND); // CH1 → Roll
  int16_t rcPitchRaw = applyDeadband((int16_t)rcChannel[1], RC_DEADBAND); // CH2 → Pitch
  int16_t rcYawRaw   = applyDeadband((int16_t)rcChannel[3], RC_DEADBAND); // CH4 → Yaw

  float desiredRoll  =  (rcRollRaw  - SERVO_MID) * RC_RATE_SCALE;
  float desiredPitch = -(rcPitchRaw - SERVO_MID) * RC_RATE_SCALE; // Negated: FS-i6S CH2 pitch is inverted
  float desiredYaw   = (rcYawRaw   - SERVO_MID) * RC_RATE_SCALE;

  // ----------------------------------------------------------------
  //  4. PID COMPUTATION
  //     setpoint = desired rate (deg/s)
  //     measured = gyro rate   (deg/s)
  //     output   = servo correction (µs)
  // ----------------------------------------------------------------
  float pitchCorr = pidPitch.compute(desiredPitch, gyroPitch, dt);
  float rollCorr  = pidRoll.compute (desiredRoll,  gyroRoll,  dt);
  float yawCorr   = pidYaw.compute  (desiredYaw,   gyroYaw,   dt);

  // ----------------------------------------------------------------
  //  5. ELEVON MIXING  (MIRRORED / OPPOSING SERVO MOUNTING)
  //
  //  The two elevon servos are installed facing OPPOSITE directions.
  //  Without compensation, the same PWM correction would drive the
  //  right surface the wrong way. Fix: negate ONLY the pitch term on
  //  the right servo (roll term stays the same because roll already
  //  needs the surfaces to move in opposite physical directions).
  //
  //  LEFT  servo: leftElevon  = 1500 + pitchCorr + rollCorr
  //  RIGHT servo: rightElevon = 1500 - pitchCorr + rollCorr
  //
  //  Expected physical result:
  //    Pitch up stick  -> both trailing edges rise  (nose pitches up)
  //    Roll right stick-> left TE rises, right TE drops (rolls right)
  //
  //  Still wrong after flashing? Use the bench-test guide below:
  //    Nose-up tilt  -> both elevons should rise; if only one does,
  //                     flip the pitchCorr sign on the bad servo.
  //    Right-bank    -> left elevon rises, right drops; if reversed,
  //                     flip the rollCorr sign on the bad servo.
  // ----------------------------------------------------------------
  int16_t leftElevon  = SERVO_MID + (int16_t)pitchCorr + (int16_t)rollCorr;
  int16_t rightElevon = SERVO_MID - (int16_t)pitchCorr + (int16_t)rollCorr;
  int16_t rudder      = SERVO_MID + (int16_t)yawCorr;

  // ----------------------------------------------------------------
  //  6. OUTPUT TO SERVOS (clamped to safe range)
  // ----------------------------------------------------------------
  servoLeft.writeMicroseconds (clampServo(leftElevon));
  servoRight.writeMicroseconds(clampServo(rightElevon));
  servoRudder.writeMicroseconds(clampServo(rudder));

  // ----------------------------------------------------------------
  //  7. TELEMETRY OUTPUT — printed every 50 ms (~20 Hz) so the ground
  //     station serial monitor can display clean, readable angle data.
  //
  //  Format (parsed by COMET FC Ground Station HTML):
  //    ROLL:<angle> PITCH:<angle> YAW:<angle> GR:<rate> GP:<rate> GY:<rate> L:<us> R:<us> RUD:<us>
  //
  //  To disable for production, comment out this entire block.
  // ----------------------------------------------------------------
  static uint32_t debugTimer = 0;
  if (millis() - debugTimer >= 50) {
    debugTimer = millis();

    // ── Angles (integrated from gyro) ──
    Serial.print("ROLL:");   Serial.print(angleRoll,  1);
    Serial.print(" PITCH:"); Serial.print(anglePitch, 1);
    Serial.print(" YAW:");   Serial.print(angleYaw,   1);

    // ── Gyro rates (deg/s) ──
    Serial.print(" GR:");    Serial.print(gyroRoll,   1);
    Serial.print(" GP:");    Serial.print(gyroPitch,  1);
    Serial.print(" GY:");    Serial.print(gyroYaw,    1);

    // ── Servo outputs (µs) ──
    Serial.print(" L:");     Serial.print(leftElevon);
    Serial.print(" R:");     Serial.print(rightElevon);
    Serial.print(" RUD:");   Serial.println(rudder);
  }
}
// =============================================================================
//  END OF FILE
// =============================================================================
