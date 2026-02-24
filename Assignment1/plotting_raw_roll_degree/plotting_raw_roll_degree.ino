/*
  SC651 Assignment (Part a):
  Roll angle estimation using gyroscope integration only.

  Discrete model (noise ignored):
    theta[k+1] = theta[k] + omega[k] * Ts

  Hardware:
    - ESP32-WROVER B
    - LSM9DS1 IMU (Arduino_LSM9DS1 library)

  Output:
    - Serial Plotter: prints roll_gyro_deg
*/

#include <Arduino_LSM9DS1.h>

// -----------------------------
// Choose which gyro axis corresponds to "roll rate"
// Most common: roll about X axis -> use gx
// If your sensor is mounted differently, switch to GY or GZ.
// -----------------------------
enum RollAxis { GX_AXIS, GY_AXIS, GZ_AXIS };
static const RollAxis ROLL_AXIS = GX_AXIS;

// -----------------------------
// State: roll angle (we'll keep it in degrees for convenience)
// -----------------------------
float roll_gyro_deg = 0.0f;

// Time bookkeeping (micros() gives microseconds)
unsigned long t_prev_us = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial */ }

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1! Check wiring / I2C.");
    while (1) { delay(100); }
  }

  // Initialize time
  t_prev_us = micros();

  Serial.println("Gyro-only roll integration started...");
  Serial.println("Rotate the board about the chosen roll axis.");
}

void loop() {
  // ---- Compute Ts (dt) in seconds using micros() ----
  unsigned long t_now_us = micros();
  float dt = (t_now_us - t_prev_us) * 1e-6f;  // microseconds -> seconds
  t_prev_us = t_now_us;

  // Guard against weird dt (e.g., first iteration or overflow edge cases)
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f; // fallback to 10 ms
  }

  // ---- Read gyroscope (Arduino_LSM9DS1 returns deg/s) ----
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // Pick the angular velocity component that corresponds to roll rate
    float omega_deg_s = 0.0f;
    if (ROLL_AXIS == GX_AXIS) omega_deg_s = gx;
    if (ROLL_AXIS == GY_AXIS) omega_deg_s = gy;
    if (ROLL_AXIS == GZ_AXIS) omega_deg_s = gz;

    // ---- Discrete integration: theta = theta + omega * Ts ----
    roll_gyro_deg += omega_deg_s * dt;

    // Optional: keep angle bounded (prevents the number from growing forever)
    // Comment this out if you want the raw integrated angle.
    if (roll_gyro_deg > 180.0f) roll_gyro_deg -= 360.0f;
    if (roll_gyro_deg < -180.0f) roll_gyro_deg += 360.0f;

    // ---- Print for Serial Plotter ----
    // Arduino Serial Plotter likes "label:value"
    Serial.print("roll_gyro_deg:");
    Serial.println(roll_gyro_deg);
  }

  // Small delay to avoid spamming serial too hard (optional)
  delay(5);
}
