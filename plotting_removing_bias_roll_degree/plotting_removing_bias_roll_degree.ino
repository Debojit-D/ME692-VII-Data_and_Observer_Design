/*
  SC651 Assignment (Part a):
  Roll angle estimation using gyroscope integration only (with bias subtraction).

  Discrete-time model (noise ignored):
      theta[k+1] = theta[k] + omega[k] * Ts

  Real gyros typically measure:
      omega_meas = omega_true + b + noise

  If the IMU is stationary, omega_true ≈ 0, so omega_meas ≈ b (bias/offset).
  When we integrate omega_meas directly, the bias accumulates linearly:
      theta drifts ≈ b * t

  To reduce drift (still "gyro-only"), subtract the estimated constant bias b_hat:
      omega_corr = omega_meas - b_hat
      theta[k+1] = theta[k] + omega_corr[k] * Ts

  Hardware:
    - ESP32-WROVER B
    - LSM9DS1 IMU (Arduino_LSM9DS1 library)

  Output:
    - Serial Plotter: prints roll_gyro_deg (bias-corrected integrated roll)
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
// Bias estimate (deg/s)
// Use the value you estimated offline from the CSV (mean bias).
// From your latest analysis: ~ +1.779315 deg/s
// -----------------------------
static const float GYRO_BIAS_DEG_S = 1.827666f;

// -----------------------------
// State: roll angle in degrees
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

  // Initialize time (first dt will be computed relative to this)
  t_prev_us = micros();

  Serial.println("Gyro-only roll integration started (with bias subtraction)...");
  Serial.print("Using GYRO_BIAS_DEG_S = ");
  Serial.println(GYRO_BIAS_DEG_S, 6);
  Serial.println("Rotate the board about the chosen roll axis.");
}

void loop() {
  // ---- Compute Ts (dt) in seconds using micros() ----
  unsigned long t_now_us = micros();
  float dt = (t_now_us - t_prev_us) * 1e-6f;  // microseconds -> seconds
  t_prev_us = t_now_us;

  // Guard against weird dt (startup spikes / rare delays)
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f; // fallback to 10 ms
  }

  // ---- Read gyroscope (Arduino_LSM9DS1 returns deg/s) ----
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // Pick the angular velocity component that corresponds to roll rate
    float omega_meas_deg_s = 0.0f;
    if (ROLL_AXIS == GX_AXIS) omega_meas_deg_s = gx;
    if (ROLL_AXIS == GY_AXIS) omega_meas_deg_s = gy;
    if (ROLL_AXIS == GZ_AXIS) omega_meas_deg_s = gz;

    // ---- Bias subtraction (still gyro-only) ----
    // omega_corr = omega_meas - b_hat
    float omega_corr_deg_s = omega_meas_deg_s - GYRO_BIAS_DEG_S;

    // ---- Discrete integration with corrected omega ----
    // theta = theta + omega_corr * dt
    roll_gyro_deg += omega_corr_deg_s * dt;

    // Optional: keep angle bounded to [-180, 180]
    if (roll_gyro_deg > 180.0f) roll_gyro_deg -= 360.0f;
    if (roll_gyro_deg < -180.0f) roll_gyro_deg += 360.0f;

    // ---- Print for Serial Plotter ----
    // Keep output simple: one value for plotting
    Serial.print("roll_gyro_deg:");
    Serial.println(roll_gyro_deg, 6);
  }

  delay(5); // optional pacing
}
