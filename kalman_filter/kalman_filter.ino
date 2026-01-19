/*
  SC651 Assignment - Part (c): Kalman Filtering for Roll Angle Estimate

  We implement the algorithm exactly as in the handout:

  System model (discrete)         (Eq. 2):
      theta[k+1] = theta[k] + omega[k] * Ts + w[k]
    We ignore the noise term in the state update, but we propagate its variance.

  Measurement model               (Eq. 3):
      theta_acc[k] = theta[k] + v[k]
    where theta_acc is computed from accelerometer.

  Accelerometer roll measurement  (Eq. 7):
      theta_acc = atan2( ay, sqrt(ax^2 + az^2) )

  Kalman filter steps:
    Prediction (Eq. 5):
      theta_hat[k+1|k] = theta_hat[k|k] + Ts * omega_gyro[k]

    Covariance prediction (Eq. 6):
      P[k+1|k] = P[k|k] + Ts^2 * sigma_w^2

    Measurement update (Eq. 8):
      K = P / (P + Ts^2 * sigma_v^2)
      theta_hat[k+1|k+1] = theta_hat[k+1|k] + K * (theta_acc[k+1] - theta_hat[k+1|k])

    Covariance update (Eq. 9):
      P = (1 - K) * P

  Output to Serial Plotter:
    - roll_gyro_deg : gyro integration only (Part a)
    - roll_acc_deg  : accelerometer only    (Part b)
    - roll_kf_deg   : Kalman filter estimate (Part c)
*/

#include <Arduino_LSM9DS1.h>
#include <math.h>

// -----------------------------------------------------------------------------
// USER SETTINGS
// -----------------------------------------------------------------------------

// Which gyro axis is your roll-rate? (must match your earlier part-a choice)
enum RollAxis { GX_AXIS, GY_AXIS, GZ_AXIS };
static const RollAxis ROLL_AXIS = GX_AXIS;

// If your accelerometer roll sign is opposite of gyro roll, flip it here.
// You already observed: gyro CCW +, accel CCW -  -> set this true.
static const bool FLIP_ACCEL_SIGN = true;

// Optional: subtract a constant gyro bias before using gyro in prediction.
// - If you want to follow the handout literally, set USE_GYRO_BIAS_CORRECTION = false.
// - If you want better practical behavior (since your bias was large), set true and set the bias value.
static const bool  USE_GYRO_BIAS_CORRECTION = true;
static const float GYRO_BIAS_DEG_S = 1.827666f;  // your estimated mean bias (deg/s)

// Sampling time:
// - The handout uses Ts (constant). :contentReference[oaicite:5]{index=5}
// - In real embedded code, dt from micros() is safer.
// We'll use dt from gyro sample timing; you can force a fixed Ts for experiments in part (d).
static const bool  USE_FIXED_TS = false;
static const float FIXED_TS_S   = 0.01f;         // 10 ms if you force fixed Ts

// Kalman noise variances (tune these in part d):
// sigma_w^2 : process noise variance (model / gyro uncertainty) :contentReference[oaicite:6]{index=6}
// sigma_v^2 : measurement noise variance (accelerometer-angle uncertainty) :contentReference[oaicite:7]{index=7}
static const float SIGMA_W2 = 1.00f;   // (deg/s)^2   <-- tune
static const float SIGMA_V2 = 0.50f;   // (deg)^2     <-- tune (see note below)

// Initial covariance P0 = Sigma_hat[k|k]  (tune in part d)
static const float P0 = 10.0f;

// Loop pacing (optional)
static const uint32_t LOOP_DELAY_MS = 5;

// -----------------------------------------------------------------------------
// STATE VARIABLES
// -----------------------------------------------------------------------------

// (a) Gyro-only roll (integrated)
float roll_gyro_deg = 0.0f;

// (b) Accel-only roll (direct measurement)
float roll_acc_deg  = 0.0f;

// (c) Kalman filter estimate and covariance
float theta_hat_deg = 0.0f;  // theta_hat[k|k]
float P             = P0;    // Sigma_hat[k|k]

// Timing
unsigned long t_prev_us = 0;

// -----------------------------------------------------------------------------
// HELPER: compute accelerometer roll (Eq. 7) in degrees
// -----------------------------------------------------------------------------
static float accel_roll_deg(float ax, float ay, float az) {
  // Eq. (7): theta_acc = atan2( ay, sqrt(ax^2 + az^2) )
  float denom = sqrtf(ax * ax + az * az);
  float theta_rad = atan2f(ay, denom);
  float theta_deg = theta_rad * (180.0f / 3.14159265f);

  if (FLIP_ACCEL_SIGN) theta_deg = -theta_deg;
  return theta_deg;
}

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait */ }

  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1! Check wiring / I2C.");
    while (1) { delay(100); }
  }

  // Initialize timing
  t_prev_us = micros();

  // Initialize KF
  theta_hat_deg = 0.0f;  // you can also initialize with first accel measurement if you want
  P = P0;

  Serial.println("Kalman roll estimation started (gyro + accel).");
  Serial.println("Serial Plotter outputs: roll_gyro_deg, roll_acc_deg, roll_kf_deg");
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  // We update when gyro data is available; dt then matches actual gyro sample spacing.
  if (!IMU.gyroscopeAvailable()) {
    delay(1);
    return;
  }

  // --------- Time step Ts (or dt) ----------
  float Ts = FIXED_TS_S;
  if (!USE_FIXED_TS) {
    unsigned long t_now_us = micros();
    Ts = (t_now_us - t_prev_us) * 1e-6f;
    t_prev_us = t_now_us;

    // Guard against crazy dt
    if (Ts <= 0.0f || Ts > 0.5f) Ts = FIXED_TS_S;
  }

  // --------- Read gyro (deg/s) ----------
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  float omega_deg_s = 0.0f;
  if (ROLL_AXIS == GX_AXIS) omega_deg_s = gx;
  if (ROLL_AXIS == GY_AXIS) omega_deg_s = gy;
  if (ROLL_AXIS == GZ_AXIS) omega_deg_s = gz;

  // Optional constant bias subtraction BEFORE using gyro in integration + KF prediction
  float omega_used_deg_s = omega_deg_s;
  if (USE_GYRO_BIAS_CORRECTION) {
    omega_used_deg_s = omega_deg_s - GYRO_BIAS_DEG_S;
  }

  // --------- (a) Gyro-only integration (Eq. 2 without noise) ----------
  roll_gyro_deg += omega_used_deg_s * Ts;

  // keep bounded for readability
  if (roll_gyro_deg > 180.0f) roll_gyro_deg -= 360.0f;
  if (roll_gyro_deg < -180.0f) roll_gyro_deg += 360.0f;

  // --------- Read accelerometer and compute roll measurement (Eq. 7) ----------
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    roll_acc_deg = accel_roll_deg(ax, ay, az);
  }
  // If accel is not available in this cycle, we just reuse the last roll_acc_deg.

  // =============================================================================
  // (c) KALMAN FILTER (Eq. 5–9)
  // =============================================================================

  // ----- Prediction step (Eq. 5) -----
  // theta_hat[k+1|k] = theta_hat[k|k] + Ts * omega_gyro[k]
  float theta_pred_deg = theta_hat_deg + Ts * omega_used_deg_s;

  // ----- Covariance prediction (Eq. 6) -----
  // P[k+1|k] = P[k|k] + Ts^2 * sigma_w^2
  // SIGMA_W2 here is "sigma_w^2" from the handout.
  float P_pred = P + (Ts * Ts) * SIGMA_W2;

  // ----- Measurement update (Eq. 8) -----
  // K = P_pred / (P_pred + Ts^2 * sigma_v^2)
  // residual = theta_acc - theta_pred
  // theta_hat = theta_pred + K * residual
  float denom = P_pred + (Ts * Ts) * SIGMA_V2;
  float K = (denom > 1e-12f) ? (P_pred / denom) : 0.0f;

  float residual = roll_acc_deg - theta_pred_deg;
  theta_hat_deg = theta_pred_deg + K * residual;

  // ----- Covariance update (Eq. 9) -----
  // P = (1 - K) * P_pred
  P = (1.0f - K) * P_pred;

  // Keep KF angle bounded (optional)
  if (theta_hat_deg > 180.0f) theta_hat_deg -= 360.0f;
  if (theta_hat_deg < -180.0f) theta_hat_deg += 360.0f;

  // =============================================================================
  // Serial Plotter output: print all 3 estimates
  // =============================================================================
  // IMPORTANT: Serial Plotter likes "name:value" format. Multiple signals can be
  // printed on one line separated by spaces.
  Serial.print("roll_gyro_deg:"); Serial.print(roll_gyro_deg, 6);
  Serial.print(" roll_acc_deg:"); Serial.print(roll_acc_deg, 6);
  Serial.print(" roll_kf_deg:");  Serial.println(theta_hat_deg, 6);

  delay(LOOP_DELAY_MS);
}
