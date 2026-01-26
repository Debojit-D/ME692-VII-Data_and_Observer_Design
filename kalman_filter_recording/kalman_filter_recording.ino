/*
  SC651 Assignment - Part (c): Kalman Filtering for Roll Angle Estimate
  NO SPIFFS: stream CSV over Serial for 60 seconds, capture on PC.

  Output CSV columns:
    time_s,Ts_s,gx_deg_s,gy_deg_s,gz_deg_s,ax_g,ay_g,az_g,
    omega_roll_deg_s,omega_used_deg_s,roll_gyro_deg,roll_acc_deg,roll_kf_deg,
    theta_pred_deg,residual_deg,K,P_pred,P
*/

#include <Arduino_LSM9DS1.h>
#include <math.h>

// ---------------- USER SETTINGS ----------------
static const float STOP_TIME_S = 60.0f;

// Which gyro axis is your roll-rate? (must match part-a)
enum RollAxis { GX_AXIS, GY_AXIS, GZ_AXIS };
static const RollAxis ROLL_AXIS = GX_AXIS;

// Accelerometer sign flip to match your convention (gyro CCW +, accel CCW -)
static const bool FLIP_ACCEL_SIGN = true;

// Optional: subtract constant gyro bias
static const bool  USE_GYRO_BIAS_CORRECTION = true;
static const float GYRO_BIAS_DEG_S = 1.827666f;

// Sampling time
static const bool  USE_FIXED_TS = false;
static const float FIXED_TS_S   = 0.1f;  // fallback / fixed Ts

// Kalman noise variances (tuning for part d)
static const float SIGMA_W2 = 1.00f;  // (deg/s)^2
static const float SIGMA_V2 = 100.0f;  // (deg)^2

// Initial covariance P0 (Sigma_hat[0|0])
static const float P0 = 500.0f;

// Loop pacing
static const uint32_t LOOP_DELAY_MS = 5;

// Optional: reduce serial bandwidth (log every Nth sample)
static const uint32_t LOG_EVERY_N = 1;

// ---------------- STATE ----------------
float roll_gyro_deg = 0.0f;
float roll_acc_deg  = 0.0f;

float theta_hat_deg = 0.0f;  // theta_hat[k|k]
float P             = P0;    // Sigma_hat[k|k]

unsigned long t_prev_us  = 0;
unsigned long t_start_ms = 0;

uint32_t sampleCount = 0;
bool started = false;

// ---------------- Helpers ----------------
static float accel_roll_deg(float ax, float ay, float az) {
  float denom = sqrtf(ax * ax + az * az);
  float theta_rad = atan2f(ay, denom);
  float theta_deg = theta_rad * (180.0f / 3.14159265f);
  if (FLIP_ACCEL_SIGN) theta_deg = -theta_deg;
  return theta_deg;
}

void setup() {
  Serial.begin(115200);
  delay(800);

  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1!");
    while (1) delay(100);
  }

  t_prev_us  = micros();
  t_start_ms = millis();

  theta_hat_deg = 0.0f;
  P = P0;

  // CSV header (single line)
  Serial.println(
    "time_s,Ts_s,"
    "gx_deg_s,gy_deg_s,gz_deg_s,"
    "ax_g,ay_g,az_g,"
    "omega_roll_deg_s,omega_used_deg_s,"
    "roll_gyro_deg,roll_acc_deg,roll_kf_deg,"
    "theta_pred_deg,residual_deg,"
    "K,P_pred,P"
  );

  started = true;
}

void loop() {
  if (!started) return;

  float t_elapsed_s = (millis() - t_start_ms) * 1e-3f;
  if (t_elapsed_s >= STOP_TIME_S) {
    Serial.println("---DONE---");
    while (1) delay(1000);  // stop forever after 60s
  }

  if (!IMU.gyroscopeAvailable()) {
    delay(1);
    return;
  }

  // dt
  float Ts = FIXED_TS_S;
  if (!USE_FIXED_TS) {
    unsigned long t_now_us = micros();
    Ts = (t_now_us - t_prev_us) * 1e-6f;
    t_prev_us = t_now_us;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = FIXED_TS_S;
  }

  // gyro
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  float omega_deg_s = 0.0f;
  if (ROLL_AXIS == GX_AXIS) omega_deg_s = gx;
  if (ROLL_AXIS == GY_AXIS) omega_deg_s = gy;
  if (ROLL_AXIS == GZ_AXIS) omega_deg_s = gz;

  float omega_used_deg_s = omega_deg_s;
  if (USE_GYRO_BIAS_CORRECTION) omega_used_deg_s -= GYRO_BIAS_DEG_S;

  // gyro-only integration
  roll_gyro_deg += omega_used_deg_s * Ts;
  if (roll_gyro_deg > 180.0f) roll_gyro_deg -= 360.0f;
  if (roll_gyro_deg < -180.0f) roll_gyro_deg += 360.0f;

  // accel
  float ax = NAN, ay = NAN, az = NAN;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    roll_acc_deg = accel_roll_deg(ax, ay, az);
  }

  // KF
  float theta_pred_deg = theta_hat_deg + Ts * omega_used_deg_s;
  float P_pred = P + (Ts * Ts) * SIGMA_W2;

  float denom = P_pred + (Ts * Ts) * SIGMA_V2;
  float K = (denom > 1e-12f) ? (P_pred / denom) : 0.0f;

  float residual = roll_acc_deg - theta_pred_deg;
  theta_hat_deg = theta_pred_deg + K * residual;
  P = (1.0f - K) * P_pred;

  if (theta_hat_deg > 180.0f) theta_hat_deg -= 360.0f;
  if (theta_hat_deg < -180.0f) theta_hat_deg += 360.0f;

  // CSV row (optional downsample)
  sampleCount++;
  if ((sampleCount % LOG_EVERY_N) == 0) {
    // Use ONE print line to keep it fast
    Serial.print(t_elapsed_s, 6); Serial.print(",");
    Serial.print(Ts, 6);          Serial.print(",");

    Serial.print(gx, 6); Serial.print(",");
    Serial.print(gy, 6); Serial.print(",");
    Serial.print(gz, 6); Serial.print(",");

    Serial.print(ax, 6); Serial.print(",");
    Serial.print(ay, 6); Serial.print(",");
    Serial.print(az, 6); Serial.print(",");

    Serial.print(omega_deg_s, 6);      Serial.print(",");
    Serial.print(omega_used_deg_s, 6); Serial.print(",");

    Serial.print(roll_gyro_deg, 6); Serial.print(",");
    Serial.print(roll_acc_deg, 6);  Serial.print(",");
    Serial.print(theta_hat_deg, 6); Serial.print(",");

    Serial.print(theta_pred_deg, 6); Serial.print(",");
    Serial.print(residual, 6);       Serial.print(",");

    Serial.print(K, 8);      Serial.print(",");
    Serial.print(P_pred, 8); Serial.print(",");
    Serial.println(P, 8);
  }

  delay(LOOP_DELAY_MS);
}
