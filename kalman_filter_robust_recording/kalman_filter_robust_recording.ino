/*
  SC651 Assignment - Part (c)/(d): Kalman Filtering for Roll Angle Estimate
  OPTION C: RAM BUFFER LOGGING (no SPIFFS writes during motion)
  --------------------------------------------------------------------
  What this does:
    - Runs your gyro+accel+KF loop normally (no flash I/O during motion).
    - Stores selected CSV columns in RAM arrays (buffer).
    - After STOP_TIME_S is reached, it DUMPS the CSV over Serial.
    - You can also press 'd' anytime to dump what has been recorded so far.
    - Press 'r' to reset buffer and start recording again.

  Output CSV columns (minimal but enough for Part (c)/(d) + residual analysis):
    time_s, Ts_s, omega_used_deg_s, roll_gyro_deg, roll_acc_deg,
    roll_kf_deg, theta_pred_deg, residual_deg, K, P_pred, P

  Notes:
    - This avoids the lag/corruption you saw from SPIFFS writes.
    - If you want MORE columns, you can add them (at cost of RAM).
*/

#include <Arduino_LSM9DS1.h>
#include <math.h>

// -----------------------------------------------------------------------------
// USER SETTINGS
// -----------------------------------------------------------------------------

static const float STOP_TIME_S = 60.0f;   // 0 -> record forever (until buffer full)

// Which gyro axis is your roll-rate?
enum RollAxis { GX_AXIS, GY_AXIS, GZ_AXIS };
static const RollAxis ROLL_AXIS = GX_AXIS;

// Accelerometer sign flip to match your convention (gyro CCW +, accel CCW -)
static const bool FLIP_ACCEL_SIGN = true;

// Optional: subtract constant gyro bias
static const bool  USE_GYRO_BIAS_CORRECTION = true;
static const float GYRO_BIAS_DEG_S = 1.827666f;

// Sampling time
static const bool  USE_FIXED_TS = false;
static const float FIXED_TS_S   = 0.01f;   // fallback / fixed Ts

// Kalman noise variances (tuning knobs for part d)
static const float SIGMA_W2 = 1.00f;  // (deg/s)^2  process noise (gyro model)
static const float SIGMA_V2 = 0.50f;  // (deg)^2    measurement noise (acc roll)

// Initial covariance
static const float P0 = 10.0f;

// Logging rate control
static const uint32_t LOOP_DELAY_MS = 5;

// -----------------------------------------------------------------------------
// RAM LOG BUFFER SETTINGS
// -----------------------------------------------------------------------------

// Estimate your sample rate ~ 100 Hz -> 60s => ~6000 samples
// We'll allocate a safe buffer. Increase if needed (watch RAM).
static const uint32_t MAX_SAMPLES = 8000;

// If set true: when buffer is full, stop recording.
// If set false: circular buffer overwrite oldest samples.
static const bool STOP_WHEN_FULL = true;

// -----------------------------------------------------------------------------
// STATE VARIABLES
// -----------------------------------------------------------------------------

float roll_gyro_deg = 0.0f;   // gyro integrated roll
float roll_acc_deg  = 0.0f;   // accel roll
float theta_hat_deg = 0.0f;   // KF estimate
float P             = P0;     // KF covariance

unsigned long t_prev_us  = 0;
unsigned long t_start_ms = 0;

bool logging_active = true;
bool dumped_once    = false;

// -----------------------------------------------------------------------------
// RAM BUFFERS
// -----------------------------------------------------------------------------
static float buf_time_s[MAX_SAMPLES];
static float buf_Ts_s[MAX_SAMPLES];
static float buf_omega_used[MAX_SAMPLES];
static float buf_roll_gyro[MAX_SAMPLES];
static float buf_roll_acc[MAX_SAMPLES];
static float buf_roll_kf[MAX_SAMPLES];
static float buf_theta_pred[MAX_SAMPLES];
static float buf_residual[MAX_SAMPLES];
static float buf_K[MAX_SAMPLES];
static float buf_P_pred[MAX_SAMPLES];
static float buf_P[MAX_SAMPLES];

static uint32_t write_idx = 0;   // next write position
static uint32_t n_samples = 0;   // number of valid samples in buffer
static bool buffer_full = false;

// -----------------------------------------------------------------------------
// Helper: accelerometer roll (Eq. 7) in degrees
// theta_acc = atan2(ay, sqrt(ax^2 + az^2))
// NOTE: Produces [-90, +90] deg range (cannot represent beyond 90).
// -----------------------------------------------------------------------------
static float accel_roll_deg(float ax, float ay, float az) {
  float denom = sqrtf(ax * ax + az * az);
  float theta_rad = atan2f(ay, denom);
  float theta_deg = theta_rad * (180.0f / 3.14159265f);
  if (FLIP_ACCEL_SIGN) theta_deg = -theta_deg;
  return theta_deg;
}

// -----------------------------------------------------------------------------
// Buffer management
// -----------------------------------------------------------------------------
static void buffer_reset() {
  write_idx = 0;
  n_samples = 0;
  buffer_full = false;
  dumped_once = false;

  roll_gyro_deg = 0.0f;
  roll_acc_deg  = 0.0f;
  theta_hat_deg = 0.0f;
  P             = P0;

  t_prev_us  = micros();
  t_start_ms = millis();

  logging_active = true;

  Serial.println("\n[RESET] Buffer cleared. Recording restarted.");
}

static void buffer_push(
  float time_s, float Ts_s, float omega_used_deg_s,
  float roll_gyro, float roll_acc, float roll_kf,
  float theta_pred, float residual, float K,
  float P_pred, float P_upd
) {
  buf_time_s[write_idx]      = time_s;
  buf_Ts_s[write_idx]        = Ts_s;
  buf_omega_used[write_idx]  = omega_used_deg_s;
  buf_roll_gyro[write_idx]   = roll_gyro;
  buf_roll_acc[write_idx]    = roll_acc;
  buf_roll_kf[write_idx]     = roll_kf;
  buf_theta_pred[write_idx]  = theta_pred;
  buf_residual[write_idx]    = residual;
  buf_K[write_idx]           = K;
  buf_P_pred[write_idx]      = P_pred;
  buf_P[write_idx]           = P_upd;

  // update counts
  if (n_samples < MAX_SAMPLES) n_samples++;

  // advance index
  write_idx++;
  if (write_idx >= MAX_SAMPLES) {
    write_idx = 0;
    buffer_full = true;
    if (STOP_WHEN_FULL) logging_active = false;
  }
}

// Dump buffer in chronological order (handles circular buffer)
static void dump_buffer_csv() {
  Serial.println("\n--- CSV DUMP START ---");
  Serial.println("time_s,Ts_s,omega_used_deg_s,roll_gyro_deg,roll_acc_deg,roll_kf_deg,theta_pred_deg,residual_deg,K,P_pred,P");

  if (n_samples == 0) {
    Serial.println("--- CSV DUMP END ---\n");
    return;
  }

  uint32_t start = 0;
  if (buffer_full && !STOP_WHEN_FULL) {
    // circular mode: oldest is write_idx
    start = write_idx;
  } else {
    // stop-when-full mode OR not full: data starts at 0
    start = 0;
  }

  for (uint32_t i = 0; i < n_samples; i++) {
    uint32_t idx = (start + i) % MAX_SAMPLES;

    Serial.print(buf_time_s[idx], 6);      Serial.print(",");
    Serial.print(buf_Ts_s[idx], 6);        Serial.print(",");
    Serial.print(buf_omega_used[idx], 6);  Serial.print(",");
    Serial.print(buf_roll_gyro[idx], 6);   Serial.print(",");
    Serial.print(buf_roll_acc[idx], 6);    Serial.print(",");
    Serial.print(buf_roll_kf[idx], 6);     Serial.print(",");
    Serial.print(buf_theta_pred[idx], 6);  Serial.print(",");
    Serial.print(buf_residual[idx], 6);    Serial.print(",");
    Serial.print(buf_K[idx], 8);           Serial.print(",");
    Serial.print(buf_P_pred[idx], 8);      Serial.print(",");
    Serial.println(buf_P[idx], 8);
  }

  Serial.println("--- CSV DUMP END ---\n");
  dumped_once = true;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(800);

  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1! Check wiring / I2C.");
    while (1) { delay(100); }
  }

  t_prev_us  = micros();
  t_start_ms = millis();

  theta_hat_deg = 0.0f;
  P = P0;

  Serial.println("KF roll estimation + RAM-buffer logging started (NO SPIFFS).");
  Serial.print("STOP_TIME_S = "); Serial.println(STOP_TIME_S);
  Serial.print("MAX_SAMPLES = "); Serial.println(MAX_SAMPLES);
  Serial.println("Commands: 'd' dump CSV, 'r' reset & record again.");
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void loop() {
  // Handle serial commands (non-blocking)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;
    if (c == 'd') dump_buffer_csv();
    if (c == 'r') buffer_reset();
  }

  if (!logging_active) {
    // If stop time reached or buffer full, auto-dump once (optional)
    if (!dumped_once && (STOP_TIME_S > 0.0f)) {
      dump_buffer_csv();
    }
    delay(50);
    return;
  }

  // Stop after STOP_TIME_S
  float t_elapsed_s = (millis() - t_start_ms) * 1e-3f;
  if (STOP_TIME_S > 0.0f && t_elapsed_s >= STOP_TIME_S) {
    logging_active = false;
    // auto dump once when finished
    dump_buffer_csv();
    Serial.println("[DONE] Recording stopped. Press 'r' to record again.");
    return;
  }

  // Wait for gyro sample
  if (!IMU.gyroscopeAvailable()) {
    delay(1);
    return;
  }

  // --------- Ts (dt) ----------
  float Ts = FIXED_TS_S;
  if (!USE_FIXED_TS) {
    unsigned long t_now_us = micros();
    Ts = (t_now_us - t_prev_us) * 1e-6f;
    t_prev_us = t_now_us;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = FIXED_TS_S;
  }

  // --------- Read gyro ----------
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  float omega_deg_s = 0.0f;
  if (ROLL_AXIS == GX_AXIS) omega_deg_s = gx;
  if (ROLL_AXIS == GY_AXIS) omega_deg_s = gy;
  if (ROLL_AXIS == GZ_AXIS) omega_deg_s = gz;

  float omega_used_deg_s = omega_deg_s;
  if (USE_GYRO_BIAS_CORRECTION) omega_used_deg_s -= GYRO_BIAS_DEG_S;

  // --------- Gyro-only integration ----------
  roll_gyro_deg += omega_used_deg_s * Ts;
  if (roll_gyro_deg > 180.0f) roll_gyro_deg -= 360.0f;
  if (roll_gyro_deg < -180.0f) roll_gyro_deg += 360.0f;

  // --------- Read accel if available ----------
  float ax = NAN, ay = NAN, az = NAN;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    roll_acc_deg = accel_roll_deg(ax, ay, az);
  }

  // --------- Kalman filter (scalar) ----------
  float theta_pred_deg = theta_hat_deg + Ts * omega_used_deg_s;
  float P_pred = P + (Ts * Ts) * SIGMA_W2;

  float denom = P_pred + (Ts * Ts) * SIGMA_V2;
  float K = (denom > 1e-12f) ? (P_pred / denom) : 0.0f;

  float residual = roll_acc_deg - theta_pred_deg;
  theta_hat_deg = theta_pred_deg + K * residual;

  P = (1.0f - K) * P_pred;

  if (theta_hat_deg > 180.0f) theta_hat_deg -= 360.0f;
  if (theta_hat_deg < -180.0f) theta_hat_deg += 360.0f;

  // --------- Store into RAM buffer ----------
  buffer_push(
    t_elapsed_s, Ts, omega_used_deg_s,
    roll_gyro_deg, roll_acc_deg, theta_hat_deg,
    theta_pred_deg, residual, K,
    P_pred, P
  );

  delay(LOOP_DELAY_MS);
}
