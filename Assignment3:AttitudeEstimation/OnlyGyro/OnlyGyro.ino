/*
  ============================================================
  GYRO-ONLY ORIENTATION — LOW-DRIFT VERSION (as much as possible)
  ============================================================

  What’s added vs simple gyro integrator:
    ✅ Better bias calibration (more samples + stillness check)
    ✅ Optional re-bias key: 'b' (place flat+still, press b)
    ✅ Gyro low-pass filtering (1st-order IIR)
    ✅ Gyro deadband (ignore tiny rates that are mostly noise)
    ✅ Quaternion update via incremental rotation (axis-angle)
       (more stable than qdot Euler integration)

  Keys:
    'p' : toggle streaming
    'z' : yaw-zero (output reference only)
    'b' : re-calibrate gyro bias (keep still for ~3s)

  Output:
    Q,w,x,y,z
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoEigenDense.h>
#include <math.h>

using Eigen::Vector3f;
using Eigen::Matrix3f;

// -----------------------------
// Streaming
// -----------------------------
static const int   STREAM_HZ = 50;
static const unsigned long STREAM_PERIOD_MS = (1000UL / STREAM_HZ);
static unsigned long lastStreamMs = 0;
bool streamEnabled = true;

// -----------------------------
// Bias calibration
// -----------------------------
static const int   GYRO_BIAS_SAMPLES = 600;   // ~3–6s depending on gyro rate
static const float STILL_RATE_DPS_MAX = 1.5f; // if |w| > this during calibration, ignore sample

// -----------------------------
// Filtering + deadband
// -----------------------------
static const float GYRO_LPF_ALPHA = 0.20f;    // 0..1  (higher = less smoothing)
static const float GYRO_DEADBAND_DPS = 0.25f; // ignore tiny rates (noise)

// -----------------------------
// Quaternion
// -----------------------------
struct Quat { float w, x, y, z; }; // scalar-first

static inline Quat quatNormalize(const Quat& q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-12f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static inline Matrix3f quatToDCM(const Quat& qn) {
  const float w = qn.w, x = qn.x, y = qn.y, z = qn.z;

  Matrix3f R;
  R(0,0) = 1.0f - 2.0f*(y*y + z*z);
  R(0,1) = 2.0f*(x*y - z*w);
  R(0,2) = 2.0f*(x*z + y*w);

  R(1,0) = 2.0f*(x*y + z*w);
  R(1,1) = 1.0f - 2.0f*(x*x + z*z);
  R(1,2) = 2.0f*(y*z - x*w);

  R(2,0) = 2.0f*(x*z - y*w);
  R(2,1) = 2.0f*(y*z + x*w);
  R(2,2) = 1.0f - 2.0f*(x*x + y*y);
  return R;
}

// -----------------------------
// Frame mapping
// -----------------------------
static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  // 180° about X: x stays, y,z flip
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

static inline float yawFromCnb(const Matrix3f& C_nb) {
  return atan2f(C_nb(1,0), C_nb(0,0));
}

static inline Quat yawQuat_NED(float psi) {
  float h = 0.5f * psi;
  return {cosf(h), 0.0f, 0.0f, sinf(h)};
}

// -----------------------------
// State
// -----------------------------
Quat q_est = {1,0,0,0};              // body->NED
Vector3f gyro_bias_b(0,0,0);         // rad/s
Vector3f gyro_filt_b(0,0,0);         // rad/s

unsigned long lastUpdateUs = 0;

// Output yaw-zero
bool yawZeroRequested = false;
Quat q_zero_inv = {1,0,0,0};

// Commands
bool doRebias = false;

// -----------------------------
// Helpers
// -----------------------------
static inline float deg2rad(float d) { return d * (float)(M_PI / 180.0f); }

static inline float softDeadband(float x, float dead) {
  float ax = fabsf(x);
  if (ax < dead) return 0.0f;
  // subtract deadband while keeping sign (prevents discontinuity)
  return (x > 0.0f) ? (x - dead) : (x + dead);
}

// Incremental quaternion from angular rate vector w (rad/s) over dt
// dq = [cos(|dtheta|/2), u*sin(|dtheta|/2)], where dtheta = w*dt
static Quat deltaQuatFromOmega(Vector3f w, float dt) {
  Vector3f dtheta = w * dt;
  float angle = dtheta.norm();
  if (angle < 1e-9f) return {1,0,0,0};

  Vector3f u = dtheta / angle;
  float h = 0.5f * angle;
  float s = sinf(h);

  return quatNormalize({cosf(h), u.x()*s, u.y()*s, u.z()*s});
}

// -----------------------------
// Bias calibration (keep board still)
// -----------------------------
static bool calibrateGyroBias(Vector3f& bias_out_rad, Vector3f& filt_reset_rad) {
  Serial.println("Gyro bias calibration: keep board STILL...");
  Vector3f sum(0,0,0);
  int used = 0;
  unsigned long t0 = millis();

  while (used < GYRO_BIAS_SAMPLES && (millis() - t0) < 8000UL) {
    if (!IMU.gyroscopeAvailable()) continue;

    float gx, gy, gz; // deg/s
    IMU.readGyroscope(gx, gy, gz);

    Vector3f w_s(gx, gy, gz);
    Vector3f w_b_deg = sensorToBodyZDown(w_s);

    float wmag = w_b_deg.norm();
    if (wmag > STILL_RATE_DPS_MAX) {
      // user moved during calib; just skip this sample
      continue;
    }

    sum += w_b_deg;
    used++;
  }

  if (used < (GYRO_BIAS_SAMPLES / 3)) {
    Serial.println("Bias calibration FAILED (not enough still samples).");
    return false;
  }

  Vector3f bias_deg = sum / (float)used;
  bias_out_rad = bias_deg * (float)(M_PI / 180.0f);

  // Reset filter state to avoid step when bias changes
  filt_reset_rad.setZero();

  Serial.print("Gyro bias (rad/s): ");
  Serial.print(bias_out_rad.x(), 6); Serial.print(", ");
  Serial.print(bias_out_rad.y(), 6); Serial.print(", ");
  Serial.println(bias_out_rad.z(), 6);

  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("GYRO-ONLY ORIENTATION — LOW-DRIFT VERSION");
  Serial.println("Keys: 'p' toggle stream, 'z' yaw-zero, 'b' re-bias gyro");
  Serial.println("CSV: Q,w,x,y,z");
  Serial.println();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Gyro rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();

  // Initial bias calibration
  Vector3f filt_reset(0,0,0);
  if (!calibrateGyroBias(gyro_bias_b, filt_reset)) {
    gyro_bias_b.setZero();
    gyro_filt_b.setZero();
  } else {
    gyro_filt_b = filt_reset;
  }

  lastUpdateUs = micros();
  Serial.println();
}

void loop() {
  // -----------------------------
  // Key commands
  // -----------------------------
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 'p' || ch == 'P') {
      streamEnabled = !streamEnabled;
      Serial.print("streamEnabled = ");
      Serial.println(streamEnabled ? "true" : "false");
    } else if (ch == 'z' || ch == 'Z') {
      yawZeroRequested = true;
      Serial.println("Yaw zero requested (next update).");
    } else if (ch == 'b' || ch == 'B') {
      doRebias = true;
    }
  }

  // Re-bias if requested (blocking for a few seconds)
  if (doRebias) {
    Vector3f filt_reset(0,0,0);
    bool ok = calibrateGyroBias(gyro_bias_b, filt_reset);
    if (ok) gyro_filt_b = filt_reset;
    doRebias = false;
    lastUpdateUs = micros();
    return;
  }

  // -----------------------------
  // Read gyro
  // -----------------------------
  if (!IMU.gyroscopeAvailable()) return;

  float gx, gy, gz; // deg/s
  IMU.readGyroscope(gx, gy, gz);

  unsigned long nowUs = micros();
  float dt = (nowUs - lastUpdateUs) * 1e-6f;
  lastUpdateUs = nowUs;

  if (dt <= 0.0f || dt > 0.1f) return;

  // sensor -> body, deg/s -> rad/s
  Vector3f w_s(gx, gy, gz);
  Vector3f w_b = sensorToBodyZDown(w_s) * (float)(M_PI / 180.0f);

  // subtract bias
  w_b -= gyro_bias_b;

  // deadband in deg/s space (conceptually), but we’re already rad/s:
  float dead_rad = deg2rad(GYRO_DEADBAND_DPS);
  w_b.x() = softDeadband(w_b.x(), dead_rad);
  w_b.y() = softDeadband(w_b.y(), dead_rad);
  w_b.z() = softDeadband(w_b.z(), dead_rad);

  // low-pass filter
  gyro_filt_b = (1.0f - GYRO_LPF_ALPHA) * gyro_filt_b + (GYRO_LPF_ALPHA) * w_b;

  // -----------------------------
  // Integrate quaternion with incremental rotation
  // q_new = q ⊗ dq   (body->NED, omega expressed in body)
  // -----------------------------
  Quat qn = quatNormalize(q_est);
  Quat dq = deltaQuatFromOmega(gyro_filt_b, dt);
  q_est = quatMul(qn, dq);
  q_est = quatNormalize(q_est);

  // -----------------------------
  // Yaw-zero (output reference)
  // -----------------------------
  if (yawZeroRequested) {
    Matrix3f C_nb = quatToDCM(quatNormalize(q_est));
    float yaw = yawFromCnb(C_nb);

    // Normal yaw-zero = remove yaw. If you want reverse behavior, use +yaw.
    Quat q_remove = yawQuat_NED(-yaw);
    q_zero_inv = quatNormalize(q_remove);

    yawZeroRequested = false;
    Serial.println("Yaw zero applied.");
  }

  Quat q_out = quatMul(q_zero_inv, q_est);
  q_out = quatNormalize(q_out);

  // Hemisphere continuity for clean plotting
  static bool havePrevOut = false;
  static Quat q_prev_out;
  if (havePrevOut) {
    float dot = q_prev_out.w*q_out.w + q_prev_out.x*q_out.x +
                q_prev_out.y*q_out.y + q_prev_out.z*q_out.z;
    if (dot < 0.0f) {
      q_out.w=-q_out.w; q_out.x=-q_out.x; q_out.y=-q_out.y; q_out.z=-q_out.z;
    }
  }
  q_prev_out = q_out;
  havePrevOut = true;

  // -----------------------------
  // Rate limit streaming
  // -----------------------------
  if (!streamEnabled) return;
  unsigned long nowMs = millis();
  if (nowMs - lastStreamMs < STREAM_PERIOD_MS) return;
  lastStreamMs = nowMs;

  // -----------------------------
  // Stream
  // -----------------------------
  Serial.print("Q,");
  Serial.print(q_out.w, 6); Serial.print(",");
  Serial.print(q_out.x, 6); Serial.print(",");
  Serial.print(q_out.y, 6); Serial.print(",");
  Serial.println(q_out.z, 6);
}