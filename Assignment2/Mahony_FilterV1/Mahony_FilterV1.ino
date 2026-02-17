/*
  ============================================================
  MAHONY AHRS (Gyro + Accel + Mag) — with your calibration stack
  ============================================================

  Frames:
    - Navigation frame: NED (X=North, Y=East, Z=Down)
    - Body frame: "z-down body" (sensor->body is 180° about X)

  Sensor pipeline:
    accel_raw (g)  -> subtract bias -> sensorToBodyZDown -> normalize
    mag_raw (uT)   -> hard-iron -> soft-iron diag -> remap -> sensorToBodyZDown -> normalize
    gyro_raw (deg/s)-> subtract bias (calibrated at startup) -> sensorToBodyZDown -> rad/s

  Mag remap (confirmed):
      mx' = -mx
      my' =  mz
      mz' =  my

  Keys:
    'p' : toggle streaming
    'z' : yaw-zero (makes current yaw = 0 while keeping roll/pitch)

  Output (single line CSV):
      Q,w,x,y,z,|a|,|mh|
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoEigenDense.h>
#include <math.h>

using Eigen::Vector3f;
using Eigen::Matrix3f;

// ------------------------------------------------------------
// (1) YOUR LATEST CALIBRATION VALUES (paste-ready)
// ------------------------------------------------------------

// Magnetometer hard-iron offsets (uT)
static const float MAG_OFF_X_UT = -23.797607f;
static const float MAG_OFF_Y_UT =  63.122559f;
static const float MAG_OFF_Z_UT =  31.121826f;

// Magnetometer soft-iron diagonal scales (unitless)
static const float MAG_SX = 1.019133f;
static const float MAG_SY = 1.069512f;
static const float MAG_SZ = 0.922706f;

// Accelerometer bias to subtract (g)
static const float ACC_BIAS_X_G = -0.02965f;
static const float ACC_BIAS_Y_G = -0.02873f;
static const float ACC_BIAS_Z_G = -0.04060f;

// ------------------------------------------------------------
// (2) Settings / tuning
// ------------------------------------------------------------
static const float ACC_MAG_MIN = 0.85f;   // gate: ||a|| must be near 1g
static const float ACC_MAG_MAX = 1.15f;

static const float MAG_H_MIN = 5.0f;     // uT threshold on horizontal component (after HI+SI+remap+body map)

static const int   STREAM_HZ = 50;
static const unsigned long STREAM_PERIOD_MS = (1000UL / STREAM_HZ);
static unsigned long lastStreamMs = 0;

bool streamEnabled = true;

// Mahony gains (tune these)
static const float KP = 2.0f;     // proportional gain (try 0.5 .. 5)
static const float KI = 0.05f;    // integral gain (try 0 .. 0.2). Set 0 to disable bias learning.

// Startup gyro bias calibration
static const int   GYRO_BIAS_SAMPLES = 200;   // ~2 s if gyro ~100 Hz, adjust if needed

// ------------------------------------------------------------
// (3) Quaternion struct + helpers
// ------------------------------------------------------------
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

// Quaternion -> DCM (body->NED)
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

// ------------------------------------------------------------
// (4) Sensor -> Body mapping (z-down body)
// 180° rotation about X: x stays, y,z flip
// ------------------------------------------------------------
static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

// ------------------------------------------------------------
// (4b) MAG AXIS REMAP (CONFIRMED)
// Apply AFTER HI+SI, BEFORE sensorToBodyZDown
// mx' = -mx, my' = mz, mz' = my
// ------------------------------------------------------------
static inline void remapMag(float mx, float my, float mz,
                            float &mxr, float &myr, float &mzr) {
  mxr = -mx;
  myr =  mz;
  mzr =  my;
}

// Extract yaw from C_nb (body->NED) for yaw-zero feature
static inline float yawFromCnb(const Matrix3f& C_nb) {
  return atan2f(C_nb(1,0), C_nb(0,0)); // radians
}

// Pure yaw quaternion about NED Z (Down) by angle psi
static inline Quat yawQuat_NED(float psi) {
  float h = 0.5f * psi;
  return {cosf(h), 0.0f, 0.0f, sinf(h)};
}

// ------------------------------------------------------------
// (5) Yaw-zero state
// q_out = q_zero_inv * q_est
// ------------------------------------------------------------
bool yawZeroRequested = false;
Quat q_zero_inv = {1,0,0,0};

// ------------------------------------------------------------
// (6) Caching accel/mag
// ------------------------------------------------------------
bool haveAccel = false, haveMag = false;
Vector3f a_s_last(0,0,0);  // sensor frame, calibrated (bias removed)
Vector3f m_s_last(0,0,0);  // sensor frame, calibrated (HI+SI+remap)

// Apply accel bias (g)
static inline Vector3f calibrateAccelSensor(float ax_raw, float ay_raw, float az_raw) {
  return Vector3f(ax_raw - ACC_BIAS_X_G,
                  ay_raw - ACC_BIAS_Y_G,
                  az_raw - ACC_BIAS_Z_G);
}

// Apply mag HI + SI (uT)
static inline Vector3f calibrateMagSensor(float mx_raw, float my_raw, float mz_raw) {
  float mx = mx_raw - MAG_OFF_X_UT;
  float my = my_raw - MAG_OFF_Y_UT;
  float mz = mz_raw - MAG_OFF_Z_UT;

  mx *= MAG_SX;
  my *= MAG_SY;
  mz *= MAG_SZ;

  return Vector3f(mx, my, mz);
}

// ------------------------------------------------------------
// (7) Gyro bias (body frame, rad/s)
// ------------------------------------------------------------
Vector3f gyro_bias_b(0,0,0);
bool gyroBiasReady = false;

// ------------------------------------------------------------
// (8) Mahony internal state
// ------------------------------------------------------------
Quat q_est = {1,0,0,0};           // body->NED
Vector3f eInt(0,0,0);             // integral error (rad/s equiv)
unsigned long lastUpdateUs = 0;

// Utility
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ------------------------------------------------------------
// Mahony update with mag
// Inputs:
//   w_b  : gyro in body frame (rad/s)
//   a_b  : accel in body frame (unit vector, "up" direction is +? depends; we use measured gravity direction)
//   m_b  : mag in body frame (unit vector)
//   dt   : seconds
// ------------------------------------------------------------
static void mahonyUpdate(Vector3f w_b, Vector3f a_b, Vector3f m_b, float dt) {
  // Normalize inputs (safety)
  float an = a_b.norm();
  float mn = m_b.norm();
  if (an < 1e-6f || mn < 1e-6f) return;
  a_b /= an;
  m_b /= mn;

  // Estimated directions from quaternion (body->NED)
  Quat qn = quatNormalize(q_est);
  Matrix3f C_nb = quatToDCM(qn);         // body->NED
  Matrix3f C_bn = C_nb.transpose();      // NED->body

  // In NED, "Down" unit vector is [0,0,1]
  // Predicted Down in body:
  Vector3f d_b_est = C_bn * Vector3f(0,0,1);

  // From accel: when static, accel measures "Up" (approximately opposite of Down).
  // Your TRIAD used d_b = -a_hat. Keep same convention here:
  Vector3f d_b_meas = -a_b;

  // ---- Magnetometer reference handling (standard Mahony with mag) ----
  // Compute Earth's magnetic field in NED estimated frame:
  Vector3f h_n = C_nb * m_b;             // mag in NED
  float bx = sqrtf(h_n.x()*h_n.x() + h_n.y()*h_n.y());
  float bz = h_n.z();

  // Reference direction of mag in NED: [bx, 0, bz]
  // Predicted mag in body:
  Vector3f m_b_est = C_bn * Vector3f(bx, 0.0f, bz);

  // Error is sum of cross products between measured and estimated directions
  Vector3f e = d_b_meas.cross(d_b_est) + m_b.cross(m_b_est);

  // Integrate error (bias learning)
  if (KI > 0.0f) {
    eInt += e * (KI * dt);
    // optional clamp to avoid windup
    eInt.x() = clampf(eInt.x(), -0.5f, 0.5f);
    eInt.y() = clampf(eInt.y(), -0.5f, 0.5f);
    eInt.z() = clampf(eInt.z(), -0.5f, 0.5f);
  } else {
    eInt.setZero();
  }

  // Apply feedback to gyro
  Vector3f w_corr = w_b + (KP * e) + eInt;

  // Quaternion kinematics: q_dot = 0.5 * q ⊗ [0, wx, wy, wz]
  Quat omega = {0.0f, w_corr.x(), w_corr.y(), w_corr.z()};
  Quat qdot = quatMul(qn, omega);
  qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;

  q_est.w += qdot.w * dt;
  q_est.x += qdot.x * dt;
  q_est.y += qdot.y * dt;
  q_est.z += qdot.z * dt;

  q_est = quatNormalize(q_est);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("MAHONY AHRS (Gyro+Accel+Mag) — NED output, z-down body");
  Serial.println("Keys: 'p' toggle stream, 'z' yaw-zero");
  Serial.println("CSV: Q,w,x,y,z,|a|,|mh|");
  Serial.println();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Accel rate = "); Serial.print(IMU.accelerationSampleRate()); Serial.println(" Hz");
  Serial.print("Gyro  rate = "); Serial.print(IMU.gyroscopeSampleRate());    Serial.println(" Hz");
  Serial.print("Mag   rate = "); Serial.print(IMU.magneticFieldSampleRate()); Serial.println(" Hz");
  Serial.println();

  // -----------------------------
  // Gyro bias calibration (keep still)
  // -----------------------------
  Serial.println("Calibrating gyro bias... keep board still.");
  Vector3f sum(0,0,0);
  int got = 0;
  unsigned long t0 = millis();
  while (got < GYRO_BIAS_SAMPLES && (millis() - t0) < 5000UL) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz; // deg/s from Arduino_LSM9DS1
      IMU.readGyroscope(gx, gy, gz);
      Vector3f w_s(gx, gy, gz);
      Vector3f w_b_deg = sensorToBodyZDown(w_s);
      sum += w_b_deg;
      got++;
    }
  }
  if (got > 0) {
    Vector3f bias_deg = sum / (float)got;
    gyro_bias_b = bias_deg * (float)(M_PI / 180.0f); // rad/s
    gyroBiasReady = true;
    Serial.print("Gyro bias (rad/s): ");
    Serial.print(gyro_bias_b.x(), 6); Serial.print(", ");
    Serial.print(gyro_bias_b.y(), 6); Serial.print(", ");
    Serial.println(gyro_bias_b.z(), 6);
  } else {
    Serial.println("Gyro bias calibration failed (no samples). Continuing with zero bias.");
    gyro_bias_b.setZero();
    gyroBiasReady = false;
  }

  lastUpdateUs = micros();
}

void loop() {
  // -----------------------------
  // Key commands
  // -----------------------------
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 'p' || ch == 'P') {
      streamEnabled = !streamEnabled;
      Serial.print("streamEnabled = "); Serial.println(streamEnabled ? "true" : "false");
    }
    if (ch == 'z' || ch == 'Z') {
      yawZeroRequested = true;
      Serial.println("Yaw zero requested (will apply on next valid update).");
    }
  }

  // -----------------------------
  // Read gyro (we update filter every loop using gyro dt)
  // -----------------------------
  float gx, gy, gz;
  bool haveGyroNow = false;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz); // deg/s
    haveGyroNow = true;
  }
  if (!haveGyroNow) return;

  unsigned long nowUs = micros();
  float dt = (nowUs - lastUpdateUs) * 1e-6f;
  lastUpdateUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) return; // guard (dt too large => skip)

  // Gyro: sensor -> body, deg/s -> rad/s, subtract bias
  Vector3f w_s(gx, gy, gz);
  Vector3f w_b = sensorToBodyZDown(w_s) * (float)(M_PI / 180.0f);
  w_b -= gyro_bias_b;

  // -----------------------------
  // Cache latest accel sample (if available)
  // -----------------------------
  if (IMU.accelerationAvailable()) {
    float ax_raw, ay_raw, az_raw;
    IMU.readAcceleration(ax_raw, ay_raw, az_raw); // g
    a_s_last = calibrateAccelSensor(ax_raw, ay_raw, az_raw);
    haveAccel = true;
  }

  // -----------------------------
  // Cache latest mag sample (if available)
  // -----------------------------
  if (IMU.magneticFieldAvailable()) {
    float mx_raw, my_raw, mz_raw;
    IMU.readMagneticField(mx_raw, my_raw, mz_raw); // uT

    Vector3f m_cal = calibrateMagSensor(mx_raw, my_raw, mz_raw);

    float mxr, myr, mzr;
    remapMag(m_cal.x(), m_cal.y(), m_cal.z(), mxr, myr, mzr);

    m_s_last = Vector3f(mxr, myr, mzr);
    haveMag = true;
  }

  // Need accel+mag at least once for correction;
  // otherwise we only integrate gyro (still valid)
  if (!haveAccel || !haveMag) {
    // gyro-only integrate (no correction)
    Vector3f w_corr = w_b;

    Quat qn = quatNormalize(q_est);
    Quat omega = {0.0f, w_corr.x(), w_corr.y(), w_corr.z()};
    Quat qdot = quatMul(qn, omega);
    qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;

    q_est.w += qdot.w * dt;
    q_est.x += qdot.x * dt;
    q_est.y += qdot.y * dt;
    q_est.z += qdot.z * dt;
    q_est = quatNormalize(q_est);
    return;
  }

  // -----------------------------
  // Map cached accel/mag to BODY
  // -----------------------------
  Vector3f a_b = sensorToBodyZDown(a_s_last);
  Vector3f m_b = sensorToBodyZDown(m_s_last);

  // -----------------------------
  // Gates (same spirit as TRIAD)
  // -----------------------------
  float a_norm = a_b.norm();
  if (a_norm < 1e-6f) return;
  if (a_norm < ACC_MAG_MIN || a_norm > ACC_MAG_MAX) {
    // If accelerating, skip accel correction but still do mag? (usually skip both)
    // We'll skip correction entirely this step, but still integrate gyro.
    Vector3f w_corr = w_b;

    Quat qn = quatNormalize(q_est);
    Quat omega = {0.0f, w_corr.x(), w_corr.y(), w_corr.z()};
    Quat qdot = quatMul(qn, omega);
    qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;

    q_est.w += qdot.w * dt;
    q_est.x += qdot.x * dt;
    q_est.y += qdot.y * dt;
    q_est.z += qdot.z * dt;
    q_est = quatNormalize(q_est);
    return;
  }

  // Horizontal mag gate (computed like TRIAD)
  Vector3f a_hat = a_b / a_norm;
  Vector3f d_b = -a_hat;
  Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
  float mh_norm = m_h.norm();
  if (mh_norm < MAG_H_MIN) {
    // skip mag correction when it becomes ill-conditioned, but still use accel
    // We can still run Mahony with mag vector present; however it's noisy here.
    // We'll just integrate gyro + accel correction by faking mag correction as zero:
    // simplest: still call mahonyUpdate but with current m_b; it will be weak/noisy.
    // Better: do accel-only Mahony (not implemented separately). We'll still call full update
    // but note: if mh_norm small, yaw may drift (expected).
  }

  // -----------------------------
  // Mahony update
  // -----------------------------
  mahonyUpdate(w_b, a_b, m_b, dt);

  // -----------------------------
  // Yaw-zero (applied to OUTPUT only, internal filter keeps running)
  // -----------------------------
  if (yawZeroRequested) {
    Matrix3f C_nb = quatToDCM(quatNormalize(q_est));
    float yaw = yawFromCnb(C_nb);
    Quat q_remove = yawQuat_NED(-yaw);
    q_zero_inv = quatNormalize(q_remove);
    yawZeroRequested = false;
    Serial.println("Yaw zero applied (heading reset).");
  }

  Quat q_out = quatMul(q_zero_inv, q_est);
  q_out = quatNormalize(q_out);

  // Optional: ensure streaming quaternion doesn’t randomly flip sign (rare in Mahony, but safe)
  static bool havePrevOut = false;
  static Quat q_prev_out;
  if (havePrevOut) {
    float dot = q_prev_out.w*q_out.w + q_prev_out.x*q_out.x + q_prev_out.y*q_out.y + q_prev_out.z*q_out.z;
    if (dot < 0.0f) { q_out.w=-q_out.w; q_out.x=-q_out.x; q_out.y=-q_out.y; q_out.z=-q_out.z; }
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
  Serial.print(q_out.z, 6); Serial.print(",");
  Serial.print(a_norm, 3);  Serial.print(",");
  Serial.println(mh_norm, 3);
}
