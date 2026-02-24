/*
  ============================================================
  MAHONY AHRS (Gyro + Accel + Mag) — FIXED + ROBUST
  ============================================================

  What’s fixed / improved vs your current Mahony:
    ✅ Keeps your exact calibration stack + z-down body mapping
    ✅ Uses accel-only Mahony when mag horizontal component is unreliable (mh_norm < MAG_H_MIN)
       → prevents noisy mag from wrecking yaw/attitude
    ✅ Optional one-time TRIAD-style initialization using accel+mag (fast lock-on at start)
    ✅ Integrator anti-windup tightened (prevents “stuck bias” / slow weird drift)
    ✅ Clean structure: integrate gyro every loop, apply correction when valid

  Frames:
    - Navigation frame: NED (X=North, Y=East, Z=Down)
    - Body frame: "z-down body" (sensor->body is 180° about X)

  Sensor pipeline:
    accel_raw (g)   -> subtract bias -> sensorToBodyZDown -> normalize
    mag_raw (uT)    -> hard-iron -> soft-iron diag -> remap -> sensorToBodyZDown
    gyro_raw (deg/s)-> startup bias -> sensorToBodyZDown -> rad/s

  Mag remap (confirmed):
      mx' = -mx
      my' =  my
      mz' =  mz

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

// Mahony gains (START HERE)
// NOTE: KP too high -> twitchy/jitter; KI too high -> “stuck” drift / slow weirdness
static const float KP = 0.18f;     // recommended start: 0.6 .. 1.5
static const float KI = 0.0002f;    // recommended start: 0.0; then 0.005..0.03 if needed

// Integral clamp (anti-windup)
static const float EINT_CLAMP = 0.20f;   // rad/s equiv (tight)

// Startup gyro bias calibration
static const int   GYRO_BIAS_SAMPLES = 250;   // ~2-3s depending on gyro rate

// Optional: initialize q_est from accel+mag once (strongly recommended)
static const bool  INIT_FROM_ACC_MAG = true;

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

// Convert DCM (body->NED) to quaternion (scalar-first)
static Quat dcmToQuat(const Matrix3f& R) {
  Quat q;
  float tr = R(0,0) + R(1,1) + R(2,2);

  if (tr > 0.0f) {
    float S = sqrtf(tr + 1.0f) * 2.0f;   // S = 4*q.w
    q.w = 0.25f * S;
    q.x = (R(2,1) - R(1,2)) / S;
    q.y = (R(0,2) - R(2,0)) / S;
    q.z = (R(1,0) - R(0,1)) / S;
  } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
    float S = sqrtf(1.0f + R(0,0) - R(1,1) - R(2,2)) * 2.0f; // S = 4*q.x
    q.w = (R(2,1) - R(1,2)) / S;
    q.x = 0.25f * S;
    q.y = (R(0,1) + R(1,0)) / S;
    q.z = (R(0,2) + R(2,0)) / S;
  } else if (R(1,1) > R(2,2)) {
    float S = sqrtf(1.0f + R(1,1) - R(0,0) - R(2,2)) * 2.0f; // S = 4*q.y
    q.w = (R(0,2) - R(2,0)) / S;
    q.x = (R(0,1) + R(1,0)) / S;
    q.y = 0.25f * S;
    q.z = (R(1,2) + R(2,1)) / S;
  } else {
    float S = sqrtf(1.0f + R(2,2) - R(0,0) - R(1,1)) * 2.0f; // S = 4*q.z
    q.w = (R(1,0) - R(0,1)) / S;
    q.x = (R(0,2) + R(2,0)) / S;
    q.y = (R(1,2) + R(2,1)) / S;
    q.z = 0.25f * S;
  }
  return quatNormalize(q);
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

// Utility
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ------------------------------------------------------------
// (4) Sensor -> Body mapping (z-down body)
// 180° rotation about X: x stays, y,z flip
// ------------------------------------------------------------
static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

// Gyro-only fix: flip yaw sign (Z) after z-down body mapping
static inline Vector3f sensorToBodyZDown_GyroFix(const Vector3f& w_s) {
  Vector3f w_b = sensorToBodyZDown(w_s); // (x, -y, -z)
  w_b.z() = -w_b.z();                    // <-- flip only Z (yaw)
  return w_b;
}

// ------------------------------------------------------------
// (4b) MAG AXIS REMAP (CONFIRMED)
// Apply AFTER HI+SI, BEFORE sensorToBodyZDown
// mx' = -mx, my' = my, mz' = mz
// ------------------------------------------------------------
static inline void remapMag(float mx, float my, float mz,
                            float &mxr, float &myr, float &mzr) {
  mxr = -mx;
  myr =  my;
  mzr =  mz;
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
// (6) Caching accel/mag (sensor frame, calibrated)
// ------------------------------------------------------------
bool haveAccel = false, haveMag = false;
Vector3f a_s_last(0,0,0);  // sensor frame, bias removed
Vector3f m_s_last(0,0,0);  // sensor frame, HI+SI+remap applied (still sensor axes)

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
Vector3f eInt(0,0,0);             // integral term (rad/s equiv)
unsigned long lastUpdateUs = 0;

bool mahonyInitialized = false;

// ------------------------------------------------------------
// (9) TRIAD-style one-shot init (uses same basis as your TRIAD)
// Returns true if init succeeded.
// ------------------------------------------------------------
static bool initFromAccMag(Vector3f a_b, Vector3f m_b) {
  float a_norm = a_b.norm();
  if (a_norm < 1e-6f) return false;
  if (a_norm < ACC_MAG_MIN || a_norm > ACC_MAG_MAX) return false;

  Vector3f a_hat = a_b / a_norm;

  // Down in body (same convention as your TRIAD)
  Vector3f d_b = -a_hat;

  // Horizontal mag
  Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
  float mh_norm = m_h.norm();
  if (mh_norm < MAG_H_MIN) return false;

  Vector3f n_b = m_h / mh_norm;          // "North" in body (magnetic)
  Vector3f e_b = d_b.cross(n_b);
  float e_norm = e_b.norm();
  if (e_norm < 1e-6f) return false;
  e_b /= e_norm;
  n_b = e_b.cross(d_b);                  // re-orthogonalize

  Matrix3f C_bn;
  C_bn.col(0) = n_b;
  C_bn.col(1) = e_b;
  C_bn.col(2) = d_b;

  Matrix3f C_nb = C_bn.transpose();      // body->NED
  q_est = dcmToQuat(C_nb);
  q_est = quatNormalize(q_est);
  eInt.setZero();
  return true;
}

// ------------------------------------------------------------
// (10) Mahony update: accel-only (no mag)
// ------------------------------------------------------------
static void mahonyUpdateAccelOnly(Vector3f w_b, Vector3f a_b, float dt) {
  float an = a_b.norm();
  if (an < 1e-6f) return;
  a_b /= an;

  Quat qn = quatNormalize(q_est);
  Matrix3f C_nb = quatToDCM(qn);
  Matrix3f C_bn = C_nb.transpose();

  Vector3f d_b_est  = C_bn * Vector3f(0,0,1); // predicted Down in body
  Vector3f d_b_meas = -a_b;                   // measured Down in body (your convention)

  Vector3f e = d_b_meas.cross(d_b_est);

  if (KI > 0.0f) {
    eInt += e * (KI * dt);
    eInt.x() = clampf(eInt.x(), -EINT_CLAMP, EINT_CLAMP);
    eInt.y() = clampf(eInt.y(), -EINT_CLAMP, EINT_CLAMP);
    eInt.z() = clampf(eInt.z(), -EINT_CLAMP, EINT_CLAMP);
  } else {
    eInt.setZero();
  }

  Vector3f w_corr = w_b + (KP * e) + eInt;

  Quat omega = {0.0f, w_corr.x(), w_corr.y(), w_corr.z()};
  Quat qdot  = quatMul(qn, omega);
  qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;

  q_est.w += qdot.w * dt;
  q_est.x += qdot.x * dt;
  q_est.y += qdot.y * dt;
  q_est.z += qdot.z * dt;

  q_est = quatNormalize(q_est);
}

// ------------------------------------------------------------
// (11) Mahony update: accel + mag
// ------------------------------------------------------------
static void mahonyUpdateAccMag(Vector3f w_b, Vector3f a_b, Vector3f m_b, float dt) {
  float an = a_b.norm();
  float mn = m_b.norm();
  if (an < 1e-6f || mn < 1e-6f) return;
  a_b /= an;
  m_b /= mn;

  Quat qn = quatNormalize(q_est);
  Matrix3f C_nb = quatToDCM(qn);         // body->NED
  Matrix3f C_bn = C_nb.transpose();      // NED->body

  Vector3f d_b_est  = C_bn * Vector3f(0,0,1);
  Vector3f d_b_meas = -a_b;

  // Mag reference (Mahony standard)
  Vector3f h_n = C_nb * m_b;
  float bx = sqrtf(h_n.x()*h_n.x() + h_n.y()*h_n.y());
  float bz = h_n.z();
  Vector3f m_b_est = C_bn * Vector3f(bx, 0.0f, bz);

  Vector3f e = d_b_meas.cross(d_b_est) + m_b.cross(m_b_est);

  if (KI > 0.0f) {
    eInt += e * (KI * dt);
    eInt.x() = clampf(eInt.x(), -EINT_CLAMP, EINT_CLAMP);
    eInt.y() = clampf(eInt.y(), -EINT_CLAMP, EINT_CLAMP);
    eInt.z() = clampf(eInt.z(), -EINT_CLAMP, EINT_CLAMP);
  } else {
    eInt.setZero();
  }

  Vector3f w_corr = w_b + (KP * e) + eInt;

  Quat omega = {0.0f, w_corr.x(), w_corr.y(), w_corr.z()};
  Quat qdot  = quatMul(qn, omega);
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
  while (got < GYRO_BIAS_SAMPLES && (millis() - t0) < 6000UL) {
    if (IMU.gyroscopeAvailable()) {
      float gx, gy, gz;
      IMU.readGyroscope(gx, gy, gz); // deg/s
      Vector3f w_s(gx, gy, gz);
      Vector3f w_b_deg = sensorToBodyZDown_GyroFix(w_s);
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
  // Read gyro (filter integrates every loop using gyro dt)
  // -----------------------------
  float gx, gy, gz;
  if (!IMU.gyroscopeAvailable()) return;
  IMU.readGyroscope(gx, gy, gz); // deg/s

  unsigned long nowUs = micros();
  float dt = (nowUs - lastUpdateUs) * 1e-6f;
  lastUpdateUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) return;

  // Gyro: sensor -> body, deg/s -> rad/s, subtract bias
  Vector3f w_s(gx, gy, gz);
  Vector3f w_b = sensorToBodyZDown_GyroFix(w_s) * (float)(M_PI / 180.0f);
  w_b -= gyro_bias_b;

  // -----------------------------
  // Cache latest accel
  // -----------------------------
  if (IMU.accelerationAvailable()) {
    float ax_raw, ay_raw, az_raw;
    IMU.readAcceleration(ax_raw, ay_raw, az_raw); // g
    a_s_last = calibrateAccelSensor(ax_raw, ay_raw, az_raw);
    haveAccel = true;
  }

  // -----------------------------
  // Cache latest mag
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

  // If we never got accel yet, just integrate gyro (no correction)
  if (!haveAccel) {
    Quat qn = quatNormalize(q_est);
    Quat omega = {0.0f, w_b.x(), w_b.y(), w_b.z()};
    Quat qdot = quatMul(qn, omega);
    qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;
    q_est.w += qdot.w * dt;
    q_est.x += qdot.x * dt;
    q_est.y += qdot.y * dt;
    q_est.z += qdot.z * dt;
    q_est = quatNormalize(q_est);
    return;
  }

  // Map cached accel to body
  Vector3f a_b = sensorToBodyZDown(a_s_last);

  float a_norm = a_b.norm();
  if (a_norm < 1e-6f) return;

  // If accelerating hard, skip correction (gyro integration only)
  if (a_norm < ACC_MAG_MIN || a_norm > ACC_MAG_MAX) {
    Quat qn = quatNormalize(q_est);
    Quat omega = {0.0f, w_b.x(), w_b.y(), w_b.z()};
    Quat qdot = quatMul(qn, omega);
    qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;
    q_est.w += qdot.w * dt;
    q_est.x += qdot.x * dt;
    q_est.y += qdot.y * dt;
    q_est.z += qdot.z * dt;
    q_est = quatNormalize(q_est);
    return;
  }

  // Normalize accel (direction)
  Vector3f a_hat = a_b / a_norm;
  Vector3f d_b = -a_hat;

  float mh_norm = 0.0f;
  bool magGood = false;
  Vector3f m_b(0,0,0);

  if (haveMag) {
    m_b = sensorToBodyZDown(m_s_last);

    // Horizontal mag magnitude (same as your TRIAD gate)
    Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
    mh_norm = m_h.norm();
    magGood = (mh_norm >= MAG_H_MIN);
  }

  // Optional: initialize once from accel+mag (fast lock)
  if (INIT_FROM_ACC_MAG && !mahonyInitialized && haveMag && magGood) {
    if (initFromAccMag(a_b, m_b)) {
      mahonyInitialized = true;
    }
  }

  // Run Mahony correction:
  //  - If magGood: accel+mag correction (stabilizes yaw)
  //  - Else: accel-only correction (stabilizes roll/pitch, yaw drifts naturally)
  if (haveMag && magGood) {
    mahonyUpdateAccMag(w_b, a_b, m_b, dt);
  } else {
    mahonyUpdateAccelOnly(w_b, a_b, dt);
  }

  // -----------------------------
  // Yaw-zero (applied to OUTPUT only)
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

  // Hemisphere continuity for output (nice for plotting)
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