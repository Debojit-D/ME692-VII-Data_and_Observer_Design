/*
  ============================================================
  COMBINED ORIENTATION STREAM — GyroOnly + TRIAD + Mahony (FIXED Z + TRIAD DIR)
  ============================================================

  FIX #1 (earlier):
    ✅ 'Z' FULL zero removes FULL current orientation (roll+pitch+yaw) for ALL THREE.

  FIX #2 (this message):
    ✅ TRIAD rotation direction flipped (inverse) relative to Gyro/Mahony/viewer.
       -> TRIAD quaternion is now inverted (conjugated) before output so motion matches.

  Keys:
    'p' : toggle streaming
    'z' : FULL zero (all axes) output reference
    'y' : yaw-only zero (keeps roll/pitch) output reference
    'b' : re-bias gyro (still ~3s)

  Output:
    Q, gw,gx,gy,gz, tw,tx,ty,tz, mw,mx,my,mz, |a|,|mh|
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoEigenDense.h>
#include <math.h>

using Eigen::Vector3f;
using Eigen::Matrix3f;

// ============================================================
// (1) USER CALIBRATION VALUES
// ============================================================

// Magnetometer hard-iron offsets (uT)
static const float MAG_OFF_X_UT = -23.797607f;
static const float MAG_OFF_Y_UT =  63.122559f;
static const float MAG_OFF_Z_UT =  31.121826f;

// Magnetometer soft-iron diagonal scales
static const float MAG_SX = 1.019133f;
static const float MAG_SY = 1.069512f;
static const float MAG_SZ = 0.922706f;

// Accelerometer bias to subtract (g)
static const float ACC_BIAS_X_G = -0.02965f;
static const float ACC_BIAS_Y_G = -0.02873f;
static const float ACC_BIAS_Z_G = -0.04060f;

// ============================================================
// (2) TUNING / SETTINGS
// ============================================================

static const int   STREAM_HZ = 50;
static const unsigned long STREAM_PERIOD_MS = (1000UL / STREAM_HZ);
static unsigned long lastStreamMs = 0;
bool streamEnabled = true;

static const float ACC_MAG_MIN = 0.85f;
static const float ACC_MAG_MAX = 1.15f;

static const float MAG_H_MIN = 5.0f;

static const int   GYRO_BIAS_SAMPLES = 600;
static const float STILL_RATE_DPS_MAX = 1.5f;

static const float GYRO_LPF_ALPHA    = 0.20f;
static const float GYRO_DEADBAND_DPS = 0.25f;

static const float KP = 0.15f;
static const float KI = 0.00f;
static const float EINT_CLAMP = 0.10f;

static const bool INIT_FROM_ACC_MAG = true;

// ✅ TRIAD direction fix switch
static const bool TRIAD_INVERT_OUTPUT = true;  // set false if you ever need to revert

// ============================================================
// (3) Quaternion + helpers
// ============================================================

struct Quat { float w, x, y, z; }; // scalar-first

static inline Quat quatNormalize(const Quat& q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-12f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline Quat quatConj(const Quat& q) {
  return { q.w, -q.x, -q.y, -q.z };
}

// assumes unit quaternion
static inline Quat quatInvUnit(const Quat& q) {
  return quatConj(q);
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static inline float quatDot(const Quat& a, const Quat& b) {
  return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
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

// DCM (body->NED) -> quaternion
static Quat dcmToQuat(const Matrix3f& R) {
  Quat q;
  float tr = R(0,0) + R(1,1) + R(2,2);

  if (tr > 0.0f) {
    float S = sqrtf(tr + 1.0f) * 2.0f;
    q.w = 0.25f * S;
    q.x = (R(2,1) - R(1,2)) / S;
    q.y = (R(0,2) - R(2,0)) / S;
    q.z = (R(1,0) - R(0,1)) / S;
  } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
    float S = sqrtf(1.0f + R(0,0) - R(1,1) - R(2,2)) * 2.0f;
    q.w = (R(2,1) - R(1,2)) / S;
    q.x = 0.25f * S;
    q.y = (R(0,1) + R(1,0)) / S;
    q.z = (R(0,2) + R(2,0)) / S;
  } else if (R(1,1) > R(2,2)) {
    float S = sqrtf(1.0f + R(1,1) - R(0,0) - R(2,2)) * 2.0f;
    q.w = (R(0,2) - R(2,0)) / S;
    q.x = (R(0,1) + R(1,0)) / S;
    q.y = 0.25f * S;
    q.z = (R(1,2) + R(2,1)) / S;
  } else {
    float S = sqrtf(1.0f + R(2,2) - R(0,0) - R(1,1)) * 2.0f;
    q.w = (R(1,0) - R(0,1)) / S;
    q.x = (R(0,2) + R(2,0)) / S;
    q.y = (R(1,2) + R(2,1)) / S;
    q.z = 0.25f * S;
  }
  return quatNormalize(q);
}

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float deg2rad(float d) { return d * (float)(M_PI / 180.0f); }

static inline float softDeadband(float x, float dead) {
  float ax = fabsf(x);
  if (ax < dead) return 0.0f;
  return (x > 0.0f) ? (x - dead) : (x + dead);
}

static Quat deltaQuatFromOmega(Vector3f w, float dt) {
  Vector3f dtheta = w * dt;
  float angle = dtheta.norm();
  if (angle < 1e-9f) return {1,0,0,0};

  Vector3f u = dtheta / angle;
  float h = 0.5f * angle;
  float s = sinf(h);
  return quatNormalize({cosf(h), u.x()*s, u.y()*s, u.z()*s});
}

// ============================================================
// (4) Frame mapping + yaw helpers
// ============================================================

static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

static inline void remapMag(float mx, float my, float mz,
                            float &mxr, float &myr, float &mzr) {
  mxr = -mx;
  myr =  my;
  mzr =  mz;
}

static inline float yawFromCnb(const Matrix3f& C_nb) {
  return atan2f(C_nb(1,0), C_nb(0,0));
}

static inline Quat yawQuat_NED(float psi) {
  float h = 0.5f * psi;
  return {cosf(h), 0.0f, 0.0f, sinf(h)};
}

// ============================================================
// (5) Calibration helpers
// ============================================================

static inline Vector3f calibrateAccelSensor(float ax_raw, float ay_raw, float az_raw) {
  return Vector3f(ax_raw - ACC_BIAS_X_G,
                  ay_raw - ACC_BIAS_Y_G,
                  az_raw - ACC_BIAS_Z_G);
}

static inline Vector3f calibrateMagSensor(float mx_raw, float my_raw, float mz_raw) {
  float mx = mx_raw - MAG_OFF_X_UT;
  float my = my_raw - MAG_OFF_Y_UT;
  float mz = mz_raw - MAG_OFF_Z_UT;

  mx *= MAG_SX;
  my *= MAG_SY;
  mz *= MAG_SZ;
  return Vector3f(mx, my, mz);
}

// ============================================================
// (6) Shared sensor caching
// ============================================================

bool haveAccel = false, haveMag = false;
Vector3f a_s_last(0,0,0);
Vector3f m_s_last(0,0,0);

// ============================================================
// (7) Gyro bias + commands
// ============================================================

Vector3f gyro_bias_b(0,0,0);
Vector3f gyro_filt_b(0,0,0);
bool doRebias = false;

static bool calibrateGyroBias(Vector3f& bias_out_rad, Vector3f& filt_reset_rad) {
  Serial.println("Gyro bias calibration: keep board STILL...");
  Vector3f sum(0,0,0);
  int used = 0;
  unsigned long t0 = millis();

  while (used < GYRO_BIAS_SAMPLES && (millis() - t0) < 8000UL) {
    if (!IMU.gyroscopeAvailable()) continue;

    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);

    Vector3f w_s(gx, gy, gz);
    Vector3f w_b_deg = sensorToBodyZDown(w_s);

    if (w_b_deg.norm() > STILL_RATE_DPS_MAX) continue;
    sum += w_b_deg;
    used++;
  }

  if (used < (GYRO_BIAS_SAMPLES / 3)) {
    Serial.println("Bias calibration FAILED (not enough still samples).");
    return false;
  }

  Vector3f bias_deg = sum / (float)used;
  bias_out_rad = bias_deg * (float)(M_PI / 180.0f);

  filt_reset_rad.setZero();

  Serial.print("Gyro bias (rad/s): ");
  Serial.print(bias_out_rad.x(), 6); Serial.print(", ");
  Serial.print(bias_out_rad.y(), 6); Serial.print(", ");
  Serial.println(bias_out_rad.z(), 6);
  return true;
}

// ============================================================
// (8) Estimator states
// ============================================================

Quat q_gyro   = {1,0,0,0};
Quat q_triad  = {1,0,0,0};
Quat q_mahony = {1,0,0,0};

bool triadValid = false;
Vector3f eInt(0,0,0);
bool mahonyInitialized = false;

unsigned long lastGyroUpdateUs = 0;

// ============================================================
// (9) Output reference (viewer zero) for ALL
// ============================================================

bool fullZeroRequested = false;
bool yawZeroRequested  = false;
Quat q_ref_inv = {1,0,0,0};
Quat q_yaw_inv = {1,0,0,0};

// ============================================================
// (10) TRIAD computation
// ============================================================

static bool computeTriadQuat(Vector3f a_b, Vector3f m_b,
                            Quat& q_out_raw,
                            float& a_norm_out,
                            float& mh_norm_out) {
  a_norm_out = a_b.norm();
  if (a_norm_out < 1e-6f) return false;
  if (a_norm_out < ACC_MAG_MIN || a_norm_out > ACC_MAG_MAX) return false;

  Vector3f a_hat = a_b / a_norm_out;

  // Down in body
  Vector3f d_b = -a_hat;

  // Horizontal mag
  Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
  mh_norm_out = m_h.norm();
  if (mh_norm_out < MAG_H_MIN) return false;

  // North in body (magnetic)
  Vector3f n_b = m_h / mh_norm_out;

  // East = Down x North (NED right-handed)
  Vector3f e_b = d_b.cross(n_b);
  float e_norm = e_b.norm();
  if (e_norm < 1e-6f) return false;
  e_b /= e_norm;

  // Re-orthogonalize North
  n_b = e_b.cross(d_b);

  // Columns are N,E,D expressed in BODY => C_bn (NED -> BODY)
  Matrix3f C_bn;
  C_bn.col(0) = n_b;
  C_bn.col(1) = e_b;
  C_bn.col(2) = d_b;

  // Body -> NED
  Matrix3f C_nb = C_bn.transpose();

  q_out_raw = dcmToQuat(C_nb);

  // ✅ TRIAD direction fix: invert quaternion if needed (gives opposite rotation)
  if (TRIAD_INVERT_OUTPUT) {
    q_out_raw = quatInvUnit(q_out_raw);
    q_out_raw = quatNormalize(q_out_raw);
  }

  return true;
}

// ============================================================
// (11) Mahony helpers
// ============================================================

static bool initMahonyFromAccMag(Vector3f a_b, Vector3f m_b) {
  Quat q_init;
  float an, mhn;
  if (!computeTriadQuat(a_b, m_b, q_init, an, mhn)) return false;

  q_mahony = quatNormalize(q_init);
  eInt.setZero();
  return true;
}

static void mahonyUpdateAccelOnly(Vector3f w_b, Vector3f a_b, float dt) {
  float an = a_b.norm();
  if (an < 1e-6f) return;
  a_b /= an;

  Quat qn = quatNormalize(q_mahony);
  Matrix3f C_nb = quatToDCM(qn);
  Matrix3f C_bn = C_nb.transpose();

  Vector3f d_b_est  = C_bn * Vector3f(0,0,1);
  Vector3f d_b_meas = -a_b;

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

  q_mahony.w += qdot.w * dt;
  q_mahony.x += qdot.x * dt;
  q_mahony.y += qdot.y * dt;
  q_mahony.z += qdot.z * dt;
  q_mahony = quatNormalize(q_mahony);
}

static void mahonyUpdateAccMag(Vector3f w_b, Vector3f a_b, Vector3f m_b, float dt) {
  float an = a_b.norm();
  float mn = m_b.norm();
  if (an < 1e-6f || mn < 1e-6f) return;
  a_b /= an;
  m_b /= mn;

  Quat qn = quatNormalize(q_mahony);
  Matrix3f C_nb = quatToDCM(qn);
  Matrix3f C_bn = C_nb.transpose();

  Vector3f d_b_est  = C_bn * Vector3f(0,0,1);
  Vector3f d_b_meas = -a_b;

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

  q_mahony.w += qdot.w * dt;
  q_mahony.x += qdot.x * dt;
  q_mahony.y += qdot.y * dt;
  q_mahony.z += qdot.z * dt;
  q_mahony = quatNormalize(q_mahony);
}

// ============================================================
// (12) Hemisphere continuity per stream
// ============================================================

static inline Quat enforceHemisphere(Quat q, bool &havePrev, Quat &prev) {
  q = quatNormalize(q);
  if (havePrev) {
    if (quatDot(prev, q) < 0.0f) { q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z; }
  }
  prev = q;
  havePrev = true;
  return q;
}

// Apply output reference: q_out = q_yaw_inv ⊗ (q_ref_inv ⊗ q_est)
static inline Quat applyZeroToQuat(const Quat& q_est) {
  Quat q1 = quatNormalize(quatMul(q_ref_inv, quatNormalize(q_est)));
  Quat q2 = quatNormalize(quatMul(q_yaw_inv, q1));
  return q2;
}

// ============================================================
// setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("COMBINED ORIENTATION STREAM — GyroOnly + TRIAD + Mahony (TRIAD DIR FIX)");
  Serial.println("Keys: 'p' toggle stream, 'z' FULL zero, 'y' yaw-zero only, 'b' re-bias gyro");
  Serial.println("CSV: Q, gw,gx,gy,gz, tw,tx,ty,tz, mw,mx,my,mz, |a|,|mh|");
  Serial.println();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Accel rate = "); Serial.print(IMU.accelerationSampleRate()); Serial.println(" Hz");
  Serial.print("Gyro  rate = "); Serial.print(IMU.gyroscopeSampleRate());    Serial.println(" Hz");
  Serial.print("Mag   rate = "); Serial.print(IMU.magneticFieldSampleRate()); Serial.println(" Hz");
  Serial.println();

  Vector3f filt_reset(0,0,0);
  if (!calibrateGyroBias(gyro_bias_b, filt_reset)) {
    gyro_bias_b.setZero();
    gyro_filt_b.setZero();
  } else {
    gyro_filt_b = filt_reset;
  }

  lastGyroUpdateUs = micros();
  lastStreamMs = millis();
  Serial.println();
}

void loop() {
  // -----------------------------
  // Keys
  // -----------------------------
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 'p' || ch == 'P') {
      streamEnabled = !streamEnabled;
      Serial.print("streamEnabled = "); Serial.println(streamEnabled ? "true" : "false");
    } else if (ch == 'z' || ch == 'Z') {
      fullZeroRequested = true;
      Serial.println("FULL zero requested (applies on next stream).");
    } else if (ch == 'y' || ch == 'Y') {
      yawZeroRequested = true;
      Serial.println("YAW-only zero requested (applies on next stream).");
    } else if (ch == 'b' || ch == 'B') {
      doRebias = true;
    }
  }

  // Re-bias gyro
  if (doRebias) {
    Vector3f filt_reset(0,0,0);
    bool ok = calibrateGyroBias(gyro_bias_b, filt_reset);
    if (ok) gyro_filt_b = filt_reset;
    doRebias = false;
    lastGyroUpdateUs = micros();
    return;
  }

  // Cache accel
  if (IMU.accelerationAvailable()) {
    float ax_raw, ay_raw, az_raw;
    IMU.readAcceleration(ax_raw, ay_raw, az_raw);
    a_s_last = calibrateAccelSensor(ax_raw, ay_raw, az_raw);
    haveAccel = true;
  }

  // Cache mag (HI+SI+remap in SENSOR)
  if (IMU.magneticFieldAvailable()) {
    float mx_raw, my_raw, mz_raw;
    IMU.readMagneticField(mx_raw, my_raw, mz_raw);

    Vector3f m_cal = calibrateMagSensor(mx_raw, my_raw, mz_raw);
    float mxr, myr, mzr;
    remapMag(m_cal.x(), m_cal.y(), m_cal.z(), mxr, myr, mzr);
    m_s_last = Vector3f(mxr, myr, mzr);
    haveMag = true;
  }

  // Read gyro
  if (!IMU.gyroscopeAvailable()) return;

  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  unsigned long nowUs = micros();
  float dt = (nowUs - lastGyroUpdateUs) * 1e-6f;
  lastGyroUpdateUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) return;

  Vector3f w_s(gx, gy, gz);
  Vector3f w_b = sensorToBodyZDown(w_s) * (float)(M_PI / 180.0f);
  w_b -= gyro_bias_b;

  // ============================================================
  // (A) Gyro-only
  // ============================================================
  float dead_rad = deg2rad(GYRO_DEADBAND_DPS);
  Vector3f w_db = w_b;
  w_db.x() = softDeadband(w_db.x(), dead_rad);
  w_db.y() = softDeadband(w_db.y(), dead_rad);
  w_db.z() = softDeadband(w_db.z(), dead_rad);

  gyro_filt_b = (1.0f - GYRO_LPF_ALPHA) * gyro_filt_b + (GYRO_LPF_ALPHA) * w_db;

  {
    Quat qn = quatNormalize(q_gyro);
    Quat dq = deltaQuatFromOmega(gyro_filt_b, dt);
    q_gyro = quatNormalize(quatMul(qn, dq));
  }

  // ============================================================
  // (B) Mahony
  // ============================================================
  bool accelGood = false;
  Vector3f a_b(0,0,0);
  float a_norm = 0.0f;

  if (haveAccel) {
    a_b = sensorToBodyZDown(a_s_last);
    a_norm = a_b.norm();
    accelGood = (a_norm > 1e-6f && a_norm >= ACC_MAG_MIN && a_norm <= ACC_MAG_MAX);
  }

  bool magGood = false;
  Vector3f m_b(0,0,0);
  float mh_norm = 0.0f;

  if (accelGood && haveMag) {
    m_b = sensorToBodyZDown(m_s_last);

    Vector3f a_hat = a_b / a_norm;
    Vector3f d_b = -a_hat;
    Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
    mh_norm = m_h.norm();
    magGood = (mh_norm >= MAG_H_MIN);
  }

  if (INIT_FROM_ACC_MAG && !mahonyInitialized && accelGood && haveMag && magGood) {
    if (initMahonyFromAccMag(a_b, m_b)) mahonyInitialized = true;
  }

  if (!accelGood) {
    Quat qn = quatNormalize(q_mahony);
    Quat omega = {0.0f, w_b.x(), w_b.y(), w_b.z()};
    Quat qdot  = quatMul(qn, omega);
    qdot.w *= 0.5f; qdot.x *= 0.5f; qdot.y *= 0.5f; qdot.z *= 0.5f;
    q_mahony.w += qdot.w * dt;
    q_mahony.x += qdot.x * dt;
    q_mahony.y += qdot.y * dt;
    q_mahony.z += qdot.z * dt;
    q_mahony = quatNormalize(q_mahony);
  } else {
    if (haveMag && magGood) mahonyUpdateAccMag(w_b, a_b, m_b, dt);
    else                    mahonyUpdateAccelOnly(w_b, a_b, dt);
  }

  // ============================================================
  // (C) TRIAD
  // ============================================================
  triadValid = false;
  float triad_a_norm = 0.0f;
  float triad_mh_norm = 0.0f;

  if (haveAccel && haveMag) {
    Vector3f a_b2 = sensorToBodyZDown(a_s_last);
    Vector3f m_b2 = sensorToBodyZDown(m_s_last);
    Quat q_raw;

    if (computeTriadQuat(a_b2, m_b2, q_raw, triad_a_norm, triad_mh_norm)) {
      q_triad = quatNormalize(q_raw);
      triadValid = true;
    }
  }

  // ============================================================
  // STREAM
  // ============================================================
  if (!streamEnabled) return;
  unsigned long nowMs = millis();
  if (nowMs - lastStreamMs < STREAM_PERIOD_MS) return;
  lastStreamMs = nowMs;

  // Reference for zeroing: use Mahony
  Quat q_ref = quatNormalize(q_mahony);

  if (fullZeroRequested) {
    q_ref_inv = quatInvUnit(q_ref);
    q_yaw_inv = {1,0,0,0};
    fullZeroRequested = false;
    yawZeroRequested = false;
    Serial.println("FULL zero applied (all axes).");
  }

  if (yawZeroRequested) {
    Quat q_vis_ref = quatNormalize(quatMul(q_ref_inv, q_ref));
    Matrix3f C_nb_ref = quatToDCM(q_vis_ref);
    float yaw = yawFromCnb(C_nb_ref);
    q_yaw_inv = quatNormalize(yawQuat_NED(-yaw));
    yawZeroRequested = false;
    Serial.println("YAW-only zero applied (keeps roll/pitch).");
  }

  Quat qg = applyZeroToQuat(q_gyro);
  Quat qm = applyZeroToQuat(q_mahony);
  Quat qt = triadValid ? applyZeroToQuat(q_triad) : Quat{1,0,0,0};

  static bool havePrevG=false, havePrevT=false, havePrevM=false;
  static Quat prevG, prevT, prevM;
  qg = enforceHemisphere(qg, havePrevG, prevG);
  qt = enforceHemisphere(qt, havePrevT, prevT);
  qm = enforceHemisphere(qm, havePrevM, prevM);

  float a_print  = triadValid ? triad_a_norm  : a_norm;
  float mh_print = triadValid ? triad_mh_norm : mh_norm;

  Serial.print("Q,");

  Serial.print(qg.w, 6); Serial.print(",");
  Serial.print(qg.x, 6); Serial.print(",");
  Serial.print(qg.y, 6); Serial.print(",");
  Serial.print(qg.z, 6); Serial.print(",");

  Serial.print(qt.w, 6); Serial.print(",");
  Serial.print(qt.x, 6); Serial.print(",");
  Serial.print(qt.y, 6); Serial.print(",");
  Serial.print(qt.z, 6); Serial.print(",");

  Serial.print(qm.w, 6); Serial.print(",");
  Serial.print(qm.x, 6); Serial.print(",");
  Serial.print(qm.y, 6); Serial.print(",");
  Serial.print(qm.z, 6); Serial.print(",");

  Serial.print(a_print, 3); Serial.print(",");
  Serial.println(mh_print, 3);
}