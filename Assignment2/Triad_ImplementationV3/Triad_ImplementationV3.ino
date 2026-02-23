/*
  ============================================================
  TRIAD (Accel + Mag) — UPDATED (with MAG axis remap)
  ============================================================

  What’s in this version:
    Your NEW magnetometer hard-iron OFFSETS + soft-iron DIAGONAL scales
    Your NEW accelerometer bias (flat + still, +Z up)
    Magnetometer AXIS REMAP from your solver result:
         mx' = -mx
         my' = my
         mz' = mz
    Normalizes accel + mag (TRIAD uses direction)
    Caches accel/mag so they don’t need to be “Available” at same instant
    Keeps your z-down body mapping + yaw-zero feature

  Output (single line CSV):
      Q,w,x,y,z,|a|,|mh|

  Frames:
    - Navigation frame: NED  (X=North, Y=East, Z=Down)
    - Body frame: "z-down body" (sensor->body is 180° about X)

  Keys:
    'p' : toggle streaming
    'z' : yaw-zero (makes current yaw = 0 while keeping roll/pitch)

  Notes:
    - Accel magnitude gate: trust gravity only when ||a|| ~ 1 g
    - Mag horizontal gate: skip if horizontal mag too small (near vertical / noisy)
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

static const float MAG_H_MIN = 5.0f;     // uT threshold on horizontal component (after HI+SI+remap)
                                         // If you see many drops, reduce to e.g. 2.0f

static const int   STREAM_HZ = 50;
static const unsigned long STREAM_PERIOD_MS = (1000UL / STREAM_HZ);
static unsigned long lastStreamMs = 0;

bool streamEnabled = true;

// ------------------------------------------------------------
// (3) Quaternion struct + helpers
// ------------------------------------------------------------
struct Quat { float w, x, y, z; }; // scalar-first

static inline Quat quatNormalize(const Quat& q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-9f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline Quat quatMul(const Quat& a, const Quat& b) {
  // Hamilton product (scalar-first)
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

// Convert a proper rotation matrix to quaternion (scalar-first)
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

// ------------------------------------------------------------
// (4) Sensor -> Body mapping (z-down body)
// 180° rotation about X: x stays, y,z flip
// ------------------------------------------------------------
static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

// ------------------------------------------------------------
// (4b) MAG AXIS REMAP (from your solver result)
// Apply AFTER HI+SI, BEFORE sensorToBodyZDown
// ------------------------------------------------------------
static inline void remapMag(float mx, float my, float mz,
                            float &mxr, float &myr, float &mzr) {
  mxr = -mx;
  myr = my;
  mzr = mz;
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
// q_out = q_zero_inv * q_raw
// ------------------------------------------------------------
bool yawZeroRequested = false;
Quat q_zero_inv = {1,0,0,0};

// ------------------------------------------------------------
// (6) Simple caching so accel/mag don’t have to update same instant
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
  // Hard-iron (offset)
  float mx = mx_raw - MAG_OFF_X_UT;
  float my = my_raw - MAG_OFF_Y_UT;
  float mz = mz_raw - MAG_OFF_Z_UT;

  // Soft-iron diagonal (scale)
  mx *= MAG_SX;
  my *= MAG_SY;
  mz *= MAG_SZ;

  return Vector3f(mx, my, mz);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("TRIAD (updated): Stream quaternion Q,w,x,y,z over Serial (NED, z=Down)");
  Serial.println("Uses: accel bias + mag hard-iron + mag soft-iron diagonal + mag axis remap");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Accel rate = "); Serial.print(IMU.accelerationSampleRate()); Serial.println(" Hz");
  Serial.print("Mag   rate = "); Serial.print(IMU.magneticFieldSampleRate()); Serial.println(" Hz");
  Serial.println();

  Serial.println("Keys:");
  Serial.println("  'p' : toggle quaternion streaming");
  Serial.println("  'z' : yaw-zero at current pose (keeps roll/pitch, zeros heading)");
  Serial.println();
  Serial.println("Streaming format: Q,w,x,y,z,|a|,|mh|");
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
      Serial.print("streamEnabled = "); Serial.println(streamEnabled ? "true" : "false");
    }
    if (ch == 'z' || ch == 'Z') {
      yawZeroRequested = true;
      Serial.println("Yaw zero requested (will apply on next valid update).");
    }
  }

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

    // 1) HI + SI calibration in SENSOR frame
    Vector3f m_cal = calibrateMagSensor(mx_raw, my_raw, mz_raw);

    // 2) Apply axis remap (permute + sign) in SENSOR frame
    float mxr, myr, mzr;
    remapMag(m_cal.x(), m_cal.y(), m_cal.z(), mxr, myr, mzr);

    // Store remapped mag as "sensor-frame" value for downstream mapping
    m_s_last = Vector3f(mxr, myr, mzr);
    haveMag = true;
  }

  // If we don't have both at least once, wait
  if (!haveAccel || !haveMag) return;

  // -----------------------------
  // Rate limit streaming
  // -----------------------------
  if (!streamEnabled) return;
  unsigned long now = millis();
  if (now - lastStreamMs < STREAM_PERIOD_MS) return;
  lastStreamMs = now;

  // -----------------------------
  // Map to BODY (z-down body)
  // -----------------------------
  Vector3f a_b = sensorToBodyZDown(a_s_last);
  Vector3f m_b = sensorToBodyZDown(m_s_last);

  // -----------------------------
  // Gate accel magnitude near 1 g
  // -----------------------------
  float a_norm = a_b.norm();
  if (a_norm < 1e-6f) return;
  if (a_norm < ACC_MAG_MIN || a_norm > ACC_MAG_MAX) return;

  // Normalize accel (direction only)
  Vector3f a_hat = a_b / a_norm;

  // -----------------------------
  // TRIAD basis construction (NED)
  // Stationary accel points UP, so "Down" in body is -a_hat
  // -----------------------------
  Vector3f d_b = -a_hat; // body Down direction

  // Remove mag component along Down to get horizontal mag
  Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
  float mh_norm = m_h.norm();
  if (mh_norm < MAG_H_MIN) return;

  // Normalize horizontal mag => "North" direction in body (magnetic north)
  Vector3f n_b = m_h / mh_norm;

  // East = Down x North
  Vector3f e_b = d_b.cross(n_b);
  float e_norm = e_b.norm();
  if (e_norm < 1e-6f) return;
  e_b /= e_norm;

  // Re-orthogonalize North (ensures orthonormal triad)
  n_b = e_b.cross(d_b);

  // Build DCM:
  // Columns are the BODY-frame basis vectors of NED axes:
  //   col0 = n_b (N in body)
  //   col1 = e_b (E in body)
  //   col2 = d_b (D in body)
  //
  // This is C_bn (NED -> BODY). We want C_nb (BODY -> NED).
  Matrix3f C_bn;
  C_bn.col(0) = n_b;
  C_bn.col(1) = e_b;
  C_bn.col(2) = d_b;

  Matrix3f C_nb = C_bn.transpose();

  // -----------------------------
  // Convert to quaternion (body->NED)
  // -----------------------------
  Quat q_raw = dcmToQuat(C_nb);

  static bool havePrev = false;
  static Quat q_prev;

  if (havePrev) {
    float dot = q_prev.w*q_raw.w + q_prev.x*q_raw.x + q_prev.y*q_raw.y + q_prev.z*q_raw.z;
    if (dot < 0.0f) { // flip to keep continuity
      q_raw.w = -q_raw.w; q_raw.x = -q_raw.x; q_raw.y = -q_raw.y; q_raw.z = -q_raw.z;
    }
  }
  q_prev = q_raw;
  havePrev = true;


  // -----------------------------
  // Yaw-zero (keep roll/pitch)
  // -----------------------------
  if (yawZeroRequested) {
    float yaw = yawFromCnb(C_nb);      // radians
    Quat q_remove = yawQuat_NED(-yaw); // remove current yaw
    q_zero_inv = quatNormalize(q_remove);
    yawZeroRequested = false;
    Serial.println("Yaw zero applied (heading reset).");
  }

  Quat q_out = quatMul(q_zero_inv, q_raw);
  q_out = quatNormalize(q_out);

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
