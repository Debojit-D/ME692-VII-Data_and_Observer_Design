/*
  ============================================================
  VERSION 2 (Professor's suggestion):
    - Compute TRIAD attitude (Accel + Mag) on ESP32
    - Output QUATERNION over Serial for a Python 3D visualizer

  Output format (single line, CSV):
      Q,w,x,y,z,|a|,|mh|
    Example:
      Q,0.991234,0.001122,-0.132200,0.000411,0.999,17.532

  Frame conventions used internally:
    - Navigation frame: NED  (X=North, Y=East, Z=Down)
    - Body frame: "z-down body" requested
      (we remap sensor->body with 180° rotation about X)

  Controls:
    'p' : toggle streaming
    'z' : yaw-zero (rotates the attitude so current yaw becomes 0 while keeping roll/pitch)

  Notes:
    - Accel magnitude gate ensures we trust gravity only near 1g.
    - Mag is hard-iron corrected only (your offsets). Soft-iron may still bias yaw.

  ============================================================
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoEigenDense.h>
#include <math.h>

using Eigen::Vector3f;
using Eigen::Matrix3f;

// -----------------------------
// Your calibration values
// -----------------------------
static const float MAG_OFF_X_UT = 13.171f;
static const float MAG_OFF_Y_UT = -2.881f;
static const float MAG_OFF_Z_UT = 46.198f;

static const float ACC_BIAS_X_G = -0.01587f;
static const float ACC_BIAS_Y_G =  0.00694f;
static const float ACC_BIAS_Z_G = -0.00165f;

// -----------------------------
// Settings / tuning
// -----------------------------
static const float ACC_MAG_MIN = 0.85f;
static const float ACC_MAG_MAX = 1.15f;

static const float MAG_H_MIN = 1.0f;   // uT gate for horizontal component

static const int   STREAM_HZ = 50;     // quaternion stream rate
static unsigned long lastStreamMs = 0;

// If you find you are "skipping too much" due to async sensor updates,
// reduce STREAM_HZ or switch to caching (we can do that next).
static const unsigned long STREAM_PERIOD_MS = (1000UL / STREAM_HZ);

bool streamEnabled = true;

// -----------------------------
// Quaternion struct + helpers
// -----------------------------
struct Quat {
  float w, x, y, z; // scalar-first
};

static inline Quat quatNormalize(const Quat& q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-9f) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static inline Quat quatConj(const Quat& q) {
  return {q.w, -q.x, -q.y, -q.z};
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

// -----------------------------
// Sensor -> Body mapping (z-down body)
// 180° rotation about X: x stays, y,z flip.
// Apply to accel and mag.
// -----------------------------
static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Extract yaw from C_nb (body->NED) for yaw-zero feature
static inline float yawFromCnb(const Matrix3f& C_nb) {
  return atan2f(C_nb(1,0), C_nb(0,0)); // radians
}

// Construct a pure yaw quaternion about NED Z (Down) by angle psi (radians)
static inline Quat yawQuat_NED(float psi) {
  // Rotation about Z axis: q = [cos(psi/2), 0, 0, sin(psi/2)]
  float h = 0.5f * psi;
  return {cosf(h), 0.0f, 0.0f, sinf(h)};
}

// -----------------------------
// Yaw zeroing state
// We store q_zero_inv so streamed quaternion is: q_out = q_zero_inv * q_raw
// where q_raw is body->NED orientation.
// -----------------------------
bool yawZeroRequested = false;
Quat q_zero_inv = {1,0,0,0}; // identity

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("TRIAD v2: Stream quaternion Q,w,x,y,z over Serial (NED, z=Down)");

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
  // Rate limit streaming (reduces Serial load + smoother visualizer)
  // -----------------------------
  unsigned long now = millis();
  if (!streamEnabled) return;
  if (now - lastStreamMs < STREAM_PERIOD_MS) return;

  // Need both sensors to update (simple version)
  if (!IMU.accelerationAvailable()) return;
  if (!IMU.magneticFieldAvailable()) return;

  lastStreamMs = now;

  // -----------------------------
  // Read raw sensors
  // -----------------------------
  float ax_raw, ay_raw, az_raw;
  float mx_raw, my_raw, mz_raw;

  IMU.readAcceleration(ax_raw, ay_raw, az_raw);   // g
  IMU.readMagneticField(mx_raw, my_raw, mz_raw);  // uT

  // -----------------------------
  // Apply calibration
  // -----------------------------
  Vector3f a_s(ax_raw - ACC_BIAS_X_G,
              ay_raw - ACC_BIAS_Y_G,
              az_raw - ACC_BIAS_Z_G);

  Vector3f m_s(mx_raw - MAG_OFF_X_UT,
              my_raw - MAG_OFF_Y_UT,
              mz_raw - MAG_OFF_Z_UT);

  // Map to BODY (z-down)
  Vector3f a_b = sensorToBodyZDown(a_s);
  Vector3f m_b = sensorToBodyZDown(m_s);

  // -----------------------------
  // Gate accel magnitude near 1 g
  // -----------------------------
  float a_norm = a_b.norm();
  if (a_norm < 1e-6f) return;
  if (a_norm < ACC_MAG_MIN || a_norm > ACC_MAG_MAX) return;

  // -----------------------------
  // TRIAD-like basis construction
  // -----------------------------
  // Down direction in BODY: accel points UP when stationary -> Down = -accel_dir
  Vector3f d_b = (-a_b / a_norm);

  // Tilt-compensate mag: remove component along Down
  Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b;
  float mh_norm = m_h.norm();
  if (mh_norm < MAG_H_MIN) return;

  // North (magnetic) in BODY
  Vector3f n_b = m_h / mh_norm;

  // East in BODY
  Vector3f e_b = d_b.cross(n_b);
  float e_norm = e_b.norm();
  if (e_norm < 1e-6f) return;
  e_b /= e_norm;

  // Re-orthogonalize north
  n_b = e_b.cross(d_b);

  // Build DCM
  // C_bn: NED -> BODY
  Matrix3f C_bn;
  C_bn.col(0) = n_b;
  C_bn.col(1) = e_b;
  C_bn.col(2) = d_b;

  // C_nb: BODY -> NED
  Matrix3f C_nb = C_bn.transpose();

  // -----------------------------
  // Convert to quaternion (body->NED)
  // -----------------------------
  Quat q_raw = dcmToQuat(C_nb);

  // -----------------------------
  // Yaw-zero feature (keeps roll/pitch)
  // We compute the current yaw from C_nb and create a "remove yaw" quaternion.
  // q_out = q_remove_yaw * q_raw  => yaw becomes ~0
  // -----------------------------
  if (yawZeroRequested) {
    float yaw = yawFromCnb(C_nb);           // radians
    Quat q_remove = yawQuat_NED(-yaw);      // remove current yaw
    q_zero_inv = quatNormalize(q_remove);   // store it
    yawZeroRequested = false;
    Serial.println("Yaw zero applied (heading reset).");
  }

  Quat q_out = quatMul(q_zero_inv, q_raw);
  q_out = quatNormalize(q_out);

  // -----------------------------
  // Stream over Serial
  // -----------------------------
  Serial.print("Q,");
  Serial.print(q_out.w, 6); Serial.print(",");
  Serial.print(q_out.x, 6); Serial.print(",");
  Serial.print(q_out.y, 6); Serial.print(",");
  Serial.print(q_out.z, 6); Serial.print(",");
  Serial.print(a_norm, 3);  Serial.print(",");
  Serial.println(mh_norm, 3);
}
