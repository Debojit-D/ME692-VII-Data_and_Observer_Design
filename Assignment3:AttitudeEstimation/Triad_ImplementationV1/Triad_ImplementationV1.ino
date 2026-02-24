/*
  ============================================================
  LSM9DS1 (Accel + Mag) --> TRIAD-style Attitude (NED, z = Down)
  using ArduinoEigen (hideakitai) for clean vector math.

  What you get:
    - roll, pitch, yaw in degrees
    - NED convention: +X = North, +Y = East, +Z = Down
    - Uses ONLY accel + magnetometer (no gyro integration)

  IMPORTANT PRACTICAL NOTES
  -------------------------
  1) Accelerometer measures "specific force". When stationary, it points ~UP,
     so gravity/down direction is opposite of accel direction.

  2) Magnetometer yaw is sensitive to soft-iron distortion and nearby metals.
     You currently applied only hard-iron offsets; yaw may be biased/jittery.

  3) Your board's sensor axes (library output) are typically +Z UP when flat.
     But you requested "z downward" for the BODY frame.
     => We remap sensor->body by a 180° rotation about X:
        x_b =  x_s
        y_b = -y_s
        z_b = -z_s
     Apply SAME mapping to accel and mag to keep them consistent.

  HOW THE MATH WORKS (TRIAD-like)
  ------------------------------
  Let body-frame unit vectors representing NED axes be:
    n_b = North axis expressed in body
    e_b = East  axis expressed in body
    d_b = Down  axis expressed in body

  Step 1: Down from accel (stationary)
    a_b ≈ Up (unit)  =>  d_b = -a_b / ||a_b||

  Step 2: Tilt-compensate magnetometer
    m_b is magnetic field in body.
    Remove component along down to get horizontal field:
      m_h = m_b - (m_b·d_b) d_b
    Normalize:
      n_b = m_h / ||m_h||   (magnetic North direction in body)

  Step 3: East from cross product
      e_b = d_b × n_b
    Normalize e_b, then re-orthogonalize n_b:
      n_b = e_b × d_b
    Now {n_b, e_b, d_b} is an orthonormal right-handed basis.

  Step 4: Build DCM
    C_bn maps NED -> Body. Its columns are NED basis vectors expressed in body:
      C_bn = [ n_b  e_b  d_b ]
    Then body->NED is transpose:
      C_nb = C_bn^T

  Step 5: Extract roll/pitch/yaw (ZYX) from C_nb

  CONTROLS
  --------
    'p' : toggle printing
    'z' : set current yaw as zero reference

  ============================================================
*/

#include <Arduino_LSM9DS1.h>
#include <ArduinoEigenDense.h>
#include <math.h>  // for NAN, isnan, atan2f, asinf

using Eigen::Vector3f;
using Eigen::Matrix3f;

// -----------------------------
// Calibration values you measured
// -----------------------------
// Magnetometer hard-iron offsets (uT): subtract these from raw mag
static const float MAG_OFF_X_UT = 13.171f;
static const float MAG_OFF_Y_UT = -2.881f;
static const float MAG_OFF_Z_UT = 46.198f;

// Accelerometer bias (g): subtract these from raw accel
static const float ACC_BIAS_X_G = -0.01587f;
static const float ACC_BIAS_Y_G =  0.00694f;
static const float ACC_BIAS_Z_G = -0.00165f;

// -----------------------------
// Settings / tuning
// -----------------------------
static const float RAD2DEG = 57.29577951308232f;

// Accel magnitude gate: only trust accel as gravity when near 1 g
static const float ACC_MAG_MIN = 0.85f;
static const float ACC_MAG_MAX = 1.15f;

// Horizontal mag gate: avoid cases where mag is nearly parallel to gravity
static const float MAG_H_MIN = 1.0f;  // uT (very loose sanity check)

// Print rate (Hz)
static const int PRINT_HZ = 20;
static unsigned long lastPrintMs = 0;

bool  printEnabled = true;

// yawZeroDeg stores the yaw offset (press 'z' to set current yaw as 0)
float yawZeroDeg = 0.0f;

// -----------------------------
// Small helpers
// -----------------------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Wrap angle to [-180, 180]
static inline float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

/*
  Extract roll, pitch, yaw from C_nb (body->NED) using ZYX convention.
  This is a common choice:
    yaw   about NED Z (Down)
    pitch about NED Y (East)
    roll  about NED X (North)

  Depending on your chosen convention, you might need sign tweaks,
  but with a consistent DCM this is the right starting point.
*/
static void dcmToRPY_NED_ZYX(const Matrix3f& C_nb, float& roll, float& pitch, float& yaw) {
  pitch = asinf(clampf(-C_nb(2, 0), -1.0f, 1.0f));
  roll  = atan2f(C_nb(2, 1), C_nb(2, 2));
  yaw   = atan2f(C_nb(1, 0), C_nb(0, 0));
}

/*
  Sensor->Body mapping to satisfy "z downward" BODY frame requirement.

  Your sensor/library reports accel Z ~ +1 g when flat (Z up).
  For a BODY frame with Z down, we rotate 180° about X:
    x_b =  x_s
    y_b = -y_s
    z_b = -z_s

  Apply same mapping to accel and mag so frames remain consistent.
*/
static inline Vector3f sensorToBodyZDown(const Vector3f& v_s) {
  return Vector3f(v_s.x(), -v_s.y(), -v_s.z());
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("TRIAD-style Attitude (Accel+Mag), NED frame (z=Down), using Eigen");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Accel rate = "); Serial.print(IMU.accelerationSampleRate()); Serial.println(" Hz");
  Serial.print("Mag   rate = "); Serial.print(IMU.magneticFieldSampleRate()); Serial.println(" Hz");

  Serial.println();
  Serial.println("Keys:");
  Serial.println("  'p' : toggle printing");
  Serial.println("  'z' : zero yaw at current pose");
  Serial.println();
  Serial.println("Output: roll_deg, pitch_deg, yaw_deg (NED, z=Down)");
  Serial.println("Tip: keep the board still; keep it away from metal/laptop for stable yaw.");
  Serial.println();
}

void loop() {
  // -----------------------------
  // Key commands
  // -----------------------------
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 'p' || ch == 'P') {
      printEnabled = !printEnabled;
      Serial.print("printEnabled = "); Serial.println(printEnabled ? "true" : "false");
    }
    if (ch == 'z' || ch == 'Z') {
      yawZeroDeg = NAN;  // sentinel: set on next valid update
      Serial.println("Yaw zero requested (will apply on next valid update).");
    }
  }

  // -----------------------------
  // Read sensors (best effort)
  // NOTE: accel and mag can have different update timing.
  // This simple version requires both to be available in the same loop.
  // If it feels "slow", we can upgrade to caching latest values.
  // -----------------------------
  if (!IMU.accelerationAvailable()) return;
  if (!IMU.magneticFieldAvailable()) return;

  float ax_raw, ay_raw, az_raw;
  float mx_raw, my_raw, mz_raw;

  IMU.readAcceleration(ax_raw, ay_raw, az_raw);   // g
  IMU.readMagneticField(mx_raw, my_raw, mz_raw);  // uT

  // -----------------------------
  // Apply calibration (bias/offset)
  // -----------------------------
  Vector3f a_s(ax_raw - ACC_BIAS_X_G,
              ay_raw - ACC_BIAS_Y_G,
              az_raw - ACC_BIAS_Z_G);   // accel in SENSOR frame (g)

  Vector3f m_s(mx_raw - MAG_OFF_X_UT,
              my_raw - MAG_OFF_Y_UT,
              mz_raw - MAG_OFF_Z_UT);   // mag in SENSOR frame (uT)

  // -----------------------------
  // Map SENSOR -> BODY so BODY has z = Down
  // -----------------------------
  Vector3f a_b = sensorToBodyZDown(a_s);
  Vector3f m_b = sensorToBodyZDown(m_s);

  // -----------------------------
  // Gate accel magnitude near 1 g
  // -----------------------------
  float a_norm = a_b.norm();
  if (a_norm < 1e-6f) return;

  if (a_norm < ACC_MAG_MIN || a_norm > ACC_MAG_MAX) {
    // If moving, accel is not pure gravity; skip update
    return;
  }

  // -----------------------------
  // TRIAD-like basis construction (NED axes expressed in BODY)
  // -----------------------------
  // Down axis in BODY:
  // accel measures UP when stationary => Down = -accel_dir
  Vector3f d_b = (-a_b / a_norm);  // unit Down in body

  // Tilt-compensated mag: remove component along Down
  Vector3f m_h = m_b - (m_b.dot(d_b)) * d_b; // horizontal magnetic field
  float mh_norm = m_h.norm();
  if (mh_norm < MAG_H_MIN) return;

  // North axis in BODY (magnetic north)
  Vector3f n_b = m_h / mh_norm;

  // East axis in BODY
  Vector3f e_b = d_b.cross(n_b);   // East = Down x North
  float e_norm = e_b.norm();
  if (e_norm < 1e-6f) return;
  e_b /= e_norm;

  // Re-orthogonalize north to ensure perfect orthonormal basis
  n_b = e_b.cross(d_b);

  // -----------------------------
  // Build DCM
  // C_bn maps NED -> BODY (columns are N,E,D in body coordinates)
  // -----------------------------
  Matrix3f C_bn;
  C_bn.col(0) = n_b;  // North
  C_bn.col(1) = e_b;  // East
  C_bn.col(2) = d_b;  // Down

  // Body -> NED
  Matrix3f C_nb = C_bn.transpose();

  // -----------------------------
  // Extract roll/pitch/yaw
  // -----------------------------
  float roll, pitch, yaw;
  dcmToRPY_NED_ZYX(C_nb, roll, pitch, yaw);

  float rollDeg  = roll  * RAD2DEG;
  float pitchDeg = pitch * RAD2DEG;
  float yawDeg   = yaw   * RAD2DEG;

  // -----------------------------
  // Yaw zeroing (relative yaw)
  // -----------------------------
  if (isnan(yawZeroDeg)) {
    yawZeroDeg = yawDeg;
    Serial.print("Yaw zero set to "); Serial.print(yawZeroDeg, 3); Serial.println(" deg");
  }
  yawDeg = wrap180(yawDeg - yawZeroDeg);

  // -----------------------------
  // Print (rate-limited)
  // -----------------------------
  unsigned long now = millis();
  if (printEnabled && (now - lastPrintMs) >= (1000UL / PRINT_HZ)) {
    lastPrintMs = now;

    Serial.print("roll=");   Serial.print(rollDeg, 2);
    Serial.print("\tpitch="); Serial.print(pitchDeg, 2);
    Serial.print("\tyaw=");   Serial.print(yawDeg, 2);

    Serial.print("\t|a|=");   Serial.print(a_norm, 3);
    Serial.print("\t|m_h|="); Serial.println(mh_norm, 3);
  }
}
