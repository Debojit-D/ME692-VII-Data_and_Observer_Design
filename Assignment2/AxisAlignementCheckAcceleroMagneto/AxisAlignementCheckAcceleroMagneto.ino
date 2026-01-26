#include <Arduino_LSM9DS1.h>

// ============================================================
// AXIS CONSISTENCY CHECK (Accel + Mag) - SNAPSHOT MODE (NO SPAM)
//
// Goal: help you see whether accel + mag axes are consistent.
//
// How to use:
//   1) Keep board FLAT, Z UP (still)  -> press 'c'  (capture FLAT)
//   2) Flip UPSIDE DOWN via ROLL (about your X) -> press 'r' (capture ROLL-FLIP)
//   3) Flip UPSIDE DOWN via PITCH (about your Y)-> press 'p' (capture PITCH-FLIP)
//   4) Press 's' to PRINT RESULTS + HALT
//
// Notes:
//   - Clockwise / anticlockwise doesn't matter.
//   - Try to avoid twisting (yaw) during the flip.
// ============================================================

// -----------------------------
// Your calibration values
// -----------------------------
static const float MAG_OFF_X_UT = 13.171f;
static const float MAG_OFF_Y_UT = -2.881f;
static const float MAG_OFF_Z_UT = 46.198f;

static const float ACC_BIAS_X_G = -0.01587f;
static const float ACC_BIAS_Y_G =  0.00694f;
static const float ACC_BIAS_Z_G = -0.00165f; // subtract this

struct Snapshot {
  bool  valid = false;

  // raw
  float ax, ay, az;   // g
  float mx, my, mz;   // uT

  // corrected
  float axc, ayc, azc; // g
  float mxc, myc, mzc; // uT
};

Snapshot sFlat, sRollFlip, sPitchFlip;

// ---------- helpers ----------
float norm3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

void applyCalibration(Snapshot &s) {
  // Accel corrected (g)
  s.axc = s.ax - ACC_BIAS_X_G;
  s.ayc = s.ay - ACC_BIAS_Y_G;
  s.azc = s.az - ACC_BIAS_Z_G;

  // Mag corrected (uT) (hard-iron)
  s.mxc = s.mx - MAG_OFF_X_UT;
  s.myc = s.my - MAG_OFF_Y_UT;
  s.mzc = s.mz - MAG_OFF_Z_UT;
}

// Read one stable sample by averaging N reads (no continuous printing)
bool readAveragedAccelMag(int N, float &ax, float &ay, float &az,
                          float &mx, float &my, float &mz) {
  float sax = 0, say = 0, saz = 0;
  float smx = 0, smy = 0, smz = 0;
  int gotA = 0, gotM = 0;

  unsigned long t0 = millis();
  while ((gotA < N || gotM < N) && (millis() - t0 < 6000)) {
    if (gotA < N && IMU.accelerationAvailable()) {
      float x, y, z;
      IMU.readAcceleration(x, y, z);
      sax += x; say += y; saz += z;
      gotA++;
    }
    if (gotM < N && IMU.magneticFieldAvailable()) {
      float x, y, z;
      IMU.readMagneticField(x, y, z);
      smx += x; smy += y; smz += z;
      gotM++;
    }
    delay(5);
  }

  if (gotA < N || gotM < N) return false;

  ax = sax / N; ay = say / N; az = saz / N;
  mx = smx / N; my = smy / N; mz = smz / N;
  return true;
}

void captureInto(Snapshot &s, const char *label) {
  const int N = 80; // average samples for stability
  Serial.print("Capturing "); Serial.print(label); Serial.println("... keep still.");

  float ax, ay, az, mx, my, mz;
  if (!readAveragedAccelMag(N, ax, ay, az, mx, my, mz)) {
    Serial.println("Capture failed (timeout). Try again.\n");
    return;
  }

  s.ax = ax; s.ay = ay; s.az = az;
  s.mx = mx; s.my = my; s.mz = mz;
  applyCalibration(s);
  s.valid = true;

  Serial.print(label);
  Serial.println(" captured.\n");
}

void printSnapshot(const char *name, const Snapshot &s) {
  Serial.println(name);

  Serial.print("  RAW  accel(g): "); Serial.print(s.ax, 5); Serial.print("\t");
                               Serial.print(s.ay, 5); Serial.print("\t");
                               Serial.println(s.az, 5);
  Serial.print("  RAW  mag (uT): "); Serial.print(s.mx, 3); Serial.print("\t");
                               Serial.print(s.my, 3); Serial.print("\t");
                               Serial.println(s.mz, 3);

  Serial.print("  CORR accel(g): "); Serial.print(s.axc, 5); Serial.print("\t");
                               Serial.print(s.ayc, 5); Serial.print("\t");
                               Serial.println(s.azc, 5);
  Serial.print("  CORR mag (uT): "); Serial.print(s.mxc, 3); Serial.print("\t");
                               Serial.print(s.myc, 3); Serial.print("\t");
                               Serial.println(s.mzc, 3);

  Serial.print("  |a| = "); Serial.print(norm3(s.axc, s.ayc, s.azc), 5);
  Serial.print("   |m| = "); Serial.println(norm3(s.mxc, s.myc, s.mzc), 3);
  Serial.println();
}

int signFlipCount(float a, float b) {
  // returns 1 if signs differ (excluding near-zero), else 0
  const float eps = 1e-6f;
  if (fabsf(a) < eps || fabsf(b) < eps) return 0;
  return ((a > 0) != (b > 0)) ? 1 : 0;
}

void printPairAnalysis(const char *title, const Snapshot &A, const Snapshot &B) {
  Serial.println(title);

  // Accel sign flips
  Serial.print("  Accel sign flips (ax,ay,az): ");
  Serial.print(signFlipCount(A.axc, B.axc)); Serial.print(", ");
  Serial.print(signFlipCount(A.ayc, B.ayc)); Serial.print(", ");
  Serial.println(signFlipCount(A.azc, B.azc));

  // Mag sign flips
  Serial.print("  Mag   sign flips (mx,my,mz): ");
  Serial.print(signFlipCount(A.mxc, B.mxc)); Serial.print(", ");
  Serial.print(signFlipCount(A.myc, B.myc)); Serial.print(", ");
  Serial.println(signFlipCount(A.mzc, B.mzc));

  // Biggest mag component change
  float dx = fabsf(B.mxc - A.mxc);
  float dy = fabsf(B.myc - A.myc);
  float dz = fabsf(B.mzc - A.mzc);

  Serial.print("  |Δm| components (x,y,z): ");
  Serial.print(dx, 3); Serial.print(", ");
  Serial.print(dy, 3); Serial.print(", ");
  Serial.println(dz, 3);

  Serial.print("  Largest Δm axis: ");
  if (dx >= dy && dx >= dz) Serial.println("X");
  else if (dy >= dx && dy >= dz) Serial.println("Y");
  else Serial.println("Z");

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Axis consistency check (Accel + Mag) - snapshot mode (no continuous print)");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.println();
  Serial.println("Keys:");
  Serial.println("  'c' : capture FLAT (Z up)");
  Serial.println("  'r' : capture ROLL-FLIP (Z down via roll about your X)");
  Serial.println("  'p' : capture PITCH-FLIP (Z down via pitch about your Y)");
  Serial.println("  's' : print results + HALT");
  Serial.println();
  Serial.println("Tip: avoid extra yaw twist during the flip.");
  Serial.println();
}

void loop() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == 'c' || ch == 'C') {
      captureInto(sFlat, "FLAT (Z up)");
    } else if (ch == 'r' || ch == 'R') {
      captureInto(sRollFlip, "ROLL-FLIP (Z down)");
    } else if (ch == 'p' || ch == 'P') {
      captureInto(sPitchFlip, "PITCH-FLIP (Z down)");
    } else if (ch == 's' || ch == 'S') {
      Serial.println("\n================ RESULTS ================\n");

      if (sFlat.valid)      printSnapshot("Snapshot: FLAT (Z up)", sFlat);
      if (sRollFlip.valid)  printSnapshot("Snapshot: ROLL-FLIP (Z down)", sRollFlip);
      if (sPitchFlip.valid) printSnapshot("Snapshot: PITCH-FLIP (Z down)", sPitchFlip);

      Serial.println("---- Pairwise checks (look for Z consistency) ----\n");

      if (sFlat.valid && sRollFlip.valid) {
        printPairAnalysis("FLAT vs ROLL-FLIP:", sFlat, sRollFlip);
      } else {
        Serial.println("FLAT vs ROLL-FLIP: (need both captures)\n");
      }

      if (sFlat.valid && sPitchFlip.valid) {
        printPairAnalysis("FLAT vs PITCH-FLIP:", sFlat, sPitchFlip);
      } else {
        Serial.println("FLAT vs PITCH-FLIP: (need both captures)\n");
      }

      Serial.println("Interpretation:");
      Serial.println("  - In BOTH flips, accel az should flip sign strongly.");
      Serial.println("  - The mag component corresponding to the sensor Z axis should also change sign strongly.");
      Serial.println("  - If mag Z does NOT react, but mag X or Y reacts most, mag axes are permuted vs accel.");
      Serial.println("\n=========================================\n");

      // HALT
      while (1) { delay(1000); }
    }
  }
}
