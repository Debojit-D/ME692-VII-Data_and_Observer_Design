#include <Arduino_LSM9DS1.h>

// ============================================================================
// SOFT-IRON (DIAGONAL) CALIBRATION ONLY
// ----------------------------------------------------------------------------
// Assumes you ALREADY have good hard-iron offsets (offX, offY, offZ).
// This sketch collects min/max of HARD-IRON CORRECTED magnetometer readings,
// then computes diagonal soft-iron scale factors (sX, sY, sZ).
//
// Correction model:
//   mx_hi = mx_raw - offX
//   my_hi = my_raw - offY
//   mz_hi = mz_raw - offZ
//
//   mx_final = mx_hi * sX
//   my_final = my_hi * sY
//   mz_final = mz_hi * sZ
//
// Commands (Serial Monitor @115200):
//   'c' : start soft-iron calibration (reset min/max)
//   's' : stop, print sX sY sZ, and HALT
//
// Movement:
//   Slow 3D tumble: roll + pitch + yaw + figure-8; cover all orientations.
// ============================================================================

// ---------------------------
// Paste your HARD-IRON offsets here (uT)
// (from your hard-iron calibration code)
// ---------------------------
const float offX = -27.203f;
const float offY =  62.756f;
const float offZ =  29.578f;

// ---------------------------
// Min/Max of HARD-IRON corrected values
// ---------------------------
float minX, minY, minZ;
float maxX, maxY, maxZ;

bool calibrating = false;

void resetMinMax() {
  minX = minY = minZ =  1e9f;
  maxX = maxY = maxZ = -1e9f;
}

// Read mag and apply ONLY hard-iron correction (offset subtraction)
bool readMagHardIronCorrected(float &mx, float &my, float &mz) {
  if (!IMU.magneticFieldAvailable()) return false;

  float x, y, z;
  IMU.readMagneticField(x, y, z); // raw in uT

  // Apply your known hard-iron offsets
  mx = x - offX;
  my = y - offY;
  mz = z - offZ;

  return true;
}

void computeSoftIronScales(float &sX, float &sY, float &sZ) {
  // Half-ranges of HARD-IRON corrected cloud
  float rX = 0.5f * (maxX - minX);
  float rY = 0.5f * (maxY - minY);
  float rZ = 0.5f * (maxZ - minZ);

  // Avoid divide-by-zero if calibration was too short
  const float eps = 1e-6f;
  if (rX < eps) rX = eps;
  if (rY < eps) rY = eps;
  if (rZ < eps) rZ = eps;

  // Target radius = average radius
  float rAvg = (rX + rY + rZ) / 3.0f;

  // Diagonal soft-iron scale factors
  sX = rAvg / rX;
  sY = rAvg / rY;
  sZ = rAvg / rZ;
}

void printSoftIronResults() {
  float sX, sY, sZ;
  computeSoftIronScales(sX, sY, sZ);

  Serial.println();
  Serial.println("====================================");
  Serial.println("=== SOFT-IRON CAL DONE (DIAGONAL) ===");
  Serial.println("====================================");

  Serial.println("NOTE: min/max are for HARD-IRON corrected mag (after offset subtraction).");
  Serial.print("min_HI (uT): "); Serial.print(minX, 3); Serial.print("\t");
                            Serial.print(minY, 3); Serial.print("\t");
                            Serial.println(minZ, 3);

  Serial.print("max_HI (uT): "); Serial.print(maxX, 3); Serial.print("\t");
                            Serial.print(maxY, 3); Serial.print("\t");
                            Serial.println(maxZ, 3);

  Serial.println();
  Serial.println("Soft-iron diagonal scale factors (unitless):");
  Serial.print("sX = "); Serial.println(sX, 6);
  Serial.print("sY = "); Serial.println(sY, 6);
  Serial.print("sZ = "); Serial.println(sZ, 6);

  Serial.println();
  Serial.println("Apply in your TRIAD / filter code like:");
  Serial.println("  mx = (mx_raw - offX) * sX;");
  Serial.println("  my = (my_raw - offY) * sY;");
  Serial.println("  mz = (mz_raw - offZ) * sZ;");

  Serial.println();
  Serial.println("Paste-ready constants:");
  Serial.print("const float sX = "); Serial.print(sX, 6); Serial.println("f;");
  Serial.print("const float sY = "); Serial.print(sY, 6); Serial.println("f;");
  Serial.print("const float sZ = "); Serial.print(sZ, 6); Serial.println("f;");

  Serial.println("====================================");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Soft-Iron (Diagonal) Calibration ONLY");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  'c' : start soft-iron calibration");
  Serial.println("  's' : stop, print sX sY sZ, HALT");
  Serial.println();
  Serial.println("Move: slowly tumble in 3D (roll + pitch + yaw + figure-8).");
  Serial.println("Avoid metal objects / laptop / phone nearby.");
  Serial.println();

  resetMinMax();
}

void loop() {
  // ---- Handle commands ----
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrating = true;
      resetMinMax();
      Serial.println();
      Serial.println("=== SOFT-IRON CAL STARTED ===");
      Serial.println("Collecting min/max of HARD-IRON corrected mag.");
      Serial.println("Slowly tumble board in 3D.");
      Serial.println("Press 's' to stop and print scale factors.");
      Serial.println("=============================");
    }

    if (ch == 's' || ch == 'S') {
      if (calibrating) {
        calibrating = false;
        printSoftIronResults();

        // Halt so you can copy numbers
        while (1) { delay(1000); }
      } else {
        Serial.println("Not calibrating. Press 'c' first.");
      }
    }
  }

  // ---- Collect min/max while calibrating ----
  if (calibrating) {
    float mx, my, mz;
    if (readMagHardIronCorrected(mx, my, mz)) {
      if (mx < minX) minX = mx;
      if (my < minY) minY = my;
      if (mz < minZ) minZ = mz;

      if (mx > maxX) maxX = mx;
      if (my > maxY) maxY = my;
      if (mz > maxZ) maxZ = mz;
    }
  }
}
