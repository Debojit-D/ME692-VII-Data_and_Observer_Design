#include <Arduino_LSM9DS1.h>

// ============================================================================
// MAGNETOMETER CALIBRATION (Hard-Iron + Soft-Iron Diagonal) — Interactive
// ----------------------------------------------------------------------------
// What it estimates:
//   1) Hard-iron offsets (uT): offX, offY, offZ  -> subtract from raw
//   2) Soft-iron diagonal scale factors: sX, sY, sZ -> multiply after offset
//
// Model used (embedded-friendly):
//   mx_hi = mx_raw - offX
//   my_hi = my_raw - offY
//   mz_hi = mz_raw - offZ
//
//   mx_cal = mx_hi * sX
//   my_cal = my_hi * sY
//   mz_cal = mz_hi * sZ
//
// Commands (Serial Monitor @115200):
//   'c' : start calibration (reset min/max and collect)
//   's' : stop calibration, print offsets + scales + paste-ready code, HALT
//   'p' : toggle live printing (raw / HI / HI+SI + magnitude)
// ----------------------------------------------------------------------------
// How to move the board during calibration:
//   - Slow 3D tumble so each axis reaches both + and - extremes
//   - Do roll + pitch + yaw rotations and a few figure-8 motions
//   - Keep away from laptops/phones/metal tables/speakers
// ============================================================================

float minX, minY, minZ;
float maxX, maxY, maxZ;

bool calibrating = false;
bool livePrint   = false;

void resetMinMax() {
  minX = minY = minZ =  1e9f;
  maxX = maxY = maxZ = -1e9f;
}

// Compute HI offsets and SI diagonal scales from min/max
void computeCalibration(float &offX, float &offY, float &offZ,
                        float &sX,   float &sY,   float &sZ) {
  // ------------------------
  // Hard-iron offsets (uT)
  // ------------------------
  offX = 0.5f * (maxX + minX);
  offY = 0.5f * (maxY + minY);
  offZ = 0.5f * (maxZ + minZ);

  // ------------------------
  // Soft-iron diagonal scales
  // Use half-ranges (rX,rY,rZ) and scale them to a common average radius.
  // ------------------------
  float rX = 0.5f * (maxX - minX);
  float rY = 0.5f * (maxY - minY);
  float rZ = 0.5f * (maxZ - minZ);

  // Protect against division by zero if calibration is too short
  const float eps = 1e-6f;
  if (rX < eps) rX = eps;
  if (rY < eps) rY = eps;
  if (rZ < eps) rZ = eps;

  float rAvg = (rX + rY + rZ) / 3.0f;

  sX = rAvg / rX;
  sY = rAvg / rY;
  sZ = rAvg / rZ;
}

float magNorm(float mx, float my, float mz) {
  return sqrtf(mx*mx + my*my + mz*mz);
}

void printCalibrationResults() {
  float offX, offY, offZ;
  float sX, sY, sZ;
  computeCalibration(offX, offY, offZ, sX, sY, sZ);

  Serial.println();
  Serial.println("=======================================");
  Serial.println("=== MAG CAL DONE (HARD + SOFT DIAG) ===");
  Serial.println("=======================================");

  Serial.print("min (uT): "); Serial.print(minX, 3); Serial.print("\t");
                         Serial.print(minY, 3); Serial.print("\t");
                         Serial.println(minZ, 3);

  Serial.print("max (uT): "); Serial.print(maxX, 3); Serial.print("\t");
                         Serial.print(maxY, 3); Serial.print("\t");
                         Serial.println(maxZ, 3);

  Serial.println();

  // Hard-iron
  Serial.println("Hard-iron offsets (uT) [subtract these from raw mag]:");
  Serial.print("offX = "); Serial.println(offX, 6);
  Serial.print("offY = "); Serial.println(offY, 6);
  Serial.print("offZ = "); Serial.println(offZ, 6);

  Serial.println();

  // Soft-iron (diagonal)
  Serial.println("Soft-iron diagonal scale factors (unitless) [multiply after offset]:");
  Serial.print("sX = "); Serial.println(sX, 6);
  Serial.print("sY = "); Serial.println(sY, 6);
  Serial.print("sZ = "); Serial.println(sZ, 6);

  Serial.println();
  Serial.println("Apply in your TRIAD / filter code like:");
  Serial.println("  // raw: mx,my,mz in uT");
  Serial.println("  mx = (mx - offX) * sX;");
  Serial.println("  my = (my - offY) * sY;");
  Serial.println("  mz = (mz - offZ) * sZ;");
  Serial.println("  // then normalize for TRIAD:");
  Serial.println("  norm = sqrt(mx*mx + my*my + mz*mz); mx/=norm; my/=norm; mz/=norm;");

  Serial.println();
  Serial.println("Paste-ready constants:");
  Serial.print("const float offX = "); Serial.print(offX, 6); Serial.println("f;");
  Serial.print("const float offY = "); Serial.print(offY, 6); Serial.println("f;");
  Serial.print("const float offZ = "); Serial.print(offZ, 6); Serial.println("f;");
  Serial.print("const float sX   = "); Serial.print(sX,   6); Serial.println("f;");
  Serial.print("const float sY   = "); Serial.print(sY,   6); Serial.println("f;");
  Serial.print("const float sZ   = "); Serial.print(sZ,   6); Serial.println("f;");

  Serial.println("=======================================");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  'c' : start calibration (collect min/max)");
  Serial.println("  's' : stop, print offsets+scales, HALT");
  Serial.println("  'p' : toggle live print (raw / HI / HI+SI + |m|)");
  Serial.println();
  Serial.println("During calibration: slowly rotate/tumble the board in 3D.");
  Serial.println("Try roll + pitch + yaw + figure-8, avoid metal objects nearby.");
  Serial.println();

  resetMinMax();
}

void loop() {
  // ------------------------
  // Read user commands
  // ------------------------
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrating = true;
      resetMinMax();
      Serial.println();
      Serial.println("=== MAG CAL STARTED (HI + SI diag) ===");
      Serial.println("Rotate/tumble board slowly in 3D.");
      Serial.println("Press 'p' for live print. Press 's' to stop and print results.");
      Serial.println("=====================================");
    }

    if (ch == 'p' || ch == 'P') {
      livePrint = !livePrint;
      Serial.print("Live print: ");
      Serial.println(livePrint ? "ON" : "OFF");
    }

    if (ch == 's' || ch == 'S') {
      if (calibrating) {
        calibrating = false;
        printCalibrationResults();

        // HALT HERE (so you can copy results)
        while (1) { delay(1000); }
      } else {
        Serial.println("Not calibrating. Press 'c' first.");
      }
    }
  }

  // ------------------------
  // During calibration, update min/max
  // ------------------------
  if (calibrating && IMU.magneticFieldAvailable()) {
    float mx_raw, my_raw, mz_raw;
    IMU.readMagneticField(mx_raw, my_raw, mz_raw);

    // Update min/max on RAW values (recommended), then compute HI+SI from those
    if (mx_raw < minX) minX = mx_raw;
    if (my_raw < minY) minY = my_raw;
    if (mz_raw < minZ) minZ = mz_raw;

    if (mx_raw > maxX) maxX = mx_raw;
    if (my_raw > maxY) maxY = my_raw;
    if (mz_raw > maxZ) maxZ = mz_raw;

    // Optional live printing for sanity
    if (livePrint) {
      float offX, offY, offZ, sX, sY, sZ;
      computeCalibration(offX, offY, offZ, sX, sY, sZ);

      // Hard-iron corrected
      float mx_hi = mx_raw - offX;
      float my_hi = my_raw - offY;
      float mz_hi = mz_raw - offZ;

      // Hard + soft corrected
      float mx_cal = mx_hi * sX;
      float my_cal = my_hi * sY;
      float mz_cal = mz_hi * sZ;

      Serial.print("RAW(uT): ");
      Serial.print(mx_raw, 2); Serial.print('\t');
      Serial.print(my_raw, 2); Serial.print('\t');
      Serial.print(mz_raw, 2);

      Serial.print(" | HI(uT): ");
      Serial.print(mx_hi, 2); Serial.print('\t');
      Serial.print(my_hi, 2); Serial.print('\t');
      Serial.print(mz_hi, 2);

      Serial.print(" | HI+SI(uT): ");
      Serial.print(mx_cal, 2); Serial.print('\t');
      Serial.print(my_cal, 2); Serial.print('\t');
      Serial.print(mz_cal, 2);

      Serial.print(" | |m|: ");
      Serial.println(magNorm(mx_cal, my_cal, mz_cal), 2);
    }
  }
}
