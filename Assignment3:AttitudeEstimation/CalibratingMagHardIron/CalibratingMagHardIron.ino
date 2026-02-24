#include <Arduino_LSM9DS1.h>

// ============================================================
// MAG HARD-IRON CALIBRATION (interactive)
//   - Press 'c' : start calibration (updates min/max)
//   - Press 's' : stop calibration, print offsets, and HALT
// ============================================================

float minX, minY, minZ;
float maxX, maxY, maxZ;

bool calibrating = false;

void resetMinMax() {
  minX = minY = minZ =  1e9f;
  maxX = maxY = maxZ = -1e9f;
}

void printCalibrationResults() {
  float offX = 0.5f * (maxX + minX);
  float offY = 0.5f * (maxY + minY);
  float offZ = 0.5f * (maxZ + minZ);

  Serial.println();
  Serial.println("=== MAG CAL DONE ===");
  Serial.print("min (uT): "); Serial.print(minX, 3); Serial.print("\t");
                         Serial.print(minY, 3); Serial.print("\t");
                         Serial.println(minZ, 3);

  Serial.print("max (uT): "); Serial.print(maxX, 3); Serial.print("\t");
                         Serial.print(maxY, 3); Serial.print("\t");
                         Serial.println(maxZ, 3);

  Serial.println();
  Serial.println("Hard-iron offsets (uT) [subtract these from raw mag]:");
  Serial.print("offX = "); Serial.println(offX, 3);
  Serial.print("offY = "); Serial.println(offY, 3);
  Serial.print("offZ = "); Serial.println(offZ, 3);

  Serial.println();
  Serial.println("Apply like:");
  Serial.println("mx_corr = mx_raw - offX;  my_corr = my_raw - offY;  mz_corr = mz_raw - offZ;");
  Serial.println("====================");
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
  Serial.println("  'c' : start calibration");
  Serial.println("  's' : stop, print offsets, HALT");
  Serial.println();
  Serial.println("During calibration: slowly rotate/tumble the board in 3D.");
  Serial.println();
}

void loop() {
  // Read user commands
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrating = true;
      resetMinMax();
      Serial.println();
      Serial.println("=== MAG CAL STARTED ===");
      Serial.println("Rotate/tumble board slowly in 3D.");
      Serial.println("Press 's' to stop and print offsets.");
      Serial.println("=======================");
    }

    if (ch == 's' || ch == 'S') {
      if (calibrating) {
        calibrating = false;
        printCalibrationResults();

        // HALT HERE (no more printing, nothing else runs)
        while (1) { delay(1000); }
      } else {
        Serial.println("Not calibrating. Press 'c' first.");
      }
    }
  }

  // During calibration, keep updating min/max (OPTIONALLY print raw values too)
  if (calibrating && IMU.magneticFieldAvailable()) {
    float x, y, z;
    IMU.readMagneticField(x, y, z);

    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (z < minZ) minZ = z;

    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
    if (z > maxZ) maxZ = z;

    // Optional: uncomment if you still want to see data while calibrating
    // Serial.print(x, 3); Serial.print('\t');
    // Serial.print(y, 3); Serial.print('\t');
    // Serial.println(z, 3);
  }
}
