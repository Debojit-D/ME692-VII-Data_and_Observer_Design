#include <Arduino_LSM9DS1.h>

// =============================
// Accel bias calibration (simple)
//   - Keep board flat + still (+Z up)
//   - Press 'c' to capture bias
//   - Prints bias once and HALTS
// =============================

static const int   N_SAMPLES = 400;   // ~4s at 100 Hz
static const float TARGET_G  = 1.0f;  // expected Z when flat (+Z up)

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }

  Serial.print("Accel sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  Serial.println();
  Serial.println("ACCEL BIAS CALIBRATION:");
  Serial.println("1) Keep board flat + still (+Z up).");
  Serial.println("2) Press 'c' to capture bias.");
  Serial.println("After printing, the code will HALT.");
  Serial.println();
}

void captureAccelBiasAndHalt() {
  float sx = 0, sy = 0, sz = 0;
  int count = 0;

  Serial.println("Capturing accel bias... keep still.");

  while (count < N_SAMPLES) {
    if (IMU.accelerationAvailable()) {
      float ax, ay, az;
      IMU.readAcceleration(ax, ay, az);
      sx += ax; sy += ay; sz += az;
      count++;
      delay(10); // ~100 Hz
    }
  }

  float axm = sx / N_SAMPLES;
  float aym = sy / N_SAMPLES;
  float azm = sz / N_SAMPLES;

  // Bias to subtract (g)
  float bax = axm;
  float bay = aym;
  float baz = azm - TARGET_G; // makes corrected az ~ +1g when flat

  Serial.println();
  Serial.println("=== ACCEL BIAS DONE ===");
  Serial.print("mean (g): "); Serial.print(axm, 5); Serial.print("\t");
                         Serial.print(aym, 5); Serial.print("\t");
                         Serial.println(azm, 5);

  Serial.println("Bias to subtract (g):");
  Serial.print("bax = "); Serial.println(bax, 5);
  Serial.print("bay = "); Serial.println(bay, 5);
  Serial.print("baz = "); Serial.println(baz, 5);

  Serial.println();
  Serial.println("Apply like:");
  Serial.println("ax_corr = ax_raw - bax;");
  Serial.println("ay_corr = ay_raw - bay;");
  Serial.println("az_corr = az_raw - baz;");
  Serial.println("=======================");

  // HALT so it doesn't keep printing / spamming
  while (1) { delay(1000); }
}

void loop() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 'c' || ch == 'C') {
      captureAccelBiasAndHalt();
    }
  }
}
