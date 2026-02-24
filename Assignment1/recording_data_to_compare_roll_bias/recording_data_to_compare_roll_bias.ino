/*
  SC651 Assignment (Part a):
  Gyro-only roll integration with BOTH:
    1) Biased roll (raw gyro integration)
    2) Unbiased roll (bias-subtracted gyro integration)

  Discrete-time model (noise ignored):
      theta[k+1] = theta[k] + omega[k] * Ts

  Gyro measurement model:
      omega_meas = omega_true + b + noise

  If board is stationary, omega_true ≈ 0 so omega_meas ≈ b.
  Integrating omega_meas causes drift ~ b * t.
  So we also compute an "unbiased" estimate using:
      omega_corr = omega_meas - b_hat

  Logging:
    - Saves CSV to ESP32 SPIFFS: /roll_unbiased.csv
    - Press 'd' in Serial Monitor to dump the CSV over serial

  Hardware:
    - ESP32-WROVER B
    - LSM9DS1 (Arduino_LSM9DS1 library)
*/

#include <Arduino_LSM9DS1.h>

// ---- ESP32 filesystem for CSV logging ----
#include "FS.h"
#include "SPIFFS.h"

// -----------------------------
// User settings
// -----------------------------
static const float STOP_TIME_S = 60.0f;          // 0 -> log forever, else stop after N seconds
static const char *CSV_PATH = "/roll_unbiased0.csv";
static const uint32_t FLUSH_EVERY_N_LINES = 50;

// -----------------------------
// Choose which gyro axis corresponds to "roll rate"
// Most common: roll about X axis -> use gx
// -----------------------------
enum RollAxis { GX_AXIS, GY_AXIS, GZ_AXIS };
static const RollAxis ROLL_AXIS = GX_AXIS;

// -----------------------------
// Bias estimate (deg/s) from your offline analysis
// -----------------------------
static const float GYRO_BIAS_DEG_S = 1.827666f;

// -----------------------------
// Roll states (degrees)
// -----------------------------
float roll_biased_deg   = 0.0f;   // integrates omega_meas
float roll_unbiased_deg = 0.0f;   // integrates (omega_meas - bias)

// Time bookkeeping
unsigned long t_prev_us = 0;
unsigned long t_start_ms = 0;

// CSV file handle
File csvFile;
uint32_t lineCount = 0;
bool logging_active = true;

// ------------------------------------------------------------
// Dump CSV file over Serial (so you can save it on your PC)
// ------------------------------------------------------------
void dumpCsvToSerial() {
  Serial.println("\n--- CSV DUMP START ---");

  File f = SPIFFS.open(CSV_PATH, FILE_READ);
  if (!f) {
    Serial.println("ERROR: Could not open CSV for reading.");
    Serial.println("--- CSV DUMP END ---\n");
    return;
  }

  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();

  Serial.println("\n--- CSV DUMP END ---\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for Serial */ }

  // -----------------------------
  // Initialize SPIFFS
  // -----------------------------
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    while (1) { delay(100); }
  }

  // Overwrite file each run (remove old file)
  if (SPIFFS.exists(CSV_PATH)) {
    SPIFFS.remove(CSV_PATH);
  }

  csvFile = SPIFFS.open(CSV_PATH, FILE_WRITE);
  if (!csvFile) {
    Serial.println("ERROR: Could not open CSV file for writing.");
    while (1) { delay(100); }
  }

  // CSV header: include both biased and unbiased rolls
  csvFile.println("time_s,dt_s,omega_meas_deg_s,omega_corr_deg_s,roll_biased_deg,roll_unbiased_deg");
  csvFile.flush();

  // -----------------------------
  // Initialize IMU
  // -----------------------------
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1! Check wiring / I2C.");
    while (1) { delay(100); }
  }

  // Initialize time
  t_prev_us = micros();
  t_start_ms = millis();

  Serial.println("Logging biased + unbiased roll to SPIFFS...");
  Serial.print("CSV file: "); Serial.println(CSV_PATH);
  Serial.print("Using bias (deg/s): "); Serial.println(GYRO_BIAS_DEG_S, 6);

  if (STOP_TIME_S == 0.0f) {
    Serial.println("STOP_TIME_S = 0 -> logging forever.");
  } else {
    Serial.print("STOP_TIME_S = "); Serial.print(STOP_TIME_S);
    Serial.println(" seconds -> will stop automatically.");
  }

  Serial.println("Type 'd' to dump the CSV over Serial.");
}

void loop() {
  // ------------------------------------------------------------
  // Handle serial commands (robust to newline characters)
  // ------------------------------------------------------------
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;
    if (c == 'd') dumpCsvToSerial();
  }

  if (!logging_active) {
    delay(50);
    return;
  }

  // ------------------------------------------------------------
  // Stop after STOP_TIME_S (if non-zero)
  // ------------------------------------------------------------
  float t_elapsed_s = (millis() - t_start_ms) * 1e-3f;
  if (STOP_TIME_S > 0.0f && t_elapsed_s >= STOP_TIME_S) {
    logging_active = false;
    csvFile.flush();
    csvFile.close();
    Serial.println("\nLogging finished. CSV file closed.");
    Serial.println("Type 'd' to dump the CSV over Serial.");
    return;
  }

  // ------------------------------------------------------------
  // Only integrate when a gyro sample is available
  // IMPORTANT: dt should correspond to successive IMU samples,
  // so we compute dt right when we read the gyro.
  // ------------------------------------------------------------
  if (!IMU.gyroscopeAvailable()) {
    delay(1);
    return;
  }

  // Compute dt between successive gyro *reads*
  unsigned long t_now_us = micros();
  float dt = (t_now_us - t_prev_us) * 1e-6f;
  t_prev_us = t_now_us;

  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f;
  }

  // Read gyro in deg/s
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  // Select roll rate component
  float omega_meas_deg_s = 0.0f;
  if (ROLL_AXIS == GX_AXIS) omega_meas_deg_s = gx;
  if (ROLL_AXIS == GY_AXIS) omega_meas_deg_s = gy;
  if (ROLL_AXIS == GZ_AXIS) omega_meas_deg_s = gz;

  // Bias-corrected omega
  float omega_corr_deg_s = omega_meas_deg_s - GYRO_BIAS_DEG_S;

  // Integrate BOTH
  roll_biased_deg   += omega_meas_deg_s * dt;
  roll_unbiased_deg += omega_corr_deg_s * dt;

  // Optional wrap to [-180, 180] for readability
  if (roll_biased_deg > 180.0f) roll_biased_deg -= 360.0f;
  if (roll_biased_deg < -180.0f) roll_biased_deg += 360.0f;

  if (roll_unbiased_deg > 180.0f) roll_unbiased_deg -= 360.0f;
  if (roll_unbiased_deg < -180.0f) roll_unbiased_deg += 360.0f;

  // Print BOTH for Serial Plotter (two traces)
  Serial.print("roll_biased_deg:");   Serial.print(roll_biased_deg, 6);
  Serial.print(" roll_unbiased_deg:");Serial.println(roll_unbiased_deg, 6);

  // Log to CSV
  csvFile.print(t_elapsed_s, 6); csvFile.print(",");
  csvFile.print(dt, 6);         csvFile.print(",");
  csvFile.print(omega_meas_deg_s, 6); csvFile.print(",");
  csvFile.print(omega_corr_deg_s, 6); csvFile.print(",");
  csvFile.print(roll_biased_deg, 6);  csvFile.print(",");
  csvFile.println(roll_unbiased_deg, 6);

  lineCount++;
  if (lineCount % FLUSH_EVERY_N_LINES == 0) {
    csvFile.flush();
  }

  delay(5);
}
