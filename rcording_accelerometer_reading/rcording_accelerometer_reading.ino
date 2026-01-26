/*
  SC651 Assignment (Part b):
  Roll angle estimation using accelerometer data only + CSV logging to SPIFFS.

  Measurement model (Eq. 7 from assignment):
    theta_acc = atan2(ay, sqrt(ax^2 + az^2))

  This version:
    - Records for STOP_TIME_S seconds (60 by default)
    - Logs to SPIFFS CSV file (ESP32 flash)
    - Type 'd' in Serial Monitor to dump the CSV over Serial

  EXTRA ROBUSTNESS ADDED (to fix your error):
    - Prints SPIFFS total/used bytes
    - If SPIFFS is nearly full, it formats SPIFFS automatically
    - Then retries opening the CSV file for writing
*/

#include <Arduino_LSM9DS1.h>
#include "FS.h"
#include "SPIFFS.h"
#include <math.h>

// -----------------------------
// User settings
// -----------------------------
static const float STOP_TIME_S = 60.0f;                          // 0 -> log forever
static const char *CSV_PATH = "/roll_acc_only_ninety_and_minus.csv";       // MUST start with '/'
static const uint32_t FLUSH_EVERY_N_LINES = 50;
static const bool FLIP_ACCEL_SIGN = true;

// If SPIFFS used space exceeds this fraction, auto-format to recover space.
static const float SPIFFS_FULL_THRESHOLD = 0.90f;  // 90%

// -----------------------------
// Time bookkeeping
// -----------------------------
unsigned long t_prev_us = 0;
unsigned long t_start_ms = 0;

// CSV file handle + counters
File csvFile;
uint32_t lineCount = 0;
bool logging_active = true;

// ------------------------------------------------------------
// Dump the CSV file over Serial
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

// ------------------------------------------------------------
// Helper: prints SPIFFS usage
// ------------------------------------------------------------
void printSpiffsInfo(const char *tag) {
  size_t total = SPIFFS.totalBytes();
  size_t used  = SPIFFS.usedBytes();
  Serial.print(tag);
  Serial.print(" SPIFFS total=");
  Serial.print(total);
  Serial.print(" used=");
  Serial.println(used);
}

// ------------------------------------------------------------
// Helper: open CSV for writing (with recovery if SPIFFS is full)
// ------------------------------------------------------------
bool openCsvForWrite() {
  // Overwrite this file if it already exists
  if (SPIFFS.exists(CSV_PATH)) {
    SPIFFS.remove(CSV_PATH);
  }

  csvFile = SPIFFS.open(CSV_PATH, FILE_WRITE);
  if (csvFile) return true;

  // If open failed, show SPIFFS usage
  printSpiffsInfo("[openCsvForWrite]");

  // If SPIFFS looks nearly full, format and retry
  size_t total = SPIFFS.totalBytes();
  size_t used  = SPIFFS.usedBytes();
  if (total > 0 && (float)used / (float)total > SPIFFS_FULL_THRESHOLD) {
    Serial.println("SPIFFS nearly full -> formatting to recover space...");
    SPIFFS.end();
    SPIFFS.format();
    if (!SPIFFS.begin(true)) {
      Serial.println("ERROR: SPIFFS mount failed after format!");
      return false;
    }
    printSpiffsInfo("[after format]");

    // Retry open
    csvFile = SPIFFS.open(CSV_PATH, FILE_WRITE);
    if (csvFile) return true;
  }

  // Still failed
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nBOOT OK - starting accel-only logger...");

  // -----------------------------
  // Initialize SPIFFS
  // -----------------------------
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    while (1) { delay(100); }
  }
  printSpiffsInfo("[startup]");

  // -----------------------------
  // Open CSV for writing (with recovery if full)
  // -----------------------------
  if (!openCsvForWrite()) {
    Serial.println("ERROR: Could not open CSV file for writing (even after recovery).");
    Serial.println("Check Arduino IDE -> Tools -> Partition Scheme has SPIFFS enabled.");
    while (1) { delay(100); }
  }

  // CSV header
  csvFile.println("time_s,dt_s,ax_g,ay_g,az_g,roll_acc_deg");
  csvFile.flush();

  // -----------------------------
  // Initialize IMU
  // -----------------------------
  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1!");
    while (1) { delay(100); }
  }

  // Initialize time
  t_prev_us = micros();
  t_start_ms = millis();

  Serial.println("Accelerometer-only roll logging started...");
  Serial.print("Logging to SPIFFS file: ");
  Serial.println(CSV_PATH);
  Serial.print("STOP_TIME_S = ");
  Serial.println(STOP_TIME_S);
  Serial.println("Type 'd' to dump the CSV over Serial.");
}

void loop() {
  // ------------------------------------------------------------
  // Handle serial commands (ignore newline chars)
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

  // Elapsed time since start (seconds)
  float t_elapsed_s = (millis() - t_start_ms) * 1e-3f;

  // Stop condition
  if (STOP_TIME_S > 0.0f && t_elapsed_s >= STOP_TIME_S) {
    logging_active = false;
    csvFile.flush();
    csvFile.close();
    Serial.println("\nLogging finished. CSV file closed.");
    Serial.println("Type 'd' to dump the CSV over Serial.");
    return;
  }

  // ------------------------------------------------------------
  // Only log when an accelerometer sample is available
  // dt corresponds to time between accel samples we actually read/log
  // ------------------------------------------------------------
  if (!IMU.accelerationAvailable()) {
    delay(1);
    return;
  }

  // Compute dt at the moment we read a fresh accel sample
  unsigned long t_now_us = micros();
  float dt = (t_now_us - t_prev_us) * 1e-6f;
  t_prev_us = t_now_us;

  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f;
  }

  // Read accelerometer (typically in "g" units)
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);

  // Roll from accelerometer (Eq. 7):
  //   theta_acc = atan2(ay, sqrt(ax^2 + az^2))
  float denom = sqrtf(ax * ax + az * az);
  float roll_acc_rad = atan2f(ay, denom);
  float roll_acc_deg = roll_acc_rad * (180.0f / 3.14159265f);

  // Align sign with your convention
  if (FLIP_ACCEL_SIGN) roll_acc_deg = -roll_acc_deg;

  // Serial output (for plotter)
  Serial.print("roll_acc_deg:");
  Serial.println(roll_acc_deg, 6);

  // CSV logging
  csvFile.print(t_elapsed_s, 6); csvFile.print(",");
  csvFile.print(dt, 6);         csvFile.print(",");
  csvFile.print(ax, 6);         csvFile.print(",");
  csvFile.print(ay, 6);         csvFile.print(",");
  csvFile.print(az, 6);         csvFile.print(",");
  csvFile.println(roll_acc_deg, 6);

  lineCount++;
  if (lineCount % FLUSH_EVERY_N_LINES == 0) {
    csvFile.flush();
  }

  delay(5);
}
