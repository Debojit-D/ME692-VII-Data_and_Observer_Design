/*
  SC651 Assignment (Part a):
  Roll angle estimation using gyroscope integration only.

  Discrete model (noise ignored):
    theta[k+1] = theta[k] + omega[k] * Ts

  Adds:
    - CSV logging to SPIFFS (ESP32 internal flash filesystem)
    - STOP_TIME_S logic:
        STOP_TIME_S = 0  -> keep logging forever
        STOP_TIME_S > 0  -> stop after that many seconds

  Output:
    - Serial Plotter: prints roll_gyro_deg (same as before)
    - CSV file: /roll_log.csv in SPIFFS

  Serial commands (optional):
    - Send 'd' to dump the CSV file to Serial Monitor
*/

#include <Arduino_LSM9DS1.h>

// ---- ESP32 filesystem for CSV logging ----
#include "FS.h"
#include "SPIFFS.h"

// -----------------------------
// User settings
// -----------------------------

// If STOP_TIME_S == 0: log forever
// If STOP_TIME_S > 0: stop after STOP_TIME_S seconds
static const float STOP_TIME_S = 60.0f;

// CSV file path in SPIFFS
static const char *CSV_PATH = "/roll1.csv";

// How often to flush data to flash (reduce write stress)
// (e.g., flush every 50 lines)
static const uint32_t FLUSH_EVERY_N_LINES = 50;

// -----------------------------
// Choose which gyro axis corresponds to "roll rate"
// Most common: roll about X axis -> use gx
// -----------------------------
enum RollAxis { GX_AXIS, GY_AXIS, GZ_AXIS };
static const RollAxis ROLL_AXIS = GX_AXIS;

// -----------------------------
// State: roll angle in degrees
// -----------------------------
float roll_gyro_deg = 0.0f;

// Time bookkeeping
unsigned long t_prev_us = 0;
unsigned long t_start_ms = 0;

// File handle for CSV
File csvFile;

// Line counter (for periodic flushing)
uint32_t lineCount = 0;

// Logging control
bool logging_active = true;

// ------------------------------------------------------------
// Helper: dump the CSV file over Serial (use Serial Monitor)
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
  // Initialize SPIFFS (flash FS)
  // -----------------------------
  // 'true' means: format if mount fails (useful first time)
  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: SPIFFS mount failed!");
    while (1) { delay(100); }
  }

  // OPTIONAL: overwrite existing file each run
  // If you want to append across runs, comment out the remove().
  if (SPIFFS.exists(CSV_PATH)) {
    SPIFFS.remove(CSV_PATH);
  }

  // Open CSV for writing
  csvFile = SPIFFS.open(CSV_PATH, FILE_WRITE);
  if (!csvFile) {
    Serial.println("ERROR: Could not open CSV file for writing.");
    while (1) { delay(100); }
  }

  // Write CSV header
  csvFile.println("time_s,dt_s,gx_deg_s,gy_deg_s,gz_deg_s,omega_roll_deg_s,roll_gyro_deg");
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

  Serial.println("Gyro-only roll integration + CSV logging started...");
  Serial.print("Logging to SPIFFS file: ");
  Serial.println(CSV_PATH);

  if (STOP_TIME_S == 0.0f) {
    Serial.println("STOP_TIME_S = 0 -> logging forever.");
  } else {
    Serial.print("STOP_TIME_S = ");
    Serial.print(STOP_TIME_S);
    Serial.println(" seconds -> will stop automatically.");
  }

  Serial.println("Type 'd' in Serial Monitor to dump the CSV.");
}

void loop() {
  // ------------------------------------------------------------
  // Optional: handle simple serial commands
  // ------------------------------------------------------------
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'd') {
      dumpCsvToSerial();
    }
  }

  // If logging is finished, do nothing (or keep printing)
  if (!logging_active) {
    delay(50);
    return;
  }

  // ------------------------------------------------------------
  // Compute dt in seconds
  // ------------------------------------------------------------
  unsigned long t_now_us = micros();
  float dt = (t_now_us - t_prev_us) * 1e-6f;  // us -> s
  t_prev_us = t_now_us;

  // Guard against weird dt (startup/rare spikes)
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.01f; // fallback to 10 ms
  }

  // Elapsed time since start (seconds)
  float t_elapsed_s = (millis() - t_start_ms) * 1e-3f;

  // ------------------------------------------------------------
  // Stop condition:
  //   STOP_TIME_S == 0  -> never stop
  //   STOP_TIME_S > 0   -> stop after STOP_TIME_S seconds
  // ------------------------------------------------------------
  if (STOP_TIME_S > 0.0f && t_elapsed_s >= STOP_TIME_S) {
    logging_active = false;

    // Close the file cleanly
    csvFile.flush();
    csvFile.close();

    Serial.println("\nLogging finished (stop_time reached). CSV file closed.");
    Serial.println("Type 'd' to dump the CSV over Serial.");

    return;
  }

  // ------------------------------------------------------------
  // Read gyroscope (Arduino_LSM9DS1 returns deg/s)
  // ------------------------------------------------------------
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // Select roll rate component
    float omega_deg_s = 0.0f;
    if (ROLL_AXIS == GX_AXIS) omega_deg_s = gx;
    if (ROLL_AXIS == GY_AXIS) omega_deg_s = gy;
    if (ROLL_AXIS == GZ_AXIS) omega_deg_s = gz;

    // ------------------------------------------------------------
    // Discrete integration: theta = theta + omega * dt
    // ------------------------------------------------------------
    roll_gyro_deg += omega_deg_s * dt;

    // Optional: keep angle bounded (prevents number from growing forever)
    if (roll_gyro_deg > 180.0f) roll_gyro_deg -= 360.0f;
    if (roll_gyro_deg < -180.0f) roll_gyro_deg += 360.0f;

    // ------------------------------------------------------------
    // Serial Plotter output (keep it simple for plotting)
    // ------------------------------------------------------------
    Serial.print("roll_gyro_deg:");
    Serial.println(roll_gyro_deg);

    // ------------------------------------------------------------
    // CSV logging
    // Columns:
    //   time_s, dt_s, gx, gy, gz, omega_roll, roll_angle
    // ------------------------------------------------------------
    csvFile.print(t_elapsed_s, 6); csvFile.print(",");
    csvFile.print(dt, 6);         csvFile.print(",");
    csvFile.print(gx, 6);         csvFile.print(",");
    csvFile.print(gy, 6);         csvFile.print(",");
    csvFile.print(gz, 6);         csvFile.print(",");
    csvFile.print(omega_deg_s, 6);csvFile.print(",");
    csvFile.println(roll_gyro_deg, 6);

    // Flush periodically to reduce flash wear + ensure data is saved
    lineCount++;
    if (lineCount % FLUSH_EVERY_N_LINES == 0) {
      csvFile.flush();
    }
  }

  delay(5); // small delay (optional)
}
