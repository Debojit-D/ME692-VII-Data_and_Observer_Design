#include <Arduino_LSM9DS1.h>

float roll_gyro_deg = 0.0f;
unsigned long t_prev_us = 0;

// Your roll axis is gy (value 2)
float gyro_bias_roll_dps = 0.0f;

void calibrateGyroBiasRoll(int samples = 600, int sample_delay_ms = 5) {
  // 600 samples * 5 ms = ~3 seconds
  Serial.println("Calibrating gyro bias... KEEP THE BOARD STILL.");

  float sum = 0.0f;
  int count = 0;

  // Give IMU a moment to settle
  delay(200);

  while (count < samples) {
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      sum += gx;            // roll axis = gx
      count++;
      delay(sample_delay_ms);
    }
  }

  gyro_bias_roll_dps = sum / (float)samples;

  Serial.print("Gyro bias (roll axis, dps) = ");
  Serial.println(gyro_bias_roll_dps, 6);
  Serial.println("Calibration done.\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU (LSM9DS1).");
    while (true) { delay(1000); }
  }

  calibrateGyroBiasRoll();   // bias estimation

  t_prev_us = micros();
  roll_gyro_deg = 0.0f;

  Serial.println("roll_gyro_deg (gyro-only, bias-corrected)");
}

void loop() {
  unsigned long t_now_us = micros();
  float Ts = (t_now_us - t_prev_us) * 1e-6f;  // seconds
  t_prev_us = t_now_us;

  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // Bias-corrected roll rate
    float omega_roll_dps = gx - gyro_bias_roll_dps;

    // Integrate
    roll_gyro_deg += omega_roll_dps * Ts;
  }

  Serial.println(roll_gyro_deg);
  delay(5);
}
