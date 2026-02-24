/*
  SC651 Assignment (Part b):
  Roll angle estimation using accelerometer data only.

  Measurement model (from assignment):
    theta_acc,k = atan2(ay, sqrt(ax^2 + az^2))        (Eq. 7)

  Notes:
  - Uses ONLY accelerometer (no gyro integration here).
  - Works best when the sensor is mostly stationary or moving slowly
    (accelerometer measures gravity direction).
*/

#include <Arduino_LSM9DS1.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait */ }

  if (!IMU.begin()) {
    Serial.println("ERROR: Failed to initialize LSM9DS1!");
    while (1) { delay(100); }
  }

  Serial.println("Accelerometer-only roll estimation started...");
  Serial.println("Output: roll_acc_deg for Serial Plotter");
}

void loop() {
  // Read accelerometer (Arduino_LSM9DS1 typically returns in g units)
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    // ------------------------------------------------------------
    // Roll from accelerometer (Eq. 7):
    //   theta_acc = atan2(ay, sqrt(ax^2 + az^2))
    // ------------------------------------------------------------
    float denom = sqrtf(ax * ax + az * az);   // sqrt(ax^2 + az^2)
    float roll_acc_rad = atan2f(ay, denom);   // atan2(ay, denom)

    // Convert radians to degrees for easier plotting/interpretation
    float roll_acc_deg = roll_acc_rad * (180.0f / 3.14159265f);
    roll_acc_deg = -roll_acc_deg;

    // Serial Plotter format: "label:value"
    Serial.print("roll_acc_deg:");
    Serial.println(roll_acc_deg, 6);
  }

  delay(5);
}
