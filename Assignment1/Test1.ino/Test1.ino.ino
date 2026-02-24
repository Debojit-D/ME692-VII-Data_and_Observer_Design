#include <Arduino_LSM9DS1.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!IMU.begin()) {
    Serial.println("IMU init failed");
    while (1) {}
  }

  Serial.println("gx,gy,gz (dps)");
}

void loop() {
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);

    // CSV format works great with Serial Plotter
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }
  delay(10);
}
