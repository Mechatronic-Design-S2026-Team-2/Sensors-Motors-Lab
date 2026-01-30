#include <Wire.h>
#include <MPU9250.h>

MPU9250 imu(Wire, 0x68);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  if (!imu.setup()) {
    Serial.println("IMU init failed");
    while (1);
  }

  Serial.println("IMU OK");
}

void loop() {
  if (imu.update()) {
    Serial.print(imu.getAccX());
    Serial.print('\t');
    Serial.println(imu.getGyroX());
  }
}
