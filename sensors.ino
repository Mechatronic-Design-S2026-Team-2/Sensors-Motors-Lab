#include <Wire.h>
#include <math.h>
#include "mpu9250.h"

/* Pins */
const uint8_t POT_PIN = A0;
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

/* IMU object */
bfs::Mpu9250 imu;

/* Timing (non-blocking scheduling) */
unsigned long lastPotMs = 0;
unsigned long lastUsMs  = 0;
unsigned long lastImuPrintMs = 0;

const unsigned long POT_PERIOD_MS = 50;   // 20 Hz
const unsigned long US_PERIOD_MS  = 100;  // 10 Hz
const unsigned long IMU_PRINT_MS  = 20;   // 50 Hz (printing)

float readUltrasonicCm() {
  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout to avoid long blocking 
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return NAN; // no echo (timeout)

  // Speed of sound 0.0343 cm/us, round trip so div by 2
  return (duration * 0.0343f) / 2.0f;
}

void setup() {
  // Uart baud rate 115200 
  Serial.begin(115200);

  // Wait briefly for Serial on native USB boards
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) {}

  // Ultrasonic setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // I2C + IMU setup (MPU9250)
  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("IMU\tERROR\tInit failed");
    while (1) {}
  }

  if (!imu.ConfigSrd(19)) {
    Serial.println("IMU\tERROR\tSRD config failed");
    while (1) {}
  }

  Serial.println("START\tOK");
}

void loop() {
  unsigned long now = millis();

  // Potentiometer
  if (now - lastPotMs >= POT_PERIOD_MS) {
    lastPotMs = now;
    int pot = analogRead(POT_PIN);

    Serial.print("POT\t");
    Serial.println(pot);
  }

  // Ultrasonic
  if (now - lastUsMs >= US_PERIOD_MS) {
    lastUsMs = now;
    float cm = readUltrasonicCm();

    Serial.print("US_CM\t");
    if (isnan(cm)) Serial.println("nan");
    else Serial.println(cm, 2);
  }

  // IMU 
  // Call Read() as often as possible; print at IMU_PRINT_MS
  if (imu.Read()) {
    if (now - lastImuPrintMs >= IMU_PRINT_MS) {
      lastImuPrintMs = now;

      Serial.print("IMU\t");
      Serial.print(imu.new_imu_data());  Serial.print('\t');
      Serial.print(imu.new_mag_data());  Serial.print('\t');

      Serial.print(imu.accel_x_mps2());  Serial.print('\t');
      Serial.print(imu.accel_y_mps2());  Serial.print('\t');
      Serial.print(imu.accel_z_mps2());  Serial.print('\t');

      Serial.print(imu.gyro_x_radps());  Serial.print('\t');
      Serial.print(imu.gyro_y_radps());  Serial.print('\t');
      Serial.print(imu.gyro_z_radps());  Serial.print('\t');

      Serial.print(imu.mag_x_ut());      Serial.print('\t');
      Serial.print(imu.mag_y_ut());      Serial.print('\t');
      Serial.print(imu.mag_z_ut());      Serial.print('\t');

      Serial.print(imu.die_temp_c());
      Serial.print('\n');
    }
  }
}
