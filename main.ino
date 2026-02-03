/*
  Sensor-controlled motors + RUN/PAUSE state machine

  Pins:
    Button: D13 (active-low, INPUT_PULLUP)
    Ultrasonic: TRIG=D11, ECHO=D12
    SG90 Servo: D10
    Stepper (28BYJ-48 + ULN2003): D4-D7 (wired to IN1..IN4)

  Added DC motor (2x PWM pins chosen: D3 and D9):
    DC Motor IN1 = D9 (PWM)
    DC Motor IN2 = D3 (PWM)
    Speed based on ultrasonic distance:
      min distance -> 0
      max/no-echo  -> 255
    Direction alternates every 1s (like your reference code) while in RUN.

  Serial: 115200
*/

#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Servo.h>
#include <Stepper.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------------- Pins ----------------------
const int POT_PIN = A0;

// Ultrasonic
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

// Servo
const int SERVO_PIN = 10;

// Button
const int BUTTON_PIN = 13; // active-low

// Stepper pins D4-D7 wired to ULN2003 IN1..IN4
const int STEPPER_D4_IN1 = 4;
const int STEPPER_D5_IN2 = 5;
const int STEPPER_D6_IN3 = 6;
const int STEPPER_D7_IN4 = 7;

// DC motor PWM pins (best available given your constraints)
const int DC_MOTOR_IN1_PWM = 9; // PWM
const int DC_MOTOR_IN2_PWM = 3; // PWM

// ---------------------- Timing ----------------------
const unsigned long POT_PERIOD_MS = 30;
const unsigned long US_PERIOD_MS  = 100;
const unsigned long IMU_PERIOD_MS = 30;

const unsigned long DEBOUNCE_MS   = 35;
const unsigned long DC_DIR_TOGGLE_MS = 1000;

// ---------------------- Servo control ----------------------
Servo servo;
const int SERVO_MIN_ANGLE = 10;
const int SERVO_MAX_ANGLE = 180;
int servoAngleCmd = SERVO_MIN_ANGLE;

// ---------------------- Stepper control ----------------------
const int stepsPerRevolution = 2048; // 28BYJ-48 geared output steps
Stepper myStepper(
  stepsPerRevolution,
  STEPPER_D4_IN1,      // IN1
  STEPPER_D6_IN3,      // IN3  (special order for Stepper.h + ULN2003)
  STEPPER_D5_IN2,      // IN2
  STEPPER_D7_IN4       // IN4
);

// Map IMU pitch to +/- quarter turn
const long STEPPER_POS_LIMIT = stepsPerRevolution / 4; // 512 steps
const int  STEPPER_MAX_STEP_PER_UPDATE = 6;
long stepperPos = 0;
long stepperTargetPos = 0;

// ---------------------- DC motor control ----------------------
int  dcSpeedCmd = 0;         // 0..255
bool dcDirCw = true;         // true=cw, false=ccw
unsigned long lastDcToggleMs = 0;

// For reduced spam printing
int  lastPrintedDcSpeed = -1;
bool lastPrintedDcDir   = false;

// ---------------------- MPU9250 ----------------------
MPU9250_asukiaaa imu;

// ---------------------- State machine ----------------------
bool motorsRunning = true;

// Debounce variables
int buttonStable = HIGH;
int buttonLastRead = HIGH;
unsigned long lastDebounceMs = 0;

// ---------------------- Sensor/report timers ----------------------
unsigned long lastPotMs = 0;
unsigned long lastUsMs  = 0;
unsigned long lastImuMs = 0;

// Keep last ultrasonic reading
float lastUsCm = NAN;

// ---------------------- Helpers ----------------------
float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Timeout 30ms ~ 5m (prevents hanging)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return NAN;

  return (duration * 0.0343f) / 2.0f;
}

int potToServoAngle(int pot) {
  long a = map(pot, 0, 1023, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  if (a < SERVO_MIN_ANGLE) a = SERVO_MIN_ANGLE;
  if (a > SERVO_MAX_ANGLE) a = SERVO_MAX_ANGLE;
  return (int)a;
}

void handleButton(unsigned long now) {
  int raw = digitalRead(BUTTON_PIN);

  if (raw != buttonLastRead) {
    buttonLastRead = raw;
    lastDebounceMs = now;
  }

  if ((now - lastDebounceMs) >= DEBOUNCE_MS && raw != buttonStable) {
    buttonStable = raw;

    // toggle on press (active-low => LOW means pressed)
    if (buttonStable == LOW) {
      motorsRunning = !motorsRunning;
      Serial.print(F("BUTTON,pressed,STATE,"));
      Serial.println(motorsRunning ? F("RUN") : F("PAUSE"));
    }
  }
}

void dcMotorClockwise(int speed) {
  analogWrite(DC_MOTOR_IN1_PWM, speed);
  analogWrite(DC_MOTOR_IN2_PWM, 0);
}

void dcMotorCounterclockwise(int speed) {
  analogWrite(DC_MOTOR_IN1_PWM, 0);
  analogWrite(DC_MOTOR_IN2_PWM, speed);
}

void dcMotorStop() {
  analogWrite(DC_MOTOR_IN1_PWM, 0);
  analogWrite(DC_MOTOR_IN2_PWM, 0);
}

int ultrasonicToDcSpeed(float cm) {
  // Choose a practical min/max mapping range.
  // Min: near sensor minimum usable range
  // Max: typical HC-SR04 max range
  const float MIN_CM = 2.0f;
  const float MAX_CM = 400.0f;

  // If no echo / timeout => treat as MAX => full speed (keeps running)
  if (isnan(cm)) return 255;

  if (cm <= MIN_CM) return 0;
  if (cm >= MAX_CM) return 255;

  // linear map to 0..255
  float t = (cm - MIN_CM) / (MAX_CM - MIN_CM);
  int s = (int)lroundf(t * 255.0f);
  if (s < 0) s = 0;
  if (s > 255) s = 255;
  return s;
}

void updateDcMotorFromUltrasonic(unsigned long now) {
  // Direction toggles like your reference code (every 1s) while RUN
  if (now - lastDcToggleMs >= DC_DIR_TOGGLE_MS) {
    lastDcToggleMs = now;
    dcDirCw = !dcDirCw;
  }

  // Speed from last ultrasonic reading
  dcSpeedCmd = ultrasonicToDcSpeed(lastUsCm);

  // Apply outputs
  if (dcSpeedCmd <= 0) {
    dcMotorStop();
  } else {
    if (dcDirCw) dcMotorClockwise(dcSpeedCmd);
    else         dcMotorCounterclockwise(dcSpeedCmd);
  }

  // Print when changed (speed or direction)
  if (dcSpeedCmd != lastPrintedDcSpeed || dcDirCw != lastPrintedDcDir) {
    lastPrintedDcSpeed = dcSpeedCmd;
    lastPrintedDcDir = dcDirCw;

    Serial.print(F("DCM,speed,"));
    Serial.print(dcSpeedCmd);
    Serial.print(F(",dir,"));
    Serial.print(dcDirCw ? F("CW") : F("CCW"));
    Serial.print(F(",MOTORS,"));
    Serial.println(motorsRunning ? F("RUN") : F("PAUSE"));
  }
}

void updateMotorsFromSensors(int pot, float pitchDeg) {
  // --- Servo from pot ---
  servoAngleCmd = potToServoAngle(pot);
  servo.write(servoAngleCmd);

  // --- Stepper target from pitch ---
  const float PITCH_LIMIT_DEG = 45.0f;
  if (pitchDeg >  PITCH_LIMIT_DEG) pitchDeg =  PITCH_LIMIT_DEG;
  if (pitchDeg < -PITCH_LIMIT_DEG) pitchDeg = -PITCH_LIMIT_DEG;

  stepperTargetPos = (long)(pitchDeg * (float)STEPPER_POS_LIMIT / PITCH_LIMIT_DEG);

  // Step toward target
  long err = stepperTargetPos - stepperPos;
  if (err != 0) {
    int stepCmd = (int)err;
    if (stepCmd >  STEPPER_MAX_STEP_PER_UPDATE) stepCmd =  STEPPER_MAX_STEP_PER_UPDATE;
    if (stepCmd < -STEPPER_MAX_STEP_PER_UPDATE) stepCmd = -STEPPER_MAX_STEP_PER_UPDATE;

    long absErr = labs(err);
    int rpm = 4;
    if (absErr > (STEPPER_POS_LIMIT / 2)) rpm = 12;
    else if (absErr > (STEPPER_POS_LIMIT / 4)) rpm = 9;
    else if (absErr > 40) rpm = 6;

    myStepper.setSpeed(rpm);
    myStepper.step(stepCmd);
    stepperPos += stepCmd;
  }
}

void updateUltrasonic(unsigned long now) {
  if (now - lastUsMs < US_PERIOD_MS) return;
  lastUsMs = now;

  lastUsCm = readUltrasonicCm();

  Serial.print(F("US_CM,"));
  if (isnan(lastUsCm)) Serial.println(F("nan"));
  else Serial.println(lastUsCm, 1);
}

void reportPot(unsigned long now, int pot) {
  if (now - lastPotMs < POT_PERIOD_MS) return;
  lastPotMs = now;

  Serial.print(F("POT,"));
  Serial.print(pot);
  Serial.print(F(",SERVO_CMD,"));
  Serial.print(servoAngleCmd);
  Serial.print(F(",MOTORS,"));
  Serial.println(motorsRunning ? F("RUN") : F("PAUSE"));
}

void reportImu(unsigned long now,
               float ax, float ay, float az,
               float gx, float gy, float gz,
               int mx, int my, int mz,
               float pitchDeg) {
  if (now - lastImuMs < IMU_PERIOD_MS) return;
  lastImuMs = now;

  Serial.print(F("IMU,"));
  Serial.print(ax, 4); Serial.print(',');
  Serial.print(ay, 4); Serial.print(',');
  Serial.print(az, 4); Serial.print(',');
  Serial.print(gx, 4); Serial.print(',');
  Serial.print(gy, 4); Serial.print(',');
  Serial.print(gz, 4); Serial.print(',');
  Serial.print(mx); Serial.print(',');
  Serial.print(my); Serial.print(',');
  Serial.print(mz);

  Serial.print(F(",PITCH_DEG,"));
  Serial.print(pitchDeg, 2);

  Serial.print(F(",STEPPER_TGT,"));
  Serial.print(stepperTargetPos);
  Serial.print(F(",STEPPER_POS,"));
  Serial.print(stepperPos);

  Serial.print(F(",DCM_SPEED,"));
  Serial.print(dcSpeedCmd);
  Serial.print(F(",DCM_DIR,"));
  Serial.println(dcDirCw ? F("CW") : F("CCW"));
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // DC motor pins
  pinMode(DC_MOTOR_IN1_PWM, OUTPUT);
  pinMode(DC_MOTOR_IN2_PWM, OUTPUT);
  dcMotorStop();

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(servoAngleCmd);

  // IMU
  Wire.begin();
  imu.setWire(&Wire);
  imu.beginAccel();
  imu.beginGyro();
  imu.beginMag();

  // Stepper baseline
  myStepper.setSpeed(6);

  Serial.println(F("BOOT,servo=pot stepper=imu dc=ultrasound button=run_pause"));
  Serial.println(F("PINS,btn=13 trig=11 echo=12 servo=10 stepper=4-7 dc_pwm=9,3"));
  Serial.println(F("STATE,RUN"));
}

void loop() {
  unsigned long now = millis();

  // State toggle
  handleButton(now);

  // Always update sensor readings + reporting
  int pot = analogRead(POT_PIN);
  updateUltrasonic(now);

  // IMU read
  imu.accelUpdate();
  imu.gyroUpdate();
  int magResult = imu.magUpdate();
  if (magResult != 0) {
    imu.beginMag();
    imu.magUpdate();
  }

  float ax = imu.accelX();
  float ay = imu.accelY();
  float az = imu.accelZ();

  float gx = imu.gyroX();
  float gy = imu.gyroY();
  float gz = imu.gyroZ();

  int mx = imu.magX();
  int my = imu.magY();
  int mz = imu.magZ();

  // Pitch from accelerometer (simple tilt estimate)
  float pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;

  // Motor updates only when RUN
  if (motorsRunning) {
    updateMotorsFromSensors(pot, pitchDeg);
    updateDcMotorFromUltrasonic(now);
  }

  // Reports (always)
  reportPot(now, pot);
  reportImu(now, ax, ay, az, gx, gy, gz, mx, my, mz, pitchDeg);
}

