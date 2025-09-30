/*
  Simplified grid-fin controller for Arduino Uno + BerryGPS-IMU v4

  - Reads IMU (BerryGPS IMU v4 assumed to use ICM-20948; install SparkFun ICM-20948 library)
  - Runs a simple complementary filter to get roll/pitch
  - Directly maps attitude error (desired - measured) to fin deflection via a proportional gain
  - Four servos 90 deg apart. One fin aligned with IMU X, one with IMU Y.

  Notes:
  - Install libraries:
      SparkFun ICM-20948 Arduino Library (SparkFun_ICM-20948_ArduinoLibrary)
  - This code uses a very small complementary filter (no Mahony/Madgwick) to keep RAM small.
  - Tune Kp and FIN_MAX_DEFLECT for your vehicle.
  - Safety: power servos from external 5V supply and common ground.
*/

#include <Wire.h>
#include <Servo.h>
#include <SparkFun_ICM-20948_ArduinoLibrary.h> // Install from Library Manager

// Pins for Arduino Uno R3 (PWM): 3,5,6,9
const int SERVO_PIN_FL = 3; // front-left (aligned -X or +X depending mounting)
const int SERVO_PIN_FR = 5; // front-right
const int SERVO_PIN_BL = 6; // back-left
const int SERVO_PIN_BR = 9; // back-right

const int FIN_NEUTRAL = 90;       // servo midpoint
const int FIN_MAX_DEFLECT = 45;   // degrees max deflection from neutral (0..45 requested)

// Control gain: fin deflection (deg) = Kp * attitude_error_deg
const float Kp_roll = 1.0;   // 1: 1 deg fin per 1 deg error (tune)
const float Kp_pitch = 1.0;  // tune

// Complementary filter alpha (0..1) higher -> trust gyro more
const float COMPLEMENTARY_ALPHA = 0.98;

ICM_20948_I2C myICM;

Servo sFL, sFR, sBL, sBR;

// attitude state (degrees)
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0; // not used for control here

// desired attitude (set via Serial commands)
float desired_roll_deg = 0.0;
float desired_pitch_deg = 0.0;

unsigned long last_us = 0;

// helper
float clampf(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

// read IMU and update complementary filter. This uses the SparkFun ICM-20948 library.
void updateIMU(float dt) {
  // dt in seconds
  if (myICM.dataReady()) {
    myICM.getAGMT();

    // Read accelerometer in g (library returns scaled values)
    float ax = myICM.accX();
    float ay = myICM.accY();
    float az = myICM.accZ();

    // Read gyro in deg/s (library returns deg/s scaled values)
    float gx = myICM.gyrX();
    float gy = myICM.gyrY();
    float gz = myICM.gyrZ();

    // Compute accel angles (in degrees)
    float roll_acc = atan2(ay, az) * 57.29577951308232;
    float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.29577951308232;

    // Integrate gyro rates
    float roll_gyro = roll + gx * dt;
    float pitch_gyro = pitch + gy * dt;

    // Complementary filter
    roll = COMPLEMENTARY_ALPHA * roll_gyro + (1.0 - COMPLEMENTARY_ALPHA) * roll_acc;
    pitch = COMPLEMENTARY_ALPHA * pitch_gyro + (1.0 - COMPLEMENTARY_ALPHA) * pitch_acc;

    yaw += gz * dt; // not fused with magnetometer here
  }
}

void apply_mixer_and_set_servos(float roll_cmd_deg, float pitch_cmd_deg) {
  // roll_cmd_deg and pitch_cmd_deg are deflections in degrees (positive/negative), map to servo angles
  // scale to FIN_MAX_DEFLECT
  float roll_effect = clampf(roll_cmd_deg, -90.0, 90.0) / 90.0 * FIN_MAX_DEFLECT;
  float pitch_effect = clampf(pitch_cmd_deg, -90.0, 90.0) / 90.0 * FIN_MAX_DEFLECT;

  // Mixer (same sign conventions as earlier):
  float defFL = FIN_NEUTRAL + (-roll_effect) + (-pitch_effect);
  float defFR = FIN_NEUTRAL + ( roll_effect) + (-pitch_effect);
  float defBL = FIN_NEUTRAL + (-roll_effect) + ( pitch_effect);
  float defBR = FIN_NEUTRAL + ( roll_effect) + ( pitch_effect);

  defFL = clampf(defFL, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);
  defFR = clampf(defFR, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);
  defBL = clampf(defBL, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);
  defBR = clampf(defBR, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);

  sFL.write((int)defFL);
  sFR.write((int)defFR);
  sBL.write((int)defBL);
  sBR.write((int)defBR);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // attach servos
  sFL.attach(SERVO_PIN_FL);
  sFR.attach(SERVO_PIN_FR);
  sBL.attach(SERVO_PIN_BL);
  sBR.attach(SERVO_PIN_BR);

  sFL.write(FIN_NEUTRAL);
  sFR.write(FIN_NEUTRAL);
  sBL.write(FIN_NEUTRAL);
  sBR.write(FIN_NEUTRAL);

  delay(200);

  // init IMU
  if (myICM.begin() != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 not detected. Check wiring / library.");
  } else {
    Serial.println("ICM-20948 initialized");
    // Optionally configure accelerometer/gyro ranges here via myICM.settings
  }

  last_us = micros();
  Serial.println("Ready. Send R<angle> or P<angle> to set desired attitude (deg). Eg: R10 or P-5");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_us) / 1e6;
  if (dt <= 0) dt = 0.001;
  last_us = now;

  updateIMU(dt);

  // simple proportional control from attitude error to fin deflection
  float err_roll = desired_roll_deg - roll;
  float err_pitch = desired_pitch_deg - pitch;

  float roll_cmd = Kp_roll * err_roll;   // degrees
  float pitch_cmd = Kp_pitch * err_pitch;

  // limit commands to reasonable range
  roll_cmd = clampf(roll_cmd, -FIN_MAX_DEFLECT, FIN_MAX_DEFLECT);
  pitch_cmd = clampf(pitch_cmd, -FIN_MAX_DEFLECT, FIN_MAX_DEFLECT);

  apply_mixer_and_set_servos(roll_cmd, pitch_cmd);

  // simple telemetry every 100 ms
  static unsigned long last_t = 0;
  if (millis() - last_t > 100) {
    last_t = millis();
    Serial.print("ROLL="); Serial.print(roll,2);
    Serial.print(",PITCH="); Serial.print(pitch,2);
    Serial.print(",dROLL="); Serial.print(desired_roll_deg,2);
    Serial.print(",dPITCH="); Serial.print(desired_pitch_deg,2);
    Serial.println();
  }

  // Serial commands to set desired attitude
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 1) {
      char c = s.charAt(0);
      float v = s.substring(1).toFloat();
      if (c == 'R') desired_roll_deg = clampf(v, -45, 45);
      if (c == 'P') desired_pitch_deg = clampf(v, -45, 45);
      if (c == 'N') { desired_roll_deg = 0; desired_pitch_deg = 0; }
      Serial.print("Set Droll="); Serial.print(desired_roll_deg);
      Serial.print(" Dpitch="); Serial.println(desired_pitch_deg);
    }
  }
}
