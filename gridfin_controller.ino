// GPS (TinyGPS++) and SoftwareSerial for GPS UART
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// LoRa (RadioHead) - assume RFM95 / SX1276 connected via SPI
#include <SPI.h>
#include <RH_RF95.h>
// --- GPS / LoRa config ---
// SoftwareSerial pins for GPS (connect GPS TX -> UNO D8, GPS RX -> UNO D7 if needed)
SoftwareSerial gpsSerial(8, 7); // RX, TX (we only need RX to read GPS TX)
TinyGPSPlus gps;

// RFM95 pins (UNO): NSS/CS on 10, DIO0/IRQ on 2, RESET on 4 (adjust wiring as needed)
#define RFM95_CS 10
#define RFM95_INT 2
#define RFM95_RST 4
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// LoRa frequency (set to your regional frequency: 915.0 for US, 868.0 for EU)
const float LORA_FREQ_MHZ = 915.0;
/*
  gridfin_controller.ino
  Prototype controller for 4 grid-fins using 4 servos, IMU for attitude estimation (Madgwick),
  simple PID inner-loop for roll/pitch rate control, and optional LoRa telemetry.

  NOTE: This is a prototype meant for bench and tethered testing only. Do NOT run on an actual
  flight platform without thorough testing, tuning, and safety checks.

  Dependencies (Arduino/PlatformIO):
    - Adafruit BusIO (if required by sensor library)
    - Adafruit_Sensor (optional)
    - Adafruit_BNO055 or an IMU library for the chosen IMU (example uses a generic I2C MPU/ICM)
    - MadgwickAHRS (Madgwick filter) or implement Madgwick/Mahony
    - RadioHead or a LoRa library for telemetry (optional)

  Hardware assumptions:
    - 4 analog/digital PWM servos on pins SERVO_PIN_FL, FR, BL, BR
    - IMU on I2C (e.g., ICM-20689/MPU-9250)
    - Optional LoRa module via SPI

  This sketch demonstrates:
    - Read IMU (gyro+accel), run Madgwick to get roll/pitch/yaw
    - Simple PID on roll and pitch to compute desired fin deflections
    - Mixer mapping roll/pitch commands to four fins
    - Telemetry over Serial (and optional LoRa placeholder)

  Replace sensor/LoRa parts with libraries matching your hardware.
*/

#include <Servo.h>
#include <Wire.h>

// ----- Configuration -----
// Pins selected for Arduino Uno R3 (PWM-capable pins): 3, 5, 6, 9
// If you change boards, ensure the pins support PWM for standard hobby servos.
const int SERVO_PIN_FL = 3; // front-left fin (or top-left) -> PWM pin 3 (UNO)
const int SERVO_PIN_FR = 5; // front-right -> PWM pin 5 (UNO)
const int SERVO_PIN_BL = 6; // back-left  -> PWM pin 6 (UNO)
const int SERVO_PIN_BR = 9; // back-right -> PWM pin 9 (UNO)

// Fin neutral angle (degrees) and maximum deflection
const int FIN_NEUTRAL = 90; // servo midpoint
const int FIN_MAX_DEFLECT = 30; // +/- degrees from neutral

// IMU settings (pseudo; replace with real IMU library usage)
const float GYRO_SENS = 1.0; // placeholder scale

// Control loop rates
const unsigned long LOOP_US = 5000; // 200 Hz control loop (5000 us)

// PID gains for roll and pitch rate (very conservative defaults)
struct PID {
  float kp;
  float ki;
  float kd;
  float integrator;
  float last_error;
  float integrator_limit;
} pid_roll = {0.08, 0.002, 0.001, 0.0, 0.0, 50.0},
  pid_pitch = {0.08, 0.002, 0.001, 0.0, 0.0, 50.0};

// Desired setpoints (outer-loop guidance should update these)
volatile float desired_roll_deg = 0.0;   // desired roll angle (deg)
volatile float desired_pitch_deg = 0.0;  // desired pitch angle (deg)

// State from IMU
volatile float roll = 0.0, pitch = 0.0, yaw = 0.0;
volatile float p_rate = 0.0, q_rate = 0.0, r_rate = 0.0; // angular rates (deg/s)

// Servo objects
Servo sFL, sFR, sBL, sBR;

// Timing
unsigned long last_loop_us = 0;
// Note: BerryGPS integration points should be considered for GPS data handling.
// Ensure to initialize gpsSerial and rf95 in setup() and handle data in loop().

// ----------------- Helper functions -----------------
float constrainf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float pid_update(PID &pid, float error, float dt) {
  pid.integrator += error * dt;
  pid.integrator = constrainf(pid.integrator, -pid.integrator_limit, pid.integrator_limit);
  float derivative = 0.0;
  if (dt > 0) derivative = (error - pid.last_error) / dt;
  float out = pid.kp * error + pid.ki * pid.integrator + pid.kd * derivative;
  pid.last_error = error;
  return out;
}

// Simple mixer: map roll_cmd (deg) and pitch_cmd (deg) to fin deflections
// Assuming fins arranged as: FL (left front), FR (right front), BL (left back), BR (right back)
// Positive roll command -> right wing down (i.e., left fins deflect opposite to right fins)
// Positive pitch command -> nose up (front fins down, rear fins up) depending on geometry

void apply_mixer(float roll_cmd_deg, float pitch_cmd_deg) {
  // scale commands to fin deflection [-FIN_MAX_DEFLECT, FIN_MAX_DEFLECT]
  float roll_effect = constrainf(roll_cmd_deg, -30.0, 30.0) / 30.0 * FIN_MAX_DEFLECT;
  float pitch_effect = constrainf(pitch_cmd_deg, -30.0, 30.0) / 30.0 * FIN_MAX_DEFLECT;

  // Example mixing (linear combination)
  float defFL =  FIN_NEUTRAL + (-roll_effect) + (-pitch_effect); // front-left
  float defFR =  FIN_NEUTRAL + ( roll_effect) + (-pitch_effect); // front-right
  float defBL =  FIN_NEUTRAL + (-roll_effect) + ( pitch_effect); // back-left
  float defBR =  FIN_NEUTRAL + ( roll_effect) + ( pitch_effect); // back-right

  // Constrain to servo travel
  defFL = constrainf(defFL, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);
  defFR = constrainf(defFR, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);
  defBL = constrainf(defBL, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);
  defBR = constrainf(defBR, FIN_NEUTRAL - FIN_MAX_DEFLECT, FIN_NEUTRAL + FIN_MAX_DEFLECT);

  sFL.write((int)defFL);
  sFR.write((int)defFR);
  sBL.write((int)defBL);
  sBR.write((int)defBR);
}

// Placeholder IMU read/update — replace with real IMU code and filter (Madgwick/EKF)
void imu_update(float dt) {
  // In a real implementation: read gyro (rad/s) and accel; run Madgwick/Mahony filter to update roll/pitch/yaw.
  // Here we simulate small drift or keep values 0 for testing.

  // Example: read raw gyro from I2C and convert to deg/s
  // p_rate = gyro_x * GYRO_SENS;
  // q_rate = gyro_y * GYRO_SENS;
  // r_rate = gyro_z * GYRO_SENS;
  // run Madgwick update to set roll,pitch,yaw

  // For prototype, keep states unchanged.
}

// Telemetry print
void send_telemetry() {
  // Send basic telemetry over Serial. Replace or augment with LoRa send.
  Serial.print("ROLL:"); Serial.print(roll,2);
  Serial.print(",PITCH:"); Serial.print(pitch,2);
  Serial.print(",YAW:"); Serial.print(yaw,2);
  Serial.print(",p:"); Serial.print(p_rate,2);
  Serial.print(",q:"); Serial.print(q_rate,2);
  Serial.print(",r:"); Serial.print(r_rate,2);
  Serial.print("\n");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Attach servos
  sFL.attach(SERVO_PIN_FL);
  sFR.attach(SERVO_PIN_FR);
  sBL.attach(SERVO_PIN_BL);
  sBR.attach(SERVO_PIN_BR);

  // Initialize to neutral
  sFL.write(FIN_NEUTRAL);
  sFR.write(FIN_NEUTRAL);
  sBL.write(FIN_NEUTRAL);
  sBR.write(FIN_NEUTRAL);

  delay(500);
  last_loop_us = micros();

  Serial.println("gridfin_controller ready");
  // Note: On Arduino Uno R3, I2C (Wire) uses pins A4 (SDA) and A5 (SCL).
  // Make sure your IMU's SDA/SCL lines are connected accordingly and powered at the correct voltage.
  // Initialize GPS serial
  gpsSerial.begin(9600);

  // Initialize LoRa (RFM95)
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("ERROR: RF95 init failed");
  } else {
    Serial.println("RF95 init OK");
    if (!rf95.setFrequency(LORA_FREQ_MHZ)) {
      Serial.println("ERROR: RF95 setFrequency failed");
    }
    // Set transmit power (0-23 dBm depending on module)
    rf95.setTxPower(13, false);
  }
}

void loop() {
  unsigned long now = micros();
  unsigned long dt_us = now - last_loop_us;
  if (dt_us < LOOP_US) return;
  last_loop_us = now;
  float dt = dt_us / 1e6f;

  // 1) Update IMU and state estimation
  imu_update(dt);

  // Read any incoming GPS characters and feed TinyGPS++
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // 2) Compute attitude error (convert to degrees)
  float err_roll = desired_roll_deg - roll;   // degrees
  float err_pitch = desired_pitch_deg - pitch;

  // 3) Inner-loop: use PID to command roll/pitch rates (here PID output in deg/s desired)
  float roll_rate_cmd = pid_update(pid_roll, err_roll, dt);
  float pitch_rate_cmd = pid_update(pid_pitch, err_pitch, dt);

  // 4) Simple feedforward: compute command as difference between desired rate and current rate
  float roll_rate_error = roll_rate_cmd - p_rate;   // p_rate should be roll rate (deg/s)
  float pitch_rate_error = pitch_rate_cmd - q_rate;

  // 5) Map rate errors to fin angle commands (here we treat output as direct angle command for simplicity)
  // In a robust design, you'd have a rate controller (PID) whose output maps to moments and then fin angles via allocation.
  float roll_cmd_deg = roll_rate_error * 0.2;   // scale factor — tune
  float pitch_cmd_deg = pitch_rate_error * 0.2;

  // 6) Apply mixer to set servos
  apply_mixer(roll_cmd_deg, pitch_cmd_deg);

  // 7) Telemetry at lower rate: print every N loops
  static int tl_count = 0;
  if (++tl_count >= 20) { // ~10 Hz telemetry at 200 Hz loop
    send_telemetry();
    tl_count = 0;
    // Also send a small LoRa telemetry packet (if radio initialized)
    if (rf95.available()) {
      // If data waiting, you might read it (not used here)
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        Serial.print("LoRa recv: ");
        for (uint8_t i = 0; i < len; i++) Serial.print((char)buf[i]);
        Serial.println();
      }
    }
    // Build a simple telemetry string
    char tbuf[128];
    float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    float lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    int sat = gps.satellites.value();
    int fix = gps.location.isValid() ? 1 : 0;
    snprintf(tbuf, sizeof(tbuf), "T,ROLL=%.2f,PITCH=%.2f,YAW=%.2f,LAT=%.6f,LON=%.6f,FIX=%d,SAT=%d", roll, pitch, yaw, lat, lon, fix, sat);
    if (rf95.init()) {
      rf95.send((uint8_t *)tbuf, strlen(tbuf));
      rf95.waitPacketSent();
    }
  }

  // 8) Respond to Serial commands (simple console commands to set desired attitudes)
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() == 0) break;
    // commands: R:+/-angle, P:+/-angle, N -> neutral
    if (s.charAt(0) == 'R') {
      desired_roll_deg = s.substring(1).toFloat();
      Serial.print("Set desired_roll: "); Serial.println(desired_roll_deg);
    } else if (s.charAt(0) == 'P') {
      desired_pitch_deg = s.substring(1).toFloat();
      Serial.print("Set desired_pitch: "); Serial.println(desired_pitch_deg);
    } else if (s == "N") {
      desired_roll_deg = 0; desired_pitch_deg = 0;
      Serial.println("Neutral");
    }
  }
}
