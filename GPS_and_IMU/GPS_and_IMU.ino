#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo servo13;
Servo servo12;
Servo servo11;
Servo servo10;

#define LSM6DSL_ADDR  0x6A   // Accelerometer + Gyro
#define LIS3MDL_ADDR  0x1C   // Magnetometer
#define PI 3.14159265358979323846

// --- GPS Setup ---
static const int RXPin = 4, TXPin = 3;   // GT-U7 TX → Pin 4 (Arduino RX)
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// --- Global sensor variables ---
float ax, ay, az;
float mx, my, mz;

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  gpsSerial.begin(GPSBaud);

  servo13.attach(13);
  servo12.attach(12);
  servo11.attach(11);
  servo10.attach(10);

  servo13.write(90);
  servo12.write(90);
  servo11.write(90);
  servo10.write(90);

  delay(5000);

  Serial.println("Initializing sensors...");

  // --- Initialize Accelerometer (LSM6DSL) ---
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(0x10);           // CTRL1_XL register
  Wire.write(0b01100000);     // 104 Hz, 2g, 100 Hz BW
  Wire.endTransmission();

  // --- Initialize Magnetometer (LIS3MDL) ---
  Wire.beginTransmission(LIS3MDL_ADDR);
  Wire.write(0x20);           // CTRL_REG1
  Wire.write(0b11111100);     // Ultra-high performance XY, 80 Hz
  Wire.endTransmission();

  Wire.beginTransmission(LIS3MDL_ADDR);
  Wire.write(0x21);           // CTRL_REG2
  Wire.write(0b00000000);     // +/-4 gauss
  Wire.endTransmission();

  Wire.beginTransmission(LIS3MDL_ADDR);
  Wire.write(0x22);           // CTRL_REG3
  Wire.write(0b00000000);     // Continuous conversion mode
  Wire.endTransmission();

  Serial.println("BerryGPS-IMU-4 + GT-U7 initialized.");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  // --- Read GPS data ---
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // --- Read IMU data ---
  readAccel();
  readMag();

  // --- Compute orientation ---
  float roll  = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float mx_corr = mx;
  float my_corr = my;
  float mz_corr = -mz;

  float mx2 = mx_corr * cos(pitch) + mz_corr * sin(pitch);
  float my2 = mx_corr * sin(roll) * sin(pitch) + my_corr * cos(roll) - mz_corr * sin(roll) * cos(pitch);

  float heading = atan2(my2, mx2) * 180.0 / PI;
  if (heading < 0) heading += 360.0;

    // --- Compute servo angle opposite to yaw ---
  float yaw = heading;                    // heading in degrees (0–360)
  if (yaw > 180) yaw -= 360;              // convert to -180–180 range

  // float servoAngle = -yaw;                // opposite of yaw
  // servoAngle = constrain(servoAngle, -30, 30);  // limit to ±30 degrees

  // // Map from [-30, 30] → [60, 120] so 0° = center (90°)
  // int servoOutput = map(servoAngle, -30, 30, 60, 120);

  if (yaw > 15){
    // servo13.write(125);
    // servo12.write(125);
    // servo11.write(125);
    // servo10.write(125);
    allServos(125);
  }
  else if (yaw < 345){
    // servo13.write(55);
    // servo12.write(55);
    // servo11.write(55);
    // servo10.write(55);
    allServos(55);
  }
  else{
    // servo13.write(90);
    // servo12.write(90);
    // servo11.write(90);
    // servo10.write(90);
    allServos(90);
  }

  // servo13.write(servoOutput);

  Serial.print(F(" | Servo Output: "));
  // Serial.println(servoOutput);

  // --- Print everything ---
  Serial.println(F("----- DATA -----"));
  Serial.print(F("Yaw (deg): "));   Serial.print(heading, 2);
  Serial.print(F(" | Pitch (deg): ")); Serial.print(pitch * 180.0 / PI, 2);
  Serial.print(F(" | Roll (deg): "));  Serial.println(roll * 180.0 / PI, 2);

  if (gps.location.isValid()) {
    Serial.print(F("Lat: ")); Serial.print(gps.location.lat(), 6);
    Serial.print(F(" | Lon: ")); Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("Lat/Lon: INVALID"));
  }

  if (gps.altitude.isValid()) {
    Serial.print(F(" | Alt: ")); Serial.print(gps.altitude.meters());
    Serial.println(F(" m"));
  } else {
    Serial.println(F(" | Alt: INVALID"));
  }

  if (gps.altitude.meters() < 1000){
    allServos(125);
    delay(500);
    allServos(55);
    delay(500);
  }

  Serial.println();
  delay(200);
}

// ======================================================
// IMU FUNCTIONS
// ======================================================
void readAccel() {
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(0x28);  // OUTX_L_XL
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSL_ADDR, 6);

  int16_t rawX = Wire.read() | (Wire.read() << 8);
  int16_t rawY = Wire.read() | (Wire.read() << 8);
  int16_t rawZ = Wire.read() | (Wire.read() << 8);

  ax = rawX * 0.000061;
  ay = rawY * 0.000061;
  az = rawZ * 0.000061;
}

void readMag() {
  Wire.beginTransmission(LIS3MDL_ADDR);
  Wire.write(0x28);  // OUT_X_L
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3MDL_ADDR, 6);

  int16_t rawX = Wire.read() | (Wire.read() << 8);
  int16_t rawY = Wire.read() | (Wire.read() << 8);
  int16_t rawZ = Wire.read() | (Wire.read() << 8);

  mx = rawX * 0.00014;
  my = rawY * 0.00014;
  mz = rawZ * 0.00014;
}

void allServos(int servoVal) {
  servo13.write(servoVal);
  servo12.write(servoVal);
  servo11.write(servoVal);
  servo10.write(servoVal);
}