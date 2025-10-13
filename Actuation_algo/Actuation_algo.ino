#include <Wire.h>
#include <Servo.h>
#include <math.h>

#define LSM6DSL_ADDR 0x6A
#define WHO_AM_I     0x0F
#define CTRL1_XL     0x10
#define CTRL2_G      0x11
#define OUTX_L_G     0x22  // first gyro register
#define OUTX_L_XL    0x28  // first accel register

Servo servoBR;
Servo servoTR;
Servo servoBL;
Servo servoTL;

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t read16(byte reg) {
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSL_ADDR, 2);
  int16_t value = Wire.read() | (Wire.read() << 8);
  return value;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  servoTR.attach(12);
  servoBR.attach(13);
  servoTL.attach(9);
  servoBL.attach(3);

  servoTR.write(90);
  servoBR.write(90);
  servoTL.write(90);
  servoBL.write(90);

  writeRegister(CTRL1_XL, 0x40); // accel 104 Hz ±2g
  writeRegister(CTRL2_G, 0x40);  // gyro 104 Hz 245 dps

  Serial.println("LSM6DSL initialized!");
}

void loop() {
  // Raw accel
  int16_t ax = read16(OUTX_L_XL);
  int16_t ay = read16(OUTX_L_XL + 2);
  int16_t az = read16(OUTX_L_XL + 4);

  // Convert to g
  float ax_g = ax * 0.000061;
  float ay_g = ay * 0.000061;
  float az_g = az * 0.000061;

  // Calculate roll and pitch (in degrees)
  float roll = atan2(ay_g, az_g) * 180.0 / PI;
  float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

  // Print
  Serial.print("Roll: ");
  Serial.print(roll, 2);
  Serial.print("  Pitch: ");
  Serial.println(pitch, 2);

  // Example fin control (simple proportional)
  int base = 90;
  int corr = constrain((int)(pitch * 2), -45, 45);

  servoTR.write(base + corr);
  servoBR.write(base - corr);
  servoTL.write(base + corr);
  servoBL.write(base - corr);

  delay(200);
}