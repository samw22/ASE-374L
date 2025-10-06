#include <Wire.h>
#include <Servo.h>

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

  servoTR.write(180);
  servoBR.write(180);
  servoTL.write(180);
  servoBL.write(180);

  // Check WHO_AM_I
  Wire.beginTransmission(LSM6DSL_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSL_ADDR, 1);
  if (Wire.available()) {
    byte id = Wire.read();
    Serial.print("WHO_AM_I: 0x");
    Serial.println(id, HEX);
  }

  // Enable accelerometer @104 Hz, ±2g
  writeRegister(CTRL1_XL, 0x40);
  // Enable gyro @104 Hz, 245 dps
  writeRegister(CTRL2_G, 0x40);

  Serial.println("LSM6DSL initialized!");
}

void loop() {
  // Raw gyro
  int16_t gx = read16(OUTX_L_G);
  int16_t gy = read16(OUTX_L_G + 2);
  int16_t gz = read16(OUTX_L_G + 4);

  // Raw accel
  int16_t ax = read16(OUTX_L_XL);
  int16_t ay = read16(OUTX_L_XL + 2);
  int16_t az = read16(OUTX_L_XL + 4);

  // Convert to physical units
  float ax_g = ax * 0.000061;   // g
  float ay_g = ay * 0.000061;
  float az_g = az * 0.000061;

  float gx_dps = gx * 0.00875;  // dps
  float gy_dps = gy * 0.00875;
  float gz_dps = gz * 0.00875;

  // Print results
  Serial.print("Accel (g) X: "); Serial.print(ax_g, 3);
  Serial.print(" Y: "); Serial.print(ay_g, 3);
  Serial.print(" Z: "); Serial.println(az_g, 3);

  Serial.print("Gyro (dps) X: "); Serial.print(gx_dps, 3);
  Serial.print(" Y: "); Serial.print(gy_dps, 3);
  Serial.print(" Z: "); Serial.println(gz_dps, 3);

  if (ax_g < 0){
    servoBR.write(ax_g*-100);
  }
  else{
    servoBR.write(ax_g*100);
  }

  Serial.println();
  delay(500);
}