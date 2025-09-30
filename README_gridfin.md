gridfin_controller prototype

What this contains
- `gridfin_controller.ino` — prototype Arduino sketch that reads an IMU (placeholder), runs a simple PID-based inner-loop, mixes roll/pitch commands into 4 servos, and outputs telemetry over Serial.

Dependencies
- Replace IMU placeholder code with your chosen IMU library. Common choices:
  - MPU9250 / ICM-20689 libraries
  - Adafruit BNO055 (provides fused orientation)
  - Use Madgwick/Mahony filter or an EKF for attitude estimation
- For LoRa telemetry: RadioHead, LMIC, or SX127x libraries depending on hardware.

Wiring (example)
- Servos
  - FL signal -> Arduino pin 3
  - FR signal -> Arduino pin 5
  - BL signal -> Arduino pin 6
  - BR signal -> Arduino pin 9
  - Servo V+ -> external 5V supply (connect common ground to Arduino GND)
  - Servo GND -> common ground
- IMU (I2C)
  - SDA -> A4 (SDA on Arduino Uno R3)
  - SCL -> A5 (SCL on Arduino Uno R3)
  - Power -> 3.3V or 5V depending on module
- LoRa (optional) — separate SPI wiring depending on module

Usage (Arduino Uno R3)
1) Open `gridfin_controller.ino` and replace the `imu_update()` placeholder with your IMU read/filter code (Madgwick/AHRS recommended).
2) Upload to your board.
3) Open Serial Monitor at 115200 baud. You will see telemetry lines like `ROLL:...,PITCH:...,YAW:...`.
4) Send simple commands over Serial to set desired attitudes:
   - `R10` sets desired roll to +10 degrees
   - `P-5` sets desired pitch to -5 degrees
   - `N` sets neutral (0,0)

Tuning
- PID gains are conservative placeholders. Tune on the bench with the platform secured, then progress to tethered tests.
- Test servo travel limits and mechanical linkages before any drop test.

Safety
- Do not power servos from the Arduino 5V regulator if multiple or large servos are used; use a separate BEC or battery and ensure common GND.
- Add fuses or current limiting to prevent damage on stalls.
- Use a kill switch and tethered testing before any free-fall tests.

Notes
- This is a prototype skeleton — replace/improve the estimator, controller, and mixer as you progress.
