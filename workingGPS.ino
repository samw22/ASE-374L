#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// ===================== User Config =====================
static const int RXPin = 4;          // GPS TX -> Arduino pin 4 (SoftwareSerial RX)
static const int TXPin = 3;          // Optional: Arduino TX -> GPS RX
static const uint32_t GPSBaud = 9600; // GT-U7 default
// #define PRINT_RAW_NMEA            // <- uncomment to mirror raw NMEA to Serial for debugging
// =======================================================

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

bool hasFix = false;
unsigned long lastReportMs = 0;
unsigned long lastBlinkMs  = 0;
bool ledState = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  // For Leonardo/Micro (ATmega32u4): wait briefly for Serial to appear.
  // Won't block forever on Uno/Nano because they don't enumerate as USB CDC.
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) { /* wait up to 1.5s */ }

  gpsSerial.begin(GPSBaud);

  Serial.println(F("GT-U7 GPS starting up..."));
  Serial.println(F("Waiting for fix..."));
  Serial.println(F("Tip: go outside with clear sky; patch antenna faces up."));
}

void loop() {
  // Feed TinyGPS++ with all incoming bytes
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
#ifdef PRINT_RAW_NMEA
    Serial.write(c);  // raw mirror (can be noisy but super useful)
#endif
    gps.encode(c);
  }

  // Once per second: status report
  if (millis() - lastReportMs >= 1000) {
    lastReportMs += 1000;

    int sats = gps.satellites.isValid() ? gps.satellites.value() : -1;
    double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;

    if (gps.location.isValid()) {
      if (!hasFix) {
        Serial.println(F("✅ GPS fix acquired!"));
        hasFix = true;
      }

      Serial.print(F("Lat: "));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F("  Lon: "));
      Serial.print(gps.location.lng(), 6);

      Serial.print(F("  Sats: "));
      Serial.print(sats);
      Serial.print(F("  HDOP: "));
      Serial.print(hdop);

      Serial.print(F("  Alt: "));
      if (gps.altitude.isValid()) {
        Serial.print(gps.altitude.meters(), 1);
        Serial.print(F(" m (age "));
        Serial.print(gps.altitude.age());
        Serial.print(F(" ms)"));
      } else {
        Serial.print(F("INVALID"));
        Serial.print(F(" (age "));
        Serial.print(gps.altitude.age());
        Serial.print(F(" ms)"));
      }

      Serial.println();
    } else {
      if (hasFix) {
        Serial.println(F("⚠️  Lost GPS fix."));
        hasFix = false;
      }
      Serial.print(F("Waiting for fix... (sats="));
      Serial.print(sats);
      Serial.print(F(", hdop="));
      Serial.print(hdop);
      Serial.print(F(", chars="));
      Serial.print(gps.charsProcessed());
      Serial.print(F(", csFail="));
      Serial.print(gps.failedChecksum());
      Serial.println(F(")"));
    }
  }

  // LED heartbeat: slow blink when no fix, fast blink when fixed
  unsigned long now = millis();
  unsigned long period = hasFix ? 250 : 800; // ms
  if (now - lastBlinkMs >= period) {
    lastBlinkMs = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
}
