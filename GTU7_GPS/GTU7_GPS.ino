#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// --- Pin connections ---
// GT-U7 TX -> Arduino pin 4  (GPS → Arduino RX)
// GT-U7 RX -> Arduino pin 3  (optional, if you want to send data to GPS)
// VCC -> 5V (or 3.3V depending on your module)
// GND -> GND

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600; // GT-U7 default baud rate

// Create GPS and serial objects
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  Serial.println("GT-U7 GPS starting up...");
}

void loop() {
  // Read data from GPS
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // When a full sentence is decoded
      displayInfo();
    }
  }

  // If no data is received for 5 seconds, print a message
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected. Check wiring.");
    while (true);
  }
}

void displayInfo() {
  Serial.println(F("----- GPS Data -----"));

  if (gps.location.isValid()) {
    Serial.print(F("Latitude : "));
    Serial.println(gps.location.lat(), 6);
    Serial.print(F("Longitude: "));
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println(F("Location : INVALID"));
  }

  if (gps.altitude.isValid()) {
    Serial.print(F("Altitude : "));
    Serial.print(gps.altitude.meters());
    Serial.println(F(" m"));
  } else {
    Serial.println(F("Altitude : INVALID"));
  }

  Serial.println();
}
