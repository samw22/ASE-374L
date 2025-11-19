#include <SPI.h>
#include <LoRa.h>

// Pin definitions for Arduino Uno
#define NSS   10   // LoRa chip select
#define RESET 9    // LoRa reset
#define DIO0  2    // LoRa DIO0

unsigned long lastCheck = 0;
bool gotPacket = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver Starting...");

  LoRa.setPins(NSS, RESET, DIO0);

  // Use same frequency as transmitter
  if (!LoRa.begin(434E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Receiver Ready!");
  Serial.println("------------------------------");
}

void loop() {

  // Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    gotPacket = true;

    Serial.print("Received packet: ");
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    Serial.print(" | RSSI: ");
    Serial.println(LoRa.packetRssi());
  }

  // Every 2 seconds, report if nothing was received
  if (millis() - lastCheck >= 2000) {
    if (!gotPacket) {
      Serial.println("No data received");
    }
    gotPacket = false;            // reset for next period
    lastCheck = millis();
  }
}
