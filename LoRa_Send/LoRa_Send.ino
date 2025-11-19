#include <SPI.h>
#include <LoRa.h>

// Pin definitions for Arduino Uno
#define NSS   10   // LoRa chip select
#define RESET 9    // LoRa reset
#define DIO0  2    // LoRa DIO0

int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Transceiver Starting...");

  LoRa.setPins(NSS, RESET, DIO0);

  // Use the confirmed 433 MHz frequency
  if (!LoRa.begin(434E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Initialized!");
  Serial.println("------------------------------");
}

void loop() {
  // --- Send a message ---
  Serial.print("Sending packet: ");
  Serial.println(counter);

  LoRa.beginPacket();
  LoRa.print("Hello from Arduino ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  // --- Listen for incoming packets ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet: ");
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    Serial.print(" | RSSI: ");
    Serial.println(LoRa.packetRssi());
  }

  delay(2000);
}