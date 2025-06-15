#include <Arduino.h>

#include "SerialUtils.h"
#include "IR_IO.h"
#include "CommandHandler.h"
#include "IR_MQTT.h"

#define LED D0
#define B_RATE 115200

IRMQTT mqtt(
  "broker.example.com",  // endereço do broker
  1883,                  // porta
  "ABC123"               // serial number da controladora
  // TODO gerar e guardar o SN na EEPROM de forma persistente e automática
);

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(B_RATE);
  while (!Serial) delay(50);

  setupIR();

  WiFi.begin("SSID", "senha"); // TODO substituir por SSID e senha reais
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nWiFi connected.");
  // TODO: reconectar ao WiFi se a conexão cair

  Serial.print("Connecting to MQTT broker at ");
  Serial.println(mqtt._broker);
  mqtt.begin();
  mqtt.loop();

  Serial.println("Setup completed.");
}

void loop() {
  handleCommand();
  mqtt.loop();
  yield();
}