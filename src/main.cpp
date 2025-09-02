#include <Arduino.h>
#include <Ticker.h>

#include "SerialUtils.h"
#include "IR_IO.h"
#include "CommandHandler.h"
#include "IR_MQTT.h"

#define LED D0
#define B_RATE 115200

Ticker envTicker;
#define ENV_PUBLISH_INTERVAL (uint32_t)5000  // Intervalo de publicação em milissegundos

#define WIFI_SSID "Renan"
#define WIFI_PASS "renan0405" // TODO substituir por SSID e senha reais ocult

// TODO: Criar arquivo de configuração separado para essas definições
#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_PORT 1883
#define _SN "ABC123"      // TODO gerar e guardar o SN na EEPROM de forma persistente e automática
#define _FW "v1.0.3"      // TODO substituir por versão real do firmware
#define _LOCATION "Lab 1" // TODO substituir por localização real do dispositivo

IRMQTT mqtt(
  MQTT_BROKER,
  MQTT_PORT,
  _SN,
  _FW,             
  _LOCATION        
);

void publishEnvironmentSettings() {
  mqtt.publishEnvSet(25.5+(millis()%3), 60.0+(millis()%3)); // TODO: valores reais do sensor
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(B_RATE);
  while (!Serial) delay(50);

  setupIR();

  WiFi.begin("Renan", "renan0405"); // TODO substituir por SSID e senha reais ocultos
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

  envTicker.attach_ms(ENV_PUBLISH_INTERVAL, publishEnvironmentSettings);
}

void loop() {
  // Serial Override
  //Serial.println("Handling command...");
  //handleCommand();
  
  mqtt.loop();
  yield();

  //Serial.println("Loop iteration completed.");
}