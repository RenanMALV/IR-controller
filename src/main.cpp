#include <Arduino.h>
#include <Ticker.h>

#include "SerialUtils.h"
#include "IR_IO.h"
//#include "CommandHandler.h"
#include "IR_MQTT.h"
#include "DHT.h"

#define LED D0
#define B_RATE 115200

#define WIFI_SSID "labFUTURO"
#define WIFI_PASS "" // TODO substituir por SSID e senha reais ocult

// TODO: Criar arquivo de configuração separado para essas definições
#define MQTT_BROKER "10.10.19.118"
#define MQTT_PORT 1883
#define MQTT_USER "controle"
#define MQTT_PASS "@96jK2nmM5DqZ47w5H7npMa9f@sKuJ"
#define _SN "ABC123"      // TODO gerar e guardar o SN na EEPROM de forma persistente e automática
#define _FW "v1.0.3"      // TODO substituir por versão real do firmware
#define _LOCATION "Lab 3" // TODO substituir por localização real do dispositivo

IRMQTT mqtt(
  MQTT_BROKER,
  MQTT_USER,
  MQTT_PASS,
  MQTT_PORT,
  _SN,
  _FW,             
  _LOCATION        
);

Ticker envTicker;
#define ENV_PUBLISH_INTERVAL (uint32_t)5000  // Intervalo de publicação das condições ambientais em milissegundos

// DHT sensor
#define DHTPIN D4       // Pino conectado ao sensor DHT
#define DHTTYPE DHT22   // DHT 22  (AM2302), DHT11, etc.
DHT dht(DHTPIN, DHTTYPE);

void publishEnvironmentSettings() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temp) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  mqtt.publishEnvSet(temp, humidity);
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(B_RATE);
  while (!Serial) delay(50);

  Serial.println("Starting Temperature and Humidity Sensor...");
  delay(1000); // Aguarda estabilização do sensor (Obrigatório pelo data sheet)
  dht.begin();

  Serial.println("Starting IR Module...");
  IR_setup();

  WiFi.begin(WIFI_SSID, WIFI_PASS); // TODO substituir por SSID e senha reais ocultos
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