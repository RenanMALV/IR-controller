#include "IR_MQTT.h"

IRMQTT::IRMQTT(const char* broker, uint16_t port, const char* sn)
  : _broker(broker), _port(port), _sn(sn), client(espClient) {}

void IRMQTT::begin() {
  client.setServer(_broker, _port);
  client.setCallback([this](char* topic, byte* payload, unsigned int length) {
    this->callback(topic, payload, length);
  });
}

void IRMQTT::loop() {
  if (!client.connected()) reconnect();
  client.loop(); // Função loop() do PubSubClient que processa mensagens recebidas e 
                 // mantém a conexão com o broker MQTT. keep-alive é gerenciado automaticamente.
}

void IRMQTT::reconnect() {
  while (!client.connected()) {
    String clientId = "IRCtrl_" + String(_sn);

    // Dados do LWT
    String willTopic = "discovery";
    StaticJsonDocument<128> willPayload;
    willPayload["sn"] = _sn;
    willPayload["status"] = "offline";

    char willMessage[128];
    serializeJson(willPayload, willMessage);

    // Conecta com LWT
    if (client.connect(clientId.c_str(), willTopic.c_str(), 1, true, willMessage)) {
        subscribeTopics();

        publishDiscovery(WiFi.localIP().toString().c_str(), "v1.0.3", "Living Room", "0xF00D", "0xF00E");
        // TODO: substituir os argumentos com valores reais

        Serial.println("MQTT connected.");
    } else {
      delay(2000);
    }
  }
}

void IRMQTT::subscribeTopics() {
  String tempTopic = "controller/" + String(_sn) + "/temperature";
  String stateTopic = "controller/" + String(_sn) + "/command/state";
  String infoTopic = "controller/" + String(_sn) + "/info";
  String locationTopic = "controller/" + String(_sn) + "/location";

  client.subscribe(tempTopic.c_str());
  client.subscribe(stateTopic.c_str());
  client.subscribe(infoTopic.c_str());
  client.subscribe(locationTopic.c_str());
}

void IRMQTT::callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) return;

  String topicStr(topic);

  if (topicStr.endsWith("/temperature")) {
    int temp = doc["value"];
    String cmd = doc["command"];
    // TODO: acionar IR com cmd e atualizar estado da temperatura
  } else if (topicStr.endsWith("/command/state")) {
    int state = doc["value"];
    String cmd = doc["command"];
    // TODO: acionar IR (liga ou desliga) com cmd
  }
  // TODO: tratar atualizações em info e location se necessário
}

void IRMQTT::publishCurrentTemp(int temp) {
  String topic = "controller/" + String(_sn) + "/temperature/current";
  StaticJsonDocument<64> doc;
  doc["value"] = temp;
  char payload[64];
  serializeJson(doc, payload);
  client.publish(topic.c_str(), payload);
}

void IRMQTT::publishCommandSent(const String& command, bool success) {
  String topic = "controller/command/notification";
  StaticJsonDocument<128> doc;
  doc["sn"] = _sn;
  doc["command"] = command;
  doc["status"] = success ? 1 : 0;
  char payload[128];
  serializeJson(doc, payload);
  client.publish(topic.c_str(), payload);
}

void IRMQTT::publishReachedTemp(int temp) {
  String topic = "controller/temperature/notification";
  StaticJsonDocument<96> doc;
  doc["sn"] = _sn;
  doc["value"] = temp;
  char payload[96];
  serializeJson(doc, payload);
  client.publish(topic.c_str(), payload);
}

void IRMQTT::publishDiscovery(const char* ip, const char* fw, const char* location, const String& onCmd, const String& offCmd) {
  String topic = "discovery";
  StaticJsonDocument<256> doc;
  doc["sn"] = _sn;
  doc["ip"] = ip;
  doc["status"] = "online";
  doc["firmware"] = fw;
  doc["location"] = location;
  doc["on_command"] = onCmd;
  doc["off_command"] = offCmd;
  char payload[256];
  serializeJson(doc, payload);
  client.publish(topic.c_str(), payload, true);  // retained
}

void IRMQTT::publishInfo(const char* status, const char* ip, const char* fw) {
  String topic = "controller/" + String(_sn) + "/info";
  StaticJsonDocument<128> doc;
  doc["status"] = status;
  doc["ip"] = ip;
  doc["firmware"] = fw;
  char payload[128];
  serializeJson(doc, payload);
  client.publish(topic.c_str(), payload);
}

void IRMQTT::publishLocation(const String& location) {
  String topic = "controller/" + String(_sn) + "/location";
  StaticJsonDocument<64> doc;
  doc["value"] = location;
  char payload[64];
  serializeJson(doc, payload);
  client.publish(topic.c_str(), payload);
}
