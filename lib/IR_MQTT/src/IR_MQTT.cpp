#include "IR_MQTT.h"

IRMQTT::IRMQTT(const char* broker, uint16_t port, const char* sn, const char* fw, const char* location)
  : _broker(broker), _port(port), _sn(sn), client(espClient), _fw(fw), _location(location) {}

void IRMQTT::begin() {
  client.setServer(_broker, _port);
  Serial.println("Server set");
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
    willPayload["ip"] = WiFi.localIP().toString();
    willPayload["status"] = "offline";
    willPayload["firmware"] = _fw;
    willPayload["location"] = _location;
    // Substituir os valores acima com dados reais
    

    char willMessage[128];
    serializeJson(willPayload, willMessage);

    // Conecta com LWT
    if (client.connect(clientId.c_str(), willTopic.c_str(), 1, true, willMessage)) {
        subscribeTopics();
        while( !publishDiscovery(WiFi.localIP().toString().c_str(), _fw, _location))
          Serial.println("Waiting for MQTT discovery...");
        Serial.println("MQTT connected.");
    } else {
      delay(2000);
    }
  }
}

void IRMQTT::subscribeTopics() {
  //String tempTopic = "controller/" + String(_sn) + "/temperature";
  //String locationTopic = "controller/" + String(_sn) + "/location";
  
  String stateTopic = "controller/" + String(_sn) + "/command/state";
  String infoTopic = "controller/" + String(_sn) + "/info";
  String configStartTopic = "controller/" + String(_sn) + "/configure/start";

  //client.subscribe(tempTopic.c_str());
  client.subscribe(stateTopic.c_str());
  client.subscribe(infoTopic.c_str());
  //client.subscribe(locationTopic.c_str());
  client.subscribe(configStartTopic.c_str());
}

void IRMQTT::callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message arrived on topic: " + String(topic));
  StaticJsonDocument<1024> jsonMsg;
  DeserializationError error = deserializeJson(jsonMsg, payload, length);
  if (error && !(String(topic).endsWith("/info") && length == 0)){
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str()); 
    return;
  }


  String topicStr(topic);

  if (topicStr.endsWith("/info")) {
    if (length == 0 || (length == 4 && strncmp((const char*)payload, "null", 4) == 0)) {
      Serial.println("Empty or null payload received... reporting device info.");
      publishInfo("online", WiFi.localIP().toString().c_str(), _fw, _location);
      return;
    }
    //else
    //  delay(1000);
    //  Serial.println("Info update received: " + String((const char*)payload));
    return;
  } else if (topicStr.endsWith("/command/state")) {
    int state = jsonMsg["value"];
    String cmdStr = jsonMsg["command"];
    String cmdType = jsonMsg["command_type"];
    String requestCode = jsonMsg["request_code"];

    unsigned long cmd = strtoul(cmdStr.c_str(), nullptr, 16); // base 16 for hex
    Serial.println("Received command: " + state);
    // TODO: acionar IR com o comando em cmd
    bool success = true; // placeholder, substituir pela resposta real do envio do comando
    publishCommandSent(requestCode, success);
    return;
  } else if (topicStr.endsWith("/configure/start")) {
    Serial.println("Configuration start");
    // TODO: recebe IR command do sensor
    unsigned long cmd = 0xB14779; // placeholder
    publishConfigEnd(cmd); // TODO: substituir pelo comando recebido do sensor
    return;
  }

}

void IRMQTT::publishEnvSet(float temp, float humidity) {
  String topic = "controller/" + String(_sn) + "/envset/current";
  StaticJsonDocument<64> jsonMsg;
  jsonMsg["temperature"] = temp;
  jsonMsg["humidity"] = humidity;
  char payload[64];
  serializeJson(jsonMsg, payload);
  client.publish(topic.c_str(), payload);
}

void IRMQTT::publishCommandSent(const String& request, bool success) {
  String topic = "controller/command/notification";
  StaticJsonDocument<128> jsonMsg;
  jsonMsg["sn"] = _sn;
  jsonMsg["request_code"] = request;
  jsonMsg["status"] = success;
  char payload[128];
  serializeJson(jsonMsg, payload);
  client.publish(topic.c_str(), payload);
}

/*void IRMQTT::publishReachedTemp(int temp) {
  String topic = "controller/temperature/notification";
  StaticJsonDocument<96> jsonMsg;
  jsonMsg["sn"] = _sn;
  jsonMsg["value"] = temp;
  char payload[96];
  serializeJson(jsonMsg, payload);
  client.publish(topic.c_str(), payload);
}*/

bool IRMQTT::publishDiscovery(const char* ip, const char* fw, const char* location) {
  String topic = "discovery";
  StaticJsonDocument<256> jsonMsg;
  jsonMsg["sn"] = _sn;
  jsonMsg["ip"] = ip;
  jsonMsg["status"] = "online";
  jsonMsg["firmware"] = fw;
  jsonMsg["location"] = location;
  char payload[256];
  serializeJson(jsonMsg, payload);
  if (client.publish(topic.c_str(), payload, true)) // retained (return true if success)
    Serial.println("Discovered Successfully!") ;
    return true;
  return false;
}

void IRMQTT::publishInfo(const char* status, const char* ip, const char* fw, const char* location) {
  String topic = "controller/" + String(_sn) + "/info";
  StaticJsonDocument<128> jsonMsg;
  jsonMsg["status"] = status;
  jsonMsg["ip"] = ip;
  jsonMsg["firmware"] = fw;
  jsonMsg["location"] = location;
  char payload[128];
  serializeJson(jsonMsg, payload);
  client.publish(topic.c_str(), payload);
}

/*void IRMQTT::publishLocation(const String& location) {
  String topic = "controller/" + String(_sn) + "/location";
  StaticJsonDocument<64> jsonMsg;
  jsonMsg["value"] = location;
  char payload[64];
  serializeJson(jsonMsg, payload);
  client.publish(topic.c_str(), payload);
}*/

void IRMQTT::publishConfigEnd(unsigned long cmd) {
  String topic = "controller/" + String(_sn) + "/configure/end";
  StaticJsonDocument<1024> jsonMsg;
  jsonMsg["command"] = cmd;
  char payload[1024];
  serializeJson(jsonMsg, payload);
  client.publish(topic.c_str(), payload);
  Serial.println("Published config end.");
}
