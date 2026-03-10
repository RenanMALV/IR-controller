#include "IR_MQTT.h"

const char* IRMQTT::getLastTopicLevel(const char* topic) {

    char* lastSlash = strrchr(topic, '/');

    if (lastSlash != NULL && *(lastSlash + 1) != '\0') {
        return lastSlash + 1;
    }

    return NULL;
}

IRMQTT::IRMQTT(const char* broker, const char* user, const char* pass, uint16_t port, const char* sn, const char* fw, const char* location)
  : _broker(broker), _user(user), _pass(pass), _port(port), _sn(sn), _fw(fw), _location(location), client(espClient) {
    espClient.setInsecure(); // Aceitar qualquer certificado TLS substituir por
    // espClient.setCACert(ca_cert);
  }

void IRMQTT::begin() {
  client.setBufferSize(4096);  // Limitado pelo tamanho da mesagem IR
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
    Serial.println("Attempting MQTT reconnection...");
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
    if (client.connect(clientId.c_str(), _user, _pass, willTopic.c_str(), 1, true, willMessage)) {
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
  String stateTopic = "controller/" + String(_sn) + "/command/state";
  String infoTopic = "controller/" + String(_sn) + "/info";
  String configStartTopic = "controller/" + String(_sn) + "/configure/start/#";
  //String envSetingsTopic = "controller/" + String(_sn) + "/envset/current";
  String commandNotificationTopic = "controller/command/notification";

  client.subscribe(stateTopic.c_str());
  client.subscribe(infoTopic.c_str());
  client.subscribe(configStartTopic.c_str());
  //client.subscribe(envSetingsTopic.c_str());
  client.subscribe(commandNotificationTopic.c_str());
}

void IRMQTT::callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message arrived on topic: " + String(topic));
  StaticJsonDocument<1024> jsonMsg;
  DeserializationError error = deserializeJson(jsonMsg, payload, length);
  if (error && 
      !(String(topic).endsWith("/info") && length == 0) &&
      !((String(topic).indexOf("/configure/start") != -1) && length == 0)) {
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
    
    String requestCode = jsonMsg["request_code"] | "";

    // Ler 'command' (RAW). Preferimos array; como fallback, aceitar string CSV.
    std::vector<uint16_t> raw;
    if (jsonMsg["raw_code"].is<JsonArray>()) {
      JsonArray arr = jsonMsg["raw_code"].as<JsonArray>();
      raw.reserve(arr.size());
      for (JsonVariant v : arr) {
        unsigned long val = v.as<unsigned long>();
        raw.push_back(static_cast<uint16_t>(val > 0xFFFF ? 0xFFFF : val));
      }
    } else if (jsonMsg["raw_code"].is<const char*>()) {
      // Fallback: "9000,4500,560,560,..."
      String csv = jsonMsg["raw_code"].as<const char*>();
      raw.clear(); raw.reserve(256);
      uint32_t acc = 0; bool inNum = false;
      for (size_t i = 0; i <= csv.length(); ++i) {
        char c = (i < csv.length()) ? csv[i] : ','; // força flush no fim
        if (isDigit(c)) { acc = acc * 10 + (c - '0'); inNum = true; }
        else if (c == ',' || c == ' ' || c == '\t') {
          if (inNum) {
            raw.push_back(static_cast<uint16_t>(acc > 0xFFFF ? 0xFFFF : acc));
            acc = 0; inNum = false;
          }
        }
      }
    } else {
      Serial.println("IR RAW: 'command' ausente ou com tipo inválido.");
      publishCommandSent(requestCode, false);
      return;
    }

    if (raw.empty()) {
      Serial.println("IR RAW: vetor vazio.");
      publishCommandSent(requestCode, false);
      return;
    }

    // Preparar envio RAW pelo módulo IR
    IR_setRaw(raw.data(), raw.size()); // prioriza RAW
    bool ok = IR_send(); // envia RAW

    Serial.printf("IR TX %s | RAW len=%u \n", ok ? "OK" : "FAIL", (unsigned)raw.size());

    publishCommandSent(requestCode, ok);
    return;
  } else if (topicStr.indexOf("/configure/start") != -1) {
    
    const char* requestCode = getLastTopicLevel(topic);

    Serial.println("Config start received. Request code: " + String(requestCode ? requestCode : "NULL"));

    if (requestCode == NULL) {
      publishConfigEnd(ConfigStatus::MALFORMED_REQUEST, NULL); // Request code mal formado
      return;
    }


    Serial.println("IR learn (RAW): iniciando janela curta de captura...");

    IR_resume();
    const uint32_t t0 = millis();
    const uint32_t timeout_ms = 10000; // TODO: tornar configurável via define config
    bool got = false;

    while (millis() - t0 < timeout_ms) {
      if (IR_poll()) { got = true; break; }
      yield();
    }
    IR_pause();

    if (!got) {
      Serial.println("IR learn: nenhum sinal recebido.");
      publishConfigEnd(ConfigStatus::TIMEOUT, requestCode); // Timeout
      return;
    }

    publishConfigEnd(ConfigStatus::SUCCESS, requestCode); // Success
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
  if (client.publish(topic.c_str(), payload, true)){ // retained (return true if success)
    Serial.println("Discovered Successfully!") ;
    return true;
  }
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

void IRMQTT::publishConfigEnd(ConfigStatus cmd, const char* requestCode) {
  String topic = "controller/" + String(_sn) + "/configure/end/";
  if (requestCode != NULL) {
    topic += requestCode;
  }

  StaticJsonDocument<2048> out;
  String payloadJson;

  // Criar array command sempre (estrutura consistente)
  JsonArray data = out.createNestedArray("command");

  if (cmd == ConfigStatus::SUCCESS) {  // Success

    out["status"] = "success";

    const uint16_t* r = IR_raw();
    size_t n = IR_raw_len();

    for (size_t i = 0; i < n; ++i) {
      data.add(r[i]);
    }

    Serial.println("Publishing SUCCESS with RAW data");

  }
  else if (cmd == ConfigStatus::TIMEOUT) {  // Timeout

    out["status"] = "timeout";
    Serial.println("Publishing TIMEOUT");

  }
  else if (cmd == ConfigStatus::MALFORMED_REQUEST) {  // Malformed request code

    out["status"] = "malformed_request_code";
    Serial.println("Publishing MALFORMED REQUEST CODE");

  }
  else {  // fallback defensivo

    out["status"] = "unknown_error";
    Serial.println("Publishing UNKNOWN ERROR");

  }

  serializeJson(out, payloadJson);

  size_t payload_len = measureJson(out);
  Serial.printf("MQTT buffer=%u | JSON len=%u\n",
                client.getBufferSize(),
                (unsigned)payload_len);

  bool pubOk = client.publish(topic.c_str(), payloadJson.c_str());

  if (pubOk)
    Serial.println("Published config end.");
  else
    Serial.println("Failed to publish config end.");
}
