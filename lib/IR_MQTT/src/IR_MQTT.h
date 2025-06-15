#ifndef IR_MQTT_H
#define IR_MQTT_H

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

class IRMQTT {
  public:
    IRMQTT(const char* broker, uint16_t port, const char* sn);
    void begin();
    void loop();
    void publishCurrentTemp(int temp);
    void publishCommandSent(const String& command, bool success);
    void publishReachedTemp(int temp);
    void publishDiscovery(const char* ip, const char* fw, const char* location, const String& onCmd, const String& offCmd);
    void publishInfo(const char* status, const char* ip, const char* fw);
    void publishLocation(const String& location);
    const char* _broker;
    
  private:
    void callback(char* topic, byte* payload, unsigned int length);
    void reconnect();
    void subscribeTopics();

    uint16_t _port;
    const char* _sn;

    WiFiClient espClient;
    PubSubClient client;
};

#endif