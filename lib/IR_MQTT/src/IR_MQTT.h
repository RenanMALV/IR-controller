#ifndef IR_MQTT_H
#define IR_MQTT_H

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

class IRMQTT {
  public:
    IRMQTT(const char* broker, uint16_t port, const char* sn, const char* fw, const char* location);
    void begin();
    void loop();
    void publishEnvSet(float temp, float humidity);
    void publishCommandSent(const String& command, bool success);
    //void publishReachedTemp(int temp);
    bool publishDiscovery(const char* ip, const char* fw, const char* location);
    void publishInfo(const char* status, const char* ip, const char* fw, const char* location);
    //void publishLocation(const String& location);
    void publishConfigEnd(unsigned long cmd);
    const char* _broker;
    
  private:
    void callback(char* topic, byte* payload, unsigned int length);
    void reconnect();
    void subscribeTopics();

    uint16_t _port;
    const char* _sn;
    const char* _fw;
    const char* _location;

    WiFiClient espClient;
    PubSubClient client;

};

#endif