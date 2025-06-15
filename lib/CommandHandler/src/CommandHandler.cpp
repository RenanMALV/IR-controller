#include <Arduino.h>
#include <ArduinoJson.h>
#include "SerialUtils.h"
#include "ir_io.h"

void handleCommand() {
  char msg[12];
  serialRead(msg);
  String command(msg);

  command.trim();
  command.toLowerCase();

  if (command == "read") {
    Serial.println("Ready to read...");
    irrecv.resume();
    while (!irrecv.decode(&results)) {
      yield();
    }
    receiveIR();
    irrecv.pause();
    Serial.println("Read completed.");
  } else if (command == "send") {
    Serial.println("Sending IR...");
    sendIR();
  } else if (command == "save") {
    Serial.println("Enter action name:");
    serialRead(msg);
    String name(msg);

    JsonDocument doc;
    doc["name"] = name;
    doc["size"] = size;
    doc["protocol"] = protocol;
    doc["value"] = results.value;

    JsonArray state = doc["state"].to<JsonArray>();
    for (u_int i = 0; i < sizeof(results.state); ++i) {
      state.add(results.state[i]);
    }

    JsonArray data = doc["data"].to<JsonArray>();
    for (int i = 0; i < size; ++i) {
      data.add(raw_array[i]);
    }

    serializeJson(doc, Serial);
    Serial.println();
  }
}