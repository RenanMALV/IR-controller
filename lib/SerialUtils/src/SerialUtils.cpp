#include <Arduino.h>
#include "SerialUtils.h"

void serialRead(char (&message)[12]) {
  int pos = 0;
  bool reading = true;
  while (reading) {
    while (Serial.available() && pos < 12) {
      char c = Serial.read();
      if (c == '\n') {
        reading = false;
        break;
      }
      message[pos++] = c;
    }
    if (pos == 12) reading = false;
  }
  while (pos < 12) message[pos++] = '\0';
}