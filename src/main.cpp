#include <Arduino.h>

#include "SerialUtils.h"
#include "IR_IO.h"
#include "CommandHandler.h"

#define LED D0
#define B_RATE 115200

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(B_RATE);
  while (!Serial) delay(50);

  setupIR();

  Serial.println("Setup completed.");
}

void loop() {
  handleCommand();
  yield();
}