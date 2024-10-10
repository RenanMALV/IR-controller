// ---- INCLUDES ----
#include <Arduino.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <string.h>
#include <vector>
#include <ArduinoJson.h>
// ------------------------

// ---- MACROS ----
#define LED D0 // debug led
#define IR_IN 14 // IR signal input pin D5
#define IR_OUT 4 // IR signal output pin D2
#define B_RATE 115200 // Baud Rate
// ------------------------

// ---- PARAMETERS ----
const uint16_t kRecvPin = IR_IN; // RECEIVER'S GPIO

const uint16_t kIrLedPin = IR_OUT; // SENDER'S GPIO

const uint32_t kBaudRate = B_RATE; // Baud rate

const uint16_t kCaptureBufferSize = 1024;  // Buffer size

const uint8_t kTimeout = 50;  // Timeout for recognizing the end of a entire message

const uint16_t kFrequency = 38000;  // Modulation frequency of IR protocol in Hz
// ---------------------------

// ---- AUXILIARY CODE ----
// IR sender creation.
IRsend irsend(kIrLedPin);
// IR receiver creation.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);
// message object
decode_results results;
// signal's protocol
decode_type_t protocol;
// Size of message
uint16_t size;
// Signal's code in Raw array
uint16_t *raw_array;

void serialRead(char (&message)[12]) {
  int messagePos = 0;
  bool reading = true;
  while(reading){
    while(Serial.available() && messagePos < 12){
      char c = Serial.read();
      if(c == '\n'){
        reading = false;
        break;
      } 
      message[messagePos] = c;
      ++messagePos;
    }
    if(messagePos == 12) reading = false;
  }
  while(messagePos < 12){
    message[messagePos] = '\0';
    ++messagePos;
  }
}

void blink(){
  digitalWrite(LED, HIGH);// turn the LED off.(Note that LOW is the voltage level but actually 
                          //the LED is on; this is because it is acive low on the ESP8266.
  delay(1000);            // wait for 1 second.
  digitalWrite(LED, LOW); // turn the LED on.
  delay(1000); // wait for 1 second.
}

void rcvRout(){
  // try to identify signal's protocol
  protocol = results.decode_type;
  size = results.bits; // Size of message
  delete [] raw_array; // dealocate last signal code
  raw_array = resultToRawArray(&results); // convert the result in a sendable array
  size = getCorrectedRawLength(&results); // size of raw array
  Serial.println();
  Serial.println("IR signal received");
}

void sendRout(){
  // determines if the transmission was successfull
  bool success = true; 
  // verify if it's a known protocol
  if (protocol == decode_type_t::UNKNOWN) {
#if SEND_RAW
    // Send raw array via IR circuit.
    irsend.sendRaw(raw_array, size, kFrequency);
#endif  // SEND_RAW
  } else if (hasACState(protocol)) {  // Does the message require a state[]?
    // It does, so send with bytes instead.
    success = irsend.send(protocol, results.state, size / 8);
  } else {  // Anything else must be a simple message protocol. ie. <= 64 bits
    success = irsend.send(protocol, results.value, size);
  }

  Serial.printf(
      "A %d-bit %s message was %ssuccessfully transmitted.\n",
      size, typeToString(protocol).c_str(),
      success ? "" : "un");
}

void trigger(){
  char msg[12];
  serialRead(msg);
  String message("");
  u_int8_t i = 0;
  while (isAlphaNumeric(msg[i]) && i < 12){
    message.concat(msg[i]);
    ++i;
  }
  
  if(msg[0]!='\0'){
    //blink();
    Serial.println();
    Serial.println("triggered action");

    if(message.equalsIgnoreCase("read")){
      Serial.println("Ready to read signal...");
      bool received = false;
      irrecv.resume(); // Resume the input of IR signals.
      while (!received){
        // signal received?
        if(irrecv.decode(&results)){
          rcvRout();
          received = true;
        }
        yield();  // This ensures the ESP doesn't WDT reset.
      }
      irrecv.pause(); // pause the entry of new signals until a read is called
      Serial.println("Signal read successfully, waiting for next command...");
    }
    else if(message.equalsIgnoreCase("send")){
      Serial.println("Sending saved signal...");
      sendRout(); // TODO send message parameter
    }
    else if(message.equalsIgnoreCase("save")){
      Serial.println("Saving signal received... Insert action name:");
      char msg[12];
      serialRead(msg);
      String message("");
      u_int8_t i = 0;
      while (isAlphaNumeric(msg[i]) && i < 12){
        message.concat(msg[i]);
        ++i;
      }
      // Serializing the "results" structure to a json form
      // this enables the ability to save decoded signals
      JsonDocument command;
      command["name"] = message;
      command["size"] = size;
      command["protocol"] = protocol;
      command["value"] = results.value;
      
      JsonArray resState = command["state"].to<JsonArray>();
      std::vector<uint8_t> stateArr(results.state, results.state + sizeof(results.state) / sizeof(results.state[0]));
      for(auto code : stateArr)
        resState.add(code);

      JsonArray data = command["data"].to<JsonArray>();
      std::vector<uint16_t> array(raw_array, raw_array+size);
      for(auto code : array)
        data.add(code);

      serializeJson(command, Serial);
    }
  }
}

// ---------------------------

void setup() {
  irrecv.enableIRIn();  // enable input of signal
  irsend.begin();       // enable transmitter

  Serial.begin(B_RATE, SERIAL_8N1);

  pinMode(LED, OUTPUT);    // LED pin as output.
  
  while (!Serial)  // Wait for the serial connection to be establised.
    delay(50);
  Serial.println();

  Serial.println("Setup of IR controller finished");
}

void loop() {
    
  trigger();

  yield();  // This ensures the ESP doesn't WDT reset.
}