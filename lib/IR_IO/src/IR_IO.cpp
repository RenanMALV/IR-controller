#include "ir_io.h"

IRsend irsend(4);  // D2
IRrecv irrecv(14, 2048, 244, false);  // D5
decode_results results;
decode_type_t protocol;
uint16_t size;
uint16_t* raw_array = nullptr;

void setupIR() {
  irrecv.enableIRIn();
  irsend.begin();
}

void receiveIR() {
  protocol = results.decode_type;
  size = results.bits;
  delete[] raw_array;
  raw_array = resultToRawArray(&results);
  size = getCorrectedRawLength(&results);
}

void sendIR() {
  if (protocol == decode_type_t::UNKNOWN) return;
  if (raw_array) {
    irsend.sendRaw(raw_array, size, 38000);
  } else if (hasACState(protocol)) {
    irsend.send(protocol, results.state, size / 8);
  } else {
    irsend.send(protocol, results.value, size);
  }
}