#ifndef IR_IO_H
#define IR_IO_H

#include <IRrecv.h>
#include <IRsend.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>

extern IRsend irsend;
extern IRrecv irrecv;
extern decode_results results;
extern uint16_t* raw_array;
extern decode_type_t protocol;
extern uint16_t size;

void setupIR();
void receiveIR();
void sendIR();

#endif