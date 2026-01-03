#pragma once
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>

// --- API do módulo IR_IO ---
// Inicializa receptor e transmissor
void IR_setup();

// Controle de recepção
void IR_resume();                 // habilita o receptor para capturar o próximo frame
void IR_pause();                  // pausa o receptor (não armazena novos frames)
bool IR_poll();                   // tenta decodificar; retorna true quando algo foi lido

// Envio (usa o último frame recebido/definido via setters abaixo)
bool IR_send();                   // envia o frame atual via IR (RAW / AC state / valor simples)

// Acesso aos dados do último frame decodificado
decode_type_t IR_protocol();
uint16_t      IR_bits();
const uint16_t* IR_raw();
size_t        IR_raw_len();

// Acesso direto ao decode_results (somente leitura)
const decode_results& IR_results();

// Setters opcionais (caso você queira carregar um frame salvo do JSON e depois enviar)
void IR_setProtocol(decode_type_t proto);
void IR_setBits(uint16_t bits);
void IR_setValue(uint64_t value);           // configura results.value
void IR_setState(const uint8_t* state, size_t len); // copia para results.state[]
void IR_setRaw(const uint16_t* data, size_t len);   // copia RAW (substitui o atual)