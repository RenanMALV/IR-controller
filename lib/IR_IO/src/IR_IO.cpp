#include "IR_IO.h"

// --- Parâmetros físicos ---
static const uint16_t kRecvPin            = 4;    // D2
static const uint16_t kIrLedPin           = 5;     // D1
static const uint16_t kCaptureBufferSize  = 2048;
static const uint8_t  kTimeout            = 244;
static const uint16_t kFrequency          = 38000;

// --- Objetos globais do módulo (escopo deste .cpp) ---
static IRsend irsend(kIrLedPin);
static IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);

// Armazena o último resultado decodificado
static decode_results g_results;

// Cache auxiliares
static decode_type_t  g_protocol = decode_type_t::UNKNOWN;
static uint16_t       g_bits     = 0;
static uint16_t*      g_raw      = nullptr;
static size_t         g_raw_len  = 0;

// --- Helpers internos ---
static void free_raw() {
  delete[] g_raw;
  g_raw = nullptr;
  g_raw_len = 0;
}

// --- API ---
void IR_setup() {
  irrecv.enableIRIn();
  irsend.begin();
}

void IR_resume() {
  irrecv.resume();
}

void IR_pause() {
  irrecv.pause();
}

bool IR_poll() {
  // Tenta decodificar; se conseguir, atualiza caches e retorna true
  if (!irrecv.decode(&g_results)) return false;

  // Atualiza metadados
  g_protocol = g_results.decode_type;
  g_bits     = g_results.bits;

  // Converte para RAW "corrigido" (o mais seguro para replay universal)
  free_raw();
  uint16_t* tmp = resultToRawArray(&g_results);
  if (tmp) {
    g_raw     = tmp;
    g_raw_len = getCorrectedRawLength(&g_results);
  }
  else {
    g_raw     = nullptr;
    g_raw_len = 0;
    return false;
  }

  return true;
}

bool IR_send() {
  if (g_protocol == decode_type_t::UNKNOWN && !g_raw) {
    // nada válido para enviar
    return false;
  }

  if (g_raw && g_raw_len > 0) {
    // Envia RAW preferencialmente se disponível
    irsend.sendRaw(g_raw, g_raw_len, kFrequency);
  } else if (hasACState(g_protocol)) {
    // Protocolos de ar-condicionado: usar state[] em bytes
    irsend.send(g_protocol, g_results.state, g_bits / 8);
  } else {
    // Protocolos "simples": value + tamanho em bits
    irsend.send(g_protocol, g_results.value, g_bits);
  }

  return true;
}

// --- Getters ---
decode_type_t IR_protocol() {
  return g_protocol;
}

uint16_t IR_bits() {
  return g_bits;
}

const uint16_t* IR_raw() {
  return g_raw;
}

size_t IR_raw_len() {
  return g_raw_len;
}

const decode_results& IR_results() {
  return g_results;
}

// --- Setters (para carregar de JSON/salvo) ---
void IR_setProtocol(decode_type_t proto) {
  g_protocol = proto;
}

void IR_setBits(uint16_t bits) {
  g_bits = bits;
}

void IR_setValue(uint64_t value) {
  g_results.value = value;
}

void IR_setState(const uint8_t* state, size_t len) {
  // g_results.state tem tamanho fixo em IRremoteESP8266; copiamos até o limite
  size_t maxlen = sizeof(g_results.state) / sizeof(g_results.state[0]);
  if (len > maxlen) len = maxlen;
  for (size_t i = 0; i < len; i++) g_results.state[i] = state[i];
}

void IR_setRaw(const uint16_t* data, size_t len) {
  free_raw();
  if (!data || len == 0) return;
  g_raw = new uint16_t[len];
  for (size_t i = 0; i < len; i++) g_raw[i] = data[i];
  g_raw_len = len;
}
