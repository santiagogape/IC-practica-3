#ifndef LORA_PROTOCOL_SHARED_H
#define LORA_PROTOCOL_SHARED_H

#include <LoRa.h>

/**
 * Dirección lógica del maestro (MKR #1)
 */
static const uint8_t MASTER_ADDRESS_shared = 0x06;

/**
 * Dirección lógica del esclavo (MKR #2)
 */
static const uint8_t SLAVE_ADDRESS_shared  = 0x05;

/**
 * Timeouts generales del protocolo (no crítico → 10 segundos).
 */
static const uint32_t TIMEOUT_SYNC_FAST_MS_shared   = 10000UL; // FAST_SYNC (V3)
static const uint32_t TIMEOUT_CALIBRATION_MS_shared = 10000UL; // esperar respuesta calibración
static const uint32_t TIMEOUT_CONFIG_MS_shared      = 10000UL; // esperar confirm config final
static const uint32_t TIMEOUT_RECOVERY_MS_shared    = 10000UL; // fase de recuperación
/** Tiempo máximo para intentar sincronizar usando SAFE (corta distancia): 10 s */
const uint32_t TIMEOUT_SYNC_SAFE_MS_shared = 10000UL;

/** Tiempo máximo para intentar sincronizar usando LONGRANGE (larga distancia): 10 s */
const uint32_t TIMEOUT_SYNC_LONG_MS_shared = 10000UL;

/** Intervalo entre envíos de mensajes de SYNC (SAFE y LONGRANGE) */
const uint32_t SYNC_SAFE_INTERVAL_MS_shared = 1000UL;



/**
 * Tabla de anchos de banda en Hz, indexados de 0 a 9.
 * Debe ser idéntica en maestro y esclavo.
 */
static const double bandwidth_kHz_shared[10] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

/**
 * Estructura de configuración LoRa compartida entre maestro y esclavo.
 * Representa los parámetros mínimos necesarios para configurar el transceiver.
 */
typedef struct {
  uint8_t bandwidth_index;   /* índice en bandwidth_kHz_shared (0..9) */
  uint8_t spreadingFactor;   /* SF [6..12]                       */
  uint8_t codingRate;        /* CR 4/x → almacenar x [5..8]     */
  uint8_t txPower;           /* Potencia en dBm [2..20] aprox.  */
} LoRaConfig_t_shared;

/**
 * Configuración SAFE para corta distancia (V3).
 * - Bw 125 kHz
 * - SF7
 * - CR 4/5
 * - TxPower 2 dBm → minimiza riesgo de saturación a pocos metros.
 */
static const LoRaConfig_t_shared LORA_SAFE_CONFIG_shared = {
  7,  // BW index -> 125 kHz
  7,  // SF7
  5,  // CR = 4/5
  2   // TxPower = 2 dBm
};


/** Configuración "LONGRANGE" para larga distancia con ruido.
 *  SF=12, BW=125 kHz, CR=4/8, TxPower=14 dBm
 */
const LoRaConfig_t_shared LORA_LONGRANGE_CONFIG_shared = {
  7,   // bandwidth_index → 125 kHz
  12,  // spreadingFactor
  8,   // codingRate (4/8)
  14   // txPower (dBm)
};


/**
 * Tipos de mensaje del protocolo compartido (V3).
 * Deben coincidir exactamente en maestro y esclavo.
 */
typedef enum : uint8_t {
  /* Sincronización inicial */
  MSG_SYNC_START_shared    = 0x00,  /* maestro → esclavo, sin payload */
  MSG_SYNC_REPLY_shared    = 0x01,  /* esclavo → maestro, sin payload */

  /* Calibración / búsqueda de configuración óptima */
  MSG_CALIBRATION_TEST_shared   = 0x10, /* maestro → esclavo */
  MSG_CALIBRATION_REPLY_shared  = 0x11, /* esclavo → maestro */

  /* Configuración final escogida por el maestro */
  MSG_CONFIG_FINAL_shared   = 0x12, /* maestro → esclavo */
  MSG_CONFIG_CONFIRM_shared = 0x13, /* esclavo → maestro */

  /* Datos periódicos + ACK genérico de aplicación */
  MSG_DATA_shared = 0x20,   /* maestro → esclavo */
  MSG_ACK_shared  = 0x21    /* esclavo → maestro (o genérico) */

} MessageType_shared;

/**
 * Calcula un CRC muy simple (XOR) sobre un buffer de bytes.
 * Se usa tanto para payloads de datos como para mensajes de control.
 */
inline uint8_t calculateCRC_shared(const uint8_t* data, uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
  }
  return crc;
}

/**
 * Aplica una configuración LoRa (LoRaConfig_t_shared) al transceiver SX127x.
 * Incluye una protección básica de rangos para evitar valores fuera de lo permitido.
 */
inline void applyLoRaConfig_shared(const LoRaConfig_t_shared& cfg) {
  uint8_t bwIndex = cfg.bandwidth_index;
  if (bwIndex > 9) bwIndex = 9;

  uint8_t sf = cfg.spreadingFactor;
  if (sf < 6)  sf = 6;
  if (sf > 12) sf = 12;

  uint8_t cr = cfg.codingRate;
  if (cr < 5) cr = 5;
  if (cr > 8) cr = 8;

  uint8_t pwr = cfg.txPower;
  if (pwr < 2)  pwr = 2;
  if (pwr > 20) pwr = 20;

  LoRa.setSignalBandwidth((long)bandwidth_kHz_shared[bwIndex]);
  LoRa.setSpreadingFactor(sf);
  LoRa.setCodingRate4(cr);
  LoRa.setTxPower(pwr, PA_OUTPUT_PA_BOOST_PIN);
}

/**
 * Codifica una configuración LoRa en 2 bytes, usando el mismo esquema
 * que tus ejemplos previos para empaquetar BW, SF, CR y TxPower.
 */
inline void encodeConfigToPayload_shared(const LoRaConfig_t_shared& cfg, uint8_t* out2bytes) {
  out2bytes[0] = (cfg.bandwidth_index << 4) | ((cfg.spreadingFactor - 6) << 1);
  out2bytes[1] = ((cfg.codingRate - 5) << 6) | ((cfg.txPower - 2) << 1);
}

/**
 * Decodifica una configuración LoRa desde 2 bytes generados por encodeConfigToPayload_shared.
 */
inline void decodeConfigFromPayload_shared(const uint8_t* in2bytes, LoRaConfig_t_shared& cfg) {
  cfg.bandwidth_index = in2bytes[0] >> 4;
  cfg.spreadingFactor = 6 + ((in2bytes[0] & 0x0F) >> 1);
  cfg.codingRate      = 5 + (in2bytes[1] >> 6);
  cfg.txPower         = 2 + ((in2bytes[1] & 0x3F) >> 1);
}

#endif
