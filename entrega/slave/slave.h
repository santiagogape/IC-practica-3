#include "lora_protocol_shared.h"

/**
 * Modos de funcionamiento del esclavo.
 * SCAN_FOR_MASTER → esperando mensajes de sincronización del maestro.
 * SYNCED          → maestro detectado; responde a calibración y config.
 * STABLE          → estado estable de recepción de datos/ACK.
 */
enum SlaveMode {
  SCAN_FOR_MASTER,
  SYNCED,
  STABLE
};

/**
 * Variables globales del esclavo.
 */
extern SlaveMode slaveMode;
extern LoRaConfig_t_shared thisNodeConf_slave;
extern bool syncedWithMaster_slave;

/**
 * Inicializa el esclavo en modo SCAN_FOR_MASTER usando la configuración SAFE.
 */
void initScanForMaster_slave();

/**
 * Lógica del modo SCAN_FOR_MASTER.
 * En esta versión V3 simple, se queda escuchando con config SAFE
 * a la espera de MSG_SYNC_START_shared desde el maestro.
 */
void handleScanForMaster_slave();

/**
 * Lógica del modo SYNCED.
 * En este ejemplo se limita a delegar toda la lógica en onReceive(),
 * pero aquí se podrían añadir timeouts o lógica de transición.
 */
void handleSynced_slave();

/**
 * Lógica del modo STABLE.
 * Similarmente, toda la lógica de tratamiento de datos entra por onReceive(),
 * pero aquí podrías añadir estadísticas o supervisión.
 */
void handleStable_slave();

/**
 * Envía un mensaje de sincronización de respuesta (MSG_SYNC_REPLY_shared)
 * al maestro que haya enviado MSG_SYNC_START_shared.
 */
void sendSyncReply_slave(uint8_t masterAddress);

/**
 * Envía una respuesta de calibración (MSG_CALIBRATION_REPLY_shared) al maestro.
 * RSSI y SNR son medidos a partir del último paquete recibido.
 */
void sendCalibrationReply_slave(uint8_t masterAddress,
                                uint16_t msgId,
                                int rssi,
                                float snr);

/**
 * Intenta aplicar una nueva configuración LoRa (sf, bwIndex, cr) en el esclavo.
 * Devuelve true si los parámetros son válidos y se aplicaron, false en caso contrario.
 */
bool applyNewConfig_slave(uint8_t sf, uint8_t bwIndex, uint8_t cr);

/**
 * Callback de recepción para LoRa (debe llamarse onReceive).
 */
void onReceive(int packetSize);

/**
 * Callback de finalización de transmisión en el esclavo (onTxDone).
 */
void onTxDone();

/**
 * Envía un ACK genérico al maestro para un msgId recibido (MSG_ACK_shared).
 */
void sendACK_slave(uint8_t recipient, uint16_t msgId);


void rotateSyncConfig_slave();

