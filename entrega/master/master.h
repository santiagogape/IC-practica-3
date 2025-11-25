#include "lora_protocol_shared.h"

/**
 * Modos de funcionamiento del maestro.
 * SYNC   → fase inicial V3 con configuración SAFE.
 * CALIBRATION → exploración de parámetros LoRa (auto-ajuste).
 * STABLE      → envío periódico de datos con la configuración escogida.
 * RECOVERY    → rollback si una nueva configuración rompe la comunicación (esqueleto).
 */
enum MasterMode {
  SYNC,
  CALIBRATION,
  STABLE,
  RECOVERY
};

/**
 * Variables globales del maestro (declaradas como extern; definidas en master.ino).
 */
extern MasterMode masterMode;
extern LoRaConfig_t_shared thisNodeConf_master;
extern LoRaConfig_t_shared remoteNodeConf_master;
extern LoRaConfig_t_shared lastGoodConfig_master;
extern int   remoteRSSI_master;
extern float remoteSNR_master;

extern volatile bool txDoneFlag_master;
extern volatile bool transmitting_master;
extern volatile bool ackReceived_master;
extern volatile bool syncReplyReceived_master;
extern volatile bool configConfirmReceived_master;

/**
 * Inicializa la fase SYNC:
 * - Aplica la configuración SAFE.
 * - Resetea flags de sincronización.
 */
void initFastSync_master();

/**
 * Lógica de la fase SYNC (se llama desde loop() mientras masterMode == SYNC).
 * Envía periódicamente MSG_SYNC_START_shared y espera MSG_SYNC_REPLY_shared
 * durante TIMEOUT_SYNC_FAST_MS_shared.
 */
void handleFastSync_master();

/**
 * Lógica de la fase CALIBRATION.
 * Invoca el algoritmo de auto-ajuste usando paquetes MSG_CALIBRATION_TEST_shared /
 * MSG_CALIBRATION_REPLY_shared y finalmente decide una configuración óptima.
 */
void handleCalibration_master();

/**
 * Lógica del estado STABLE.
 * Envía periódicamente MSG_DATA_shared al esclavo usando la configuración actual.
 */
void handleStable_master();

/**
 * Lógica del estado RECOVERY (esqueleto).
 * Permite en el futuro implementar rollback si una configuración rompe el enlace.
 */
void handleRecovery_master();

/**
 * Envío de un mensaje con payload y tipo dado, gestionando reintentos y espera de ACK.
 * Devuelve true si se recibió ACK dentro del timeout, false en caso contrario.
 */
bool sendMessageWithRetry_master(uint8_t* payload,
                                 uint8_t payloadLength,
                                 uint16_t msgId,
                                 uint8_t msgType);

/**
 * Envía un ACK genérico al remitente indicado para el msgId recibido.
 */
void sendACK_master(uint8_t recipient, uint16_t msgId);

/**
 * ISR de recepción para la librería LoRa (callback registrado en setup()).
 * Debe llamarse exactamente "onReceive" para que LoRa.h lo invoque.
 */
void onReceive(int packetSize);

/**
 * ISR de finalización de transmisión para la librería LoRa.
 * Debe llamarse exactamente "onTxDone" (o TxFinished si se registra así).
 */
void TxFinished();

/**
 * Algoritmo de auto-ajuste de configuración LoRa en el maestro.
 * Basado en tu función autoAdjustConfigImproved original, adaptada a los nuevos
 * tipos de mensaje compartidos.
 */
void autoAdjustConfigImproved_master();
