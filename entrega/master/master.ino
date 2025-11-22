#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#include "lora_protocol_shared.h"
#include "master.h"

#define TX_LAPSE_MS          10000UL
#define MAX_RETRIES          3
#define ACK_TIMEOUT_MS       500UL
#define CALIBRATION_RETRIES  2

/* Variables globales del maestro (definidas aquí, declaradas extern en master.h) */
MasterMode masterMode = SYNC;

LoRaConfig_t_shared thisNodeConf_master   = LORA_SAFE_CONFIG_shared;
LoRaConfig_t_shared remoteNodeConf_master = { 0, 0, 0, 0 };
LoRaConfig_t_shared lastGoodConfig_master = LORA_SAFE_CONFIG_shared;

int   remoteRSSI_master = -200;
float remoteSNR_master  = -200.0f;

volatile bool txDoneFlag_master         = true;
volatile bool transmitting_master       = false;
volatile bool ackReceived_master        = false;
volatile bool syncReplyReceived_master  = false;
volatile bool configConfirmReceived_master = false;

/* Prototipo local de CRC para compat con código previo (usa versión shared). */
static uint8_t calculateCRC_local(uint8_t* data, uint8_t length) {
  return calculateCRC_shared(data, length);
}

/* --------------------------------------------------------------------
 *  SETUP
 * ------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* esperar consola */ }

  Serial.println("\n=== LoRa MASTER V3 - SYNC + CALIBRATION + STABLE ===");

  if (!init_PMIC()) {
    Serial.println("Init PMIC failed!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (true) {
      delay(500);
    }
  }

  /* Config inicial SAFE (V3) */
  thisNodeConf_master   = LORA_SAFE_CONFIG_shared;
  lastGoodConfig_master = thisNodeConf_master;
  applyLoRaConfig_shared(thisNodeConf_master);

  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);
  LoRa.receive();

  initFastSync_master();

  Serial.println("LoRa MASTER init completed.\n");
}

/* --------------------------------------------------------------------
 *  LOOP
 * ------------------------------------------------------------------*/
void loop() {
  switch (masterMode) {
    case SYNC:
      handleFastSync_master();
      break;
    case CALIBRATION:
      handleCalibration_master();
      break;
    case STABLE:
      handleStable_master();
      break;
    case RECOVERY:
      handleRecovery_master();
      break;
  }
}

/* --------------------------------------------------------------------
 *  INIT SYNC
 * ------------------------------------------------------------------*/
void initFastSync_master() {
  syncReplyReceived_master   = false;
  configConfirmReceived_master = false;
  ackReceived_master         = false;

  thisNodeConf_master   = LORA_SAFE_CONFIG_shared;
  lastGoodConfig_master = thisNodeConf_master;
  applyLoRaConfig_shared(thisNodeConf_master);

  Serial.println("[MASTER] SYNC: using SAFE config (SF7, BW125kHz, CR4/5, Pwr=2dBm)");
}

/* --------------------------------------------------------------------
 *  SYNC (V3)
 * ------------------------------------------------------------------*/
void handleFastSync_master() {
  static bool initialized = false;
  static uint32_t fastSyncStart_ms = 0;
  static uint32_t lastSyncSent_ms  = 0;

  uint32_t now = millis();

  if (!initialized) {
    fastSyncStart_ms = now;
    lastSyncSent_ms  = 0;
    initialized = true;
  }

  if (syncReplyReceived_master) {
    Serial.println("[MASTER] SYNC: SYNC_REPLY recibido, pasando a CALIBRATION");
    masterMode = CALIBRATION;
    initialized = false;
    return;
  }

  if (now - lastSyncSent_ms >= SYNC_SAFE_INTERVAL_MS_shared) {
    Serial.println("[MASTER] SYNC: enviando MSG_SYNC_START_shared");
    LoRa.beginPacket();
    LoRa.write(SLAVE_ADDRESS_shared);        // destinatario
    LoRa.write(MASTER_ADDRESS_shared);       // remitente
    LoRa.write((uint8_t)MSG_SYNC_START_shared);
    LoRa.write((uint8_t)0);                  // msgId alto
    LoRa.write((uint8_t)0);                  // msgId bajo
    LoRa.write((uint8_t)0);                  // longitud (0, sin payload)
    LoRa.endPacket();
    LoRa.receive();

    lastSyncSent_ms = now;
  }

  if (now - fastSyncStart_ms > TIMEOUT_SYNC_FAST_MS_shared) {
    Serial.println("[MASTER] SYNC: timeout sin SYNC_REPLY, pasando a CALIBRATION de todas formas.");
    masterMode = CALIBRATION;
    initialized = false;
  }
}

/* --------------------------------------------------------------------
 *  CALIBRATION
 * ------------------------------------------------------------------*/
void handleCalibration_master() {
  static bool calibrationDone = false;
  if (!calibrationDone) {
    autoAdjustConfigImproved_master();
    calibrationDone = true;
    masterMode = STABLE;
    Serial.println("[MASTER] CALIBRATION: terminada, pasando a STABLE.");
  }
}

/* --------------------------------------------------------------------
 *  STABLE - envío periódico de datos
 * ------------------------------------------------------------------*/
void handleStable_master() {
  static uint32_t lastSendTime_ms = 0;
  static uint32_t txInterval_ms   = TX_LAPSE_MS;
  static uint16_t msgCount        = 0;
  static uint32_t tx_begin_ms     = 0;

  uint32_t now = millis();

  if (!transmitting_master && (now - lastSendTime_ms > txInterval_ms)) {
    uint8_t payload[8];
    uint8_t payloadLength = 0;

    uint8_t cfgBytes[2];
    encodeConfigToPayload_shared(thisNodeConf_master, cfgBytes);
    payload[payloadLength++] = cfgBytes[0];
    payload[payloadLength++] = cfgBytes[1];
    payload[payloadLength++] = (uint8_t)(-LoRa.packetRssi() * 2);
    payload[payloadLength++] = (uint8_t)(148 + LoRa.packetSnr());

    transmitting_master = true;
    txDoneFlag_master   = false;
    tx_begin_ms         = now;

    bool success = sendMessageWithRetry_master(payload, payloadLength, msgCount, (uint8_t)MSG_DATA_shared);

    if (success) {
      Serial.print("✓ [MASTER] DATA packet ");
      Serial.print(msgCount++);
      Serial.println(" sent with ACK");
    } else {
      Serial.print("✗ [MASTER] DATA packet ");
      Serial.print(msgCount);
      Serial.println(" failed after retries");
    }
  }

  if (transmitting_master && txDoneFlag_master) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    uint32_t lapse_ms  = (tx_begin_ms - lastSendTime_ms);
    lastSendTime_ms    = tx_begin_ms;

    float duty_cycle = (lapse_ms > 0) ? (100.0f * TxTime_ms / lapse_ms) : 0.0f;
    Serial.print("[MASTER] DATA TX time: ");
    Serial.print(TxTime_ms);
    Serial.print(" ms, duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %");

    if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 100;
    }

    transmitting_master = false;
    LoRa.receive();
  }
}

/* --------------------------------------------------------------------
 *  RECOVERY (esqueleto)
 * ------------------------------------------------------------------*/
void handleRecovery_master() {
  /* Por ahora no implementamos lógica avanzada de RECOVERY.
   * Podrás añadir aquí rollback a lastGoodConfig_master y
   * comprobaciones extra cuando quieras evolucionar a V4.
   */
  Serial.println("[MASTER] RECOVERY state not implemented yet.");
  masterMode = STABLE;
}

/* --------------------------------------------------------------------
 *  Envío con ACK y reintentos
 * ------------------------------------------------------------------*/
bool sendMessageWithRetry_master(uint8_t* payload,
                                 uint8_t payloadLength,
                                 uint16_t msgId,
                                 uint8_t msgType) {
  for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
    if (attempt > 0) {
      Serial.print("[MASTER] Retry ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(MAX_RETRIES - 1);
      delay(100 * attempt);
    }

    ackReceived_master = false;

    while (!LoRa.beginPacket()) {
      delay(10);
    }

    LoRa.write(SLAVE_ADDRESS_shared);
    LoRa.write(MASTER_ADDRESS_shared);
    LoRa.write(msgType);
    LoRa.write((uint8_t)(msgId >> 8));
    LoRa.write((uint8_t)(msgId & 0xFF));
    LoRa.write(payloadLength);
    if (payloadLength > 0) {
      LoRa.write(payload, (size_t)payloadLength);
      uint8_t crc = calculateCRC_local(payload, payloadLength);
      LoRa.write(crc);
    } else {
      LoRa.write((uint8_t)0); /* CRC dummy cuando no hay payload */
    }
    LoRa.endPacket();

    uint32_t ackWaitStart = millis();
    LoRa.receive();

    while ((millis() - ackWaitStart) < ACK_TIMEOUT_MS) {
      if (ackReceived_master) {
        return true;
      }
      delay(10);
    }
  }
  return false;
}

/* --------------------------------------------------------------------
 *  ACK genérico
 * ------------------------------------------------------------------*/
void sendACK_master(uint8_t recipient, uint16_t msgId) {
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(MASTER_ADDRESS_shared);
  LoRa.write((uint8_t)MSG_ACK_shared);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write((uint8_t)0); /* longitud 0, sin payload */
  LoRa.endPacket();
}

/* --------------------------------------------------------------------
 *  onReceive (callback LoRa)
 * ------------------------------------------------------------------*/
void onReceive(int packetSize) {
  if (transmitting_master && !txDoneFlag_master) {
    txDoneFlag_master = true;
  }
  if (packetSize == 0) return;

  uint8_t buffer[64];
  int recipient    = LoRa.read();
  uint8_t sender   = LoRa.read();
  uint8_t msgType  = LoRa.read();
  uint16_t msgId   = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t length   = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && receivedBytes < sizeof(buffer)) {
    buffer[receivedBytes++] = (uint8_t)LoRa.read();
  }

  if (recipient != MASTER_ADDRESS_shared && recipient != 0xFF) {
    return;
  }

  if (receivedBytes > 0) {
    uint8_t rxCRC = buffer[receivedBytes - 1];
    uint8_t calcCRC = calculateCRC_shared(buffer, receivedBytes - 1);
    if (rxCRC != calcCRC) {
      Serial.println("[MASTER] ✗ CRC mismatch, dropping packet");
      return;
    }
    receivedBytes--;
  }

  remoteRSSI_master = LoRa.packetRssi();
  remoteSNR_master  = LoRa.packetSnr();

  if (msgType == (uint8_t)MSG_SYNC_REPLY_shared) {
    Serial.println("[MASTER] ✓ MSG_SYNC_REPLY_shared recibido");
    syncReplyReceived_master = true;
    return;
  }

  if (msgType == (uint8_t)MSG_ACK_shared) {
    ackReceived_master = true;
    return;
  }

  if (msgType == (uint8_t)MSG_CALIBRATION_REPLY_shared && receivedBytes >= 2) {
    int encRSSI = (int)buffer[0];
    int encSNR  = (int)buffer[1];
    remoteRSSI_master = -encRSSI / 2;
    remoteSNR_master  = encSNR - 148;
    return;
  }

  if (msgType == (uint8_t)MSG_CONFIG_CONFIRM_shared) {
    Serial.println("[MASTER] ✓ MSG_CONFIG_CONFIRM_shared recibido");
    configConfirmReceived_master = true;
    return;
  }

  /* Para MSG_DATA_shared recibido desde el esclavo (opcional) podrías hacer debug aquí */
}

/* --------------------------------------------------------------------
 *  TxFinished (callback LoRa)
 * ------------------------------------------------------------------*/
void TxFinished() {
  txDoneFlag_master = true;
}

/* --------------------------------------------------------------------
 *  Auto-ajuste (versión simplificada basada en tu código)
 * ------------------------------------------------------------------*/
void autoAdjustConfigImproved_master() {
  const float MIN_SNR  = -12.0f;
  const int   MIN_RSSI = -118;

  Serial.println("\n[MASTER] AUTO-ADJUST: explorando configuraciones...");

  struct Result {
    uint8_t sf;
    uint8_t bw;
    uint8_t cr;
    uint32_t txTime;
    float snr;
    int rssi;
    float reliability;
    bool valid;
  };

  Result best = {0, 0, 0, 0xFFFFFFFFUL, -200.0f, -200, 0.0f, false};

  uint8_t testPayload[4] = {0xAA, 0x55, 0, 0};

  for (int bw = 9; bw >= 6; bw--) {
    for (int sf = 7; sf <= 12; sf++) {
      for (int cr = 5; cr <= 8; cr++) {

        Serial.print("[MASTER] Test SF=");
        Serial.print(sf);
        Serial.print(" BW=");
        Serial.print((int)bandwidth_kHz_shared[bw]);
        Serial.print("kHz CR=4/");
        Serial.println(cr);

        LoRa.setSpreadingFactor(sf);
        LoRa.setSignalBandwidth((long)bandwidth_kHz_shared[bw]);
        LoRa.setCodingRate4(cr);
        delay(10);

        int successes = 0;
        float sumSNR = 0.0f;
        int   sumRSSI = 0;
        uint32_t sumTime = 0;

        for (int trial = 0; trial < CALIBRATION_RETRIES; trial++) {
          remoteSNR_master  = -200.0f;
          remoteRSSI_master = -200;

          testPayload[2] = (uint8_t)sf;
          testPayload[3] = (uint8_t)bw;

          uint32_t t0 = millis();
          LoRa.beginPacket();
          LoRa.write(SLAVE_ADDRESS_shared);
          LoRa.write(MASTER_ADDRESS_shared);
          LoRa.write((uint8_t)MSG_CALIBRATION_TEST_shared);
          LoRa.write((uint8_t)0);
          LoRa.write((uint8_t)0);
          LoRa.write((uint8_t)4);
          LoRa.write(testPayload, 4);
          LoRa.write(calculateCRC_shared(testPayload, 4));
          LoRa.endPacket();

          LoRa.receive();
          delay(400);

          uint32_t txTime = millis() - t0;

          if (remoteSNR_master > -150.0f &&
              remoteSNR_master >= MIN_SNR &&
              remoteRSSI_master >= MIN_RSSI) {
            successes++;
            sumSNR  += remoteSNR_master;
            sumRSSI += remoteRSSI_master;
            sumTime += txTime;
          }
        }

        if (successes == 0) {
          continue;
        }

        float reliability = 100.0f * successes / CALIBRATION_RETRIES;
        float avgSNR  = sumSNR / successes;
        int   avgRSSI = sumRSSI / successes;
        uint32_t avgTime = sumTime / (uint32_t)successes;

        if (reliability >= 100.0f && avgTime < best.txTime) {
          best.sf   = (uint8_t)sf;
          best.bw   = (uint8_t)bw;
          best.cr   = (uint8_t)cr;
          best.txTime = avgTime;
          best.snr    = avgSNR;
          best.rssi   = avgRSSI;
          best.reliability = reliability;
          best.valid  = true;
        }
      }
    }
  }

  if (!best.valid) {
    Serial.println("[MASTER] AUTO-ADJUST: no se encontró config válida, manteniendo SAFE.");
    thisNodeConf_master = LORA_SAFE_CONFIG_shared;
    applyLoRaConfig_shared(thisNodeConf_master);
    return;
  }

  thisNodeConf_master.spreadingFactor = best.sf;
  thisNodeConf_master.bandwidth_index = best.bw;
  thisNodeConf_master.codingRate      = best.cr;
  applyLoRaConfig_shared(thisNodeConf_master);
  lastGoodConfig_master = thisNodeConf_master;

  uint8_t finalPayload[3] = {best.sf, best.bw, best.cr};
  sendMessageWithRetry_master(finalPayload, 3, 999, (uint8_t)MSG_CONFIG_FINAL_shared);

  Serial.println("[MASTER] AUTO-ADJUST: configuración óptima aplicada.");
}
