#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#include "slave.h"

/* Variables globales del esclavo (definidas aquí, declaradas extern en slave.h) */
SlaveMode slaveMode = SCAN_FOR_MASTER;

LoRaConfig_t_shared thisNodeConf_slave = LORA_SAFE_CONFIG_shared;
bool syncedWithMaster_slave = false;

struct Stats_slave {
  uint32_t packetsReceived;
  uint32_t packetsSent;
  uint32_t crcErrors;
  uint32_t syncMessages;
  uint32_t calibrationRequests;
} stats_slave = {0, 0, 0, 0, 0};

/* Fase interna de sincronización en el esclavo */
enum SyncPhase_slave_t {
  SYNC_PHASE_SAFE_slave = 0,
  SYNC_PHASE_LONG_slave = 1
};

static SyncPhase_slave_t syncPhase_slave = SYNC_PHASE_SAFE_slave;
static uint32_t syncPhaseStart_ms_slave  = 0;

/* --------------------------------------------------------------------
 *  SETUP
 * ------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* esperar consola */ }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("\n=== LoRa SLAVE V3 - SYNC SAFE + LONGRANGE + CALIBRATION ===");

  if (!init_PMIC()) {
    Serial.println("Init PMIC failed!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  thisNodeConf_slave = LORA_SAFE_CONFIG_shared;
  applyLoRaConfig_shared(thisNodeConf_slave);

  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();

  initScanForMaster_slave();

  Serial.println("LoRa SLAVE init completed.\n");
}

/* --------------------------------------------------------------------
 *  LOOP
 * ------------------------------------------------------------------*/
void loop() {
  switch (slaveMode) {
    case SCAN_FOR_MASTER:
      handleScanForMaster_slave();
      break;
    case SYNCED:
      handleSynced_slave();
      break;
    case STABLE:
      handleStable_slave();
      break;
  }

  delay(50);
}

/* --------------------------------------------------------------------
 *  INIT SCAN_FOR_MASTER
 * ------------------------------------------------------------------*/
void initScanForMaster_slave() {
  syncedWithMaster_slave = false;
  slaveMode              = SCAN_FOR_MASTER;

  syncPhase_slave        = SYNC_PHASE_SAFE_slave;
  syncPhaseStart_ms_slave = millis();

  thisNodeConf_slave = LORA_SAFE_CONFIG_shared;
  applyLoRaConfig_shared(thisNodeConf_slave);

  Serial.println("[SLAVE] SCAN_FOR_MASTER: SAFE phase (10 s) con LORA_SAFE_CONFIG_shared.");
}

/* --------------------------------------------------------------------
 *  SCAN_FOR_MASTER (dos fases: SAFE → LONGRANGE)
 * ------------------------------------------------------------------*/
void handleScanForMaster_slave() {
  uint32_t now = millis();

  // parpadeo lento para indicar búsqueda
  digitalWrite(LED_BUILTIN, (now / 500) % 2);

  // si ya se sincronizó por SYNC_START o por CALIBRATION, pasar a SYNCED
  if (syncedWithMaster_slave) {
    slaveMode = SYNCED;
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }

  if (syncPhase_slave == SYNC_PHASE_SAFE_slave) {
    if (now - syncPhaseStart_ms_slave > TIMEOUT_SYNC_SAFE_MS_shared) {
      // pasar a LONGRANGE
      syncPhase_slave        = SYNC_PHASE_LONG_slave;
      syncPhaseStart_ms_slave = now;

      thisNodeConf_slave = LORA_LONGRANGE_CONFIG_shared;
      applyLoRaConfig_shared(thisNodeConf_slave);

      Serial.println("[SLAVE] SCAN: timeout SAFE, cambiando a fase LONGRANGE (10 s).");
    }
  } else { // SYNC_PHASE_LONG_slave
    if (now - syncPhaseStart_ms_slave > TIMEOUT_SYNC_LONG_MS_shared) {
      // no se logró SYNC ni con SAFE ni con LONGRANGE
      Serial.println("[SLAVE] SCAN: timeout LONGRANGE, esperando que maestro entre en CALIBRATION.");
      // Nos quedamos con config LONGRANGE, pero permitimos mensajes de CALIBRATION.
      // No cambiamos slaveMode aún; onReceive manejará las fases siguientes.
    }
  }
}

/* --------------------------------------------------------------------
 *  SYNCED
 * ------------------------------------------------------------------*/
void handleSynced_slave() {
  digitalWrite(LED_BUILTIN, HIGH);
  // La lógica de calibración y configuración final sucede en onReceive.
}

/* --------------------------------------------------------------------
 *  STABLE
 * ------------------------------------------------------------------*/
void handleStable_slave() {
  digitalWrite(LED_BUILTIN, HIGH);
  // En esta versión, la recepción de DATA/ACK se maneja en onReceive.
}

/* --------------------------------------------------------------------
 *  sendSyncReply_slave
 * ------------------------------------------------------------------*/
void sendSyncReply_slave(uint8_t masterAddress) {
  LoRa.beginPacket();
  LoRa.write(masterAddress);
  LoRa.write(SLAVE_ADDRESS_shared);
  LoRa.write((uint8_t)MSG_SYNC_REPLY_shared);
  LoRa.write((uint8_t)0);
  LoRa.write((uint8_t)0);
  LoRa.write((uint8_t)0); /* longitud 0, sin payload */
  LoRa.endPacket();

  stats_slave.packetsSent++;

  Serial.println("[SLAVE] MSG_SYNC_REPLY_shared enviado.");
}

/* --------------------------------------------------------------------
 *  sendCalibrationReply_slave
 * ------------------------------------------------------------------*/
void sendCalibrationReply_slave(uint8_t masterAddress,
                                uint16_t msgId,
                                int rssi,
                                float snr) {
  uint8_t payload[2];
  payload[0] = (uint8_t)(-rssi * 2);       /* mismo encoding que en maestro */
  payload[1] = (uint8_t)(148 + snr);

  LoRa.beginPacket();
  LoRa.write(masterAddress);
  LoRa.write(SLAVE_ADDRESS_shared);
  LoRa.write((uint8_t)MSG_CALIBRATION_REPLY_shared);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write((uint8_t)2);                  /* longitud payload */
  LoRa.write(payload, 2);
  LoRa.write(calculateCRC_shared(payload, 2));
  LoRa.endPacket();

  stats_slave.packetsSent++;

  Serial.print("[SLAVE] CALIB_REPLY: RSSI=");
  Serial.print(rssi);
  Serial.print(" dBm, SNR=");
  Serial.print(snr, 1);
  Serial.println(" dB");
}

/* --------------------------------------------------------------------
 *  applyNewConfig_slave
 * ------------------------------------------------------------------*/
bool applyNewConfig_slave(uint8_t sf, uint8_t bwIndex, uint8_t cr) {
  if (sf < 6 || sf > 12) {
    Serial.println("[SLAVE] Invalid SF");
    return false;
  }
  if (bwIndex > 9) {
    Serial.println("[SLAVE] Invalid BW index");
    return false;
  }
  if (cr < 5 || cr > 8) {
    Serial.println("[SLAVE] Invalid CR");
    return false;
  }

  thisNodeConf_slave.spreadingFactor = sf;
  thisNodeConf_slave.bandwidth_index = bwIndex;
  thisNodeConf_slave.codingRate      = cr;
  applyLoRaConfig_shared(thisNodeConf_slave);
  delay(10);

  Serial.print("[SLAVE] New config applied: SF=");
  Serial.print(sf);
  Serial.print(" BW=");
  Serial.print((int)bandwidth_kHz_shared[bwIndex]);
  Serial.print("kHz CR=4/");
  Serial.println(cr);

  return true;
}

/* --------------------------------------------------------------------
 *  sendACK_slave
 * ------------------------------------------------------------------*/
void sendACK_slave(uint8_t recipient, uint16_t msgId) {
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(SLAVE_ADDRESS_shared);
  LoRa.write((uint8_t)MSG_ACK_shared);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write((uint8_t)0); /* longitud 0, sin payload */
  LoRa.endPacket();

  stats_slave.packetsSent++;
}

/* --------------------------------------------------------------------
 *  onReceive (callback LoRa)
 * ------------------------------------------------------------------*/
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  stats_slave.packetsReceived++;

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

  if (recipient != SLAVE_ADDRESS_shared && recipient != 0xFF) {
    return;
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (receivedBytes > 0) {
    uint8_t rxCRC   = buffer[receivedBytes - 1];
    uint8_t calcCRC = calculateCRC_shared(buffer, receivedBytes - 1);
    if (rxCRC != calcCRC) {
      stats_slave.crcErrors++;
      Serial.println("[SLAVE] ✗ CRC ERROR");
      return;
    }
    receivedBytes--;
  }

  /* ----------- SYNC ------------ */
  if (msgType == (uint8_t)MSG_SYNC_START_shared && !syncedWithMaster_slave) {
    stats_slave.syncMessages++;
    syncedWithMaster_slave = true;
    slaveMode = SYNCED;

    Serial.print("[SLAVE] SYNC from master 0x");
    Serial.print(sender, HEX);
    Serial.print(" RSSI=");
    Serial.print(rssi);
    Serial.print(" dBm SNR=");
    Serial.print(snr, 1);
    Serial.println(" dB");

    sendSyncReply_slave(sender);
    LoRa.receive();
    return;
  }

  /* Si aún no está sincronizado, solo aceptamos CALIBRATION_TEST como fallback */
  if (!syncedWithMaster_slave && msgType != (uint8_t)MSG_CALIBRATION_TEST_shared) {
    return;
  }

  /* -------- CALIBRATION TEST -------- */
  if (msgType == (uint8_t)MSG_CALIBRATION_TEST_shared) {
    stats_slave.calibrationRequests++;

    // Si aún no está sincronizado, consideramos que a partir de ahora hay comunicación
    if (!syncedWithMaster_slave) {
      syncedWithMaster_slave = true;
      slaveMode = SYNCED;
      Serial.println("[SLAVE] Fallback: sincronizado con maestro vía CALIBRATION_TEST.");
    }

    if (receivedBytes >= 4) {
      uint8_t testSF = buffer[2];
      uint8_t testBW = buffer[3];

      thisNodeConf_slave.spreadingFactor = testSF;
      thisNodeConf_slave.bandwidth_index = testBW;
      applyLoRaConfig_shared(thisNodeConf_slave);
      delay(10);

      Serial.print("[SLAVE] CALIB_TEST: adapting to SF=");
      Serial.print(testSF);
      Serial.print(" BW idx=");
      Serial.println(testBW);
    }

    sendCalibrationReply_slave(sender, msgId, rssi, snr);
    LoRa.receive();
    return;
  }

  /* -------- CONFIG FINAL -------- */
  if (msgType == (uint8_t)MSG_CONFIG_FINAL_shared) {
    if (receivedBytes >= 3) {
      uint8_t sf = buffer[0];
      uint8_t bw = buffer[1];
      uint8_t cr = buffer[2];

      if (applyNewConfig_slave(sf, bw, cr)) {
        sendACK_slave(sender, msgId);

        /* Confirmación explícita al maestro */
        LoRa.beginPacket();
        LoRa.write(sender);
        LoRa.write(SLAVE_ADDRESS_shared);
        LoRa.write((uint8_t)MSG_CONFIG_CONFIRM_shared);
        LoRa.write((uint8_t)(msgId >> 8));
        LoRa.write((uint8_t)(msgId & 0xFF));
        LoRa.write((uint8_t)0);
        LoRa.endPacket();

        slaveMode = STABLE;
      }
    }
    LoRa.receive();
    return;
  }

  /* -------- DATA -------- */
  if (msgType == (uint8_t)MSG_DATA_shared) {
    sendACK_slave(sender, msgId);

    Serial.print("[SLAVE] DATA received, RSSI=");
    Serial.print(rssi);
    Serial.print(" dBm, SNR=");
    Serial.print(snr, 1);
    Serial.println(" dB");

    if (receivedBytes >= 2) {
      LoRaConfig_t_shared remoteCfg;
      decodeConfigFromPayload_shared(buffer, remoteCfg);
      int remoteRSSI = 0;
      float remoteSNR = 0.0f;
      if (receivedBytes >= 4) {
        remoteRSSI = -(int(buffer[2])) / 2;
        remoteSNR  = float(int(buffer[3]) - 148);
      }

      Serial.print("[SLAVE] Master cfg: BW=");
      Serial.print((int)bandwidth_kHz_shared[remoteCfg.bandwidth_index]);
      Serial.print("kHz SF=");
      Serial.print(remoteCfg.spreadingFactor);
      Serial.print(" CR=4/");
      Serial.print(remoteCfg.codingRate);
      Serial.print(" TxPwr=");
      Serial.print(remoteCfg.txPower);
      Serial.println(" dBm");

      Serial.print("[SLAVE] Master RSSI=");
      Serial.print(remoteRSSI);
      Serial.print(" dBm, SNR=");
      Serial.print(remoteSNR, 1);
      Serial.println(" dB");
    }

    LoRa.receive();
    return;
  }

  /* -------- ACK -------- */
  if (msgType == (uint8_t)MSG_ACK_shared) {
    LoRa.receive();
    return;
  }
}

/* --------------------------------------------------------------------
 *  onTxDone (callback LoRa)
 * ------------------------------------------------------------------*/
void onTxDone() {
  stats_slave.packetsSent++;
}
