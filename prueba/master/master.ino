#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

// Direcciones
const uint8_t LOCAL_ADDR = 0x06;
const uint8_t SLAVE_ADDR = 0x05;
const uint8_t SYNC_WORD  = 0x12;

// Config LoRa base
const long   LORA_FREQ = 868E6;
const long   BW_TABLE[] = {125000, 250000, 500000, 62500, 41700, 31250};
const uint8_t BW_COUNT = sizeof(BW_TABLE) / sizeof(BW_TABLE[0]);

// Tipos de mensaje
const uint8_t CMD_TEST_TX   = 0x01;
const uint8_t CMD_ACK_TX    = 0x02;
const uint8_t CMD_TEST_CFG  = 0x03;
const uint8_t CMD_ACK_CFG   = 0x04;
const uint8_t CMD_SET_FINAL = 0x05;
const uint8_t CMD_ACK_FINAL = 0x06;
const uint8_t CMD_DATA      = 0x10;
const uint8_t CMD_ACK_DATA  = 0x11;

// Barridos a probar
const uint8_t TX_LIST[] = {2,3,4,5,6,7,8,9,10};
const uint8_t TX_COUNT  = sizeof(TX_LIST) / sizeof(TX_LIST[0]);
const uint8_t SF_LIST[] = {7,8,9,10,11,12};
const uint8_t SF_COUNT  = sizeof(SF_LIST) / sizeof(SF_LIST[0]);

// Resultados de pruebas TX
struct TxResult {
  uint8_t tx;
  int     rssi;
  float   snr;
};
TxResult txResults[TX_COUNT];
uint8_t txResCount = 0;

// Resultados de pruebas BW/SF
struct CfgResult {
  uint8_t bwIndex;
  uint8_t sf;
  int     rssi;
  float   snr;
};
CfgResult cfgResults[BW_COUNT * SF_COUNT];
uint8_t cfgResCount = 0;

// Estado maestro
enum Phase {
  PHASE_TX_SYNC,
  PHASE_CFG_SYNC,
  PHASE_DONE
};
Phase phase = PHASE_TX_SYNC;

uint8_t msgIdCounter = 0;
bool awaitingAck = false;
uint8_t awaitingType = 0;
uint8_t awaitingId   = 0;

bool ackDataReceived = false;

// Mejor config encontrada
uint8_t bestTX       = 5;
uint8_t bestBWIndex  = 0;
uint8_t bestSF       = 7;

// Prototipos
void onReceive(int packetSize);
void applyRadio(long bw, uint8_t sf, uint8_t tx);
void sendPacket(uint8_t dest, uint8_t type, uint8_t id, const uint8_t* payload, uint8_t len);
bool sendAndWaitAck(uint8_t dest, uint8_t type, const uint8_t* payload, uint8_t len, uint32_t timeoutMs);

// ======================================================
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== MAESTRO LoRa SYNC BIN ===");

  if (!init_PMIC()) {
    Serial.println("Aviso: BQ24195L no inicializado");
  } else {
    Serial.println("OK: BQ24195L inicializado");
  }

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error: LoRa init failed");
    while (true);
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setPreambleLength(8);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  applyRadio(BW_TABLE[0], 7, 5);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("Maestro listo para sincronización.\n");
}

// ======================================================
void loop() {
  if (phase == PHASE_TX_SYNC) {
    Serial.println("== FASE 1: Barrido TX ==");
    txResCount = 0;

    for (uint8_t i = 0; i < TX_COUNT; i++) {
      uint8_t tx = TX_LIST[i];
      uint8_t id = msgIdCounter++;

      uint8_t payload[1] = { tx };
      bool ok = sendAndWaitAck(SLAVE_ADDR, CMD_TEST_TX, payload, 1, 2000);
      if (!ok) {
        Serial.print("  TX "); Serial.print(tx);
        Serial.println(" sin ACK, se ignora");
        continue;
      }
      // RSSI/SNR del ACK se guardan en onReceive
      // guardado en txResults[txResCount-1]
      delay(200);
    }

    if (txResCount == 0) {
      Serial.println("  Sin ningún TX válido, usando defecto");
      bestTX = 5;
    } else {
      float bestScore = -1000.0;
      uint8_t bestIdx = 0;
      for (uint8_t i = 0; i < txResCount; i++) {
        float score = txResults[i].snr * 2.0 + (txResults[i].rssi + 120) * 0.5;
        Serial.print("  TX="); Serial.print(txResults[i].tx);
        Serial.print(" RSSI="); Serial.print(txResults[i].rssi);
        Serial.print(" SNR="); Serial.print(txResults[i].snr);
        Serial.print(" score="); Serial.println(score);
        if (score > bestScore) {
          bestScore = score;
          bestIdx = i;
        }
      }
      bestTX = txResults[bestIdx].tx;
      Serial.print(">> Mejor TX encontrado="); Serial.println(bestTX);
    }

    phase = PHASE_CFG_SYNC;
  }
  else if (phase == PHASE_CFG_SYNC) {
    Serial.println("\n== FASE 2: Barrido BW/SF ==");
    cfgResCount = 0;

    for (uint8_t bi = 0; bi < BW_COUNT; bi++) {
      for (uint8_t si = 0; si < SF_COUNT; si++) {
        uint8_t sf = SF_LIST[si];
        uint8_t id = msgIdCounter++;

        // Pre-configurar maestro a esos parámetros
        applyRadio(BW_TABLE[bi], sf, bestTX);

        uint8_t payload[2] = { bi, sf };
        bool ok = sendAndWaitAck(SLAVE_ADDR, CMD_TEST_CFG, payload, 2, 2500);
        if (!ok) {
          Serial.print("  BW idx="); Serial.print(bi);
          Serial.print(" SF="); Serial.print(sf);
          Serial.println(" sin ACK, se ignora");
          continue;
        }

        delay(300);
      }
    }

    if (cfgResCount == 0) {
      Serial.println("  Sin configs válidas, usando BW/SF por defecto");
      bestBWIndex = 0;
      bestSF      = 7;
    } else {
      float bestScore = -1000.0;
      uint8_t bestIdx = 0;
      for (uint8_t i = 0; i < cfgResCount; i++) {
        float score = cfgResults[i].snr * 2.0 + (cfgResults[i].rssi + 120) * 0.5;
        Serial.print("  BW="); Serial.print(BW_TABLE[cfgResults[i].bwIndex]);
        Serial.print(" SF="); Serial.print(cfgResults[i].sf);
        Serial.print(" RSSI="); Serial.print(cfgResults[i].rssi);
        Serial.print(" SNR="); Serial.print(cfgResults[i].snr);
        Serial.print(" score="); Serial.println(score);
        if (score > bestScore) {
          bestScore = score;
          bestIdx = i;
        }
      }
      bestBWIndex = cfgResults[bestIdx].bwIndex;
      bestSF      = cfgResults[bestIdx].sf;
      Serial.print(">> Mejor BW/SF: BW=");
      Serial.print(BW_TABLE[bestBWIndex]);
      Serial.print(" SF=");
      Serial.println(bestSF);
    }

    // Enviar configuración final y esperar ACK_FINAL
    Serial.println("\n== FASE 3: Enviar config final ==");
    applyRadio(BW_TABLE[bestBWIndex], bestSF, bestTX);
    uint8_t payload[3] = { bestTX, bestBWIndex, bestSF };
    bool ok = sendAndWaitAck(SLAVE_ADDR, CMD_SET_FINAL, payload, 3, 2500);
    if (ok) {
      Serial.println("Config final aplicada en esclavo con éxito.");
      phase = PHASE_DONE;
    } else {
      Serial.println("Fallo al aplicar config final en esclavo, pero maestro la mantiene.");
      phase = PHASE_DONE;
    }
  }
  else if (phase == PHASE_DONE) {
    // Enviar mensajes de datos de prueba cada 3s
    static uint32_t lastDataMs = 0;
    if (millis() - lastDataMs > 3000) {
      lastDataMs = millis();
      const char *txt = "Hola LoRa!";
      uint8_t len = strlen(txt);
      uint8_t id = msgIdCounter++;

      uint8_t buf[32];
      memcpy(buf, txt, len);

      bool ok = sendAndWaitAck(SLAVE_ADDR, CMD_DATA, buf, len, 1500);
      if (ok) {
        Serial.println("DATA enviado con ACK correcto.");
      } else {
        Serial.println("DATA enviado sin ACK.");
      }
    }
  }

  delay(10);
}

// ======================================================
void applyRadio(long bw, uint8_t sf, uint8_t tx) {
  LoRa.idle();
  LoRa.setSignalBandwidth(bw);
  LoRa.setSpreadingFactor(sf);
  LoRa.setTxPower(tx, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.receive();
  Serial.print("-> Radio maestro: BW=");
  Serial.print(bw);
  Serial.print(" SF=");
  Serial.print(sf);
  Serial.print(" TX=");
  Serial.println(tx);
}

// Enviar y esperar ACK
bool sendAndWaitAck(uint8_t dest, uint8_t type, const uint8_t* payload, uint8_t len, uint32_t timeoutMs) {
  uint8_t id = msgIdCounter++;
  awaitingAck  = true;
  awaitingType = type;
  awaitingId   = id;

  sendPacket(dest, type, id, payload, len);

  uint32_t t0 = millis();
  while (awaitingAck && (millis() - t0) < timeoutMs) {
    LoRa.receive();
    delay(10);
  }

  if (awaitingAck) {
    Serial.print("Timeout esperando ACK para type=0x");
    Serial.print(type, HEX);
    Serial.print(" id=");
    Serial.println(id);
    awaitingAck = false;
    return false;
  }

  return true;
}

void sendPacket(uint8_t dest, uint8_t type, uint8_t id, const uint8_t* payload, uint8_t len) {
  LoRa.idle();
  while (!LoRa.beginPacket()) { delay(1); }

  LoRa.write(dest);
  LoRa.write(LOCAL_ADDR);
  LoRa.write(type);
  LoRa.write(id);
  for (uint8_t i = 0; i < len; i++) LoRa.write(payload[i]);

  LoRa.endPacket();
  LoRa.receive();

  Serial.print("TX type=0x"); Serial.print(type, HEX);
  Serial.print(" id="); Serial.print(id);
  Serial.print(" dest=0x"); Serial.println(dest, HEX);
}

// ======================================================
void onReceive(int packetSize) {
  if (packetSize < 4) return;

  uint8_t dest = LoRa.read();
  uint8_t src  = LoRa.read();
  uint8_t type = LoRa.read();
  uint8_t id   = LoRa.read();

  uint8_t payload[16];
  uint8_t pLen = 0;
  while (LoRa.available() && pLen < sizeof(payload)) {
    payload[pLen++] = (uint8_t)LoRa.read();
  }

  if (dest != LOCAL_ADDR && dest != 0xFF) {
    return;
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  Serial.print("RX type=0x"); Serial.print(type, HEX);
  Serial.print(" id="); Serial.print(id);
  Serial.print(" de 0x"); Serial.print(src, HEX);
  Serial.print(" RSSI="); Serial.print(rssi);
  Serial.print(" SNR="); Serial.println(snr);

  // Guardar resultados de pruebas
  if (type == CMD_ACK_TX && txResCount < TX_COUNT) {
    txResults[txResCount].tx   = TX_LIST[txResCount];
    txResults[txResCount].rssi = rssi;
    txResults[txResCount].snr  = snr;
    txResCount++;
  }

  if (type == CMD_ACK_CFG && cfgResCount < (BW_COUNT * SF_COUNT)) {
    cfgResults[cfgResCount].bwIndex = cfgResCount % BW_COUNT; // aproximado
    cfgResults[cfgResCount].sf      = SF_LIST[(cfgResCount / BW_COUNT) % SF_COUNT];
    cfgResults[cfgResCount].rssi    = rssi;
    cfgResults[cfgResCount].snr     = snr;
    cfgResCount++;
  }

  // Gestión de ACK genérico
  if ((type == CMD_ACK_TX && awaitingType == CMD_TEST_TX && id == awaitingId) ||
      (type == CMD_ACK_CFG && awaitingType == CMD_TEST_CFG && id == awaitingId) ||
      (type == CMD_ACK_FINAL && awaitingType == CMD_SET_FINAL && id == awaitingId) ||
      (type == CMD_ACK_DATA && awaitingType == CMD_DATA && id == awaitingId)) {
    awaitingAck = false;
  }

  // CMD_ACK_DATA extra
  if (type == CMD_ACK_DATA && pLen >= 1) {
    ackDataReceived = true;
  }
}
