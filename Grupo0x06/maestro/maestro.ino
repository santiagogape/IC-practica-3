
/* ------------------------------------------------------------
 * MAESTRO LoRa - Sincronización completa (TX + BW/SF) + Mensajería M
 * Dirección local: 0x06, destino: 0x05, SyncWord: 0x12
 * - Handshake robusto (TX y BW/SF)
 * - Verificación/curación con QS/SS y RS
 * - Envío fiable de "MXXXXXXXXX" con ACK "MA"
 * - ID mensaje 16-bit correcto
 * ------------------------------------------------------------ */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

// ---------- Parámetros generales ----------
const uint8_t localAddress = 0x06;
uint8_t destination        = 0x05;
const uint8_t SYNC_WORD    = 0x12;

#define TX_LAPSE_MS       15000
#define WAIT_TIMEOUT_MS    2000

// Config original (para reporting y fallback)
#define ORIGINAL_BW 125000L
#define ORIGINAL_SF 7
#define ORIGINAL_TX 3

// ---------- Matrices de prueba ----------
const uint8_t sync_txpower[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
const uint8_t SYNC_TX_COUNT  = sizeof(sync_txpower) / sizeof(sync_txpower[0]);

const long    sync_bw[] = {125000, 250000, 500000, 62500, 41700, 31250, 20800, 15600, 10400, 7800};
const uint8_t sync_sf[] = {7,      7,      7,      8,     9,     10,    11,    12,    12,    12};
const uint8_t SYNC_TESTS_COUNT = sizeof(sync_bw) / sizeof(sync_bw[0]);

// ---------- Estado del protocolo ----------
enum SyncState {
  IDLE,
  SENDING,
  WAIT_SA_AFTER_ST,
  WAIT_SA_AFTER_TX,
  WAIT_SA_AFTER_SE_TX,
  WAIT_SA_AFTER_SI,
  WAIT_SA_AFTER_CONFIG,
  WAIT_SA_AFTER_SE_BWSF
};
volatile SyncState syncState = IDLE;
volatile SyncState protocolPendingState = IDLE;

enum ProtocolPhase { PHASE_TX_SYNC, PHASE_BWSF_SYNC, PHASE_COMPLETE };
ProtocolPhase currentPhase = PHASE_TX_SYNC;

// --------- Variables runtime ----------
volatile bool txDoneFlag = true;
static bool transmitting = false;
static uint32_t tx_begin_ms = 0;

static uint16_t msgCount = 0;
static uint32_t txInterval_ms = TX_LAPSE_MS;

// Config actual
long     currentBW = ORIGINAL_BW;
uint8_t  currentSF = ORIGINAL_SF;
uint8_t  currentTX = ORIGINAL_TX;

// Índices de prueba
uint8_t  txIndex   = 0;
uint8_t  syncIndex = 0;

// Buffers de envío
char     pendingMsg[50];
uint8_t  pendingMsgLen = 0;
volatile bool pendingSend = false;

// Copias previas (si necesitas rollback maestro)
volatile uint8_t prev_spreadingFactor = ORIGINAL_SF;
volatile long    prev_bandwidth       = ORIGINAL_BW;
volatile uint8_t prev_txpower         = ORIGINAL_TX;

// Resultados TX y finalización
struct TxTestResult { uint8_t txpower; int rssi; float snr; float quality; };
TxTestResult txResults[21];
uint8_t      txResultsCount = 0;
uint8_t      bestTxIndex    = 0;
volatile bool txFinalizationPending = false;
uint8_t      bestTXCache = ORIGINAL_TX;

// ---------- Soporte QS/SS/RS y Mensajería M ----------
static volatile bool lastStatusReceived = false; // llegó SS...
static volatile bool lastMAReceived     = false; // llegó MA (ACK de M)
static long         slaveBW_cache       = 0;
static uint8_t      slaveSF_cache       = 0;
static uint8_t      slaveTX_cache       = 0;

static uint16_t mMsgId = 0;        // ID 16-bit para mensajes M
#define M_PAYLOAD_MAX 200

// ---------- Prototipos ----------
void onReceive(int packetSize);
void TxFinished();
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount);
void calculateBestTxPower();
bool requestSlaveStatus(uint32_t timeout_ms = 1500);
bool healToSlaveState(uint32_t timeout_ms = 1500);
bool verifyAndHealSync(uint32_t timeout_ms = 2000);
bool sendMReliable(const char* text, uint8_t maxRetries = 3, uint32_t ackTimeout_ms = 1500);
void enviarM_con_verificacion(const char* texto);
void maestroMessagingTick();

// ======================================================
// Setup
// ======================================================
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== MAESTRO LoRa - Sincronización TX + BW/SF + M ===");

  if (!init_PMIC()) Serial.println("Aviso: BQ24195L no inicializado o no presente");
  else              Serial.println("OK: BQ24195L inicializado");

  if (!LoRa.begin(868E6)) {
    Serial.println("Error: LoRa init failed");
    while (true);
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();

  LoRa.setSignalBandwidth(currentBW);
  LoRa.setSpreadingFactor(currentSF);
  LoRa.setTxPower(currentTX, PA_OUTPUT_PA_BOOST_PIN);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);
  LoRa.receive();

  Serial.print("Config inicial - BW: "); Serial.print(currentBW);
  Serial.print(" Hz, SF: "); Serial.print(currentSF);
  Serial.print(", TX: "); Serial.println(currentTX);
  Serial.println("================================================\n");
}

// ======================================================
/* Loop principal:
 * - Gestiona envío pendiente y timeouts de SA
 * - Arranca fases automáticamente
 * - (Opcional) Envío por Serial tras completar sincronización
 */
// ======================================================
void loop() {
  // Envío pendiente
  if (pendingSend && !transmitting) {
    transmitting = true;
    txDoneFlag   = false;
    tx_begin_ms  = millis();
    sendMessage(pendingMsg, pendingMsgLen, msgCount);
    Serial.print(">> Enviando: '"); Serial.print(pendingMsg); Serial.println("'");
    pendingSend = false;
    syncState   = SENDING;
  }

  // Fin de TX (async)
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print(" TX completado en "); Serial.print(TxTime_ms); Serial.println(" ms");

    if (protocolPendingState != IDLE) {
      syncState = protocolPendingState;
      protocolPendingState = IDLE;
      Serial.print(" Estado: ");
      switch (syncState) {
        case WAIT_SA_AFTER_ST:     Serial.println("WAIT_SA_AFTER_ST"); break;
        case WAIT_SA_AFTER_TX:     Serial.println("WAIT_SA_AFTER_TX"); break;
        case WAIT_SA_AFTER_SE_TX:  Serial.println("WAIT_SA_AFTER_SE_TX"); break;
        case WAIT_SA_AFTER_SI:     Serial.println("WAIT_SA_AFTER_SI"); break;
        case WAIT_SA_AFTER_CONFIG: Serial.println("WAIT_SA_AFTER_CONFIG"); break;
        case WAIT_SA_AFTER_SE_BWSF:Serial.println("WAIT_SA_AFTER_SE_BWSF"); break;
        default:                   Serial.println("OTRO"); break;
      }
    }
    LoRa.receive();
    transmitting = false;
  }

  // Timeouts esperando SA (reacciones por fase)
  static uint32_t waitStartTime = 0;
  static SyncState lastStateForTimer = IDLE;
  if (lastStateForTimer != syncState) {
    waitStartTime = millis();
    lastStateForTimer = syncState;
  }
  if ((syncState == WAIT_SA_AFTER_ST ||
       syncState == WAIT_SA_AFTER_TX ||
       syncState == WAIT_SA_AFTER_SE_TX ||
       syncState == WAIT_SA_AFTER_SI ||
       syncState == WAIT_SA_AFTER_CONFIG ||
       syncState == WAIT_SA_AFTER_SE_BWSF) &&
      (millis() - waitStartTime > WAIT_TIMEOUT_MS)) {

    Serial.println("\n!!! TIMEOUT esperando SA !!!");

    if (currentPhase == PHASE_TX_SYNC) {
      if (txFinalizationPending) {
        Serial.println("Timeout en FINALIZACIÓN TX: reintentando ciclo");
        if (syncState == WAIT_SA_AFTER_ST) {
          strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;
        } else if (syncState == WAIT_SA_AFTER_TX) {
          snprintf(pendingMsg, sizeof(pendingMsg), "T%u", bestTXCache);
          pendingMsgLen = strlen(pendingMsg); pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_TX; syncState = IDLE;
        } else if (syncState == WAIT_SA_AFTER_SE_TX) {
          strcpy(pendingMsg, "SE"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_SE_TX; syncState = IDLE;
        }
        LoRa.receive();
      } else {
        // En pruebas TX: avanzar
        Serial.println("Avanzando a siguiente TX de prueba...\n");
        LoRa.idle();
        LoRa.setTxPower(prev_txpower, PA_OUTPUT_PA_BOOST_PIN);
        LoRa.receive();
        currentTX = prev_txpower;

        txIndex++;
        if (txIndex < SYNC_TX_COUNT) {
          delay(300);
          strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;
          Serial.print(" Iniciando ciclo TX ["); Serial.print(txIndex); Serial.println("] con ST\n");
        } else {
          if (txResultsCount == 0) {
            Serial.println("\n!!! Sin resultados de TX Power, reiniciando a configuración original !!!\n");
            LoRa.idle();
            LoRa.setSignalBandwidth(ORIGINAL_BW);
            LoRa.setSpreadingFactor(ORIGINAL_SF);
            LoRa.setTxPower(ORIGINAL_TX, PA_OUTPUT_PA_BOOST_PIN);
            LoRa.receive();
            currentBW = ORIGINAL_BW; currentSF = ORIGINAL_SF; currentTX = ORIGINAL_TX;
            txIndex = 0; syncIndex = 0; currentPhase = PHASE_TX_SYNC;
            syncState = IDLE; protocolPendingState = IDLE; pendingSend = false;
            delay(500);
          } else {
            calculateBestTxPower(); // programa ST -> Tbest -> SE
          }
        }
      }
    } else {
      // En fase BW/SF: reintentos/avance
      Serial.println("Timeout en BW/SF, continuar con la siguiente configuración...\n");
      if (syncState == WAIT_SA_AFTER_SI) {
        strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
      } else if (syncState == WAIT_SA_AFTER_CONFIG) {
        syncIndex++;
        if (syncIndex < SYNC_TESTS_COUNT) {
          long testBW = sync_bw[syncIndex]; uint8_t testSF = sync_sf[syncIndex];
          snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
          pendingMsgLen = strlen(pendingMsg); pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_CONFIG; syncState = IDLE;
        } else {
          Serial.println("\n>>> FIN FASE BW/SF (agotadas) <<<\n");
          currentPhase = PHASE_COMPLETE; syncState = IDLE;
          protocolPendingState = IDLE; pendingSend = false;
        }
      } else if (syncState == WAIT_SA_AFTER_SE_BWSF) {
        syncIndex++;
        if (syncIndex < SYNC_TESTS_COUNT) {
          long testBW = sync_bw[syncIndex]; uint8_t testSF = sync_sf[syncIndex];
          snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
          pendingMsgLen = strlen(pendingMsg); pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_CONFIG; syncState = IDLE;
        } else {
          Serial.println("\n>>> FIN FASE BW/SF (agotadas) <<<\n");
          currentPhase = PHASE_COMPLETE; syncState = IDLE;
          protocolPendingState = IDLE; pendingSend = false;
        }
      } else {
        strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
      }
      LoRa.receive();
    }
  }

  // Arranque automático del protocolo
  static bool protocolStarted = false;
  if (!protocolStarted && syncState == IDLE && !transmitting) {
    if (currentPhase == PHASE_TX_SYNC) {
      Serial.println("\n>>> FASE 1: Sincronización TX (ciclos ST->T->SE) <<<\n");
      strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_ST; protocolStarted = true;
    } else if (currentPhase == PHASE_BWSF_SYNC) {
      Serial.println("\n>>> FASE 2: Sincronización BW/SF <<<\n");
      strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SI; protocolStarted = true;
    }
  }

  // (Opcional) envíos desde Serial cuando finaliza la sincronización
  maestroMessagingTick();

  delay(10);
}

// ======================================================
// Envío de mensaje con cabecera (async, con TxDone)
// ======================================================
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCountRef) {
  while (!LoRa.beginPacket()) { delay(5); }
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(msgCountRef >> 8));
  LoRa.write((uint8_t)(msgCountRef & 0xFF));
  LoRa.write(msgLength);
  LoRa.print(outgoing);
  LoRa.endPacket(true);
  msgCountRef++;
}

// ======================================================
// onReceive: procesa SA para el protocolo, SS (estado) y MA (ACK de M)
// ======================================================
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  char buffer[60];
  uint8_t recipient = LoRa.read();
  uint8_t sender    = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer)-1)) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  buffer[receivedBytes] = '\0';

  if (incomingLength != receivedBytes) {
    Serial.println("Error: longitud incorrecta"); return;
  }
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXX");
    Serial.print("X Recipient "); Serial.print(recipient); Serial.print(" sender "); Serial.print(sender); Serial.println("  X");
    Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXX");
    Serial.println("Mensaje no es para mí"); return;
  }

  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();
  Serial.print("<< Recibido: '"); Serial.print(buffer);
  Serial.print("'  RSSI: "); Serial.print(rssi);
  Serial.print(" dBm  SNR: "); Serial.println(snr);

  // ---- Captura SS y MA (para QS/M) ----
  if (receivedBytes >= 2 && buffer[0] == 'S' && buffer[1] == 'S') {
    String s = String(buffer);
    int ixX = s.indexOf('X');
    int ixY = s.indexOf('Y');
    int ixT = s.indexOf('T');
    if (ixX >= 2 && ixY > ixX && ixT > ixY) {
      long bw = s.substring(ixX + 1, ixY).toInt();
      int  sf = s.substring(ixY + 1, ixT).toInt();
      int  tx = s.substring(ixT + 1).toInt();
      slaveBW_cache = bw;
      slaveSF_cache = (uint8_t)sf;
      slaveTX_cache = (uint8_t)tx;
      lastStatusReceived = true;
      Serial.print("<< SS parseado: BW="); Serial.print(slaveBW_cache);
      Serial.print(" SF="); Serial.print(slaveSF_cache);
      Serial.print(" TX="); Serial.println(slaveTX_cache);
    }
  }
  if (receivedBytes >= 2 && buffer[0] == 'M' && buffer[1] == 'A') {
    lastMAReceived = true;
    Serial.println("<< ACK MA detectado");
  }

  // ---- Protocolo: sólo procesamos ACK "SA" ----
  if (!(receivedBytes >= 2 && buffer[0] == 'S' && buffer[1] == 'A')) {
    return;
  }
  Serial.println(" SA recibido");

  // ===== FASE TX POWER =====
  if (currentPhase == PHASE_TX_SYNC) {
    if (syncState == WAIT_SA_AFTER_ST) {
      uint8_t txToSend = txFinalizationPending ? bestTXCache : sync_txpower[txIndex];
      snprintf(pendingMsg, sizeof(pendingMsg), "T%u", txToSend);
      pendingMsgLen = strlen(pendingMsg); pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_TX;
      Serial.print(" Preparando T"); Serial.println(txToSend);
    }
    else if (syncState == WAIT_SA_AFTER_TX) {
      uint8_t newTX = txFinalizationPending ? bestTXCache : sync_txpower[txIndex];
      Serial.print(" Cambiando MAESTRO a TX="); Serial.println(newTX);
      prev_txpower = currentTX;

      LoRa.idle();
      LoRa.setTxPower(newTX, PA_OUTPUT_PA_BOOST_PIN);
      LoRa.receive();

      currentTX = newTX;
      delay(80);

      strcpy(pendingMsg, "SE"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SE_TX;
      Serial.println(" Preparando SE (cierre TX)");
    }
    else if (syncState == WAIT_SA_AFTER_SE_TX) {
      if (txFinalizationPending) {
        Serial.println(" Ciclo TX FINAL completado -> pasar a BW/SF");
        txFinalizationPending = false;
        currentPhase = PHASE_BWSF_SYNC;
        strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
      } else {
        Serial.println(" Ciclo TX (prueba) completado!");
        txResults[txResultsCount].txpower = currentTX;
        txResults[txResultsCount].rssi    = rssi;
        txResults[txResultsCount].snr     = snr;
        txResults[txResultsCount].quality = snr * 2.0 + (rssi + 100) * 0.5;
        txResultsCount++;
        txIndex++;

        if (txIndex < SYNC_TX_COUNT) {
          Serial.print("\n>>> Siguiente ciclo TX ("); Serial.print(txIndex);
          Serial.print("/"); Serial.print(SYNC_TX_COUNT); Serial.println(") <<<\n");
          delay(250);
          strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;
        } else {
          calculateBestTxPower(); // programa ST -> Tbest -> SE
        }
      }
    }
  }

  // ===== FASE BW/SF =====
  if (currentPhase == PHASE_BWSF_SYNC) {
    if (syncState == WAIT_SA_AFTER_SI) {
      long testBW = sync_bw[syncIndex]; uint8_t testSF = sync_sf[syncIndex];
      snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
      pendingMsgLen = strlen(pendingMsg); pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_CONFIG;
      Serial.print(" Preparando config ["); Serial.print(syncIndex);
      Serial.print("]: BW="); Serial.print(testBW);
      Serial.print(", SF="); Serial.println(testSF);
    }
    else if (syncState == WAIT_SA_AFTER_CONFIG) {
      long newBW = sync_bw[syncIndex]; uint8_t newSF = sync_sf[syncIndex];
      Serial.print(" Cambiando MAESTRO a BW="); Serial.print(newBW);
      Serial.print(", SF="); Serial.println(newSF);

      LoRa.idle();
      LoRa.setSignalBandwidth(newBW);
      LoRa.setSpreadingFactor(newSF);
      LoRa.receive();

      prev_spreadingFactor = currentSF;
      prev_bandwidth       = currentBW;
      currentBW = newBW; currentSF = newSF;

      delay(80);
      strcpy(pendingMsg, "SE"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SE_BWSF;
      Serial.println(" Preparando SE (cierre BW/SF)");
    }
    else if (syncState == WAIT_SA_AFTER_SE_BWSF) {
      Serial.println(" Ciclo BW/SF completado!");
      Serial.print(" Config actual - BW: "); Serial.print(currentBW);
      Serial.print(" Hz, SF: "); Serial.println(currentSF);

      syncIndex++;
      if (syncIndex < SYNC_TESTS_COUNT) {
        Serial.print("\n>>> Siguiente configuración ("); Serial.print(syncIndex);
        Serial.print("/"); Serial.print(SYNC_TESTS_COUNT); Serial.println(") <<<\n");
        delay(250);
        long testBW = sync_bw[syncIndex]; uint8_t testSF = sync_sf[syncIndex];
        snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
        pendingMsgLen = strlen(pendingMsg); pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_CONFIG;
      } else {
        Serial.println("\n===========================================");
        Serial.println(">>> SINCRONIZACIÓN COMPLETA <<<");
        Serial.println("===========================================\n");
        currentPhase = PHASE_COMPLETE; syncState = IDLE;
        protocolPendingState = IDLE; pendingSend = false;
      }
    }
  }
}

// ======================================================
// TxDone ISR
// ======================================================
void TxFinished() { txDoneFlag = true; }

// ======================================================
// Cálculo de mejor TX y preparación del ciclo final
// ======================================================
void calculateBestTxPower() {
  Serial.println("\n===========================================");
  Serial.println(">>> Análisis TX Power <<<");
  Serial.println("===========================================");

  if (txResultsCount == 0) {
    Serial.println("No hay resultados; reinicio a original.\n");
    LoRa.idle();
    LoRa.setSignalBandwidth(ORIGINAL_BW);
    LoRa.setSpreadingFactor(ORIGINAL_SF);
    LoRa.setTxPower(ORIGINAL_TX, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.receive();
    currentBW = ORIGINAL_BW; currentSF = ORIGINAL_SF; currentTX = ORIGINAL_TX;
    txIndex = 0; syncIndex = 0; currentPhase = PHASE_TX_SYNC;
    syncState = IDLE; protocolPendingState = IDLE; pendingSend = false;
    delay(500);
    return;
  }

  float bestQuality = txResults[0].quality; bestTxIndex = 0;
  for (uint8_t i = 0; i < txResultsCount; i++) {
    Serial.print("TX="); Serial.print(txResults[i].txpower);
    Serial.print(" RSSI="); Serial.print(txResults[i].rssi);
    Serial.print(" SNR=");  Serial.print(txResults[i].snr);
    Serial.print(" Calidad="); Serial.println(txResults[i].quality);
    if (txResults[i].quality > bestQuality) { bestQuality = txResults[i].quality; bestTxIndex = i; }
  }

  bestTXCache = txResults[bestTxIndex].txpower;
  Serial.println("-------------------------------------------");
  Serial.print(">>> MEJOR TX POWER: "); Serial.println(bestTXCache);
  Serial.println("===========================================\n");

  txFinalizationPending = true; // haremos ST -> Tbest -> SE

  strcpy(pendingMsg, "ST");
  pendingMsgLen = 2; pendingSend = true;
  protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;

  Serial.print("Iniciando FINALIZACIÓN de TX con ST; mejor TX=");
  Serial.println(bestTXCache);
}

// ======================================================
// QS/SS/RS + Mensajería M (fiable con ACK MA)
// ======================================================
bool requestSlaveStatus(uint32_t timeout_ms) {
  lastStatusReceived = false;

  while (!LoRa.beginPacket()) { delay(5); }
  LoRa.write(destination); LoRa.write(localAddress);
  LoRa.write((uint8_t)0);  LoRa.write((uint8_t)0);
  LoRa.write((uint8_t)2);  LoRa.print("QS");
  LoRa.endPacket(true);

  uint32_t t0 = millis();
  while (!lastStatusReceived && (millis() - t0) < timeout_ms) {
    LoRa.receive(); delay(10);
  }
  return lastStatusReceived;
}

bool healToSlaveState(uint32_t timeout_ms) {
  bool changed = false;

  if (slaveTX_cache != 0 && slaveTX_cache != currentTX) {
    Serial.print("Curación: MAESTRO -> TX="); Serial.println(slaveTX_cache);
    LoRa.idle(); LoRa.setTxPower(slaveTX_cache, PA_OUTPUT_PA_BOOST_PIN); LoRa.receive();
    currentTX = slaveTX_cache; changed = true; delay(60);
  }
  if ((slaveBW_cache != 0 && slaveBW_cache != currentBW) ||
      (slaveSF_cache != 0 && slaveSF_cache != currentSF)) {
    Serial.print("Curación: MAESTRO -> BW="); Serial.print(slaveBW_cache);
    Serial.print(" SF="); Serial.println(slaveSF_cache);
    LoRa.idle(); LoRa.setSignalBandwidth(slaveBW_cache); LoRa.setSpreadingFactor(slaveSF_cache); LoRa.receive();
    currentBW = slaveBW_cache; currentSF = slaveSF_cache; changed = true; delay(60);
  }

  if (changed) {
    while (!LoRa.beginPacket()) { delay(5); }
    LoRa.write(destination); LoRa.write(localAddress);
    LoRa.write((uint8_t)0); LoRa.write((uint8_t)0);
    LoRa.write((uint8_t)2); LoRa.print("SE");
    LoRa.endPacket(true);

    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) { LoRa.receive(); delay(10); break; }
    return true;
  }
  return true;
}

bool verifyAndHealSync(uint32_t timeout_ms) {
  Serial.println("Verificando sincronización con QS...");
  if (!requestSlaveStatus(timeout_ms)) {
    Serial.println("QS sin respuesta; enviando RS...");
    while (!LoRa.beginPacket()) { delay(5); }
    LoRa.write(destination); LoRa.write(localAddress);
    LoRa.write((uint8_t)0);  LoRa.write((uint8_t)0);
    LoRa.write((uint8_t)2);  LoRa.print("RS");
    LoRa.endPacket(true);

    delay(200);
    if (!requestSlaveStatus(timeout_ms)) {
      Serial.println("Sin estado tras RS; abortando envío de M.");
      return false;
    }
  }

  bool okBW = (slaveBW_cache == currentBW);
  bool okSF = (slaveSF_cache == currentSF);
  bool okTX = (slaveTX_cache == currentTX);

  if (okBW && okSF && okTX) {
    Serial.println("Sincronización OK (BW/SF/TX iguales).");
    return true;
  }
  Serial.println("Diferencias detectadas; curando maestro al estado del esclavo...");
  return healToSlaveState(timeout_ms);
}

bool sendMReliable(const char* text, uint8_t maxRetries, uint32_t ackTimeout_ms) {
  if (!text) return false;

  char payload[M_PAYLOAD_MAX + 2];
  payload[0] = 'M';
  size_t tlen = strlen(text);
  if (tlen > M_PAYLOAD_MAX) tlen = M_PAYLOAD_MAX;
  memcpy(&payload[1], text, tlen);
  payload[1 + tlen] = '\0';
  uint8_t msgLength = (uint8_t)(1 + tlen);

  for (uint8_t attempt = 0; attempt <= maxRetries; ++attempt) {
    lastMAReceived = false;

    while (!LoRa.beginPacket()) { delay(5); }
    LoRa.write(destination); LoRa.write(localAddress);
    LoRa.write((uint8_t)(mMsgId >> 8));
    LoRa.write((uint8_t)(mMsgId & 0xFF));
    LoRa.write(msgLength);
    LoRa.print(payload);
    LoRa.endPacket(true);

    Serial.print(">> Enviando M (intento ");
    Serial.print(attempt + 1); Serial.print("/"); Serial.print(maxRetries + 1);
    Serial.print("): '"); Serial.print(payload + 1); Serial.println("'");

    uint32_t t0 = millis();
    while (!lastMAReceived && (millis() - t0) < ackTimeout_ms) { LoRa.receive(); delay(10); }
    if (lastMAReceived) { Serial.println("ACK MA recibido. M entregado."); mMsgId++; return true; }
    Serial.println("Sin ACK MA; reintento...");
  }
  Serial.println("Fallo en envío M: sin ACK MA tras reintentos.");
  return false;
}

void enviarM_con_verificacion(const char* texto) {
  if (!verifyAndHealSync(2000)) {
    Serial.println("Sincronización no válida; no se envía M.");
    return;
  }
  sendMReliable(texto, /*maxRetries=*/3, /*ackTimeout_ms=*/1500);
}

// Envía M desde el monitor serie cuando la sincronización ha terminado
void maestroMessagingTick() {
  if (currentPhase != PHASE_COMPLETE) return;

  static char lineBuf[M_PAYLOAD_MAX + 1];
  static size_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[idx] = '\0';
      if (idx > 0) enviarM_con_verificacion(lineBuf);
      idx = 0;
      return;
    }
    if (idx < M_PAYLOAD_MAX) lineBuf[idx++] = c;
  }
}
