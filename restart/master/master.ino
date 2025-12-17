/* ------------------------------------------------------------
 * MAESTRO LoRa - Sincronización completa + Selección óptima
 * Dirección local: 0x06, destino: 0x05, SyncWord: 0x12
 * - Handshake robusto (TX y BW/SF)
 * - Medición de tiempo de transferencia
 * - Selección automática de mejor configuración
 * - Mensajería M con ACK "MA"
 * ------------------------------------------------------------ */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

// ---------- Parámetros generales ----------
const uint8_t localAddress = 0x06;
uint8_t destination        = 0x05;
const uint8_t SYNC_WORD    = 0x12;

#define TX_LAPSE_MS       15000
#define WAIT_TIMEOUT_MS    2500

// Config original
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

// Copias previas
volatile uint8_t prev_spreadingFactor = ORIGINAL_SF;
volatile long    prev_bandwidth       = ORIGINAL_BW;
volatile uint8_t prev_txpower         = ORIGINAL_TX;

// ========== ESTRUCTURAS PARA MEDICIÓN ==========
struct TxTestResult { 
  uint8_t txpower; 
  int rssi; 
  float snr; 
  float quality; 
};
TxTestResult txResults[21];
uint8_t      txResultsCount = 0;
uint8_t      bestTxIndex    = 0;
volatile bool txFinalizationPending = false;
uint8_t      bestTXCache = ORIGINAL_TX;

// Estructura para resultados BW/SF con medición de tiempo
struct BwSfTestResult {
  long bw;
  uint8_t sf;
  int rssi;
  float snr;
  uint32_t transferTime_ms;  // NUEVO: tiempo de transferencia
  float quality;
};
BwSfTestResult bwsfResults[10];
uint8_t bwsfResultsCount = 0;
uint8_t bestBwSfIndex = 0;
volatile bool bwsfFinalizationPending = false;
long    bestBW_cache = ORIGINAL_BW;
uint8_t bestSF_cache = ORIGINAL_SF;

// Medición de tiempo por ciclo BW/SF
uint32_t cycleStartTime_ms = 0;

// ---------- Soporte QS/SS/RS y Mensajería M ----------
static volatile bool lastStatusReceived = false;
static volatile bool lastMAReceived     = false;
static long         slaveBW_cache       = 0;
static uint8_t      slaveSF_cache       = 0;
static uint8_t      slaveTX_cache       = 0;

static uint16_t mMsgId = 0;
#define M_PAYLOAD_MAX 200

// ---------- Prototipos ----------
void onReceive(int packetSize);
void TxFinished();
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount);
void calculateBestTxPower();
void calculateBestBwSf();
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
  Serial.println("=== MAESTRO LoRa - Sincronización Optimizada ===");

  if (!init_PMIC()) Serial.println("Aviso: BQ24195L no inicializado");
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
// Loop principal
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

  // Fin de TX
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print(" TX completado en "); Serial.print(TxTime_ms); Serial.println(" ms");

    if (protocolPendingState != IDLE) {
      syncState = protocolPendingState;
      protocolPendingState = IDLE;
    }
    LoRa.receive();
    transmitting = false;
  }

  // Timeouts esperando SA
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
        Serial.println("Timeout en FINALIZACIÓN TX: reintentando");
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
        Serial.println("Avanzando a siguiente TX...\n");
        LoRa.idle();
        LoRa.setTxPower(prev_txpower, PA_OUTPUT_PA_BOOST_PIN);
        LoRa.receive();
        currentTX = prev_txpower;

        txIndex++;
        if (txIndex < SYNC_TX_COUNT) {
          delay(300);
          strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;
        } else {
          if (txResultsCount == 0) {
            Serial.println("\n!!! Sin resultados TX, reiniciando !!!\n");
            LoRa.idle();
            LoRa.setSignalBandwidth(ORIGINAL_BW);
            LoRa.setSpreadingFactor(ORIGINAL_SF);
            LoRa.setTxPower(ORIGINAL_TX, PA_OUTPUT_PA_BOOST_PIN);
            LoRa.receive();
            currentBW = ORIGINAL_BW; currentSF = ORIGINAL_SF; currentTX = ORIGINAL_TX;
            txIndex = 0; syncIndex = 0; currentPhase = PHASE_TX_SYNC;
            syncState = IDLE; protocolPendingState = IDLE;
            delay(500);
          } else {
            calculateBestTxPower();
          }
        }
      }
    } else if (currentPhase == PHASE_BWSF_SYNC) {
      if (bwsfFinalizationPending) {
        Serial.println("Timeout en FINALIZACIÓN BW/SF: reintentando");
        if (syncState == WAIT_SA_AFTER_SI) {
          strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
        } else if (syncState == WAIT_SA_AFTER_CONFIG) {
          snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", bestBW_cache, bestSF_cache);
          pendingMsgLen = strlen(pendingMsg); pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_CONFIG; syncState = IDLE;
        } else if (syncState == WAIT_SA_AFTER_SE_BWSF) {
          strcpy(pendingMsg, "SE"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_SE_BWSF; syncState = IDLE;
        }
        LoRa.receive();
      } else {
        Serial.println("Timeout en BW/SF, avanzar...\n");
        LoRa.idle();
        LoRa.setSignalBandwidth(prev_bandwidth);
        LoRa.setSpreadingFactor(prev_spreadingFactor);
        LoRa.receive();
        currentBW = prev_bandwidth;
        currentSF = prev_spreadingFactor;

        syncIndex++;
        if (syncIndex < SYNC_TESTS_COUNT) {
          delay(300);
          strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
        } else {
          if (bwsfResultsCount == 0) {
            Serial.println("\n!!! Sin resultados BW/SF !!!\n");
            currentPhase = PHASE_COMPLETE; syncState = IDLE;
          } else {
            calculateBestBwSf();
          }
        }
      }
    }
  }

  // Arranque automático
  static bool protocolStarted = false;
  if (!protocolStarted && syncState == IDLE && !transmitting) {
    if (currentPhase == PHASE_TX_SYNC) {
      Serial.println("\n>>> FASE 1: Sincronización TX <<<\n");
      strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_ST; protocolStarted = true;
    } else if (currentPhase == PHASE_BWSF_SYNC) {
      Serial.println("\n>>> FASE 2: Sincronización BW/SF <<<\n");
      strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SI; protocolStarted = true;
    }
  }

  maestroMessagingTick();
  delay(10);
}

// ======================================================
// Envío de mensaje
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
// onReceive: procesamiento mejorado con medición de tiempo
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

  if (incomingLength != receivedBytes || (recipient != localAddress && recipient != 0xFF)) {
    return;
  }

  int   rssi = LoRa.packetRssi();
  float snr  = LoRa.packetSnr();
  Serial.print("<< Recibido: '"); Serial.print(buffer);
  Serial.print("'  RSSI: "); Serial.print(rssi);
  Serial.print(" dBm  SNR: "); Serial.println(snr);

  // Captura SS y MA
  if (receivedBytes >= 2 && buffer[0] == 'S' && buffer[1] == 'S') {
    String s = String(buffer);
    int ixX = s.indexOf('X');
    int ixY = s.indexOf('Y');
    int ixT = s.indexOf('T');
    if (ixX >= 2 && ixY > ixX && ixT > ixY) {
      slaveBW_cache = s.substring(ixX + 1, ixY).toInt();
      slaveSF_cache = (uint8_t)s.substring(ixY + 1, ixT).toInt();
      slaveTX_cache = (uint8_t)s.substring(ixT + 1).toInt();
      lastStatusReceived = true;
    }
  }
  if (receivedBytes >= 2 && buffer[0] == 'M' && buffer[1] == 'A') {
    lastMAReceived = true;
  }

  // Sólo procesamos SA
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
    }
    else if (syncState == WAIT_SA_AFTER_TX) {
      uint8_t newTX = txFinalizationPending ? bestTXCache : sync_txpower[txIndex];
      prev_txpower = currentTX;
      LoRa.idle();
      currentTX = newTX;
      LoRa.setTxPower(newTX, PA_OUTPUT_PA_BOOST_PIN);
      LoRa.receive();
      delay(80);

      strcpy(pendingMsg, "SE"); pendingMsgLen = 2; pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SE_TX;
    }
    else if (syncState == WAIT_SA_AFTER_SE_TX) {
      if (txFinalizationPending) {
        Serial.println(" Ciclo TX FINAL completado -> pasar a BW/SF\n");
        txFinalizationPending = false;
        currentPhase = PHASE_BWSF_SYNC;
        strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
      } else {
        txResults[txResultsCount].txpower = currentTX;
        txResults[txResultsCount].rssi    = rssi;
        txResults[txResultsCount].snr     = snr;
        txResults[txResultsCount].quality = snr * 2.0 + (rssi + 100) * 0.5;
        txResultsCount++;
        txIndex++;

        if (txIndex < SYNC_TX_COUNT) {
          delay(250);
          strcpy(pendingMsg, "ST"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;
        } else {
          calculateBestTxPower();
        }
      }
    }
  }

  // ===== FASE BW/SF CON MEDICIÓN DE TIEMPO =====
  if (currentPhase == PHASE_BWSF_SYNC) {
    if (syncState == WAIT_SA_AFTER_SI) {
      long testBW = bwsfFinalizationPending ? bestBW_cache : sync_bw[syncIndex];
      uint8_t testSF = bwsfFinalizationPending ? bestSF_cache : sync_sf[syncIndex];
      
      snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
      pendingMsgLen = strlen(pendingMsg); pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_CONFIG;
      
      // INICIO de medición de tiempo del ciclo
      cycleStartTime_ms = millis();
    }
    else if (syncState == WAIT_SA_AFTER_CONFIG) {
      long newBW = bwsfFinalizationPending ? bestBW_cache : sync_bw[syncIndex];
      uint8_t newSF = bwsfFinalizationPending ? bestSF_cache : sync_sf[syncIndex];

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
    }
    else if (syncState == WAIT_SA_AFTER_SE_BWSF) {
      // FIN de medición: calcular tiempo total del ciclo
      uint32_t cycleTime = millis() - cycleStartTime_ms;
      
      if (bwsfFinalizationPending) {
        Serial.println(" Ciclo BW/SF FINAL completado!");
        Serial.println("\n===========================================");
        Serial.println(">>> SINCRONIZACIÓN COMPLETA <<<");
        Serial.print(">>> Config óptima: BW="); Serial.print(currentBW);
        Serial.print(" Hz, SF="); Serial.print(currentSF);
        Serial.print(", TX="); Serial.println(currentTX);
        Serial.println("===========================================\n");
        
        bwsfFinalizationPending = false;
        currentPhase = PHASE_COMPLETE; syncState = IDLE;
        protocolPendingState = IDLE; pendingSend = false;
      } else {
        Serial.print(" Ciclo completado en "); 
        Serial.print(cycleTime); 
        Serial.println(" ms");
        
        bwsfResults[bwsfResultsCount].bw = currentBW;
        bwsfResults[bwsfResultsCount].sf = currentSF;
        bwsfResults[bwsfResultsCount].rssi = rssi;
        bwsfResults[bwsfResultsCount].snr = snr;
        bwsfResults[bwsfResultsCount].transferTime_ms = cycleTime;
        bwsfResults[bwsfResultsCount].quality = snr * 2.0 + (rssi + 100) * 0.5;
        bwsfResultsCount++;
        
        syncIndex++;
        if (syncIndex < SYNC_TESTS_COUNT) {
          delay(250);
          strcpy(pendingMsg, "SI"); pendingMsgLen = 2; pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
        } else {
          calculateBestBwSf();
        }
      }
    }
  }
}

void TxFinished() { txDoneFlag = true; }

// ======================================================
// Cálculo de mejor TX
// ======================================================
void calculateBestTxPower() {
  Serial.println("\n===========================================");
  Serial.println(">>> Análisis TX Power <<<");
  Serial.println("===========================================");

  if (txResultsCount == 0) {
    Serial.println("Sin resultados TX\n");
    LoRa.idle();
    LoRa.setSignalBandwidth(ORIGINAL_BW);
    LoRa.setSpreadingFactor(ORIGINAL_SF);
    LoRa.setTxPower(ORIGINAL_TX, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.receive();
    currentBW = ORIGINAL_BW; currentSF = ORIGINAL_SF; currentTX = ORIGINAL_TX;
    txIndex = 0; syncIndex = 0; currentPhase = PHASE_TX_SYNC;
    syncState = IDLE; protocolPendingState = IDLE;
    delay(500);
    return;
  }

  float bestQuality = txResults[0].quality; bestTxIndex = 0;
  for (uint8_t i = 0; i < txResultsCount; i++) {
    Serial.print("TX="); Serial.print(txResults[i].txpower);
    Serial.print(" RSSI="); Serial.print(txResults[i].rssi);
    Serial.print(" SNR=");  Serial.print(txResults[i].snr);
    Serial.print(" Calidad="); Serial.println(txResults[i].quality);
    if (txResults[i].quality > bestQuality) { 
      bestQuality = txResults[i].quality; 
      bestTxIndex = i; 
    }
  }

  bestTXCache = txResults[bestTxIndex].txpower;
  Serial.println("-------------------------------------------");
  Serial.print(">>> MEJOR TX POWER: "); Serial.println(bestTXCache);
  Serial.println("===========================================\n");

  txFinalizationPending = true;
  strcpy(pendingMsg, "ST");
  pendingMsgLen = 2; pendingSend = true;
  protocolPendingState = WAIT_SA_AFTER_ST; syncState = IDLE;
}

// ======================================================
// NUEVO: Cálculo de mejor BW/SF basado en menor tiempo
// ======================================================
void calculateBestBwSf() {
  Serial.println("\n===========================================");
  Serial.println(">>> Análisis BW/SF (Tiempo de Transferencia) <<<");
  Serial.println("===========================================");

  if (bwsfResultsCount == 0) {
    Serial.println("Sin resultados BW/SF\n");
    currentPhase = PHASE_COMPLETE; syncState = IDLE;
    return;
  }

  // Buscar la configuración con MENOR tiempo de transferencia
  uint32_t bestTime = bwsfResults[0].transferTime_ms;
  bestBwSfIndex = 0;
  
  for (uint8_t i = 0; i < bwsfResultsCount; i++) {
    Serial.print("BW="); Serial.print(bwsfResults[i].bw);
    Serial.print(" Hz, SF="); Serial.print(bwsfResults[i].sf);
    Serial.print(" | Tiempo="); Serial.print(bwsfResults[i].transferTime_ms);
    Serial.print(" ms | RSSI="); Serial.print(bwsfResults[i].rssi);
    Serial.print(" dBm, SNR="); Serial.print(bwsfResults[i].snr);
    Serial.print(" | Calidad="); Serial.println(bwsfResults[i].quality);
    
    // Priorizar MENOR tiempo
    if (bwsfResults[i].transferTime_ms < bestTime) {
      bestTime = bwsfResults[i].transferTime_ms;
      bestBwSfIndex = i;
    }
  }

  bestBW_cache = bwsfResults[bestBwSfIndex].bw;
  bestSF_cache = bwsfResults[bestBwSfIndex].sf;
  
  Serial.println("-------------------------------------------");
  Serial.print(">>> MEJOR CONFIG (menor tiempo): BW=");
  Serial.print(bestBW_cache);
  Serial.print(" Hz, SF="); Serial.print(bestSF_cache);
  Serial.print(" | Tiempo="); Serial.print(bestTime);
  Serial.println(" ms");
  Serial.println("===========================================\n");

  bwsfFinalizationPending = true;
  strcpy(pendingMsg, "SI");
  pendingMsgLen = 2; pendingSend = true;
  protocolPendingState = WAIT_SA_AFTER_SI; syncState = IDLE;
}

// ======================================================
// Funciones QS/SS/RS y Mensajería M
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
    if (lastMAReceived) { 
      Serial.println("ACK MA recibido. M entregado."); 
      mMsgId++; 
      return true; 
    }
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
  sendMReliable(texto, 3, 1500);
}

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