/* ---------------------------------------------------------------------
 *  Maestro Completo - Sistema de Sincronización Avanzado
 *  Práctica 3 - GII-IoT
 *  
 *  Fases de operación:
 *  1. Sincronización TxPower (mejor RSSI)
 *  2. Sincronización BW/SF (mejor SNR y tiempo)
 *  3. Aplicar mejor configuración
 *  4. Health check periódico
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS 1000
#define MAX_RETRIES 3
#define TIMEOUT_MS 5000
#define HEALTH_CHECK_INTERVAL 10000  // Health check cada 10 segundos

// NOTA: Ajustar estas variables
#define LOCALADDRESS 0x06
#define DESTINATION 0x05
#define START_TX 2
#define START_BW 125000
#define START_SF 7

const uint8_t sync_txpower[] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
const uint8_t SYNC_TX_COUNT = sizeof(sync_txpower) / sizeof(sync_txpower[0]);

const long sync_bw[] = { 125000, 250000, 500000, 62500, 41700, 31250, 20800, 15600, 10400, 7800 };
const uint8_t sync_sf[] = { 7, 7, 7, 8, 9, 10, 11, 12, 12, 12 };
const uint8_t SYNC_TESTS_COUNT = sizeof(sync_bw) / sizeof(sync_bw[0]);

// Estructuras para almacenar resultados
struct TestResult {
  uint8_t txPower;
  int16_t rssi;
  float snr;
  bool valid;
};

struct BwSfResult {
  long bandwidth;
  uint8_t spreadingFactor;
  float snr;
  uint32_t txTime;
  bool valid;
};

TestResult testResults[sizeof(sync_txpower) / sizeof(sync_txpower[0])];
BwSfResult bwSfResults[sizeof(sync_bw) / sizeof(sync_bw[0])];

// Mejores valores encontrados
struct BestConfig {
  uint8_t txPower;
  long bandwidth;
  uint8_t spreadingFactor;
  int16_t rssi;
  float snr;
  uint32_t txTime;
};

BestConfig bestConfig;

volatile uint8_t txIndex = 0;
volatile uint8_t bwSfIndex = 0;
volatile uint8_t retryCount = 0;
volatile bool txDoneFlag = true;
volatile bool saReceived = false;
char pendingMsg[50];

enum SyncState {
  IDLE,
  // Fase 1: TxPower
  SYNC_START,
  WAITSA_AFTERST,
  SEND_T,
  WAITSA_AFTERT,
  // Fase 2: BW/SF
  SEND_BWSF,
  WAITSA_AFTER_BWSF,
  // Fase 3: Finalización y Best Config
  SEND_SE,
  WAIT_SA_AFTER_SE_TX,
  SEND_BEST_CONFIG,
  WAIT_SA_AFTER_BEST,
  // Fase 4: Health Check
  HEALTH_CHECK_MODE
};

volatile SyncState syncState;
volatile uint32_t lastTxTime = 0;
volatile uint32_t lastHealthCheck = 0;

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== LoRa Maestro - Sistema Completo de Sincronización ===\n");

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  } else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  // Configuración inicial
  LoRa.setSignalBandwidth(START_BW);
  LoRa.setSpreadingFactor(START_SF);
  LoRa.setSyncWord(0x12);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setTxPower(START_TX, PA_OUTPUT_PA_BOOST_PIN);

  LoRa.onReceive(onReceive);
  LoRa.receive();
  LoRa.onTxDone(TxFinished);

  Serial.println("LoRa init succeeded.\n");
  
  // Inicializar arrays
  for (uint8_t i = 0; i < SYNC_TX_COUNT; i++) {
    testResults[i].txPower = sync_txpower[i];
    testResults[i].rssi = 0;
    testResults[i].snr = 0.0;
    testResults[i].valid = false;
  }
  
  for (uint8_t i = 0; i < SYNC_TESTS_COUNT; i++) {
    bwSfResults[i].bandwidth = sync_bw[i];
    bwSfResults[i].spreadingFactor = sync_sf[i];
    bwSfResults[i].snr = 0.0;
    bwSfResults[i].txTime = 0;
    bwSfResults[i].valid = false;
  }
  
  syncState = IDLE;
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() {
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
  static bool transmitting = false;

  // Verificar timeout
  if ((syncState == WAITSA_AFTERST || syncState == WAITSA_AFTERT || 
       syncState == WAITSA_AFTER_BWSF || syncState == WAIT_SA_AFTER_SE_TX ||
       syncState == WAIT_SA_AFTER_BEST) 
      && !saReceived && (millis() - lastTxTime > TIMEOUT_MS)) {
    
    Serial.println("TIMEOUT: No se recibió SA");
    retryCount++;
    
    if (retryCount >= MAX_RETRIES) {
      Serial.print("Máximo de reintentos alcanzado - ");
      
      if (syncState == WAITSA_AFTERST) {
        Serial.println("Abortando");
        syncState = IDLE;
      } else if (syncState == WAITSA_AFTERT) {
        Serial.println("Pasando al siguiente TxPower");
        txIndex++;
        syncState = SEND_T;
      } else if (syncState == WAITSA_AFTER_BWSF) {
        Serial.println("Pasando al siguiente BW/SF");
        bwSfIndex++;
        syncState = SEND_BWSF;
      } else if (syncState == WAIT_SA_AFTER_SE_TX) {
        Serial.println("Pasando a calcular mejor config");
        syncState = SEND_BEST_CONFIG;
      } else if (syncState == WAIT_SA_AFTER_BEST) {
        Serial.println("Pasando a Health Check");
        syncState = HEALTH_CHECK_MODE;
      }
      retryCount = 0;
    } else {
      Serial.print("Reintento ");
      Serial.print(retryCount);
      Serial.print("/");
      Serial.println(MAX_RETRIES);
      
      if (syncState == WAITSA_AFTERST) syncState = IDLE;
      else if (syncState == WAITSA_AFTERT) syncState = SEND_T;
      else if (syncState == WAITSA_AFTER_BWSF) syncState = SEND_BWSF;
      else if (syncState == WAIT_SA_AFTER_SE_TX) syncState = SEND_SE;
      else if (syncState == WAIT_SA_AFTER_BEST) syncState = SEND_BEST_CONFIG;
    }
    
    transmitting = false;
    lastSendTime_ms = 0;
  }

  // FASE 4: HEALTH CHECK MODE
  if (syncState == HEALTH_CHECK_MODE && !transmitting && 
      (millis() - lastHealthCheck > HEALTH_CHECK_INTERVAL)) {
    
    snprintf(pendingMsg, sizeof(pendingMsg), "HC");
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
    lastHealthCheck = millis();

    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.println("[Health Check] Enviando HC");
  }

  // FASE 3: SEND_BEST_CONFIG
  if (!transmitting && syncState == SEND_BEST_CONFIG && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    // Calcular mejor configuración
    calculateBestConfig();
    
    // Aplicar mejor configuración
    LoRa.setTxPower(bestConfig.txPower, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSignalBandwidth(bestConfig.bandwidth);
    LoRa.setSpreadingFactor(bestConfig.spreadingFactor);
    
    // Enviar configuración al esclavo
    snprintf(pendingMsg, sizeof(pendingMsg),
             "BEST:TX%u,BW%ld,SF%u",
             bestConfig.txPower,
             bestConfig.bandwidth,
             bestConfig.spreadingFactor);
    
    Serial.println("\n=== APLICANDO MEJOR CONFIGURACIÓN ===");
    Serial.print("TxPower: ");
    Serial.print(bestConfig.txPower);
    Serial.println(" dBm");
    Serial.print("Bandwidth: ");
    Serial.print(bestConfig.bandwidth);
    Serial.println(" Hz");
    Serial.print("Spreading Factor: ");
    Serial.println(bestConfig.spreadingFactor);
    Serial.print("RSSI: ");
    Serial.print(bestConfig.rssi);
    Serial.println(" dBm");
    Serial.print("SNR: ");
    Serial.println(bestConfig.snr);
    Serial.print("Tx Time: ");
    Serial.print(bestConfig.txTime);
    Serial.println(" ms");
    Serial.println("=====================================\n");
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
    lastTxTime = millis();
    saReceived = false;
    
    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    syncState = WAIT_SA_AFTER_BEST;
  }

  // FASE 2: SEND_SE (End of BW/SF tests)
  if (!transmitting && syncState == SEND_SE && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    snprintf(pendingMsg, sizeof(pendingMsg), "SE");
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
    lastTxTime = millis();
    saReceived = false;

    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.println("\nEnviando SE (fin de pruebas BW/SF)");
    syncState = WAIT_SA_AFTER_SE_TX;
  }

  // FASE 2: SEND_BWSF (Bandwidth/Spreading Factor tests)
  if (!transmitting && syncState == SEND_BWSF && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    if (bwSfIndex >= SYNC_TESTS_COUNT) {
      Serial.println("\n=== Todas las pruebas BW/SF completadas ===\n");
      bwSfIndex = 0;
      syncState = SEND_SE;
      return;
    }
    
    // Aplicar nueva configuración BW/SF
    snprintf(pendingMsg, sizeof(pendingMsg), 
             "B%ld,SF%u", 
             sync_bw[bwSfIndex], 
             sync_sf[bwSfIndex]);
    
    Serial.print("\n[Test BW/SF ");
    Serial.print(bwSfIndex + 1);
    Serial.print("/");
    Serial.print(SYNC_TESTS_COUNT);
    Serial.print("] BW=");
    Serial.print(sync_bw[bwSfIndex]);
    Serial.print(", SF=");
    Serial.print(sync_sf[bwSfIndex]);
    Serial.print(" - ");
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
    lastTxTime = millis();
    saReceived = false;

    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.print("Sending '");
    Serial.print(pendingMsg);
    Serial.print("' ");
    
    syncState = WAITSA_AFTER_BWSF;
  }

  // FASE 1: SEND_T (TxPower tests)
  if (!transmitting && syncState == SEND_T && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    if (txIndex >= SYNC_TX_COUNT) {
      Serial.println("\n=== Todas las pruebas TxPower completadas ===");
      Serial.println("=== Iniciando pruebas BW/SF ===\n");
      txIndex = 0;
      bwSfIndex = 0;
      syncState = SEND_BWSF;
      return;
    }
    
    LoRa.setTxPower(sync_txpower[txIndex], PA_OUTPUT_PA_BOOST_PIN);
    
    snprintf(pendingMsg, sizeof(pendingMsg), "T%u", sync_txpower[txIndex]);
    
    Serial.print("\n[Test TX ");
    Serial.print(txIndex + 1);
    Serial.print("/");
    Serial.print(SYNC_TX_COUNT);
    Serial.print("] TxPower: ");
    Serial.print(sync_txpower[txIndex]);
    Serial.print(" dBm - ");
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
    lastTxTime = millis();
    saReceived = false;

    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.print("Sending '");
    Serial.print(pendingMsg);
    Serial.print("' ");
    
    syncState = WAITSA_AFTERT;
  }

  // FASE 1: IDLE - Iniciar sincronización
  if (syncState == IDLE && !transmitting && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    snprintf(pendingMsg, sizeof(pendingMsg), "ST");

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
    lastTxTime = millis();
    saReceived = false;

    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.println("=== INICIANDO SINCRONIZACIÓN ===");
    Serial.print("Sending '");
    Serial.print(pendingMsg);
    Serial.print("' ");
    syncState = WAITSA_AFTERST;
  }

  // Transmisión completada
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");
    
    // Guardar tiempo de transmisión para BW/SF
    if (syncState == WAITSA_AFTER_BWSF) {
      bwSfResults[bwSfIndex].txTime = TxTime_ms;
    }

    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    if (duty_cycle <= 1.0f) {
      txInterval_ms = random(TX_LAPSE_MS) + 1000;
    } else {
      txInterval_ms = TxTime_ms * 100;
    }

    transmitting = false;
    LoRa.receive();
  }
}

// --------------------------------------------------------------------
// Calculate Best Configuration
// --------------------------------------------------------------------
void calculateBestConfig() {
  // Mejor TxPower (mayor RSSI)
  int8_t bestTxIdx = -1;
  int16_t bestRSSI = -200;
  
  for (uint8_t i = 0; i < SYNC_TX_COUNT; i++) {
    if (testResults[i].valid && testResults[i].rssi > bestRSSI) {
      bestRSSI = testResults[i].rssi;
      bestTxIdx = i;
    }
  }
  
  // Mejor BW/SF (mejor SNR y menor tiempo)
  int8_t bestBwSfIdx = -1;
  float bestScore = -1000.0;
  
  for (uint8_t i = 0; i < SYNC_TESTS_COUNT; i++) {
    if (bwSfResults[i].valid) {
      // Score = SNR - (txTime/100)
      // Prioriza SNR alto y tiempo bajo
      float score = bwSfResults[i].snr - (bwSfResults[i].txTime / 100.0);
      if (score > bestScore) {
        bestScore = score;
        bestBwSfIdx = i;
      }
    }
  }
  
  // Asignar mejor configuración
  if (bestTxIdx >= 0) {
    bestConfig.txPower = testResults[bestTxIdx].txPower;
    bestConfig.rssi = testResults[bestTxIdx].rssi;
  } else {
    bestConfig.txPower = START_TX;
    bestConfig.rssi = 0;
  }
  
  if (bestBwSfIdx >= 0) {
    bestConfig.bandwidth = bwSfResults[bestBwSfIdx].bandwidth;
    bestConfig.spreadingFactor = bwSfResults[bestBwSfIdx].spreadingFactor;
    bestConfig.snr = bwSfResults[bestBwSfIdx].snr;
    bestConfig.txTime = bwSfResults[bestBwSfIdx].txTime;
  } else {
    bestConfig.bandwidth = START_BW;
    bestConfig.spreadingFactor = START_SF;
    bestConfig.snr = 0.0;
    bestConfig.txTime = 0;
  }
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t& msgCount) {
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(DESTINATION);
  LoRa.write(LOCALADDRESS);
  LoRa.write((uint8_t)(msgCount >> 7));
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(msgLength);
  LoRa.print(outgoing);
  LoRa.endPacket(true);
  msgCount++;
}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  char buffer[50];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer) - 1))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  buffer[receivedBytes] = '\0';

  if (incomingLength != receivedBytes) {
    Serial.print("Error: length mismatch ");
    Serial.print(incomingLength);
    Serial.print(" vs ");
    Serial.println(receivedBytes);
    return;
  }

  if ((recipient & LOCALADDRESS) != LOCALADDRESS) {
    return;
  }

  int16_t rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // Health Check Response
  if (strcmp(buffer, "HCR") == 0 && syncState == HEALTH_CHECK_MODE) {
    Serial.print("[Health Check] Respuesta recibida - RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.println(snr);
    return;
  }

  // SA Response
  if (strcmp(buffer, "SA") == 0) {
    saReceived = true;
    Serial.println("SA recibido");
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.println(snr);
    
    if (syncState == WAITSA_AFTERST) {
      syncState = SEND_T;
      retryCount = 0;
      txIndex = 0;
      Serial.println("\n=== Iniciando pruebas TxPower ===\n");
    } 
    else if (syncState == WAITSA_AFTERT) {
      testResults[txIndex].rssi = rssi;
      testResults[txIndex].snr = snr;
      testResults[txIndex].valid = true;
      
      Serial.print("Guardado [");
      Serial.print(txIndex);
      Serial.print("]: TX=");
      Serial.print(testResults[txIndex].txPower);
      Serial.print(", RSSI=");
      Serial.print(rssi);
      Serial.print(", SNR=");
      Serial.println(snr);
      
      
      txIndex++;
      syncState = SEND_T;
      retryCount = 0;
    }
    else if (syncState == WAITSA_AFTER_BWSF) {
      bwSfResults[bwSfIndex].snr = snr;
      bwSfResults[bwSfIndex].valid = true;
      
      Serial.print("Guardado [");
      Serial.print(bwSfIndex);
      Serial.print("]: BW=");
      Serial.print(bwSfResults[bwSfIndex].bandwidth);
      Serial.print(", SF=");
      Serial.print(bwSfResults[bwSfIndex].spreadingFactor);
      Serial.print(", SNR=");
      Serial.print(snr);
      Serial.print(", Time=");
      Serial.print(bwSfResults[bwSfIndex].txTime);
      Serial.println(" ms");
      LoRa.setSignalBandwidth(sync_bw[bwSfIndex]);
      LoRa.setSpreadingFactor(sync_sf[bwSfIndex]);
      
      bwSfIndex++;
      syncState = SEND_BWSF;
      retryCount = 0;
    }
    else if (syncState == WAIT_SA_AFTER_SE_TX) {
      Serial.println("SA recibido - Pruebas BW/SF finalizadas");
      syncState = SEND_BEST_CONFIG;
      retryCount = 0;
    }
    else if (syncState == WAIT_SA_AFTER_BEST) {
      Serial.println("\n=== CONFIGURACIÓN APLICADA ===");
      Serial.println("=== Entrando en modo Health Check ===\n");
      syncState = HEALTH_CHECK_MODE;
      lastHealthCheck = millis();
      retryCount = 0;
    }
  }
}

void TxFinished() {
  txDoneFlag = true;
}