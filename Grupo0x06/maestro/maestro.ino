/* ---------------------------------------------------------------------
 *  MAESTRO LoRa - Protocolo de Sincronización con TX Power
 *  
 *  Protocolo TX Power:
 *  1. Maestro envía "ST" → Esclavo responde "SA"
 *  2. Maestro envía "TXX" (XX=valor) → Esclavo responde "SA" (esclavo cambia TX)
 *  3. Maestro cambia su TX power
 *  4. Maestro envía "SE" con nuevo TX → Esclavo responde "SA"
 *  5. Repite desde paso 2 con siguiente TX power
 *  6. Al finalizar, se queda con mejor configuración de ruido
 *  
 *  Protocolo BW/SF (después de TX Power):
 *  1. Maestro envía "SI" → Esclavo responde "SA"
 *  2. Maestro envía "XBWYSF" → Esclavo responde "SA" (esclavo cambia config)
 *  3. Maestro cambia a la nueva configuración
 *  4. Maestro envía "SE" con nueva config → Esclavo responde "SA"
 *  5. Repite desde paso 2 con siguiente configuración
 *
 *  Autores:
 * · Nicolás Rey Alonso
 * · José Manuel Díaz Hernández
 * · Santiago galindo Peralta
 * · Alberto Martel Rodríguez
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS 15000

const uint8_t localAddress = 0x06;
uint8_t destination = 0x05;

volatile bool txDoneFlag = true;

// Array de TX Power a probar (2 a 20)
const uint8_t sync_txpower[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
const uint8_t SYNC_TX_COUNT = sizeof(sync_txpower) / sizeof(sync_txpower[0]);

// Array de configuraciones BW/SF a probar
const long sync_bw[] = {125000, 250000, 500000, 62500, 41700, 31250, 20800, 15600, 10400, 7800};
const uint8_t sync_sf[] = {7, 7, 7, 8, 9, 10, 11, 12, 12, 12};
const uint8_t SYNC_TESTS_COUNT = sizeof(sync_bw) / sizeof(sync_bw[0]);

// Estados del protocolo
enum SyncState {
  IDLE,
  SENDING,
  WAIT_SA_AFTER_ST,       // Esperando SA después de ST (inicio TX power)
  WAIT_SA_AFTER_TX,       // Esperando SA después de TXX
  WAIT_SA_AFTER_SE_TX,    // Esperando SA después de SE (TX power)
  WAIT_SA_AFTER_SI,       // Esperando SA después de SI (inicio BW/SF)
  WAIT_SA_AFTER_CONFIG,   // Esperando SA después de XBWYSF
  WAIT_SA_AFTER_SE_BWSF   // Esperando SA después de SE (BW/SF)
};

volatile SyncState syncState = IDLE;
volatile SyncState protocolPendingState = IDLE;

const uint32_t WAIT_TIMEOUT_MS = 15000;
uint32_t waitStartTime = 0;

char pendingMsg[50];
uint8_t pendingMsgLen = 0;
volatile bool pendingSend = false;

// Configuración actual
long currentBW = 125000;
uint8_t currentSF = 7;
uint8_t currentTX = 3;

// Índices de configuración
uint8_t txIndex = 0;
uint8_t syncIndex = 0;

// Configuración previa (para rollback)
volatile uint8_t prev_spreadingFactor = 7;
volatile long prev_bandwidth = 125000;
volatile uint8_t prev_txpower = 3;

// Variables para tracking de mejor TX power
struct TxTestResult {
  uint8_t txpower;
  int rssi;
  float snr;
  float quality; // Métrica combinada (mayor es mejor)
};

TxTestResult txResults[20]; // Máximo 20 tests
uint8_t txResultsCount = 0;
uint8_t bestTxIndex = 0;

// Variables de control de transmisión
static uint32_t lastSendTime_ms = 0;
static uint16_t msgCount = 0;
static uint32_t txInterval_ms = TX_LAPSE_MS;
static uint32_t tx_begin_ms = 0;
static bool transmitting = false;

// Fase del protocolo
enum ProtocolPhase {
  PHASE_TX_SYNC,
  PHASE_BWSF_SYNC,
  PHASE_COMPLETE
};

ProtocolPhase currentPhase = PHASE_TX_SYNC;

void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount);
void onReceive(int packetSize);
void TxFinished();
void calculateBestTxPower();

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== MAESTRO LoRa - Sincronización TX + BW/SF ===");

  if (!init_PMIC()) {
    Serial.println("Error: Inicialización BQ24195L fallida");
  } else {
    Serial.println("OK: BQ24195L inicializado");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("Error: LoRa init failed");
    while (true);
  }

  // Configuración inicial
  LoRa.setSignalBandwidth(125000);
  LoRa.setSpreadingFactor(7);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);
  currentBW = 125000;
  currentSF = 7;
  currentTX = 3;
  LoRa.setSyncWord(0x12);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);
  LoRa.receive();

  Serial.print("Config inicial - BW: ");
  Serial.print(currentBW);
  Serial.print(" Hz, SF: ");
  Serial.print(currentSF);
  Serial.print(", TX: ");
  Serial.println(currentTX);
  Serial.println("================================================\n");
}

void loop() {
  // Envío pendiente del protocolo
  if (pendingSend && !transmitting) {
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(pendingMsg, pendingMsgLen, msgCount);

    Serial.print(">> Enviando: '");
    Serial.print(pendingMsg);
    Serial.println("'");

    pendingSend = false;
    syncState = SENDING;
  }

  // Transmisión completada
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("   TX completado en ");
    Serial.print(TxTime_ms);
    Serial.println(" ms");

    // Activar estado pendiente
    if (protocolPendingState != IDLE) {
      syncState = protocolPendingState;
      protocolPendingState = IDLE;
      waitStartTime = millis();
      
      Serial.print("   Estado: ");
      switch (syncState) {
        case WAIT_SA_AFTER_ST:
          Serial.println("WAIT_SA_AFTER_ST");
          break;
        case WAIT_SA_AFTER_TX:
          Serial.println("WAIT_SA_AFTER_TX");
          break;
        case WAIT_SA_AFTER_SE_TX:
          Serial.println("WAIT_SA_AFTER_SE_TX");
          break;
        case WAIT_SA_AFTER_SI:
          Serial.println("WAIT_SA_AFTER_SI");
          break;
        case WAIT_SA_AFTER_CONFIG:
          Serial.println("WAIT_SA_AFTER_CONFIG");
          break;
        case WAIT_SA_AFTER_SE_BWSF:
          Serial.println("WAIT_SA_AFTER_SE_BWSF");
          break;
        default:
          Serial.println("OTRO");
          break;
      }
      LoRa.receive();
    } else {
      LoRa.receive();
    }

    transmitting = false;
  }

  // Timeout handling
  if ((syncState == WAIT_SA_AFTER_ST || 
       syncState == WAIT_SA_AFTER_TX || 
       syncState == WAIT_SA_AFTER_SE_TX ||
       syncState == WAIT_SA_AFTER_SI || 
       syncState == WAIT_SA_AFTER_CONFIG || 
       syncState == WAIT_SA_AFTER_SE_BWSF) &&
      (millis() - waitStartTime > WAIT_TIMEOUT_MS)) {
    
    Serial.println("\n!!! TIMEOUT esperando SA !!!");
    
    if (currentPhase == PHASE_TX_SYNC) {
      // En fase TX: volver a config anterior y probar siguiente
      Serial.println("Volviendo a TX power anterior y probando siguiente...\n");
      
      LoRa.setTxPower(prev_txpower, PA_OUTPUT_PA_BOOST_PIN);
      currentTX = prev_txpower;
      
      txIndex++;
      
      if (txIndex < SYNC_TX_COUNT) {
        delay(500);
        
        uint8_t testTX = sync_txpower[txIndex];
        snprintf(pendingMsg, sizeof(pendingMsg), "T%u", testTX);
        pendingMsgLen = strlen(pendingMsg);
        pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_TX;
        syncState = IDLE;
        
        Serial.print("   Probando siguiente TX: ");
        Serial.println(testTX);
      } else {
        // Terminó fase TX, seleccionar mejor y pasar a BW/SF
        calculateBestTxPower();
        currentPhase = PHASE_BWSF_SYNC;
        syncState = IDLE;
        protocolPendingState = IDLE;
      }
    } else {
      // En fase BW/SF: reiniciar protocolo
      Serial.println("Reiniciando protocolo BW/SF...\n");
      
      syncState = IDLE;
      protocolPendingState = IDLE;
      pendingSend = false;
      syncIndex = 0;
      
      LoRa.setSignalBandwidth(prev_bandwidth);
      LoRa.setSpreadingFactor(prev_spreadingFactor);
      currentBW = prev_bandwidth;
      currentSF = prev_spreadingFactor;
    }
    
    LoRa.receive();
  }

  // Iniciar protocolo automáticamente
  static bool protocolStarted = false;
  if (!protocolStarted && syncState == IDLE && !transmitting) {
    if (currentPhase == PHASE_TX_SYNC) {
      Serial.println("\n>>> FASE 1: Sincronización TX Power <<<\n");
      strcpy(pendingMsg, "ST");
      pendingMsgLen = 2;
      pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_ST;
      protocolStarted = true;
    } else if (currentPhase == PHASE_BWSF_SYNC) {
      Serial.println("\n>>> FASE 2: Sincronización BW/SF <<<\n");
      strcpy(pendingMsg, "SI");
      pendingMsgLen = 2;
      pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SI;
      protocolStarted = true;
    }
  }

  delay(10);
}

void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount) {
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(msgCount >> 7));
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(msgLength);
  LoRa.print(outgoing);
  LoRa.endPacket(true); // async mode
  msgCount++;
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  char buffer[50];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer) - 1)) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  buffer[receivedBytes] = '\0';

  if (incomingLength != receivedBytes) {
    Serial.println("Error: longitud incorrecta");
    return;
  }

  if ((recipient & localAddress) != localAddress && recipient != 0xFF) {
    Serial.println("Mensaje no es para mí");
    return;
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  Serial.print("<< Recibido: '");
  Serial.print(buffer);
  Serial.print("' | RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm | SNR: ");
  Serial.println(snr);

  // Procesamiento del protocolo
  if (receivedBytes >= 2 && buffer[0] == 'S' && buffer[1] == 'A') {
    Serial.println("   SA recibido");

    // ========== FASE TX POWER ==========
    if (currentPhase == PHASE_TX_SYNC) {
      
      if (syncState == WAIT_SA_AFTER_ST) {
        // Enviar primer TX power
        uint8_t testTX = sync_txpower[txIndex];
        
        snprintf(pendingMsg, sizeof(pendingMsg), "T%u", testTX);
        pendingMsgLen = strlen(pendingMsg);
        pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_TX;
        
        Serial.print("   Preparando TX [");
        Serial.print(txIndex);
        Serial.print("]: ");
        Serial.println(testTX);
      }
      else if (syncState == WAIT_SA_AFTER_TX) {
        // Esclavo confirmó cambio de TX
        uint8_t newTX = sync_txpower[txIndex];
        
        Serial.print("   Cambiando maestro a TX=");
        Serial.println(newTX);
        
        prev_txpower = currentTX;
        LoRa.setTxPower(newTX, PA_OUTPUT_PA_BOOST_PIN);
        currentTX = newTX;
        
        delay(100);
        LoRa.receive();
        
        // Enviar SE para probar
        strcpy(pendingMsg, "SE");
        pendingMsgLen = 2;
        pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_SE_TX;
        
        Serial.println("   Preparando SE con nuevo TX");
      }
      else if (syncState == WAIT_SA_AFTER_SE_TX) {
        // Ciclo TX completado, guardar resultado
        Serial.println("   Ciclo TX completado!");
        
        txResults[txResultsCount].txpower = currentTX;
        txResults[txResultsCount].rssi = rssi;
        txResults[txResultsCount].snr = snr;
        // Calidad: prioriza SNR alto y RSSI no demasiado bajo
        txResults[txResultsCount].quality = snr * 2.0 + (rssi + 100) * 0.5;
        
        Serial.print("   TX=");
        Serial.print(currentTX);
        Serial.print(" | Calidad=");
        Serial.println(txResults[txResultsCount].quality);
        
        txResultsCount++;
        txIndex++;
        
        if (txIndex < SYNC_TX_COUNT) {
          Serial.print("\n>>> Siguiente TX Power (");
          Serial.print(txIndex);
          Serial.print("/");
          Serial.print(SYNC_TX_COUNT);
          Serial.println(") <<<\n");
          
          delay(500);
          
          uint8_t testTX = sync_txpower[txIndex];
          snprintf(pendingMsg, sizeof(pendingMsg), "T%u", testTX);
          pendingMsgLen = strlen(pendingMsg);
          pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_TX;
          
          Serial.print("   Preparando TX [");
          Serial.print(txIndex);
          Serial.print("]: ");
          Serial.println(testTX);
        } else {
          // Terminó fase TX
          calculateBestTxPower();
          currentPhase = PHASE_BWSF_SYNC;
          syncState = IDLE;
          protocolPendingState = IDLE;
          pendingSend = false;
        }
      }
    }
    
    // ========== FASE BW/SF ==========
    else if (currentPhase == PHASE_BWSF_SYNC) {
      
      if (syncState == WAIT_SA_AFTER_SI) {
        // Enviar primera configuración BW/SF
        long testBW = sync_bw[syncIndex];
        uint8_t testSF = sync_sf[syncIndex];
        
        snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
        pendingMsgLen = strlen(pendingMsg);
        pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_CONFIG;
        
        Serial.print("   Preparando config [");
        Serial.print(syncIndex);
        Serial.print("]: BW=");
        Serial.print(testBW);
        Serial.print(", SF=");
        Serial.println(testSF);
      }
      else if (syncState == WAIT_SA_AFTER_CONFIG) {
        // Esclavo confirmó cambio de config
        long newBW = sync_bw[syncIndex];
        uint8_t newSF = sync_sf[syncIndex];
        
        Serial.print("   Cambiando maestro a BW=");
        Serial.print(newBW);
        Serial.print(", SF=");
        Serial.println(newSF);
        
        LoRa.setSignalBandwidth(newBW);
        LoRa.setSpreadingFactor(newSF);
        prev_spreadingFactor = currentSF;
        prev_bandwidth = currentBW;
        currentBW = newBW;
        currentSF = newSF;
        
        delay(100);
        LoRa.receive();
        
        strcpy(pendingMsg, "SE");
        pendingMsgLen = 2;
        pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_SE_BWSF;
        
        Serial.println("   Preparando SE con nueva config");
      }
      else if (syncState == WAIT_SA_AFTER_SE_BWSF) {
        // Ciclo BW/SF completado
        Serial.println("   Ciclo BW/SF completado!");
        Serial.print("   Config actual - BW: ");
        Serial.print(currentBW);
        Serial.print(" Hz, SF: ");
        Serial.println(currentSF);
        
        syncIndex++;
        
        if (syncIndex < SYNC_TESTS_COUNT) {
          Serial.print("\n>>> Siguiente configuración (");
          Serial.print(syncIndex);
          Serial.print("/");
          Serial.print(SYNC_TESTS_COUNT);
          Serial.println(") <<<\n");
          
          delay(500);
          
          long testBW = sync_bw[syncIndex];
          uint8_t testSF = sync_sf[syncIndex];
          
          snprintf(pendingMsg, sizeof(pendingMsg), "X%ldY%u", testBW, testSF);
          pendingMsgLen = strlen(pendingMsg);
          pendingSend = true;
          protocolPendingState = WAIT_SA_AFTER_CONFIG;
          
          Serial.print("   Preparando config [");
          Serial.print(syncIndex);
          Serial.print("]: BW=");
          Serial.print(testBW);
          Serial.print(", SF=");
          Serial.println(testSF);
        } else {
          Serial.println("\n===========================================");
          Serial.println(">>> SINCRONIZACIÓN COMPLETA <<<");
          Serial.println("===========================================\n");
          currentPhase = PHASE_COMPLETE;
          syncState = IDLE;
          protocolPendingState = IDLE;
          pendingSend = false;
        }
      }
    }
  }
}

void calculateBestTxPower() {
  Serial.println("\n===========================================");
  Serial.println(">>> Análisis TX Power <<<");
  Serial.println("===========================================");
  
  if (txResultsCount == 0) {
    Serial.println("No hay resultados de TX power");
    return;
  }
  
  // Encontrar mejor calidad
  float bestQuality = txResults[0].quality;
  bestTxIndex = 0;
  
  for (uint8_t i = 0; i < txResultsCount; i++) {
    Serial.print("TX=");
    Serial.print(txResults[i].txpower);
    Serial.print(" | RSSI=");
    Serial.print(txResults[i].rssi);
    Serial.print(" | SNR=");
    Serial.print(txResults[i].snr);
    Serial.print(" | Calidad=");
    Serial.println(txResults[i].quality);
    
    if (txResults[i].quality > bestQuality) {
      bestQuality = txResults[i].quality;
      bestTxIndex = i;
    }
  }
  
  uint8_t bestTX = txResults[bestTxIndex].txpower;
  
  Serial.println("-------------------------------------------");
  Serial.print(">>> MEJOR TX POWER: ");
  Serial.print(bestTX);
  Serial.print(" (Calidad: ");
  Serial.print(txResults[bestTxIndex].quality);
  Serial.println(")");
  Serial.println("===========================================\n");
  
  // Configurar con el mejor TX power
  LoRa.setTxPower(bestTX, PA_OUTPUT_PA_BOOST_PIN);
  currentTX = bestTX;
  
  delay(1000);
}

void TxFinished() {
  txDoneFlag = true;
}