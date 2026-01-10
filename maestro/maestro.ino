/* ---------------------------------------------------------------------
 *  PROTOCOLO LORA - MAESTRO
 *  Calibración completa: BW, SF, CR, TxPower
 *  Mantiene sincronización usando Safe Config como fallback
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

// =====================================================================
// DIRECCIONES
// =====================================================================
const uint8_t localAddress = 0x05;
const uint8_t destination = 0x06;

// =====================================================================
// TIMEOUTS
// =====================================================================
#define ACK_TIMEOUT_MS       3000
#define SYNC_TIMEOUT_MS      120000
#define TX_INTERVAL_MS       5000
#define CALIB_INTERVAL_MS    1500

// =====================================================================
// TIPOS DE MENSAJE
// =====================================================================
#define MSG_TYPE_ACK            0x00
#define MSG_TYPE_FORBIDDEN      0x01
#define MSG_TYPE_MSG_SEND       0x02
#define MSG_TYPE_SYNC_ATTEMPT   0x04
#define MSG_TYPE_TEST_CONFIG    0x05  // Solicitar prueba de config
#define MSG_TYPE_PROBE          0x06  // Probe de conectividad
#define MSG_TYPE_FINAL_CONFIG   0x07  // Config final
#define MSG_TYPE_POWER_TEST     0x08  // Test de potencia

#define ROLE_MASTER 0x80
#define MAX_PAYLOAD_SIZE 50

// =====================================================================
// ESTADOS
// =====================================================================
typedef enum {
  STATE_SYNC,
  STATE_CALIB_MODULATION,
  STATE_CALIB_POWER,
  STATE_DATA
} ProtocolState_t;

// =====================================================================
// CONFIGURACIÓN
// =====================================================================
typedef struct {
  uint32_t bandwidth;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

// Resultado de prueba
typedef struct {
  LoRaConfig_t config;
  int rssi;
  float snr;
  uint32_t airTime;  // Estimación tiempo en aire
  bool valid;
} TestResult_t;

// Safe Config: robusta para sincronización
LoRaConfig_t safeConfig = {125000, 10, 5, 20};
LoRaConfig_t currentConfig;
LoRaConfig_t testConfig;
LoRaConfig_t bestModConfig;
LoRaConfig_t optimalConfig;

// Arrays de valores a probar
const uint32_t testBandwidths[] = {500000, 250000, 125000};
const uint8_t testSFs[] = {7, 8, 9, 10, 11, 12};
const uint8_t testCRs[] = {5, 6, 7, 8};
const uint8_t numBWs = 3;
const uint8_t numSFs = 6;
const uint8_t numCRs = 4;

#define TXPOWER_MIN 3
#define TXPOWER_MAX 20
#define RSSI_THRESHOLD -110
#define SNR_THRESHOLD -5.0

// Índices de calibración
uint8_t idxBW = 0, idxSF = 0, idxCR = 0;
uint8_t testTxPower = TXPOWER_MAX;

// Mejor resultado de modulación
TestResult_t bestResult;
bool foundValidModulation = false;

// Sub-estados de calibración
typedef enum {
  CALIB_SEND_TEST_REQ,
  CALIB_WAIT_TEST_ACK,
  CALIB_SWITCH_AND_PROBE,
  CALIB_WAIT_PROBE_ACK,
  CALIB_REVERT_SAFE
} CalibSubState_t;
CalibSubState_t calibSubState = CALIB_SEND_TEST_REQ;

// Estado global
ProtocolState_t currentState = STATE_SYNC;

// Flags y timers
volatile bool messageReceived = false;
bool waitingForAck = false;
uint32_t ackWaitStart = 0;
uint32_t lastSendTime = 0;
uint32_t probeStartTime = 0;
uint8_t lastCmdType = 0;

// Buffer RX
uint8_t rxBuffer[MAX_PAYLOAD_SIZE];
uint8_t rxLength = 0;
uint8_t rxSender = 0;
uint8_t rxRecipient = 0;
int rxRSSI = 0;
float rxSNR = 0;
int remoteRSSI = 0;
float remoteSNR = 0;

// =====================================================================
// PROTOTIPOS
// =====================================================================
void setupLoRa();
void applyConfig(LoRaConfig_t cfg);
void sendPacket(uint8_t* data, uint8_t len);
void onReceive(int packetSize);

void handleSync();
void handleCalibModulation();
void handleCalibPower();
void handleData();

void sendSyncAttempt();
void sendTestConfigReq(LoRaConfig_t cfg);
void sendProbe();
void sendPowerTest(uint8_t pwr);
void sendFinalConfig(LoRaConfig_t cfg);
void sendMsgSend(const char* msg);
void sendAck();

void processMessage();
void nextModulationTest();
void evaluateBestModulation();
float calcAirTimeScore(LoRaConfig_t cfg);

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println(F("=== MAESTRO LoRa - Calibracion Completa ==="));
  
  if (!init_PMIC()) {
    Serial.println(F("PMIC Error"));
  }
  
  setupLoRa();
  
  currentConfig = safeConfig;
  bestResult.valid = false;
  foundValidModulation = false;
  
  currentState = STATE_SYNC;
  lastSendTime = millis();
  
  Serial.println(F("Estado: SYNC"));
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  if (messageReceived) {
    processMessage();
    messageReceived = false;
  }
  
  switch (currentState) {
    case STATE_SYNC:
      handleSync();
      break;
    case STATE_CALIB_MODULATION:
      handleCalibModulation();
      break;
    case STATE_CALIB_POWER:
      handleCalibPower();
      break;
    case STATE_DATA:
      handleData();
      break;
  }
}

// =====================================================================
// CONFIGURACIÓN LORA
// =====================================================================
void setupLoRa() {
  if (!LoRa.begin(868E6)) {
    Serial.println(F("LoRa Error"));
    while (1);
  }
  applyConfig(safeConfig);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println(F("LoRa OK"));
}

void applyConfig(LoRaConfig_t cfg) {
  LoRa.setSignalBandwidth(cfg.bandwidth);
  LoRa.setSpreadingFactor(cfg.spreadingFactor);
  LoRa.setCodingRate4(cfg.codingRate);
  LoRa.setTxPower(cfg.txPower, PA_OUTPUT_PA_BOOST_PIN);
  currentConfig = cfg;
}

// =====================================================================
// ENVÍO DE PAQUETES (BLOQUEANTE)
// =====================================================================
void sendPacket(uint8_t* data, uint8_t len) {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(len);
  LoRa.write(data, len);
  LoRa.endPacket();  // Bloqueante
  LoRa.receive();
  lastSendTime = millis();
}

// =====================================================================
// RECEPCIÓN
// =====================================================================
void onReceive(int packetSize) {
  if (packetSize == 0) return;
  
  rxRecipient = LoRa.read();
  rxSender = LoRa.read();
  uint8_t len = LoRa.read();
  
  rxLength = 0;
  while (LoRa.available() && rxLength < MAX_PAYLOAD_SIZE) {
    rxBuffer[rxLength++] = LoRa.read();
  }
  
  rxRSSI = LoRa.packetRssi();
  rxSNR = LoRa.packetSnr();
  
  if (rxRecipient != localAddress) return;
  
  messageReceived = true;
}

// =====================================================================
// PROCESAMIENTO DE MENSAJES
// =====================================================================
void processMessage() {
  uint8_t msgType = rxBuffer[0] & 0x0F;
  
  Serial.print(F("<< RX Tipo: 0x")); Serial.print(msgType, HEX);
  Serial.print(F(" RSSI:")); Serial.print(rxRSSI);
  Serial.print(F(" SNR:")); Serial.println(rxSNR);
  
  // Extraer métricas remotas del ACK si las hay
  if (msgType == MSG_TYPE_ACK && rxLength >= 5) {
    remoteRSSI = (int16_t)((rxBuffer[2] << 8) | rxBuffer[1]);
    remoteSNR = ((int16_t)((rxBuffer[4] << 8) | rxBuffer[3])) / 10.0f;
    Serial.print(F("   Remote RSSI:")); Serial.print(remoteRSSI);
    Serial.print(F(" SNR:")); Serial.println(remoteSNR);
  }
  
  switch (msgType) {
    case MSG_TYPE_ACK: {
      waitingForAck = false;
      
      if (lastCmdType == MSG_TYPE_SYNC_ATTEMPT && currentState == STATE_SYNC) {
        Serial.println(F(">> SYNC OK - Iniciando calibracion modulacion"));
        currentState = STATE_CALIB_MODULATION;
        idxBW = 0; idxSF = 0; idxCR = 0;
        calibSubState = CALIB_SEND_TEST_REQ;
        bestResult.valid = false;
        foundValidModulation = false;
      }
      else if (currentState == STATE_CALIB_MODULATION) {
        if (lastCmdType == MSG_TYPE_TEST_CONFIG && calibSubState == CALIB_WAIT_TEST_ACK) {
          calibSubState = CALIB_SWITCH_AND_PROBE;
        }
        else if (lastCmdType == MSG_TYPE_PROBE && calibSubState == CALIB_WAIT_PROBE_ACK) {
          uint32_t probeTime = millis() - probeStartTime;
          
          Serial.print(F("   >> TEST OK: BW=")); Serial.print(testConfig.bandwidth);
          Serial.print(F(" SF=")); Serial.print(testConfig.spreadingFactor);
          Serial.print(F(" CR=")); Serial.print(testConfig.codingRate);
          Serial.print(F(" Time=")); Serial.println(probeTime);
          
          if (remoteRSSI > RSSI_THRESHOLD && remoteSNR > SNR_THRESHOLD) {
            float score = calcAirTimeScore(testConfig);
            float bestScore = bestResult.valid ? calcAirTimeScore(bestResult.config) : 9999999;
            
            if (score < bestScore) {
              bestResult.config = testConfig;
              bestResult.rssi = remoteRSSI;
              bestResult.snr = remoteSNR;
              bestResult.airTime = probeTime;
              bestResult.valid = true;
              foundValidModulation = true;
              Serial.println(F("   >> Nueva mejor config!"));
            }
          }
          calibSubState = CALIB_REVERT_SAFE;
        }
      }
      else if (currentState == STATE_CALIB_POWER) {
        if (lastCmdType == MSG_TYPE_POWER_TEST) {
          if (remoteRSSI > RSSI_THRESHOLD && remoteSNR > SNR_THRESHOLD) {
            Serial.print(F(">> Power ")); Serial.print(testTxPower);
            Serial.println(F(" dBm OK"));
            optimalConfig = bestModConfig;
            optimalConfig.txPower = testTxPower;
            sendFinalConfig(optimalConfig);
            waitingForAck = true;
            ackWaitStart = millis();
          } else {
            testTxPower++;
            if (testTxPower > TXPOWER_MAX) {
              optimalConfig = bestModConfig;
              optimalConfig.txPower = TXPOWER_MAX;
              sendFinalConfig(optimalConfig);
              waitingForAck = true;
              ackWaitStart = millis();
            }
          }
        } else if (lastCmdType == MSG_TYPE_FINAL_CONFIG) {
          Serial.println(F(">> FINAL_CONFIG ACK - Entrando a DATA"));
          currentState = STATE_DATA;
          waitingForAck = false;
          ackWaitStart = 0;
        }
      }
      else if (currentState == STATE_DATA) {
        Serial.println(F(">> ACK recibido en DATA"));
      }
      lastCmdType = 0;
      break;
    }
      
    case MSG_TYPE_FORBIDDEN:
      Serial.println(F(">> FORBIDDEN recibido"));
      waitingForAck = false;
      if (currentState == STATE_CALIB_MODULATION) {
        calibSubState = CALIB_REVERT_SAFE;
      }
      break;
      
    case MSG_TYPE_MSG_SEND:
      Serial.println(F(">> MSG recibido"));
      sendAck();
      break;
  }
}

// =====================================================================
// ESTADO: SYNC
// =====================================================================
void handleSync() {
  if (waitingForAck) {
    if (millis() - ackWaitStart > ACK_TIMEOUT_MS) {
      Serial.println(F("Sync timeout, reintentando..."));
      waitingForAck = false;
      lastCmdType = 0;
    }
    return;
  }
  
  if (millis() - lastSendTime > 2000) {
    sendSyncAttempt();
    waitingForAck = true;
    ackWaitStart = millis();
  }
}

// =====================================================================
// ESTADO: CALIBRACIÓN MODULACIÓN
// =====================================================================
void handleCalibModulation() {
  // Construir config de prueba actual
  testConfig.bandwidth = testBandwidths[idxBW];
  testConfig.spreadingFactor = testSFs[idxSF];
  testConfig.codingRate = testCRs[idxCR];
  testConfig.txPower = TXPOWER_MAX;
  
  switch (calibSubState) {
    case CALIB_SEND_TEST_REQ:
      // Asegurar que estamos en safe config
      applyConfig(safeConfig);
      delay(20);
      
      Serial.print(F(">> Test BW=")); Serial.print(testConfig.bandwidth);
      Serial.print(F(" SF=")); Serial.print(testConfig.spreadingFactor);
      Serial.print(F(" CR=")); Serial.println(testConfig.codingRate);
      
      sendTestConfigReq(testConfig);
      waitingForAck = true;
      ackWaitStart = millis();
      calibSubState = CALIB_WAIT_TEST_ACK;
      break;
      
    case CALIB_WAIT_TEST_ACK:
      if (millis() - ackWaitStart > ACK_TIMEOUT_MS) {
        Serial.println(F("   Test ACK timeout"));
        waitingForAck = false;
        lastCmdType = 0;
        calibSubState = CALIB_REVERT_SAFE;
      }
      break;
      
    case CALIB_SWITCH_AND_PROBE:
      // Cambiar a config de prueba
      delay(150);  // Dar tiempo al esclavo
      applyConfig(testConfig);
      delay(50);
      
      probeStartTime = millis();
      sendProbe();
      waitingForAck = true;
      ackWaitStart = millis();
      calibSubState = CALIB_WAIT_PROBE_ACK;
      break;
      
    case CALIB_WAIT_PROBE_ACK:
      if (millis() - ackWaitStart > ACK_TIMEOUT_MS) {
        Serial.println(F("   Probe timeout - config no valida"));
        waitingForAck = false;
        lastCmdType = 0;
        calibSubState = CALIB_REVERT_SAFE;
      }
      break;
      
    case CALIB_REVERT_SAFE:
      applyConfig(safeConfig);
      delay(100);
      nextModulationTest();
      break;
  }
}

void nextModulationTest() {
  idxCR++;
  if (idxCR >= numCRs) {
    idxCR = 0;
    idxSF++;
    if (idxSF >= numSFs) {
      idxSF = 0;
      idxBW++;
      if (idxBW >= numBWs) {
        // Terminamos todas las pruebas
        evaluateBestModulation();
        return;
      }
    }
  }
  calibSubState = CALIB_SEND_TEST_REQ;
}

void evaluateBestModulation() {
  Serial.println(F("\n========== RESULTADOS MODULACION =========="));
  
  if (foundValidModulation && bestResult.valid) {
    Serial.println(F("Mejor configuracion encontrada:"));
    Serial.print(F("  BW: ")); Serial.println(bestResult.config.bandwidth);
    Serial.print(F("  SF: ")); Serial.println(bestResult.config.spreadingFactor);
    Serial.print(F("  CR: ")); Serial.println(bestResult.config.codingRate);
    Serial.print(F("  RSSI: ")); Serial.println(bestResult.rssi);
    Serial.print(F("  SNR: ")); Serial.println(bestResult.snr);
    
    bestModConfig = bestResult.config;
    
    // Iniciar calibración de potencia
    Serial.println(F("\n>> Iniciando calibracion de potencia"));
    currentState = STATE_CALIB_POWER;
    testTxPower = TXPOWER_MIN;
    
    // Primero establecer la modulación óptima en el esclavo
    bestModConfig.txPower = TXPOWER_MAX;
    sendFinalConfig(bestModConfig);
    waitingForAck = true;
    ackWaitStart = millis();
    
  } else {
    Serial.println(F("No se encontro config valida, usando safe"));
    optimalConfig = safeConfig;
    sendFinalConfig(optimalConfig);
    waitingForAck = true;
    ackWaitStart = millis();
    currentState = STATE_DATA;
  }
}

// =====================================================================
// ESTADO: CALIBRACIÓN POTENCIA
// =====================================================================
void handleCalibPower() {
  if (waitingForAck) {
    if (millis() - ackWaitStart > ACK_TIMEOUT_MS) {
      if (lastCmdType == MSG_TYPE_POWER_TEST) {
        Serial.println(F("Power test timeout"));
        waitingForAck = false;
        lastCmdType = 0;
        testTxPower++;
        if (testTxPower > TXPOWER_MAX) {
          optimalConfig = bestModConfig;
          optimalConfig.txPower = TXPOWER_MAX;
          sendFinalConfig(optimalConfig);
          waitingForAck = true;
          ackWaitStart = millis();
        }
      } else if (lastCmdType == MSG_TYPE_FINAL_CONFIG) {
        Serial.println(F("Final config timeout, reenviando"));
        sendFinalConfig(optimalConfig);
        waitingForAck = true;
        ackWaitStart = millis();
      } else {
        waitingForAck = false;
        lastCmdType = 0;
      }
    }
    return;
  }
  
  // Ya tenemos la modulación óptima establecida
  // Probar con potencias crecientes desde el mínimo
  if (millis() - lastSendTime > CALIB_INTERVAL_MS) {
    if (testTxPower <= TXPOWER_MAX) {
      LoRaConfig_t pwrTest = bestModConfig;
      pwrTest.txPower = testTxPower;
      applyConfig(pwrTest);
      
      Serial.print(F(">> Test Power: ")); Serial.println(testTxPower);
      sendPowerTest(testTxPower);
      waitingForAck = true;
      ackWaitStart = millis();
    }
  }
}

// =====================================================================
// ESTADO: DATA
// =====================================================================
void handleData() {
  static bool configApplied = false;
  
  if (!configApplied) {
    applyConfig(optimalConfig);
    configApplied = true;
    Serial.println(F("\n========== ESTADO DATA =========="));
    Serial.print(F("Config final: BW=")); Serial.print(optimalConfig.bandwidth);
    Serial.print(F(" SF=")); Serial.print(optimalConfig.spreadingFactor);
    Serial.print(F(" CR=")); Serial.print(optimalConfig.codingRate);
    Serial.print(F(" Pwr=")); Serial.println(optimalConfig.txPower);
  }
  
  if (waitingForAck) {
    if (millis() - ackWaitStart > ACK_TIMEOUT_MS) {
      Serial.println(F("Data timeout"));
      waitingForAck = false;
      lastCmdType = 0;
    }
    return;
  }
  
  if (millis() - lastSendTime > TX_INTERVAL_MS) {
    sendMsgSend("Hola desde Maestro!");
    waitingForAck = true;
    ackWaitStart = millis();
  }
}

// =====================================================================
// FUNCIONES DE ENVÍO
// =====================================================================
void sendSyncAttempt() {
  Serial.println(F(">> TX SYNC_ATTEMPT"));
  uint8_t p[8];
  p[0] = ROLE_MASTER | MSG_TYPE_SYNC_ATTEMPT;
  memcpy(&p[1], &safeConfig.bandwidth, 4);
  p[5] = safeConfig.spreadingFactor;
  p[6] = safeConfig.codingRate;
  p[7] = safeConfig.txPower;
  lastCmdType = MSG_TYPE_SYNC_ATTEMPT;
  sendPacket(p, 8);
}

void sendTestConfigReq(LoRaConfig_t cfg) {
  uint8_t p[8];
  p[0] = ROLE_MASTER | MSG_TYPE_TEST_CONFIG;
  memcpy(&p[1], &cfg.bandwidth, 4);
  p[5] = cfg.spreadingFactor;
  p[6] = cfg.codingRate;
  p[7] = cfg.txPower;
  lastCmdType = MSG_TYPE_TEST_CONFIG;
  sendPacket(p, 8);
}

void sendProbe() {
  uint8_t p[1] = { ROLE_MASTER | MSG_TYPE_PROBE };
  lastCmdType = MSG_TYPE_PROBE;
  sendPacket(p, 1);
}

void sendPowerTest(uint8_t pwr) {
  uint8_t p[2];
  p[0] = ROLE_MASTER | MSG_TYPE_POWER_TEST;
  p[1] = pwr;
  lastCmdType = MSG_TYPE_POWER_TEST;
  sendPacket(p, 2);
}

void sendFinalConfig(LoRaConfig_t cfg) {
  Serial.println(F(">> TX FINAL_CONFIG"));
  uint8_t p[8];
  p[0] = ROLE_MASTER | MSG_TYPE_FINAL_CONFIG;
  memcpy(&p[1], &cfg.bandwidth, 4);
  p[5] = cfg.spreadingFactor;
  p[6] = cfg.codingRate;
  p[7] = cfg.txPower;
  lastCmdType = MSG_TYPE_FINAL_CONFIG;
  sendPacket(p, 8);
}

void sendMsgSend(const char* msg) {
  uint8_t p[MAX_PAYLOAD_SIZE];
  p[0] = ROLE_MASTER | MSG_TYPE_MSG_SEND;
  uint8_t len = strlen(msg);
  if (len > MAX_PAYLOAD_SIZE - 2) len = MAX_PAYLOAD_SIZE - 2;
  memcpy(&p[1], msg, len);
  lastCmdType = MSG_TYPE_MSG_SEND;
  sendPacket(p, len + 1);
}

void sendAck() {
  uint8_t p[5];
  p[0] = ROLE_MASTER | MSG_TYPE_ACK;
  int16_t r = (int16_t)rxRSSI;
  int16_t s = (int16_t)(rxSNR * 10);
  p[1] = r & 0xFF; p[2] = (r >> 8) & 0xFF;
  p[3] = s & 0xFF; p[4] = (s >> 8) & 0xFF;
  lastCmdType = MSG_TYPE_ACK;
  sendPacket(p, 5);
}

// =====================================================================
// CÁLCULO SCORE AIRTIME (menor es mejor)
// =====================================================================
float calcAirTimeScore(LoRaConfig_t cfg) {
  // Fórmula aproximada: AirTime proporcional a (2^SF) / BW
  // Menor score = más rápido = mejor
  float score = (float)(1UL << cfg.spreadingFactor) / (float)cfg.bandwidth;
  // Penalizar CR alto (más redundancia = más lento)
  score *= (float)(cfg.codingRate) / 5.0f;
  return score * 1000000.0f;  // Escalar para legibilidad
}
