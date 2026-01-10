/* ---------------------------------------------------------------------
 *  PROTOCOLO LORA - ESCLAVO
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
const uint8_t localAddress = 0x06;
const uint8_t destination = 0x05;

// =====================================================================
// TIMEOUTS
// =====================================================================
#define REVERT_TIMEOUT_MS    2000   // Si no llega PROBE, volver a safe
#define SYNC_TIMEOUT_MS      120000
#define TX_INTERVAL_MS       5000

// =====================================================================
// TIPOS DE MENSAJE
// =====================================================================
#define MSG_TYPE_ACK            0x00
#define MSG_TYPE_FORBIDDEN      0x01
#define MSG_TYPE_MSG_SEND       0x02
#define MSG_TYPE_SYNC_ATTEMPT   0x04
#define MSG_TYPE_TEST_CONFIG    0x05
#define MSG_TYPE_PROBE          0x06
#define MSG_TYPE_FINAL_CONFIG   0x07
#define MSG_TYPE_POWER_TEST     0x08

#define ROLE_SLAVE 0x00
#define MAX_PAYLOAD_SIZE 50

// =====================================================================
// ESTADOS
// =====================================================================
typedef enum {
  STATE_SYNC,
  STATE_CALIB,
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

// Safe Config: igual que maestro
LoRaConfig_t safeConfig = {125000, 10, 5, 20};
LoRaConfig_t currentConfig;
LoRaConfig_t testConfig;

// Estado
ProtocolState_t currentState = STATE_SYNC;

// Timer para revertir a safe config
bool pendingRevert = false;
uint32_t lastMsgTime = 0;
uint32_t lastSendTime = 0;

// Flags
volatile bool messageReceived = false;

// Buffer RX
uint8_t rxBuffer[MAX_PAYLOAD_SIZE];
uint8_t rxLength = 0;
uint8_t rxSender = 0;
uint8_t rxRecipient = 0;
int rxRSSI = 0;
float rxSNR = 0;

// =====================================================================
// PROTOTIPOS
// =====================================================================
void setupLoRa();
void applyConfig(LoRaConfig_t cfg);
void sendPacket(uint8_t* data, uint8_t len);
void onReceive(int packetSize);
void processMessage();
void sendAckWithMetrics();
void sendAck();
void sendForbidden();
void checkRevertTimeout();
void sendMsgSend(const char* msg);

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println(F("=== ESCLAVO LoRa - Calibracion Completa ==="));
  
  if (!init_PMIC()) {
    Serial.println(F("PMIC Error"));
  }
  
  setupLoRa();
  
  currentConfig = safeConfig;
  currentState = STATE_SYNC;
  lastMsgTime = millis();
  
  Serial.println(F("Estado: SYNC - Esperando maestro..."));
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  if (messageReceived) {
    processMessage();
    messageReceived = false;
  }
  
  checkRevertTimeout();
  
  // En estado DATA, podemos enviar mensajes periódicos
  if (currentState == STATE_DATA) {
    if (millis() - lastSendTime > TX_INTERVAL_MS * 2) {
      sendMsgSend("Hola desde Esclavo!");
      lastSendTime = millis();
    }
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
  lastMsgTime = millis();
}

// =====================================================================
// TIMEOUT PARA REVERTIR A SAFE CONFIG
// =====================================================================
void checkRevertTimeout() {
  if (pendingRevert && (millis() - lastMsgTime) > REVERT_TIMEOUT_MS) {
    Serial.println(F(">> Revert timeout -> Safe Config"));
    applyConfig(safeConfig);
    pendingRevert = false;
  }
}

// =====================================================================
// PROCESAMIENTO DE MENSAJES
// =====================================================================
void processMessage() {
  uint8_t msgType = rxBuffer[0] & 0x0F;
  
  Serial.print(F("<< RX Tipo: 0x")); Serial.print(msgType, HEX);
  Serial.print(F(" RSSI:")); Serial.print(rxRSSI);
  Serial.print(F(" SNR:")); Serial.println(rxSNR);
  
  switch (msgType) {
    case MSG_TYPE_SYNC_ATTEMPT:
      handleSyncAttempt();
      break;
      
    case MSG_TYPE_TEST_CONFIG:
      handleTestConfig();
      break;
      
    case MSG_TYPE_PROBE:
      handleProbe();
      break;
      
    case MSG_TYPE_POWER_TEST:
      handlePowerTest();
      break;
      
    case MSG_TYPE_FINAL_CONFIG:
      handleFinalConfig();
      break;
      
    case MSG_TYPE_MSG_SEND:
      handleMsgSend();
      break;
      
    case MSG_TYPE_ACK:
      Serial.println(F(">> ACK recibido"));
      break;
      
    default:
      Serial.print(F(">> Tipo desconocido: 0x"));
      Serial.println(msgType, HEX);
      break;
  }
}

// =====================================================================
// HANDLERS DE MENSAJES
// =====================================================================
void handleSyncAttempt() {
  Serial.println(F(">> SYNC_ATTEMPT recibido"));
  
  if (rxLength >= 8) {
    // Extraer config propuesta (debería ser safeConfig)
    uint32_t bw;
    memcpy(&bw, &rxBuffer[1], 4);
    uint8_t sf = rxBuffer[5];
    uint8_t cr = rxBuffer[6];
    uint8_t pwr = rxBuffer[7];
    
    Serial.print(F("   Config: BW=")); Serial.print(bw);
    Serial.print(F(" SF=")); Serial.print(sf);
    Serial.print(F(" CR=")); Serial.print(cr);
    Serial.print(F(" Pwr=")); Serial.println(pwr);
  }
  
  // Aplicar safe config y responder
  applyConfig(safeConfig);
  currentState = STATE_CALIB;
  pendingRevert = false;
  
  sendAckWithMetrics();
  Serial.println(F(">> Estado: CALIB"));
}

void handleTestConfig() {
  Serial.println(F(">> TEST_CONFIG recibido"));
  
  if (rxLength < 8) {
    sendForbidden();
    return;
  }
  
  // Extraer config a probar
  memcpy(&testConfig.bandwidth, &rxBuffer[1], 4);
  testConfig.spreadingFactor = rxBuffer[5];
  testConfig.codingRate = rxBuffer[6];
  testConfig.txPower = rxBuffer[7];
  
  Serial.print(F("   Test: BW=")); Serial.print(testConfig.bandwidth);
  Serial.print(F(" SF=")); Serial.print(testConfig.spreadingFactor);
  Serial.print(F(" CR=")); Serial.println(testConfig.codingRate);
  
  // PRIMERO: Enviar ACK en safe config (antes de cambiar!)
  sendAck();
  
  // Esperar a que el ACK se envíe completamente
  delay(100);
  
  // Cambiar a la config de prueba
  applyConfig(testConfig);
  Serial.println(F("   Cambiado a test config, esperando PROBE..."));
  
  // Armar watchdog: si no llegan mensajes en REVERT_TIMEOUT_MS volvemos a safe
  pendingRevert = true;
}

void handleProbe() {
  Serial.println(F(">> PROBE recibido"));
  
  // Cancelar revert timer
  pendingRevert = true;  // mantenemos watchdog: revertiremos solo si se queda en silencio
  
  // Responder con métricas (en la config de prueba actual)
  sendAckWithMetrics();
  // No revertimos inmediato: se hará solo si no llegan más mensajes en REVERT_TIMEOUT_MS
}

void handlePowerTest() {
  Serial.println(F(">> POWER_TEST recibido"));
  
  if (rxLength >= 2) {
    uint8_t testPwr = rxBuffer[1];
    Serial.print(F("   Power: ")); Serial.println(testPwr);
  }
  
  // Responder con métricas
  sendAckWithMetrics();
}

void handleFinalConfig() {
  Serial.println(F(">> FINAL_CONFIG recibido"));
  
  if (rxLength < 8) {
    sendForbidden();
    return;
  }
  
  // Extraer config final
  LoRaConfig_t finalCfg;
  memcpy(&finalCfg.bandwidth, &rxBuffer[1], 4);
  finalCfg.spreadingFactor = rxBuffer[5];
  finalCfg.codingRate = rxBuffer[6];
  finalCfg.txPower = rxBuffer[7];
  
  Serial.println(F("========== CONFIG FINAL =========="));
  Serial.print(F("  BW: ")); Serial.println(finalCfg.bandwidth);
  Serial.print(F("  SF: ")); Serial.println(finalCfg.spreadingFactor);
  Serial.print(F("  CR: ")); Serial.println(finalCfg.codingRate);
  Serial.print(F("  Pwr: ")); Serial.println(finalCfg.txPower);
  Serial.println(F("=================================="));
  
  // Aplicar config final
  applyConfig(finalCfg);
  currentState = STATE_DATA;
  pendingRevert = false;
  
  sendAck();
  Serial.println(F(">> Estado: DATA"));
}

void handleMsgSend() {
  Serial.println(F(">> MSG_SEND recibido"));
  
  if (rxLength > 1) {
    char msg[MAX_PAYLOAD_SIZE];
    uint8_t len = rxLength - 1;
    if (len > MAX_PAYLOAD_SIZE - 1) len = MAX_PAYLOAD_SIZE - 1;
    memcpy(msg, &rxBuffer[1], len);
    msg[len] = '\0';
    Serial.print(F("   Contenido: ")); Serial.println(msg);
  }
  
  sendAck();
}

// =====================================================================
// FUNCIONES DE ENVÍO
// =====================================================================
void sendAckWithMetrics() {
  Serial.println(F(">> TX ACK con metricas"));
  uint8_t p[5];
  p[0] = ROLE_SLAVE | MSG_TYPE_ACK;
  int16_t r = (int16_t)rxRSSI;
  int16_t s = (int16_t)(rxSNR * 10);
  p[1] = r & 0xFF; p[2] = (r >> 8) & 0xFF;
  p[3] = s & 0xFF; p[4] = (s >> 8) & 0xFF;
  sendPacket(p, 5);
}

void sendAck() {
  Serial.println(F(">> TX ACK"));
  uint8_t p[5];
  p[0] = ROLE_SLAVE | MSG_TYPE_ACK;
  int16_t r = (int16_t)rxRSSI;
  int16_t s = (int16_t)(rxSNR * 10);
  p[1] = r & 0xFF; p[2] = (r >> 8) & 0xFF;
  p[3] = s & 0xFF; p[4] = (s >> 8) & 0xFF;
  sendPacket(p, 5);
}

void sendForbidden() {
  Serial.println(F(">> TX FORBIDDEN"));
  uint8_t p[1] = { ROLE_SLAVE | MSG_TYPE_FORBIDDEN };
  sendPacket(p, 1);
}

void sendMsgSend(const char* msg) {
  uint8_t p[MAX_PAYLOAD_SIZE];
  p[0] = ROLE_SLAVE | MSG_TYPE_MSG_SEND;
  uint8_t len = strlen(msg);
  if (len > MAX_PAYLOAD_SIZE - 2) len = MAX_PAYLOAD_SIZE - 2;
  memcpy(&p[1], msg, len);
  sendPacket(p, len + 1);
}
