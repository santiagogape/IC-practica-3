/* ---------------------------------------------------------------------
 *  Implementación del Protocolo LoRa - ESCLAVO
 *  Práctica 3 - Asignatura (GII-IoT)
 *  
 *  Protocolo definido en protocolo_lora.tex
 *  - Estados: SYNC, CALIBRATING, DATA
 *  - Mensajes: ACK, FORBIDDEN, MSG_SEND, SYNC_CHECK, SYNC_ATTEMPT, CALIBRATION, FINAL_CONFIG
 *  - Responde a calibración TxPower con RSSI/SNR
 *  
 *  Requiere librería Arduino LoRa modificada y Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

// =====================================================================
// CONFIGURACIÓN DEL PROTOCOLO
// =====================================================================

// Direcciones de red
const uint8_t localAddress = 0x06;     // Dirección del Esclavo
uint8_t destination = 0x05;            // Dirección del Maestro

// Timeouts y reintentos (según protocolo)
#define SYNC_TIMEOUT_MS      120000    // 2 minutos sin mensajes -> resync
#define ACK_TIMEOUT_MS       3000      // Timeout para esperar ACK
#define MAX_RETRIES          3         // Número máximo de reintentos
#define TX_INTERVAL_MS       10000     // Intervalo entre transmisiones

// Tamaño máximo de payload
#define MAX_PAYLOAD_SIZE     50

// =====================================================================
// TIPOS DE MENSAJE (según protocolo)
// =====================================================================
#define MSG_TYPE_ACK         0x00      // 00 - Confirmación
#define MSG_TYPE_FORBIDDEN   0x01      // 01 - Mensaje rechazado
#define MSG_TYPE_MSG_SEND    0x02      // 10 - Envío de datos
#define MSG_TYPE_SYNC_CHECK  0x03      // 11 - Verificación de sync
#define MSG_TYPE_SYNC_ATTEMPT 0x04     // Sincronización inicial
#define MSG_TYPE_CALIBRATION 0x05      // Mensaje de calibración TxPower
#define MSG_TYPE_FINAL_CONFIG 0x06     // Configuración final tras calibración

// Bit de rol (Master = 1, Slave = 0)
#define ROLE_MASTER          0x80
#define ROLE_SLAVE           0x00

// =====================================================================
// ESTADOS DEL PROTOCOLO
// =====================================================================
typedef enum {
  STATE_SYNC,           // Estado inicial - esperar conexión
  STATE_CALIBRATING,    // Calibrando TxPower
  STATE_DATA            // Estado operacional
} ProtocolState_t;

// =====================================================================
// ESTRUCTURA DE CONFIGURACIÓN LORA
// =====================================================================
typedef struct {
  uint32_t bandwidth;        // Ancho de banda en Hz
  uint8_t spreadingFactor;   // Factor de dispersión [6-12]
  uint8_t codingRate;        // Coding rate [5-8]
  uint8_t txPower;           // Potencia de TX [2-20] dBm
} LoRaConfig_t;

// Bandwidths válidos según protocolo
const uint32_t validBandwidths[] = {125000, 250000, 500000};
const int numValidBandwidths = 3;

// =====================================================================
// VARIABLES GLOBALES
// =====================================================================

// Estado del protocolo
ProtocolState_t currentState = STATE_SYNC;

// Configuración actual
LoRaConfig_t currentConfig = {125000, 10, 5, 3};   // Config inicial con TxPower mínimo

// Flags de transmisión
volatile bool txDoneFlag = true;
volatile bool transmitting = false;

// Contadores y timers
uint16_t msgCount = 0;
uint32_t lastMessageTime_ms = 0;
uint32_t lastSendTime_ms = 0;
uint32_t txBegin_ms = 0;
uint32_t txInterval_ms = TX_INTERVAL_MS;

// Control de sincronización
uint8_t syncRetries = 0;
bool waitingForAck = false;
uint32_t ackWaitStart_ms = 0;
uint8_t pendingMsgType = 0;

// Buffer de recepción
volatile bool messageReceived = false;
uint8_t rxBuffer[MAX_PAYLOAD_SIZE + 10];
uint8_t rxLength = 0;
uint8_t rxSender = 0;
uint8_t rxRecipient = 0;
int rxRSSI = 0;
float rxSNR = 0;

// TxPower del mensaje de calibración recibido (para respuesta)
uint8_t lastCalibrationTxPower = 0;

// =====================================================================
// PROTOTIPOS DE FUNCIONES
// =====================================================================
void setupLoRa();
void applyLoRaConfig(LoRaConfig_t* config);
void sendMessage(uint8_t* payload, uint8_t payloadLength);
void sendAckWithMetrics();
void sendAck();
void sendForbidden();
void sendMsgSend(const char* data, uint8_t len);
void onReceive(int packetSize);
void TxFinished();
void processReceivedMessage();
void handleSyncState();
void handleDataState();
void checkSyncTimeout();
bool validateSyncParams(uint32_t bw, uint8_t sf, uint8_t txPwr);
void printBinaryPayload(uint8_t* payload, uint8_t payloadLength);
void printConfig(LoRaConfig_t* config);

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("========================================");
  Serial.println("   PROTOCOLO LORA - NODO ESCLAVO");
  Serial.println("   Con soporte calibracion TxPower");
  Serial.println("========================================");
  Serial.print("Direccion local: 0x");
  Serial.println(localAddress, HEX);
  Serial.print("Direccion destino: 0x");
  Serial.println(destination, HEX);

  // Inicializar PMIC
  if (!init_PMIC()) {
    Serial.println("ERROR: Inicializacion de BQ24195L fallida!");
  } else {
    Serial.println("OK: BQ24195L inicializado");
  }

  // Inicializar LoRa
  setupLoRa();
  
  // Iniciar en estado SYNC
  currentState = STATE_SYNC;
  lastMessageTime_ms = millis();
  
  Serial.println("\n>> Estado inicial: SYNC");
  Serial.println(">> Esperando SyncAttempt del maestro...\n");
}

// =====================================================================
// LOOP PRINCIPAL
// =====================================================================
void loop() {
  // Procesar mensaje recibido si hay uno pendiente
  if (messageReceived) {
    processReceivedMessage();
    messageReceived = false;
  }

  // Verificar timeout de sincronización
  checkSyncTimeout();

  // Máquina de estados
  switch (currentState) {
    case STATE_SYNC:
      handleSyncState();
      break;
    case STATE_CALIBRATING:
      // El esclavo solo responde en calibración, no inicia
      break;
    case STATE_DATA:
      handleDataState();
      break;
  }

  // Manejar finalización de transmisión
  if (transmitting && txDoneFlag) {
    uint32_t txTime_ms = millis() - txBegin_ms;
    Serial.print(">> TX completado en ");
    Serial.print(txTime_ms);
    Serial.println(" ms");

    // Ajustar intervalo para duty cycle del 1%
    uint32_t lapse_ms = txBegin_ms - lastSendTime_ms;
    lastSendTime_ms = txBegin_ms;
    if (lapse_ms > 0) {
      float dutyCycle = (100.0f * txTime_ms) / lapse_ms;
      if (dutyCycle > 1.0f) {
        txInterval_ms = txTime_ms * 100;
      }
    }

    transmitting = false;
    LoRa.receive();
  }
}

// =====================================================================
// CONFIGURACIÓN LORA
// =====================================================================
void setupLoRa() {
  if (!LoRa.begin(868E6)) {
    Serial.println("ERROR: LoRa init fallido!");
    while (true);
  }

  applyLoRaConfig(&currentConfig);
  
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);
  LoRa.receive();

  Serial.println("OK: LoRa inicializado a 868 MHz");
  printConfig(&currentConfig);
}

void applyLoRaConfig(LoRaConfig_t* config) {
  LoRa.setSignalBandwidth(config->bandwidth);
  LoRa.setSpreadingFactor(config->spreadingFactor);
  LoRa.setCodingRate4(config->codingRate);
  LoRa.setTxPower(config->txPower, PA_OUTPUT_PA_BOOST_PIN);
}

// =====================================================================
// MANEJO DE ESTADO SYNC
// =====================================================================
void handleSyncState() {
  // El esclavo solo espera mensajes del maestro en estado SYNC
  // No hace nada activamente
}

// =====================================================================
// MANEJO DE ESTADO DATA
// =====================================================================
void handleDataState() {
  // El esclavo puede enviar datos periódicamente si lo desea
  if (!transmitting && !waitingForAck && (millis() - lastSendTime_ms > txInterval_ms)) {
    char testMsg[32];
    snprintf(testMsg, sizeof(testMsg), "SLAVE#%d TxPwr=%d", msgCount, currentConfig.txPower);
    sendMsgSend(testMsg, strlen(testMsg));
  }

  if (waitingForAck && (millis() - ackWaitStart_ms > ACK_TIMEOUT_MS)) {
    Serial.println("!! Timeout esperando ACK de MSG_SEND");
    waitingForAck = false;
    syncRetries++;
    
    if (syncRetries >= MAX_RETRIES) {
      Serial.println(">> Volviendo a estado SYNC por falta de ACK\n");
      currentState = STATE_SYNC;
      syncRetries = 0;
      txInterval_ms = TX_INTERVAL_MS;
    }
  }
}

// =====================================================================
// VERIFICAR TIMEOUT DE SINCRONIZACIÓN
// =====================================================================
void checkSyncTimeout() {
  if (currentState == STATE_DATA) {
    if (millis() - lastMessageTime_ms > SYNC_TIMEOUT_MS) {
      Serial.println("\n!! Timeout de sincronizacion (2 min sin mensajes)");
      Serial.println(">> Volviendo a estado SYNC...\n");
      currentState = STATE_SYNC;
      waitingForAck = false;
      txInterval_ms = TX_INTERVAL_MS;
    }
  }
}

// =====================================================================
// ENVÍO DE MENSAJES
// =====================================================================
void sendMessage(uint8_t* payload, uint8_t payloadLength) {
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(payloadLength);
  LoRa.write(payload, payloadLength);
  
  transmitting = true;
  txDoneFlag = false;
  txBegin_ms = millis();
  
  LoRa.endPacket(true);
}

// ACK con métricas de señal (RSSI y SNR) - usado durante calibración
void sendAckWithMetrics() {
  Serial.println(">> Enviando ACK con metricas");
  Serial.print("   RSSI medido: ");
  Serial.print(rxRSSI);
  Serial.print(" dBm, SNR medido: ");
  Serial.print(rxSNR, 1);
  Serial.println(" dB");
  
  uint8_t payload[5];
  payload[0] = ROLE_SLAVE | MSG_TYPE_ACK;
  
  // RSSI como int16_t (little-endian)
  int16_t rssiVal = (int16_t)rxRSSI;
  payload[1] = (uint8_t)(rssiVal & 0xFF);
  payload[2] = (uint8_t)((rssiVal >> 8) & 0xFF);
  
  // SNR * 10 como int16_t (little-endian)
  int16_t snrVal = (int16_t)(rxSNR * 10);
  payload[3] = (uint8_t)(snrVal & 0xFF);
  payload[4] = (uint8_t)((snrVal >> 8) & 0xFF);
  
  sendMessage(payload, 5);
  msgCount++;
}

// ACK simple sin métricas
void sendAck() {
  Serial.println(">> Enviando ACK simple");
  
  uint8_t payload[1];
  payload[0] = ROLE_SLAVE | MSG_TYPE_ACK;
  
  sendMessage(payload, 1);
  msgCount++;
}

void sendForbidden() {
  Serial.println(">> Enviando FORBIDDEN");
  
  uint8_t payload[1];
  payload[0] = ROLE_SLAVE | MSG_TYPE_FORBIDDEN;
  
  sendMessage(payload, 1);
  msgCount++;
}

void sendMsgSend(const char* data, uint8_t len) {
  Serial.print(">> Enviando MSG_SEND: ");
  Serial.println(data);
  
  uint8_t payload[MAX_PAYLOAD_SIZE];
  payload[0] = ROLE_SLAVE | MSG_TYPE_MSG_SEND;
  
  uint8_t copyLen = min(len, (uint8_t)(MAX_PAYLOAD_SIZE - 1));
  memcpy(&payload[1], data, copyLen);
  
  sendMessage(payload, copyLen + 1);
  msgCount++;
  
  waitingForAck = true;
  ackWaitStart_ms = millis();
  pendingMsgType = MSG_TYPE_MSG_SEND;
}

// =====================================================================
// RECEPCIÓN DE MENSAJES (Callback)
// =====================================================================
void onReceive(int packetSize) {
  if (transmitting && !txDoneFlag) txDoneFlag = true;
  if (packetSize == 0) return;

  rxRecipient = LoRa.read();
  rxSender = LoRa.read();
  uint8_t msgSize = LoRa.read();

  rxLength = 0;
  while (LoRa.available() && rxLength < sizeof(rxBuffer)) {
    rxBuffer[rxLength++] = LoRa.read();
  }

  rxRSSI = LoRa.packetRssi();
  rxSNR = LoRa.packetSnr();

  if ((rxRecipient & localAddress) != localAddress) {
    return;
  }

  if (msgSize != rxLength) {
    Serial.println("!! Error: longitud de mensaje no coincide");
    return;
  }

  messageReceived = true;
  lastMessageTime_ms = millis();
}

// =====================================================================
// PROCESAMIENTO DE MENSAJES RECIBIDOS
// =====================================================================
void processReceivedMessage() {
  if (rxLength == 0) return;

  uint8_t header = rxBuffer[0];
  uint8_t msgType = header & 0x0F;

  Serial.println("\n----------------------------------------");
  Serial.print("<< Mensaje recibido de 0x");
  Serial.println(rxSender, HEX);
  Serial.print("   Tipo: ");

  switch (msgType) {
    case MSG_TYPE_ACK:
      Serial.println("ACK");
      handleReceivedAck();
      break;

    case MSG_TYPE_FORBIDDEN:
      Serial.println("FORBIDDEN");
      handleReceivedForbidden();
      break;

    case MSG_TYPE_MSG_SEND:
      Serial.println("MSG_SEND");
      handleReceivedMsgSend();
      break;

    case MSG_TYPE_SYNC_ATTEMPT:
      Serial.println("SYNC_ATTEMPT");
      handleReceivedSyncAttempt();
      break;

    case MSG_TYPE_CALIBRATION:
      Serial.println("CALIBRATION");
      handleReceivedCalibration();
      break;

    case MSG_TYPE_FINAL_CONFIG:
      Serial.println("FINAL_CONFIG");
      handleReceivedFinalConfig();
      break;

    default:
      Serial.print("DESCONOCIDO (0x");
      Serial.print(msgType, HEX);
      Serial.println(")");
      break;
  }

  Serial.print("   RSSI: ");
  Serial.print(rxRSSI);
  Serial.print(" dBm, SNR: ");
  Serial.print(rxSNR, 1);
  Serial.println(" dB");
  Serial.println("----------------------------------------\n");
}

void handleReceivedAck() {
  if (!waitingForAck) return;
  
  waitingForAck = false;
  syncRetries = 0;

  if (currentState == STATE_DATA && pendingMsgType == MSG_TYPE_MSG_SEND) {
    Serial.println("   >> ACK de MSG_SEND recibido!");
  }
}

void handleReceivedForbidden() {
  Serial.println("   >> Mensaje rechazado por el maestro");
  waitingForAck = false;
}

void handleReceivedMsgSend() {
  if (currentState != STATE_DATA) {
    Serial.println("   >> No en estado DATA, enviando FORBIDDEN");
    sendForbidden();
    return;
  }

  if (rxLength > 1) {
    char msg[MAX_PAYLOAD_SIZE];
    uint8_t msgLen = rxLength - 1;
    memcpy(msg, &rxBuffer[1], msgLen);
    msg[msgLen] = '\0';
    
    Serial.print("   Contenido: ");
    Serial.println(msg);
  }
  
  sendAck();
}

void handleReceivedSyncAttempt() {
  Serial.println("   >> Procesando SYNC_ATTEMPT");
  
  if (rxLength < 7) {
    Serial.println("   !! Payload insuficiente");
    sendForbidden();
    return;
  }

  // Extraer parámetros (según protocolo)
  // BW (4 bytes, little-endian)
  uint32_t proposedBW = (uint32_t)rxBuffer[1] |
                        ((uint32_t)rxBuffer[2] << 8) |
                        ((uint32_t)rxBuffer[3] << 16) |
                        ((uint32_t)rxBuffer[4] << 24);
  
  // SF (1 byte)
  uint8_t proposedSF = rxBuffer[5];
  
  // TxPower (1 byte)
  uint8_t proposedTxPower = rxBuffer[6];

  Serial.print("   Propuesta: BW=");
  Serial.print(proposedBW);
  Serial.print(" Hz, SF=");
  Serial.print(proposedSF);
  Serial.print(", TxPower=");
  Serial.print(proposedTxPower);
  Serial.println(" dBm");

  // Validar parámetros
  if (!validateSyncParams(proposedBW, proposedSF, proposedTxPower)) {
    Serial.println("   !! Parametros invalidos");
    sendForbidden();
    return;
  }

  // Aplicar configuración inicial
  currentConfig.bandwidth = proposedBW;
  currentConfig.spreadingFactor = proposedSF;
  currentConfig.txPower = proposedTxPower;
  applyLoRaConfig(&currentConfig);

  Serial.println("   >> Configuracion inicial aceptada");
  Serial.println("   >> Transicion a estado CALIBRATING");
  currentState = STATE_CALIBRATING;
  
  // Enviar ACK con métricas
  sendAckWithMetrics();
}

void handleReceivedCalibration() {
  Serial.println("   >> Procesando mensaje de CALIBRACION");
  
  if (rxLength < 2) {
    Serial.println("   !! Payload insuficiente");
    sendForbidden();
    return;
  }

  // Extraer TxPower del mensaje de calibración
  lastCalibrationTxPower = rxBuffer[1];
  
  Serial.print("   TxPower de prueba: ");
  Serial.print(lastCalibrationTxPower);
  Serial.println(" dBm");

  // Responder con ACK incluyendo RSSI y SNR medidos
  sendAckWithMetrics();
}

void handleReceivedFinalConfig() {
  Serial.println("   >> Procesando CONFIGURACION FINAL");
  
  if (rxLength < 7) {
    Serial.println("   !! Payload insuficiente");
    sendForbidden();
    return;
  }

  // Extraer parámetros finales (mismo formato que SYNC_ATTEMPT)
  uint32_t finalBW = (uint32_t)rxBuffer[1] |
                     ((uint32_t)rxBuffer[2] << 8) |
                     ((uint32_t)rxBuffer[3] << 16) |
                     ((uint32_t)rxBuffer[4] << 24);
  
  uint8_t finalSF = rxBuffer[5];
  uint8_t finalTxPower = rxBuffer[6];

  Serial.println("\n========================================");
  Serial.println("   CONFIGURACION OPTIMA RECIBIDA");
  Serial.println("========================================");
  Serial.print("   BW: ");
  Serial.print(finalBW);
  Serial.println(" Hz");
  Serial.print("   SF: ");
  Serial.println(finalSF);
  Serial.print("   TxPower: ");
  Serial.print(finalTxPower);
  Serial.println(" dBm");
  Serial.println("========================================\n");

  // Validar parámetros
  if (!validateSyncParams(finalBW, finalSF, finalTxPower)) {
    Serial.println("   !! Parametros invalidos");
    sendForbidden();
    return;
  }

  // Aplicar configuración óptima
  currentConfig.bandwidth = finalBW;
  currentConfig.spreadingFactor = finalSF;
  currentConfig.txPower = finalTxPower;
  applyLoRaConfig(&currentConfig);

  Serial.println("   >> Configuracion optima aplicada!");
  Serial.println("   >> Transicion a estado DATA");
  currentState = STATE_DATA;
  txInterval_ms = TX_INTERVAL_MS;
  
  // Enviar ACK simple (ya no necesita métricas)
  sendAck();
}

// =====================================================================
// FUNCIONES AUXILIARES
// =====================================================================
bool validateSyncParams(uint32_t bw, uint8_t sf, uint8_t txPwr) {
  bool validBW = false;
  for (int i = 0; i < numValidBandwidths; i++) {
    if (bw == validBandwidths[i]) {
      validBW = true;
      break;
    }
  }
  if (!validBW) return false;
  if (sf < 6 || sf > 12) return false;
  if (txPwr < 2 || txPwr > 20) return false;
  return true;
}

void TxFinished() {
  txDoneFlag = true;
}

void printBinaryPayload(uint8_t* payload, uint8_t payloadLength) {
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}

void printConfig(LoRaConfig_t* config) {
  Serial.println("   Configuracion LoRa:");
  Serial.print("   - BW: ");
  Serial.print(config->bandwidth);
  Serial.println(" Hz");
  Serial.print("   - SF: ");
  Serial.println(config->spreadingFactor);
  Serial.print("   - CR: ");
  Serial.println(config->codingRate);
  Serial.print("   - TxPower: ");
  Serial.print(config->txPower);
  Serial.println(" dBm");
}
