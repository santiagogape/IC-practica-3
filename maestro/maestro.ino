/* ---------------------------------------------------------------------
 *  Implementación del Protocolo LoRa - MAESTRO
 *  Práctica 3 - Asignatura (GII-IoT)
 *  
 *  Protocolo definido en protocolo_lora.tex
 *  - Estados: SYNC, CALIBRATING, DATA
 *  - Mensajes: ACK, FORBIDDEN, MSG_SEND, SYNC_CHECK, SYNC_ATTEMPT, CALIBRATION, FINAL_CONFIG
 *  - Calibración automática de TxPower (3-20 dBm)
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
const uint8_t localAddress = 0x05;     // Dirección del Maestro
uint8_t destination = 0x06;            // Dirección del Esclavo

// Timeouts y reintentos (según protocolo)
#define SYNC_TIMEOUT_MS      120000    // 2 minutos sin mensajes -> resync
#define ACK_TIMEOUT_MS       3000      // Timeout para esperar ACK
#define MAX_RETRIES          3         // Número máximo de reintentos
#define TX_INTERVAL_MS       10000     // Intervalo entre transmisiones
#define CALIBRATION_INTERVAL_MS 2000   // Intervalo entre pruebas de calibración

// Tamaño máximo de payload
#define MAX_PAYLOAD_SIZE     50

// Rango de calibración de TxPower
#define TXPOWER_MIN          3
#define TXPOWER_MAX          20

// =====================================================================
// TIPOS DE MENSAJE (según protocolo)
// =====================================================================
#define MSG_TYPE_ACK         0x00      // 00 - Confirmación (incluye RSSI/SNR)
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

// Estructura para almacenar resultados de calibración
typedef struct {
  uint8_t txPower;
  int rssi;
  float snr;
  bool valid;
} CalibrationResult_t;

// Bandwidths válidos según protocolo
const uint32_t validBandwidths[] = {125000, 250000, 500000};
const int numValidBandwidths = 3;

// =====================================================================
// VARIABLES GLOBALES
// =====================================================================

// Estado del protocolo
ProtocolState_t currentState = STATE_SYNC;

// Configuración actual y pendiente
LoRaConfig_t currentConfig = {125000, 10, 5, 3};   // Config inicial con TxPower mínimo
LoRaConfig_t pendingConfig = {125000, 10, 5, 3};
LoRaConfig_t optimalConfig = {125000, 10, 5, 3};   // Config óptima encontrada

// Resultados de calibración
CalibrationResult_t calibrationResults[TXPOWER_MAX - TXPOWER_MIN + 1];
uint8_t currentCalibrationTxPower = TXPOWER_MIN;
uint8_t calibrationAttempts = 0;
bool calibrationComplete = false;

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

// RSSI y SNR reportados por el esclavo
int remoteRSSI = 0;
float remoteSNR = 0;

// =====================================================================
// PROTOTIPOS DE FUNCIONES
// =====================================================================
void setupLoRa();
void applyLoRaConfig(LoRaConfig_t* config);
void sendMessage(uint8_t* payload, uint8_t payloadLength);
void sendAck();
void sendForbidden();
void sendSyncAttempt();
void sendCalibrationProbe(uint8_t txPower);
void sendFinalConfig();
void sendMsgSend(const char* data, uint8_t len);
void onReceive(int packetSize);
void TxFinished();
void processReceivedMessage();
void handleSyncState();
void handleCalibrationState();
void handleDataState();
void checkSyncTimeout();
void initCalibration();
void processCalibrationAck();
void findOptimalTxPower();
bool validateSyncParams(uint32_t bw, uint8_t sf, uint8_t txPwr);
void printBinaryPayload(uint8_t* payload, uint8_t payloadLength);
void printConfig(LoRaConfig_t* config);
void printCalibrationResults();

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("========================================");
  Serial.println("   PROTOCOLO LORA - NODO MAESTRO");
  Serial.println("   Con calibracion de TxPower");
  Serial.println("========================================");
  Serial.print("Direccion local: 0x");
  Serial.println(localAddress, HEX);
  Serial.print("Direccion destino: 0x");
  Serial.println(destination, HEX);
  Serial.print("Rango TxPower: ");
  Serial.print(TXPOWER_MIN);
  Serial.print(" - ");
  Serial.print(TXPOWER_MAX);
  Serial.println(" dBm");

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
  syncRetries = 0;
  lastMessageTime_ms = millis();
  
  Serial.println("\n>> Estado inicial: SYNC");
  Serial.println(">> Esperando iniciar calibracion...\n");
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
      handleCalibrationState();
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
// INICIALIZAR CALIBRACIÓN
// =====================================================================
void initCalibration() {
  Serial.println("\n========================================");
  Serial.println("   INICIANDO CALIBRACION DE TxPower");
  Serial.println("========================================");
  
  // Resetear resultados de calibración
  for (int i = 0; i <= TXPOWER_MAX - TXPOWER_MIN; i++) {
    calibrationResults[i].txPower = TXPOWER_MIN + i;
    calibrationResults[i].rssi = -200;  // Valor inválido
    calibrationResults[i].snr = -50;    // Valor inválido
    calibrationResults[i].valid = false;
  }
  
  currentCalibrationTxPower = TXPOWER_MIN;
  calibrationAttempts = 0;
  calibrationComplete = false;
  currentState = STATE_CALIBRATING;
  txInterval_ms = CALIBRATION_INTERVAL_MS;
  
  Serial.print("Probando TxPower desde ");
  Serial.print(TXPOWER_MIN);
  Serial.print(" hasta ");
  Serial.print(TXPOWER_MAX);
  Serial.println(" dBm\n");
}

// =====================================================================
// MANEJO DE ESTADO SYNC
// =====================================================================
void handleSyncState() {
  // Si estamos esperando ACK del SyncAttempt inicial
  if (waitingForAck) {
    if (millis() - ackWaitStart_ms > ACK_TIMEOUT_MS) {
      Serial.println("!! Timeout esperando ACK de SyncAttempt");
      syncRetries++;
      
      if (syncRetries >= MAX_RETRIES) {
        Serial.println("!! Maximo de reintentos alcanzado");
        syncRetries = 0;
      }
      waitingForAck = false;
    }
    return;
  }

  // Enviar SyncAttempt inicial para establecer conexión
  if (!transmitting && (millis() - lastSendTime_ms > txInterval_ms)) {
    sendSyncAttempt();
    waitingForAck = true;
    ackWaitStart_ms = millis();
    pendingMsgType = MSG_TYPE_SYNC_ATTEMPT;
  }
}

// =====================================================================
// MANEJO DE ESTADO CALIBRACIÓN
// =====================================================================
void handleCalibrationState() {
  // Si estamos esperando ACK de la prueba de calibración
  if (waitingForAck) {
    if (millis() - ackWaitStart_ms > ACK_TIMEOUT_MS) {
      Serial.print("!! Timeout en calibracion TxPower=");
      Serial.println(currentCalibrationTxPower);
      
      calibrationAttempts++;
      if (calibrationAttempts >= MAX_RETRIES) {
        // Marcar como inválido y pasar al siguiente
        int idx = currentCalibrationTxPower - TXPOWER_MIN;
        calibrationResults[idx].valid = false;
        
        currentCalibrationTxPower++;
        calibrationAttempts = 0;
        
        if (currentCalibrationTxPower > TXPOWER_MAX) {
          // Calibración completa
          findOptimalTxPower();
        }
      }
      waitingForAck = false;
    }
    return;
  }

  // Enviar siguiente prueba de calibración
  if (!transmitting && (millis() - lastSendTime_ms > txInterval_ms)) {
    if (currentCalibrationTxPower <= TXPOWER_MAX) {
      sendCalibrationProbe(currentCalibrationTxPower);
      waitingForAck = true;
      ackWaitStart_ms = millis();
      pendingMsgType = MSG_TYPE_CALIBRATION;
    }
  }
}

// =====================================================================
// MANEJO DE ESTADO DATA
// =====================================================================
void handleDataState() {
  if (!transmitting && !waitingForAck && (millis() - lastSendTime_ms > txInterval_ms)) {
    char testMsg[32];
    snprintf(testMsg, sizeof(testMsg), "MSG#%d TxPwr=%d", msgCount, optimalConfig.txPower);
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
      Serial.println(">> Reiniciando calibracion...\n");
      currentState = STATE_SYNC;
      syncRetries = 0;
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

void sendSyncAttempt() {
  Serial.println("\n>> Enviando SYNC_ATTEMPT inicial");
  
  uint8_t payload[7];
  uint8_t idx = 0;
  
  payload[idx++] = ROLE_MASTER | MSG_TYPE_SYNC_ATTEMPT;
  
  // BW (4 bytes, little-endian)
  payload[idx++] = (uint8_t)(currentConfig.bandwidth & 0xFF);
  payload[idx++] = (uint8_t)((currentConfig.bandwidth >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((currentConfig.bandwidth >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((currentConfig.bandwidth >> 24) & 0xFF);
  
  // SF (1 byte)
  payload[idx++] = currentConfig.spreadingFactor;
  
  // TxPower inicial (1 byte)
  payload[idx++] = currentConfig.txPower;
  
  Serial.print("   BW=");
  Serial.print(currentConfig.bandwidth);
  Serial.print(" Hz, SF=");
  Serial.print(currentConfig.spreadingFactor);
  Serial.print(", TxPower=");
  Serial.print(currentConfig.txPower);
  Serial.println(" dBm");
  
  sendMessage(payload, idx);
  msgCount++;
}

void sendCalibrationProbe(uint8_t txPower) {
  Serial.print("\n>> Probando TxPower = ");
  Serial.print(txPower);
  Serial.println(" dBm");
  
  // Aplicar el TxPower de prueba
  LoRa.setTxPower(txPower, PA_OUTPUT_PA_BOOST_PIN);
  
  uint8_t payload[2];
  payload[0] = ROLE_MASTER | MSG_TYPE_CALIBRATION;
  payload[1] = txPower;
  
  sendMessage(payload, 2);
  msgCount++;
}

void sendFinalConfig() {
  Serial.println("\n>> Enviando CONFIGURACION FINAL");
  
  uint8_t payload[7];
  uint8_t idx = 0;
  
  payload[idx++] = ROLE_MASTER | MSG_TYPE_FINAL_CONFIG;
  
  // BW (4 bytes, little-endian)
  payload[idx++] = (uint8_t)(optimalConfig.bandwidth & 0xFF);
  payload[idx++] = (uint8_t)((optimalConfig.bandwidth >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((optimalConfig.bandwidth >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((optimalConfig.bandwidth >> 24) & 0xFF);
  
  // SF (1 byte)
  payload[idx++] = optimalConfig.spreadingFactor;
  
  // TxPower óptimo (1 byte)
  payload[idx++] = optimalConfig.txPower;
  
  Serial.println("   Configuracion optima:");
  printConfig(&optimalConfig);
  
  // Aplicar configuración óptima
  applyLoRaConfig(&optimalConfig);
  currentConfig = optimalConfig;
  
  sendMessage(payload, idx);
  msgCount++;
  
  waitingForAck = true;
  ackWaitStart_ms = millis();
  pendingMsgType = MSG_TYPE_FINAL_CONFIG;
}

void sendAck() {
  Serial.println(">> Enviando ACK");
  
  uint8_t payload[1];
  payload[0] = ROLE_MASTER | MSG_TYPE_ACK;
  
  sendMessage(payload, 1);
  msgCount++;
}

void sendMsgSend(const char* data, uint8_t len) {
  Serial.print(">> Enviando MSG_SEND: ");
  Serial.println(data);
  
  uint8_t payload[MAX_PAYLOAD_SIZE];
  payload[0] = ROLE_MASTER | MSG_TYPE_MSG_SEND;
  
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

    default:
      Serial.print("DESCONOCIDO (0x");
      Serial.print(msgType, HEX);
      Serial.println(")");
      break;
  }

  Serial.print("   RSSI local: ");
  Serial.print(rxRSSI);
  Serial.print(" dBm, SNR local: ");
  Serial.print(rxSNR, 1);
  Serial.println(" dB");
  Serial.println("----------------------------------------\n");
}

void handleReceivedAck() {
  // Extraer RSSI y SNR reportados por el esclavo (si están presentes)
  if (rxLength >= 5) {
    // El esclavo envía: Header(1) + RSSI(2, signed) + SNR(2, signed*10)
    int16_t reportedRSSI = (int16_t)((rxBuffer[2] << 8) | rxBuffer[1]);
    int16_t reportedSNR_x10 = (int16_t)((rxBuffer[4] << 8) | rxBuffer[3]);
    remoteRSSI = reportedRSSI;
    remoteSNR = reportedSNR_x10 / 10.0f;
    
    Serial.print("   RSSI remoto: ");
    Serial.print(remoteRSSI);
    Serial.print(" dBm, SNR remoto: ");
    Serial.print(remoteSNR, 1);
    Serial.println(" dB");
  }

  if (!waitingForAck) return;
  
  waitingForAck = false;
  syncRetries = 0;

  if (currentState == STATE_SYNC && pendingMsgType == MSG_TYPE_SYNC_ATTEMPT) {
    Serial.println("   >> ACK de SyncAttempt - Iniciando calibracion");
    initCalibration();
  }
  else if (currentState == STATE_CALIBRATING && pendingMsgType == MSG_TYPE_CALIBRATION) {
    processCalibrationAck();
  }
  else if (pendingMsgType == MSG_TYPE_FINAL_CONFIG) {
    Serial.println("   >> ACK de configuracion final!");
    Serial.println("   >> Transicion a estado DATA");
    currentState = STATE_DATA;
    txInterval_ms = TX_INTERVAL_MS;
  }
  else if (currentState == STATE_DATA && pendingMsgType == MSG_TYPE_MSG_SEND) {
    Serial.println("   >> ACK de MSG_SEND recibido!");
  }
}

void processCalibrationAck() {
  int idx = currentCalibrationTxPower - TXPOWER_MIN;
  
  // Guardar resultados usando el RSSI/SNR remoto (lo que el esclavo ve)
  calibrationResults[idx].txPower = currentCalibrationTxPower;
  calibrationResults[idx].rssi = remoteRSSI;
  calibrationResults[idx].snr = remoteSNR;
  calibrationResults[idx].valid = true;
  
  Serial.print("   >> Calibracion TxPower=");
  Serial.print(currentCalibrationTxPower);
  Serial.print(" -> RSSI=");
  Serial.print(remoteRSSI);
  Serial.print(" dBm, SNR=");
  Serial.print(remoteSNR, 1);
  Serial.println(" dB");
  
  // Siguiente TxPower
  currentCalibrationTxPower++;
  calibrationAttempts = 0;
  
  if (currentCalibrationTxPower > TXPOWER_MAX) {
    findOptimalTxPower();
  }
}

void findOptimalTxPower() {
  Serial.println("\n========================================");
  Serial.println("   RESULTADOS DE CALIBRACION");
  Serial.println("========================================");
  
  printCalibrationResults();
  
  // Buscar el TxPower óptimo
  // Criterio: mejor SNR primero, luego mejor RSSI como desempate
  // También consideramos eficiencia energética (preferir menor TxPower si calidad similar)
  
  int bestIdx = -1;
  float bestScore = -1000;
  
  for (int i = 0; i <= TXPOWER_MAX - TXPOWER_MIN; i++) {
    if (!calibrationResults[i].valid) continue;
    
    // Score combinado: SNR tiene más peso que RSSI
    // Penalizamos ligeramente TxPower alto para eficiencia energética
    float score = (calibrationResults[i].snr * 2.0f) + 
                  (calibrationResults[i].rssi + 100) * 0.5f -
                  (calibrationResults[i].txPower - TXPOWER_MIN) * 0.1f;
    
    if (score > bestScore) {
      bestScore = score;
      bestIdx = i;
    }
  }
  
  if (bestIdx >= 0) {
    optimalConfig.bandwidth = currentConfig.bandwidth;
    optimalConfig.spreadingFactor = currentConfig.spreadingFactor;
    optimalConfig.codingRate = currentConfig.codingRate;
    optimalConfig.txPower = calibrationResults[bestIdx].txPower;
    
    Serial.println("\n>> TxPower optimo encontrado:");
    Serial.print("   TxPower = ");
    Serial.print(optimalConfig.txPower);
    Serial.println(" dBm");
    Serial.print("   RSSI = ");
    Serial.print(calibrationResults[bestIdx].rssi);
    Serial.println(" dBm");
    Serial.print("   SNR = ");
    Serial.print(calibrationResults[bestIdx].snr, 1);
    Serial.println(" dB");
    
    // Enviar configuración final al esclavo
    sendFinalConfig();
  }
  else {
    Serial.println("!! No se encontro TxPower valido!");
    Serial.println(">> Usando TxPower por defecto (10 dBm)");
    optimalConfig.txPower = 10;
    sendFinalConfig();
  }
}

void handleReceivedForbidden() {
  Serial.println("   >> Mensaje rechazado por el esclavo");
  waitingForAck = false;
  
  if (currentState == STATE_SYNC) {
    syncRetries++;
  }
}

void handleReceivedMsgSend() {
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

void printCalibrationResults() {
  Serial.println("TxPower | RSSI (dBm) | SNR (dB) | Estado");
  Serial.println("--------|------------|----------|--------");
  
  for (int i = 0; i <= TXPOWER_MAX - TXPOWER_MIN; i++) {
    Serial.print("   ");
    if (calibrationResults[i].txPower < 10) Serial.print(" ");
    Serial.print(calibrationResults[i].txPower);
    Serial.print("   |    ");
    
    if (calibrationResults[i].valid) {
      if (calibrationResults[i].rssi > -100) Serial.print(" ");
      Serial.print(calibrationResults[i].rssi);
      Serial.print("    |   ");
      if (calibrationResults[i].snr >= 0) Serial.print(" ");
      Serial.print(calibrationResults[i].snr, 1);
      Serial.println("   |   OK");
    }
    else {
      Serial.println("   --     |    --    |  FAIL");
    }
  }
}
