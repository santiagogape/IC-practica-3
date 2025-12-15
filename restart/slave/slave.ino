/* ---------------------------------------------------------------------
 *  ESCLAVO Completo - Sistema de Sincronización Avanzado
 *  Práctica 3 - GII-IoT
 *  
 *  Fases de operación:
 *  1. Responder a sincronización TxPower
 *  2. Responder a sincronización BW/SF
 *  3. Aplicar mejor configuración recibida
 *  4. Responder a Health checks
 * ---------------------------------------------------------------------
 */
#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS 1000
#define MAX_RETRIES 3
#define TIMEOUT_MS 5000

// NOTA: Ajustar estas variables
#define LOCALADDRESS 0x05
#define DESTINATION 0x06
#define START_TX 2
#define START_BW 125000
#define START_SF 7

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

TestResult receivedResults[20];
BwSfResult bwSfReceivedResults[10];
uint8_t resultCount = 0;
uint8_t bwSfResultCount = 0;

enum SyncState {
  IDLE,
  SYNC_INIT,
  SYNC_WAITT,
  SA_AFTER_T,
  SA_AFTER_BWSF,
  SYNC_T_FINAL_SA,
  HEALTH_CHECK_MODE
};

volatile SyncState syncState;
volatile uint8_t next_tx = 0;
volatile long next_bw = 0;
volatile uint8_t next_sf = 0;
volatile bool txDoneFlag = true;
volatile uint8_t retryCount = 0;
volatile uint32_t lastRxTime = 0;

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== LoRa Esclavo - Sistema Completo de Sincronización ===\n");

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
  for (uint8_t i = 0; i < 20; i++) {
    receivedResults[i].txPower = 0;
    receivedResults[i].rssi = 0;
    receivedResults[i].snr = 0.0;
    receivedResults[i].valid = false;
  }
  
  for (uint8_t i = 0; i < 10; i++) {
    bwSfReceivedResults[i].bandwidth = 0;
    bwSfReceivedResults[i].spreadingFactor = 0;
    bwSfReceivedResults[i].snr = 0.0;
    bwSfReceivedResults[i].txTime = 0;
    bwSfReceivedResults[i].valid = false;
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

  // Estado: SYNC_T_FINAL_SA - Responder después de SE
  if (syncState == SYNC_T_FINAL_SA && !transmitting && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    char message[50];
    snprintf(message, sizeof(message), "SA");

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(message, uint8_t(strlen(message)), msgCount);
    Serial.println("\nEnviando SA final (después de SE)");
    
    syncState = IDLE;
    retryCount = 0;
  }

  // Estado: SA_AFTER_BWSF - Responder después de BW/SF test
  if (syncState == SA_AFTER_BWSF && !transmitting && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    char message[50];
    snprintf(message, sizeof(message), "SA");

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(message, uint8_t(strlen(message)), msgCount);
    Serial.print("Enviando SA (BW=");
    Serial.print(next_bw);
    Serial.print(", SF=");
    Serial.print(next_sf);
    Serial.println(")");
    
    // Aplicar nueva configuración BW/SF
    LoRa.setSignalBandwidth(next_bw);
    LoRa.setSpreadingFactor(next_sf);
    Serial.print("Configuración aplicada: BW=");
    Serial.print(next_bw);
    Serial.print(" Hz, SF=");
    Serial.println(next_sf);
    
    syncState = SYNC_WAITT;
    retryCount = 0;
  }

  // Estado: SA_AFTER_T - Responder después de TxPower test
  if (syncState == SA_AFTER_T && !transmitting && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    char message[50];
    snprintf(message, sizeof(message), "SA");

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(message, uint8_t(strlen(message)), msgCount);
    Serial.print("Enviando SA (TxPower=");
    Serial.print(next_tx);
    Serial.println(" dBm)");
    
    // Aplicar nueva potencia de transmisión
    LoRa.setTxPower(next_tx, PA_OUTPUT_PA_BOOST_PIN);
    Serial.print("TxPower ajustado a: ");
    Serial.print(next_tx);
    Serial.println(" dBm");
    
    syncState = SYNC_WAITT;
    retryCount = 0;
  }

  // Estado: SYNC_INIT - Primera respuesta SA después de ST
  if (syncState == SYNC_INIT && !transmitting && 
      ((millis() - lastSendTime_ms) > txInterval_ms)) {
    
    char message[50];
    snprintf(message, sizeof(message), "SA");

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(message, uint8_t(strlen(message)), msgCount);
    Serial.println("\nEnviando SA inicial (después de ST)");
    
    syncState = SYNC_WAITT;
    retryCount = 0;
  }

  // Transmisión completada
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");

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

  lastRxTime = millis();

  int16_t rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  // Health Check Request
  if (strcmp(buffer, "HC") == 0) {
    Serial.print("[Health Check] Recibido - RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.println(snr);
    
    // Responder con HCR
    static uint16_t hcMsgCount = 0;
    char hcResponse[50];
    snprintf(hcResponse, sizeof(hcResponse), "HCR");
    sendMessage(hcResponse, uint8_t(strlen(hcResponse)), hcMsgCount);
    Serial.println("[Health Check] Enviando HCR");
    return;
  }

  Serial.println("\n--- Mensaje Recibido ---");
  Serial.print("From: 0x");
  Serial.println(sender, HEX);
  Serial.print("Message: ");
  Serial.println(buffer);
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm, SNR: ");
  Serial.println(snr);
  Serial.println("------------------------\n");

  // Procesar mensaje ST (Start)
  if (strcmp(buffer, "ST") == 0) {
    Serial.println(">>> SINCRONIZACIÓN INICIADA (ST)");
    syncState = SYNC_INIT;
    retryCount = 0;
    resultCount = 0;
    bwSfResultCount = 0;
    
    // Guardar primer resultado
    if (resultCount < 20) {
      receivedResults[resultCount].txPower = START_TX;
      receivedResults[resultCount].rssi = rssi;
      receivedResults[resultCount].snr = snr;
      receivedResults[resultCount].valid = true;
      resultCount++;
    }
  }
  // Procesar mensaje T (TxPower test)
  else if (buffer[0] == 'T' && strlen(buffer) <= 4) {
    uint8_t tx = (uint8_t)atoi(&buffer[1]);
    Serial.print(">>> TEST TXPOWER: ");
    Serial.print(tx);
    Serial.println(" dBm");
    
    next_tx = tx;
    syncState = SA_AFTER_T;
    retryCount = 0;
    
    // Guardar resultado
    if (resultCount < 20) {
      receivedResults[resultCount].txPower = tx;
      receivedResults[resultCount].rssi = rssi;
      receivedResults[resultCount].snr = snr;
      receivedResults[resultCount].valid = true;
      resultCount++;
      
      Serial.print("Guardado [");
      Serial.print(resultCount - 1);
      Serial.print("]: TX=");
      Serial.print(tx);
      Serial.print(", RSSI=");
      Serial.print(rssi);
      Serial.print(", SNR=");
      Serial.println(snr);
    }
  }
  // Procesar mensaje BW/SF (Bandwidth/Spreading Factor test)
  else if (buffer[0] == 'B' && strstr(buffer, ",SF") != NULL) {
    // Parse "B125000,SF7"
    char* comma = strchr(buffer, ',');
    if (comma != NULL) {
      *comma = '\0';
      long bw = atol(&buffer[1]);
      uint8_t sf = (uint8_t)atoi(comma + 3);
      
      Serial.print(">>> TEST BW/SF: BW=");
      Serial.print(bw);
      Serial.print(" Hz, SF=");
      Serial.println(sf);
      
      next_bw = bw;
      next_sf = sf;
      syncState = SA_AFTER_BWSF;
      retryCount = 0;
      
      // Guardar resultado
      if (bwSfResultCount < 10) {
        bwSfReceivedResults[bwSfResultCount].bandwidth = bw;
        bwSfReceivedResults[bwSfResultCount].spreadingFactor = sf;
        bwSfReceivedResults[bwSfResultCount].snr = snr;
        bwSfReceivedResults[bwSfResultCount].valid = true;
        bwSfResultCount++;
        
        Serial.print("Guardado [");
        Serial.print(bwSfResultCount - 1);
        Serial.print("]: BW=");
        Serial.print(bw);
        Serial.print(", SF=");
        Serial.print(sf);
        Serial.print(", SNR=");
        Serial.println(snr);
      }
    }
  }
  // Procesar mensaje SE (Sync End)
  else if (strcmp(buffer, "SE") == 0) {
    Serial.println(">>> PRUEBAS FINALIZADAS (SE)");
    syncState = SYNC_T_FINAL_SA;
    retryCount = 0;
    
    // Mostrar resumen de resultados TxPower
    Serial.println("\n===== RESUMEN TXPOWER =====");
    Serial.print("Total pruebas: ");
    Serial.println(resultCount);
    for (uint8_t i = 0; i < resultCount; i++) {
      if (receivedResults[i].valid) {
        Serial.print("[");
        Serial.print(i);
        Serial.print("] TX=");
        Serial.print(receivedResults[i].txPower);
        Serial.print(" dBm, RSSI=");
        Serial.print(receivedResults[i].rssi);
        Serial.print(" dBm, SNR=");
        Serial.println(receivedResults[i].snr);
      }
    }
    
    // Mostrar resumen de resultados BW/SF
    Serial.println("\n===== RESUMEN BW/SF =====");
    Serial.print("Total pruebas: ");
    Serial.println(bwSfResultCount);
    for (uint8_t i = 0; i < bwSfResultCount; i++) {
      if (bwSfReceivedResults[i].valid) {
        Serial.print("[");
        Serial.print(i);
        Serial.print("] BW=");
        Serial.print(bwSfReceivedResults[i].bandwidth);
        Serial.print(" Hz, SF=");
        Serial.print(bwSfReceivedResults[i].spreadingFactor);
        Serial.print(", SNR=");
        Serial.println(bwSfReceivedResults[i].snr);
      }
    }
    Serial.println("===========================\n");
  }
  // Procesar mensaje BEST (Mejor configuración)
  else if (strncmp(buffer, "BEST:", 5) == 0) {
    Serial.println("\n***************************************");
    Serial.println("*** MEJOR CONFIGURACIÓN RECIBIDA ***");
    Serial.println(buffer);
    
    // Parse "BEST:TX14,BW125000,SF7"
    char* txStart = strstr(buffer, "TX");
    char* bwStart = strstr(buffer, "BW");
    char* sfStart = strstr(buffer, "SF");
    
    if (txStart && bwStart && sfStart) {
      uint8_t bestTx = (uint8_t)atoi(txStart + 2);
      long bestBw = atol(bwStart + 2);
      uint8_t bestSf = (uint8_t)atoi(sfStart + 2);
      
      Serial.print("TxPower: ");
      Serial.print(bestTx);
      Serial.println(" dBm");
      Serial.print("Bandwidth: ");
      Serial.print(bestBw);
      Serial.println(" Hz");
      Serial.print("Spreading Factor: ");
      Serial.println(bestSf);
      
      // Aplicar configuración
      LoRa.setTxPower(bestTx, PA_OUTPUT_PA_BOOST_PIN);
      LoRa.setSignalBandwidth(bestBw);
      LoRa.setSpreadingFactor(bestSf);
      
      Serial.println("\n=== CONFIGURACIÓN APLICADA ===");
      Serial.println("=== Modo Health Check activado ===\n");
      
      syncState = HEALTH_CHECK_MODE;
      
      // Enviar SA de confirmación
      static uint16_t bestMsgCount = 0;
      char saMsg[50];
      snprintf(saMsg, sizeof(saMsg), "SA");
      sendMessage(saMsg, uint8_t(strlen(saMsg)), bestMsgCount);
      Serial.println("Enviando SA de confirmación");
    }
    
    Serial.println("***************************************\n");
  }
}

void TxFinished() {
  txDoneFlag = true;
}