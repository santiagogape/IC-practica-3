/* ---------------------------------------------------------------------
 *  MAESTRO LoRa - Protocolo de Sincronización CORREGIDO
 *  
 *  Protocolo:
 *  1. Maestro envía "SI" → Esclavo responde "SA"
 *  2. Maestro envía "XBWYSF" → Esclavo responde "SA" (esclavo cambia config)
 *  3. Maestro cambia a la nueva configuración
 *  4. Maestro envía "SE" con nueva config → Esclavo responde "SA"
 *  5. Repite desde paso 2 con siguiente configuración
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS 15000

const uint8_t localAddress = 0x06;
uint8_t destination = 0x05;

volatile bool txDoneFlag = true;

// Array de configuraciones a probar
const long sync_bw[] = {125000, 250000, 500000, 62500, 41700, 31250, 20800, 15600, 10400, 7800};
const uint8_t sync_sf[] = {7, 7, 7, 8, 9, 10, 11, 12, 12, 12};
const uint8_t SYNC_TESTS_COUNT = sizeof(sync_bw) / sizeof(sync_bw[0]);

// Estados del protocolo
enum SyncState {
  IDLE,
  SENDING,
  WAIT_SA_AFTER_SI,
  WAIT_SA_AFTER_CONFIG,
  WAIT_SA_AFTER_SE
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

// Índice de configuración
uint8_t syncIndex = 0;

//Las conf
volatile uint8_t prev_spreadingFactor = 7;
volatile long prev_bandwidth = 125000;

// Variables de control de transmisión
static uint32_t lastSendTime_ms = 0;
static uint16_t msgCount = 0;
static uint32_t txInterval_ms = TX_LAPSE_MS;
static uint32_t tx_begin_ms = 0;
static bool transmitting = false;

void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount);
void onReceive(int packetSize);
void TxFinished();

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== MAESTRO LoRa - Protocolo Sincronización ===");

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
  currentBW = 125000;
  currentSF = 7;
  LoRa.setSyncWord(0x12);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.enableCrc();
  
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);
  LoRa.receive();

  Serial.print("Config inicial - BW: ");
  Serial.print(currentBW);
  Serial.print(" Hz, SF: ");
  Serial.println(currentSF);
  Serial.println("===========================================\n");
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
        case WAIT_SA_AFTER_SI:
          Serial.println("WAIT_SA_AFTER_SI");
          break;
        case WAIT_SA_AFTER_CONFIG:
          Serial.println("WAIT_SA_AFTER_CONFIG");
          break;
        case WAIT_SA_AFTER_SE:
          Serial.println("WAIT_SA_AFTER_SE");
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
  if ((syncState == WAIT_SA_AFTER_SI || 
       syncState == WAIT_SA_AFTER_CONFIG || 
       syncState == WAIT_SA_AFTER_SE) &&
      (millis() - waitStartTime > WAIT_TIMEOUT_MS)) {
    
    Serial.println("\n!!! TIMEOUT esperando SA !!!");
    Serial.println("Reiniciando protocolo...\n");
    
    syncState = IDLE;
    protocolPendingState = IDLE;
    pendingSend = false;
    syncIndex = 0;
    
    // Volver a config inicial
    LoRa.setSignalBandwidth(prev_bandwidth);
    LoRa.setSpreadingFactor(prev_spreadingFactor);
    currentBW = 125000;
    currentSF = 7;
    
    LoRa.receive();
  }

  // Iniciar protocolo automáticamente
  static bool protocolStarted = false;
  //  if (!protocolStarted && syncState == IDLE && !transmitting) {
  if (syncState == IDLE && !transmitting) {
    Serial.println("\n>>> Iniciando protocolo de sincronización <<<\n");
    strcpy(pendingMsg, "SI");
    pendingMsgLen = 2;
    pendingSend = true;
    protocolPendingState = WAIT_SA_AFTER_SI;
    protocolStarted = true;
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

  Serial.print("<< Recibido: '");
  Serial.print(buffer);
  Serial.print("' | RSSI: ");
  Serial.print(LoRa.packetRssi());
  Serial.print(" dBm | SNR: ");
  Serial.println(LoRa.packetSnr());

  // Procesamiento del protocolo
  if (receivedBytes >= 2 && buffer[0] == 'S' && buffer[1] == 'A') {
    Serial.println("   SA recibido");

    if (syncState == WAIT_SA_AFTER_SI) {
      // Enviar primera configuración
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
      // Esclavo confirmó que cambió de config
      // Ahora el maestro cambia a la misma config
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
      
      delay(100); // Dar tiempo para estabilizar
      LoRa.receive();
      
      // Ahora enviar "SE" con la nueva configuración
      strcpy(pendingMsg, "SE");
      pendingMsgLen = 2;
      pendingSend = true;
      protocolPendingState = WAIT_SA_AFTER_SE;
      
      Serial.println("   Preparando SE con nueva config");
    }
    else if (syncState == WAIT_SA_AFTER_SE) {
      // Ciclo completado
      Serial.println("   Ciclo de sincronización completado!");
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
        
        // Enviar siguiente configuración
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
        Serial.println(">>> TODAS LAS CONFIGURACIONES PROBADAS <<<");
        Serial.println("===========================================\n");
        syncState = IDLE;
        protocolPendingState = IDLE;
        pendingSend = false;
      }
    }
  }
}

void TxFinished() {
  txDoneFlag = true;
}