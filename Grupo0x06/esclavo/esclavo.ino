/* ---------------------------------------------------------------------
 *  Esclavo LoRa con SINCRONIZACIÓN INICIAL ROBUSTA
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

const uint8_t localAddress = 0x05;
uint8_t masterAddress = 0x06;

volatile bool transmitting = false;
volatile bool txDoneFlag = true;

// Tipos de mensaje
enum MessageType {
  MSG_SYNC = 0x00,
  MSG_CALIBRATION = 0x01,
  MSG_CONFIG_FINAL = 0x02,
  MSG_DATA = 0x03,
  MSG_ACK = 0x04
};

typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3};

// CONFIGURACIÓN INICIAL COMÚN (debe coincidir con el maestro)
LoRaConfig_t thisNodeConf = {7, 10, 5, 14}; // BW=125kHz, SF=10, CR=4/5, Pwr=14dBm

bool syncedWithMaster = false;

struct Stats {
  uint32_t packetsReceived;
  uint32_t packetsSent;
  uint32_t crcErrors;
  uint32_t calibrationRequests;
  uint32_t syncAttempts;
} stats = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║    LoRa SLAVE con Sincronización Robusta         ║");
  Serial.println("╚═══════════════════════════════════════════════════╝\n");

  if (!init_PMIC()) {
    Serial.println("⚠ Init PMIC failed (or not present)");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("✗ LoRa init failed. Check wiring.");
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();

  Serial.println("✓ LoRa slave initialized");
  Serial.print("Address: 0x");
  Serial.println(localAddress, HEX);
  Serial.print("Initial config: BW=");
  Serial.print((int)bandwidth_kHz[thisNodeConf.bandwidth_index]);
  Serial.print(" kHz, SF=");
  Serial.print(thisNodeConf.spreadingFactor);
  Serial.print(", CR=4/");
  Serial.print(thisNodeConf.codingRate);
  Serial.print(", Power=");
  Serial.print(thisNodeConf.txPower);
  Serial.println(" dBm");
  
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║          ESPERANDO SINCRONIZACIÓN...              ║");
  Serial.println("╚═══════════════════════════════════════════════════╝\n");
  Serial.println("→ Escaneando configuraciones comunes...");
}

void loop() {
  static uint32_t lastStatsTime = 0;
  static uint32_t lastConfigSwitch = 0;
  static int currentConfigIndex = 0;
  
  // Si no está sincronizado, rotar entre configuraciones comunes
  if (!syncedWithMaster) {
    if (millis() - lastConfigSwitch > 3000) { // Cambiar cada 3 segundos
      struct SyncConfig {
        uint8_t bw;
        uint8_t sf;
      };
      
      SyncConfig commonConfigs[] = {
        {7, 10},  // 125kHz, SF10
        {7, 9},   // 125kHz, SF9
        {7, 11},  // 125kHz, SF11
        {7, 12},  // 125kHz, SF12
        {8, 10},  // 250kHz, SF10
      };
      
      int numConfigs = sizeof(commonConfigs) / sizeof(SyncConfig);
      
      currentConfigIndex = (currentConfigIndex + 1) % numConfigs;
      
      thisNodeConf.bandwidth_index = commonConfigs[currentConfigIndex].bw;
      thisNodeConf.spreadingFactor = commonConfigs[currentConfigIndex].sf;
      
      LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
      LoRa.setSignalBandwidth((long)bandwidth_kHz[thisNodeConf.bandwidth_index]);
      delay(10);
      LoRa.receive();
      
      Serial.print("→ Escaneando: BW=");
      Serial.print((int)bandwidth_kHz[thisNodeConf.bandwidth_index]);
      Serial.print(" kHz, SF=");
      Serial.print(thisNodeConf.spreadingFactor);
      Serial.println(" (esperando maestro...)");
      
      lastConfigSwitch = millis();
      stats.syncAttempts++;
      
      // Parpadeo lento mientras busca
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  } else {
    // Ya sincronizado - LED fijo
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Mostrar estadísticas cada 30 segundos
    if (millis() - lastStatsTime > 30000) {
      printStats();
      lastStatsTime = millis();
    }
  }
  
  delay(100);
}

uint8_t calculateCRC(uint8_t* data, uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
  }
  return crc;
}

void sendACK(uint8_t recipient, uint16_t msgId) {
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(localAddress);
  LoRa.write(MSG_ACK);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write(0);
  LoRa.endPacket();
  
  stats.packetsSent++;
}

void sendCalibrationResponse(uint8_t recipient, int rssi, float snr) {
  uint8_t reply[4];
  
  reply[0] = (thisNodeConf.bandwidth_index << 4) | ((thisNodeConf.spreadingFactor - 6) << 1);
  reply[1] = ((thisNodeConf.codingRate - 5) << 6) | ((thisNodeConf.txPower - 2) << 1);
  
  int16_t encodedRSSI = int16_t(-rssi * 2);
  if (encodedRSSI < 0) encodedRSSI = 0;
  if (encodedRSSI > 255) encodedRSSI = 255;
  reply[2] = (uint8_t)encodedRSSI;
  
  int16_t encodedSNR = int16_t(148 + round(snr));
  if (encodedSNR < 0) encodedSNR = 0;
  if (encodedSNR > 255) encodedSNR = 255;
  reply[3] = (uint8_t)encodedSNR;
  
  uint8_t crc = calculateCRC(reply, 4);
  
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(localAddress);
  LoRa.write(MSG_DATA);
  LoRa.write(0);
  LoRa.write(0);
  LoRa.write(4);
  LoRa.write(reply, 4);
  LoRa.write(crc);
  LoRa.endPacket();
  
  stats.packetsSent++;
  
  Serial.print("→ Calibration response sent (RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm, SNR: ");
  Serial.print(snr, 1);
  Serial.println(" dB)");
}

bool applyNewConfig(uint8_t sf, uint8_t bw, uint8_t cr) {
  if (sf < 7 || sf > 12) {
    Serial.println("✗ Invalid SF");
    return false;
  }
  if (bw > 9) {
    Serial.println("✗ Invalid BW index");
    return false;
  }
  if (cr < 5 || cr > 8) {
    Serial.println("✗ Invalid CR");
    return false;
  }
  
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║         APPLYING NEW CONFIGURATION               ║");
  Serial.println("╚═══════════════════════════════════════════════════╝");
  
  Serial.print("Previous: SF=");
  Serial.print(thisNodeConf.spreadingFactor);
  Serial.print(" BW=");
  Serial.print((int)bandwidth_kHz[thisNodeConf.bandwidth_index]);
  Serial.print(" kHz CR=4/");
  Serial.println(thisNodeConf.codingRate);
  
  thisNodeConf.spreadingFactor = sf;
  thisNodeConf.bandwidth_index = bw;
  thisNodeConf.codingRate = cr;
  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  delay(10);
  
  Serial.print("New:      SF=");
  Serial.print(thisNodeConf.spreadingFactor);
  Serial.print(" BW=");
  Serial.print((int)bandwidth_kHz[thisNodeConf.bandwidth_index]);
  Serial.print(" kHz CR=4/");
  Serial.println(thisNodeConf.codingRate);
  Serial.println("✓ Configuration applied successfully\n");
  
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  
  return true;
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;
  
  stats.packetsReceived++;
  
  uint8_t buffer[64];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint8_t msgType = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();
  
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (uint8_t)LoRa.read();
  }
  
  if ((recipient != localAddress) && (recipient != 0xFF)) {
    return;
  }
  
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  if (receivedBytes > 0) {
    uint8_t receivedCRC = buffer[receivedBytes - 1];
    uint8_t calculatedCRC = calculateCRC(buffer, receivedBytes - 1);
    
    if (receivedCRC != calculatedCRC) {
      stats.crcErrors++;
      Serial.println("\n✗ CRC ERROR - Corrupted packet!");
      return;
    }
    receivedBytes--;
  }
  
  // MENSAJE DE SINCRONIZACIÓN
  if (msgType == MSG_SYNC && !syncedWithMaster) {
    Serial.println("\n╔═══════════════════════════════════════════════════╗");
    Serial.println("║        ✓ SINCRONIZADO CON EL MAESTRO!            ║");
    Serial.println("╚═══════════════════════════════════════════════════╝");
    
    Serial.print("Config de sync: BW=");
    Serial.print((int)bandwidth_kHz[thisNodeConf.bandwidth_index]);
    Serial.print(" kHz, SF=");
    Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(" | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.print(snr, 1);
    Serial.println(" dB\n");
    
    syncedWithMaster = true;
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Responder al maestro
    uint8_t syncReply[2] = {0xBB, 0xCC};
    LoRa.beginPacket();
    LoRa.write(sender);
    LoRa.write(localAddress);
    LoRa.write(MSG_DATA);
    LoRa.write(0);
    LoRa.write(0);
    LoRa.write(2);
    LoRa.write(syncReply, 2);
    LoRa.write(calculateCRC(syncReply, 2));
    LoRa.endPacket();
    
    LoRa.receive();
    return;
  }
  
  // Solo procesar otros mensajes si ya está sincronizado
  if (!syncedWithMaster && msgType != MSG_SYNC) {
    return;
  }
  
  Serial.println("\n┌─────────────────────────────────────────┐");
  Serial.print("│ RX from 0x");
  Serial.print(sender, HEX);
  Serial.print(" | Type: ");
  Serial.print(msgType);
  Serial.print(" | ID: ");
  Serial.print(incomingMsgId);
  Serial.println("    │");
  Serial.print("│ RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm | SNR: ");
  Serial.print(snr, 1);
  Serial.println(" dB       │");
  Serial.println("└─────────────────────────────────────────┘");
  
  switch (msgType) {
    
    case MSG_CALIBRATION:
      stats.calibrationRequests++;
      Serial.println("→ Calibration request received");
      
      // Durante calibración, el maestro puede cambiar los parámetros
      // Adaptarse automáticamente si detectamos que el maestro cambió
      if (receivedBytes >= 4) {
        uint8_t testSF = buffer[2];
        uint8_t testBW = buffer[3];
        
        // Si detectamos parámetros diferentes, adaptarnos
        if (testSF != thisNodeConf.spreadingFactor || 
            testBW != thisNodeConf.bandwidth_index) {
          
          Serial.print("→ Adaptando a config de prueba: SF=");
          Serial.print(testSF);
          Serial.print(" BW=");
          Serial.println(testBW);
          
          thisNodeConf.spreadingFactor = testSF;
          thisNodeConf.bandwidth_index = testBW;
          
          LoRa.setSpreadingFactor(testSF);
          LoRa.setSignalBandwidth((long)bandwidth_kHz[testBW]);
          delay(10);
        }
      }
      
      sendCalibrationResponse(sender, rssi, snr);
      LoRa.receive();
      break;
      
    case MSG_CONFIG_FINAL:
      Serial.println("→ Final configuration received");
      
      if (receivedBytes >= 3) {
        uint8_t newSF = buffer[0];
        uint8_t newBW = buffer[1];
        uint8_t newCR = buffer[2];
        
        if (applyNewConfig(newSF, newBW, newCR)) {
          sendACK(sender, incomingMsgId);
          LoRa.receive();
        }
      } else {
        Serial.println("✗ Config message too short");
      }
      break;
      
    case MSG_DATA:
      Serial.println("→ Data message received");
      
      sendACK(sender, incomingMsgId);
      
      if (receivedBytes > 0) {
        Serial.print("Payload (");
        Serial.print(receivedBytes);
        Serial.print(" bytes): ");
        for (int i = 0; i < receivedBytes; i++) {
          Serial.print("0x");
          if (buffer[i] < 0x10) Serial.print("0");
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        
        if (receivedBytes == 4) {
          uint8_t remoteBW = buffer[0] >> 4;
          uint8_t remoteSF = 6 + ((buffer[0] & 0x0F) >> 1);
          uint8_t remoteCR = 5 + (buffer[1] >> 6);
          uint8_t remotePwr = 2 + ((buffer[1] & 0x3F) >> 1);
          int remoteRSSI = -int(buffer[2]) / 2;
          float remoteSNR = int(buffer[3]) - 148;
          
          Serial.print("Master status: BW=");
          Serial.print((int)bandwidth_kHz[remoteBW]);
          Serial.print(" kHz, SF=");
          Serial.print(remoteSF);
          Serial.print(", CR=4/");
          Serial.print(remoteCR);
          Serial.print(", Pwr=");
          Serial.print(remotePwr);
          Serial.print(" dBm, RSSI=");
          Serial.print(remoteRSSI);
          Serial.print(" dBm, SNR=");
          Serial.print(remoteSNR, 1);
          Serial.println(" dB");
        }
      }
      
      LoRa.receive();
      break;
      
    case MSG_ACK:
      Serial.println("→ ACK received");
      LoRa.receive();
      break;
      
    default:
      Serial.print("→ Unknown message type: ");
      Serial.println(msgType);
      LoRa.receive();
      break;
  }
}

void onTxDone() {
  transmitting = false;
  txDoneFlag = true;
}

void printStats() {
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║                  STATISTICS                       ║");
  Serial.println("╠═══════════════════════════════════════════════════╣");
  
  if (!syncedWithMaster) {
    Serial.print("║ Sync attempts:          ");
    Serial.print(stats.syncAttempts);
    Serial.println("                         ║");
    Serial.println("║ Status:                 NOT SYNCED               ║");
  } else {
    Serial.println("║ Status:                 SYNCED ✓                 ║");
    Serial.print("║ Packets received:       ");
    Serial.print(stats.packetsReceived);
    Serial.println("                         ║");
    Serial.print("║ Packets sent:           ");
    Serial.print(stats.packetsSent);
    Serial.println("                         ║");
    Serial.print("║ CRC errors:             ");
    Serial.print(stats.crcErrors);
    Serial.println("                         ║");
    Serial.print("║ Calibration requests:   ");
    Serial.print(stats.calibrationRequests);
    Serial.println("                         ║");
  }
  
  Serial.println("╠═══════════════════════════════════════════════════╣");
  Serial.print("║ Current SF:             ");
  Serial.print(thisNodeConf.spreadingFactor);
  Serial.println("                         ║");
  Serial.print("║ Current BW:             ");
  Serial.print((int)bandwidth_kHz[thisNodeConf.bandwidth_index]);
  Serial.println(" kHz                   ║");
  Serial.print("║ Current CR:             4/");
  Serial.print(thisNodeConf.codingRate);
  Serial.println("                       ║");
  Serial.print("║ Current Power:          ");
  Serial.print(thisNodeConf.txPower);
  Serial.println(" dBm                     ║");
  Serial.println("╚═══════════════════════════════════════════════════╝\n");
}