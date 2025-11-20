/* ---------------------------------------------------------------------
 *  Esclavo LoRa MEJORADO compatible con maestro mejorado
 *  - Sistema ACK/NACK
 *  - Verificación CRC
 *  - Tipos de mensaje
 *  - Respuestas robustas durante calibración
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

const uint8_t localAddress = 0x05;
uint8_t masterAddress = 0x06;

volatile bool transmitting = false;
volatile bool txDoneFlag = true;

// Tipos de mensaje (deben coincidir con el maestro)
enum MessageType {
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

// Configuración inicial del esclavo
LoRaConfig_t thisNodeConf = {7, 9, 5, 14}; // BW=125kHz, SF=9, CR=4/5, Pwr=14dBm

// Estadísticas
struct Stats {
  uint32_t packetsReceived;
  uint32_t packetsSent;
  uint32_t crcErrors;
  uint32_t calibrationRequests;
} stats = {0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║     LoRa SLAVE MEJORADO - AutoAdjust Ready       ║");
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

  // Configuración inicial
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
  Serial.println(" dBm\n");
  Serial.println("Waiting for master...\n");
}

void loop() {
  static uint32_t lastStatsTime = 0;
  
  // Mostrar estadísticas cada 30 segundos
  if (millis() - lastStatsTime > 30000) {
    printStats();
    lastStatsTime = millis();
  }
  
  // Parpadeo de LED para indicar que está vivo
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 2000) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
  
  delay(100);
}

// -------------------- Cálculo de CRC (debe coincidir con maestro) --------------------
uint8_t calculateCRC(uint8_t* data, uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
  }
  return crc;
}

// -------------------- Envío de ACK --------------------
void sendACK(uint8_t recipient, uint16_t msgId) {
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(localAddress);
  LoRa.write(MSG_ACK);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write(0); // sin payload
  LoRa.endPacket();
  
  stats.packetsSent++;
  
  Serial.print("→ ACK sent for msgId ");
  Serial.println(msgId);
}

// -------------------- Respuesta a calibración --------------------
void sendCalibrationResponse(uint8_t recipient, int rssi, float snr) {
  // Formato de respuesta: 4 bytes con config actual + RSSI/SNR medidos
  uint8_t reply[4];
  
  reply[0] = (thisNodeConf.bandwidth_index << 4) | ((thisNodeConf.spreadingFactor - 6) << 1);
  reply[1] = ((thisNodeConf.codingRate - 5) << 6) | ((thisNodeConf.txPower - 2) << 1);
  
  // Codificar RSSI y SNR
  int16_t encodedRSSI = int16_t(-rssi * 2);
  if (encodedRSSI < 0) encodedRSSI = 0;
  if (encodedRSSI > 255) encodedRSSI = 255;
  reply[2] = (uint8_t)encodedRSSI;
  
  int16_t encodedSNR = int16_t(148 + round(snr));
  if (encodedSNR < 0) encodedSNR = 0;
  if (encodedSNR > 255) encodedSNR = 255;
  reply[3] = (uint8_t)encodedSNR;
  
  uint8_t crc = calculateCRC(reply, 4);
  
  // Envío bloqueante para garantizar llegada durante calibración
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(localAddress);
  LoRa.write(MSG_DATA);
  LoRa.write(0); // msgId high
  LoRa.write(0); // msgId low
  LoRa.write(4); // length
  LoRa.write(reply, 4);
  LoRa.write(crc);
  LoRa.endPacket(); // bloqueante
  
  stats.packetsSent++;
  
  Serial.print("→ Calibration response sent (RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm, SNR: ");
  Serial.print(snr, 1);
  Serial.println(" dB)");
}

// -------------------- Aplicar nueva configuración --------------------
bool applyNewConfig(uint8_t sf, uint8_t bw, uint8_t cr) {
  // Validaciones
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
  
  // Aplicar nueva configuración
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
  
  // Parpadear LED rápido para indicar cambio
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
  digitalWrite(LED_BUILTIN, HIGH);
  
  return true;
}

// -------------------- onReceive mejorado --------------------
void onReceive(int packetSize) {
  if (packetSize == 0) return;
  
  stats.packetsReceived++;
  
  // Leer cabecera
  uint8_t buffer[64];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint8_t msgType = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();
  
  // Leer payload
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (uint8_t)LoRa.read();
  }
  
  // Verificar destinatario
  if ((recipient != localAddress) && (recipient != 0xFF)) {
    return; // No es para mí
  }
  
  // Obtener métricas de señal
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  
  // Verificar CRC si hay datos
  if (receivedBytes > 0) {
    uint8_t receivedCRC = buffer[receivedBytes - 1];
    uint8_t calculatedCRC = calculateCRC(buffer, receivedBytes - 1);
    
    if (receivedCRC != calculatedCRC) {
      stats.crcErrors++;
      Serial.println("\n✗ CRC ERROR - Corrupted packet!");
      Serial.print("Expected: 0x");
      Serial.print(calculatedCRC, HEX);
      Serial.print(" | Received: 0x");
      Serial.println(receivedCRC, HEX);
      return;
    }
    receivedBytes--; // Quitar CRC del payload
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
  
  // Procesar según tipo de mensaje
  switch (msgType) {
    
    case MSG_CALIBRATION:
      stats.calibrationRequests++;
      Serial.println("→ Calibration request received");
      
      // Durante calibración, el maestro puede cambiar parámetros
      // Respondemos con nuestro estado y las métricas que recibimos
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
          // Enviar ACK de confirmación
          sendACK(sender, incomingMsgId);
          LoRa.receive();
        }
      } else {
        Serial.println("✗ Config message too short");
      }
      break;
      
    case MSG_DATA:
      Serial.println("→ Data message received");
      
      // Enviar ACK
      sendACK(sender, incomingMsgId);
      
      // Procesar datos (ejemplo: mostrar payload)
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
        
        // Decodificar si es formato estándar de 4 bytes
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
      // El esclavo normalmente no procesa ACKs, pero podría hacerlo
      LoRa.receive();
      break;
      
    default:
      Serial.print("→ Unknown message type: ");
      Serial.println(msgType);
      LoRa.receive();
      break;
  }
}

// -------------------- onTxDone --------------------
void onTxDone() {
  transmitting = false;
  txDoneFlag = true;
}

// -------------------- Imprimir estadísticas --------------------
void printStats() {
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║                  STATISTICS                       ║");
  Serial.println("╠═══════════════════════════════════════════════════╣");
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

// -------------------- Reset de estadísticas (opcional) --------------------
void resetStats() {
  stats.packetsReceived = 0;
  stats.packetsSent = 0;
  stats.crcErrors = 0;
  stats.calibrationRequests = 0;
  Serial.println("✓ Statistics reset");
}