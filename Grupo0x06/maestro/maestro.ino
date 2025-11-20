/* ---------------------------------------------------------------------
 *  Versión MEJORADA con seguridad y exploración exhaustiva de configs
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          10000
#define MAX_RETRIES          3
#define ACK_TIMEOUT_MS       500
#define CALIBRATION_RETRIES  2

const uint8_t localAddress = 0x06;
uint8_t destination = 0x05;

// Tipos de mensaje
enum MessageType {
  MSG_CALIBRATION = 0x01,
  MSG_CONFIG_FINAL = 0x02,
  MSG_DATA = 0x03,
  MSG_ACK = 0x04
};

volatile bool txDoneFlag = true;
volatile bool transmitting = false;
volatile bool ackReceived = false;

bool configSyncDone = false;

typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 6, 10, 5, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = -200.0f;

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  Serial.println("\n=== LoRa Duplex MEJORADO - AutoAdjust Seguro ===");

  if (!init_PMIC()) {
    Serial.println("Init PMIC failed!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(TxFinished);
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");
}

void loop() {
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;

  if (!configSyncDone) {
    autoAdjustConfigImproved();
    return;
  }

  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {
    uint8_t payload[50];
    uint8_t payloadLength = 0;

    payload[payloadLength]    = (thisNodeConf.bandwidth_index << 4);
    payload[payloadLength++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
    payload[payloadLength]    = ((thisNodeConf.codingRate - 5) << 6);
    payload[payloadLength++] |= ((thisNodeConf.txPower - 2) << 1);

    payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
    payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    // Envío con reintentos
    bool success = sendMessageWithRetry(payload, payloadLength, msgCount, MSG_DATA);
    
    if (success) {
      Serial.print("✓ Packet ");
      Serial.print(msgCount++);
      Serial.println(" sent successfully");
    } else {
      Serial.print("✗ Packet ");
      Serial.print(msgCount);
      Serial.println(" failed after retries");
    }
  }

  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" ms");

    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 100;
    }

    transmitting = false;
    LoRa.receive();
  }
}

// -------------------- Envío con ACK y reintentos --------------------
bool sendMessageWithRetry(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount, uint8_t msgType) {
  for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
    if (attempt > 0) {
      Serial.print("Retry ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(MAX_RETRIES - 1);
      delay(100 * attempt); // backoff exponencial
    }

    ackReceived = false;
    
    while(!LoRa.beginPacket()) {
      delay(10);
    }
    
    LoRa.write(destination);
    LoRa.write(localAddress);
    LoRa.write(msgType);
    LoRa.write((uint8_t)(msgCount >> 8));
    LoRa.write((uint8_t)(msgCount & 0xFF));
    LoRa.write(payloadLength);
    LoRa.write(payload, (size_t)payloadLength);
    
    // CRC simple
    uint8_t crc = calculateCRC(payload, payloadLength);
    LoRa.write(crc);
    
    LoRa.endPacket();
    
    // Esperar ACK
    uint32_t ackWaitStart = millis();
    LoRa.receive();
    
    while ((millis() - ackWaitStart) < ACK_TIMEOUT_MS) {
      if (ackReceived) {
        return true;
      }
      delay(10);
    }
  }
  
  return false; // Falló después de todos los reintentos
}

// -------------------- Cálculo de CRC simple --------------------
uint8_t calculateCRC(uint8_t* data, uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
  }
  return crc;
}

// -------------------- onReceive mejorado --------------------
void onReceive(int packetSize) {
  if (transmitting && !txDoneFlag) txDoneFlag = true;
  if (packetSize == 0) return;

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

  // Verificar destinatario
  if ((recipient != localAddress) && (recipient != 0xFF)) {
    Serial.println("✗ Message not for me");
    return;
  }

  // Verificar CRC si hay datos
  if (receivedBytes > 0) {
    uint8_t receivedCRC = buffer[receivedBytes - 1];
    uint8_t calculatedCRC = calculateCRC(buffer, receivedBytes - 1);
    
    if (receivedCRC != calculatedCRC) {
      Serial.println("✗ CRC mismatch! Corrupted packet.");
      return;
    }
    receivedBytes--; // quitar CRC del payload
  }

  // Actualizar RSSI y SNR
  remoteRSSI = LoRa.packetRssi();
  remoteSNR  = LoRa.packetSnr();

  // Procesar según tipo de mensaje
  if (msgType == MSG_ACK) {
    ackReceived = true;
    Serial.println("✓ ACK received");
    return;
  }

  Serial.println("\n--- Received ---");
  Serial.print("From: 0x");
  Serial.print(sender, HEX);
  Serial.print(" | Type: ");
  Serial.print(msgType);
  Serial.print(" | ID: ");
  Serial.print(incomingMsgId);
  Serial.print(" | RSSI: ");
  Serial.print(remoteRSSI);
  Serial.print(" dBm | SNR: ");
  Serial.print(remoteSNR, 1);
  Serial.println(" dB");

  // Enviar ACK
  sendACK(sender, incomingMsgId);

  // Procesar datos estándar de configuración
  if (receivedBytes == 4 && msgType == MSG_DATA) {
    remoteNodeConf.bandwidth_index = buffer[0] >> 4;
    remoteNodeConf.spreadingFactor = 6 + ((buffer[0] & 0x0F) >> 1);
    remoteNodeConf.codingRate = 5 + (buffer[1] >> 6);
    remoteNodeConf.txPower = 2 + ((buffer[1] & 0x3F) >> 1);
    
    Serial.print("Remote: BW=");
    Serial.print(bandwidth_kHz[remoteNodeConf.bandwidth_index]);
    Serial.print(" kHz, SF=");
    Serial.print(remoteNodeConf.spreadingFactor);
    Serial.print(", CR=");
    Serial.print(remoteNodeConf.codingRate);
    Serial.print(", Pwr=");
    Serial.print(remoteNodeConf.txPower);
    Serial.println(" dBm");
  }
}

// -------------------- Enviar ACK --------------------
void sendACK(uint8_t recipient, uint16_t msgId) {
  LoRa.beginPacket();
  LoRa.write(recipient);
  LoRa.write(localAddress);
  LoRa.write(MSG_ACK);
  LoRa.write((uint8_t)(msgId >> 8));
  LoRa.write((uint8_t)(msgId & 0xFF));
  LoRa.write(0); // sin payload
  LoRa.endPacket();
}

void TxFinished() {
  txDoneFlag = true;
}

void printBinaryPayload(uint8_t * payload, uint8_t payloadLength) {
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}

// -------------------- AutoAdjust MEJORADO --------------------
void autoAdjustConfigImproved() {
  const float MIN_SNR  = -12.0;
  const int   MIN_RSSI = -118;

  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║  AUTO-ADJUST MEJORADO - Exploración Exhaustiva   ║");
  Serial.println("╚═══════════════════════════════════════════════════╝\n");

  struct Result {
    uint8_t sf;
    uint8_t bw;
    uint8_t cr;
    uint32_t txTime;
    float snr;
    int rssi;
    float reliability; // % éxito en reintentos
    bool valid;
  };

  Result best = {0, 0, 0, 0xFFFFFFFF, -200.0, -200, 0.0, false};
  int totalTests = 0;
  int validTests = 0;

  uint8_t testPayload[4] = {0xAA, 0x55, 0, 0};

  // EXPLORACIÓN EXHAUSTIVA: BW, SF y CR
  for (int bw = 9; bw >= 6; bw--) {        // 500→125→62.5→41.7 kHz
    for (int sf = 7; sf <= 12; sf++) {     // SF7→SF12
      for (int cr = 5; cr <= 8; cr++) {    // CR 4/5 → 4/8
        
        totalTests++;
        
        Serial.print("\n[Test ");
        Serial.print(totalTests);
        Serial.print("] SF=");
        Serial.print(sf);
        Serial.print(" BW=");
        Serial.print((int)bandwidth_kHz[bw]);
        Serial.print(" kHz CR=4/");
        Serial.println(cr);

        // Configurar y dar tiempo de estabilización
        LoRa.setSpreadingFactor(sf);
        LoRa.setSignalBandwidth((long)bandwidth_kHz[bw]);
        LoRa.setCodingRate4(cr);
        delay(10);

        // Múltiples intentos para calcular fiabilidad
        int successes = 0;
        float sumSNR = 0;
        int sumRSSI = 0;
        uint32_t sumTime = 0;

        for (int trial = 0; trial < CALIBRATION_RETRIES; trial++) {
          remoteSNR  = -200.0;
          remoteRSSI = -200;

          testPayload[2] = sf;
          testPayload[3] = bw;

          uint32_t t0 = millis();
          LoRa.beginPacket();
          LoRa.write(destination);
          LoRa.write(localAddress);
          LoRa.write(MSG_CALIBRATION);
          LoRa.write(0);
          LoRa.write(0);
          LoRa.write(4);
          LoRa.write(testPayload, 4);
          LoRa.write(calculateCRC(testPayload, 4));
          LoRa.endPacket();
          
          LoRa.receive();
          delay(400);

          uint32_t txTime = millis() - t0;

          if (remoteSNR > -150 && remoteSNR >= MIN_SNR && remoteRSSI >= MIN_RSSI) {
            successes++;
            sumSNR += remoteSNR;
            sumRSSI += remoteRSSI;
            sumTime += txTime;
          }
          
          delay(50);
        }

        float reliability = (100.0 * successes) / CALIBRATION_RETRIES;
        
        Serial.print("  → Fiabilidad: ");
        Serial.print(reliability, 0);
        Serial.println("%");

        if (successes == 0) {
          Serial.println("  → ✗ Config fallida (0% éxito)");
          continue;
        }

        float avgSNR = sumSNR / successes;
        int avgRSSI = sumRSSI / successes;
        uint32_t avgTime = sumTime / successes;

        Serial.print("  → SNR: ");
        Serial.print(avgSNR, 1);
        Serial.print(" dB | RSSI: ");
        Serial.print(avgRSSI);
        Serial.print(" dBm | Tiempo: ");
        Serial.print(avgTime);
        Serial.println(" ms");

        // Criterio de selección: tiempo más bajo con 100% fiabilidad
        if (reliability >= 100.0 && avgTime < best.txTime) {
          best.sf = sf;
          best.bw = bw;
          best.cr = cr;
          best.txTime = avgTime;
          best.snr = avgSNR;
          best.rssi = avgRSSI;
          best.reliability = reliability;
          best.valid = true;
          validTests++;

          Serial.println("  → ✓ ¡NUEVA MEJOR CONFIG!");
        } else if (reliability >= 100.0) {
          validTests++;
        }
      }
    }
  }

  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║            RESULTADOS DE CALIBRACIÓN             ║");
  Serial.println("╚═══════════════════════════════════════════════════╝");
  Serial.print("Tests totales: ");
  Serial.println(totalTests);
  Serial.print("Configs válidas: ");
  Serial.println(validTests);

  if (!best.valid) {
    Serial.println("\n⚠ ERROR: No se encontró configuración válida.");
    Serial.println("Manteniendo config por defecto.\n");
    configSyncDone = true;
    return;
  }

  // Aplicar configuración óptima
  thisNodeConf.spreadingFactor = best.sf;
  thisNodeConf.bandwidth_index = best.bw;
  thisNodeConf.codingRate = best.cr;

  LoRa.setSpreadingFactor(best.sf);
  LoRa.setSignalBandwidth((long)bandwidth_kHz[best.bw]);
  LoRa.setCodingRate4(best.cr);
  delay(10);

  // Notificar configuración final al esclavo
  uint8_t finalPayload[3] = {best.sf, best.bw, best.cr};
  sendMessageWithRetry(finalPayload, 3, 999, MSG_CONFIG_FINAL);

  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║          CONFIGURACIÓN ÓPTIMA APLICADA           ║");
  Serial.println("╠═══════════════════════════════════════════════════╣");
  Serial.print("║  SF: ");
  Serial.print(best.sf);
  Serial.println("                                           ║");
  Serial.print("║  BW: ");
  Serial.print((int)bandwidth_kHz[best.bw]);
  Serial.println(" kHz                                  ║");
  Serial.print("║  CR: 4/");
  Serial.print(best.cr);
  Serial.println("                                         ║");
  Serial.print("║  Tiempo TX: ");
  Serial.print(best.txTime);
  Serial.println(" ms                              ║");
  Serial.print("║  SNR: ");
  Serial.print(best.snr, 1);
  Serial.println(" dB                                  ║");
  Serial.print("║  RSSI: ");
  Serial.print(best.rssi);
  Serial.println(" dBm                                ║");
  Serial.print("║  Fiabilidad: ");
  Serial.print(best.reliability, 0);
  Serial.println("%                              ║");
  Serial.println("╚═══════════════════════════════════════════════════╝\n");

  configSyncDone = true;
}