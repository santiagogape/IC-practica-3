/* ---------------------------------------------------------------------
 *  Versión corregida del ejemplo con auto-ajuste (maestro)
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          10000

const uint8_t localAddress = 0x06;
uint8_t destination = 0x05;

volatile bool txDoneFlag = true;
volatile bool transmitting = false;

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

LoRaConfig_t bestConfig;
float bestSNR = -1000.0f;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Duplex corrected - AutoAdjust");

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
    autoAdjustConfig();
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

    sendMessage(payload, payloadLength, msgCount);
    Serial.print("Sending packet ");
    Serial.print(msgCount++);
    Serial.print(": ");
    printBinaryPayload(payload, payloadLength);
  }

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

    if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 100;
    }

    transmitting = false;
    LoRa.receive();
  }
}

// -------------------- sendMessage corregida (uso general) --------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) {
  // beginPacket puede fallar si el radio está ocupado -> esperamos
  while(!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(msgCount >> 8));   // CORRECCIÓN: >>8 (no >>7)
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(payloadLength);
  LoRa.write(payload, (size_t)payloadLength);
  LoRa.endPacket(true); // asíncrono (normal en operación), onTxDone marcará txDoneFlag
}

// -------------------- función de envío bloqueante usada en calibración --------------------
void sendMessageBlocking(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(msgCount >> 8));
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(payloadLength);
  LoRa.write(payload, (size_t)payloadLength);
  LoRa.endPacket(); // bloqueante: retorna cuando la transmisión ha terminado
  // después de endPacket() la transmisión ha finalizado físicamente
}

// -------------------- onReceive corregido --------------------
void onReceive(int packetSize) {
  if (transmitting && !txDoneFlag) txDoneFlag = true;

  if (packetSize == 0) return;

  uint8_t buffer[32];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();

  // CORRECCIÓN: usar <<8 (no <<7)
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (uint8_t)LoRa.read();
  }

  if (incomingLength != receivedBytes) {
    Serial.print("Receiving error: declared message length ");
    Serial.print(incomingLength);
    Serial.print(" does not match length ");
    Serial.println(receivedBytes);
    // we'll continue: still update RSSI/SNR for diagnostics
  }

  // CORRECCIÓN: comprobar destinatario correctamente (aceptar broadcast 0xFF)
  if ((recipient != localAddress) && (recipient != 0xFF)) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }

  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Payload length: " + String(incomingLength));
  Serial.print("Payload: ");
  printBinaryPayload(buffer, receivedBytes);

  // ACTUALIZAR siempre RSSI y SNR con la última recepción (ayuda en auto-ajuste)
  remoteRSSI = LoRa.packetRssi();
  remoteSNR  = LoRa.packetSnr();
  Serial.print("\nRSSI: " + String(remoteRSSI));
  Serial.print(" dBm\nSNR: " + String(remoteSNR));
  Serial.println(" dB");

  // Si es un paquete de estado estándar de 4 bytes (como tu formato original)
  if (receivedBytes == 4) {
    remoteNodeConf.bandwidth_index = buffer[0] >> 4;
    remoteNodeConf.spreadingFactor = 6 + ((buffer[0] & 0x0F) >> 1);
    remoteNodeConf.codingRate = 5 + (buffer[1] >> 6);
    remoteNodeConf.txPower = 2 + ((buffer[1] & 0x3F) >> 1);
    remoteRSSI = -int(buffer[2]) / 2.0f;
    remoteSNR  = int(buffer[3]) - 148;

    Serial.print("Remote config: BW: ");
    Serial.print(bandwidth_kHz[remoteNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(remoteNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(remoteNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(remoteNodeConf.txPower);
    Serial.print(" dBm, RSSI: ");
    Serial.print(remoteRSSI);
    Serial.print(" dBm, SNR: ");
    Serial.print(remoteSNR, 1);
    Serial.println(" dB\n");
  } else {
    Serial.print("Non-standard payload size: ");
    Serial.print(receivedBytes);
    Serial.println(" bytes\n");
  }
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

// -------------------- autoAdjustConfig corregido --------------------
// -------------------- autoAdjustConfig QUE CUMPLE EL ENUNCIADO --------------------
void autoAdjustConfig() {

  const float MIN_SNR  = -12.0;   // umbral SNR mínimo aceptable
  const int   MIN_RSSI = -118;    // umbral RSSI mínimo aceptable

  Serial.println("\n=== AUTO-ADJUST CONFIG (Optimizar tiempo con SNR/RSSI mínimos) ===");

  struct Result {
    uint8_t sf;
    uint8_t bw;
    uint32_t txTime;
    float snr;
    int rssi;
    bool valid;
  };

  Result best = {0, 0, 0xFFFFFFFF, -200.0, -200, false};

  uint8_t testPayload[2] = {0, 0};

  // PROBAR CONFIGS DE MÁS RÁPIDAS A MÁS LENTAS
  for (int bw = 9; bw >= 7; bw--) {       // 500 → 250 → 125 kHz
    for (int sf = 7; sf <= 12; sf++) {    // SF7 → SF12

      Serial.print("\nProbando SF=");
      Serial.print(sf);
      Serial.print(" BWidx=");
      Serial.println(bw);

      remoteSNR  = -200.0;
      remoteRSSI = -200;

      LoRa.setSpreadingFactor(sf);
      LoRa.setSignalBandwidth((long)bandwidth_kHz[bw]);
      delay(6);

      testPayload[0] = sf;
      testPayload[1] = bw;

      uint32_t t0 = millis();
      sendMessageBlocking(testPayload, 2, 0);
      LoRa.receive();
      delay(300);     // tiempo para recibir respuesta

      uint32_t txTime = millis() - t0;

      Serial.print("  → Tiempo TX real: ");
      Serial.print(txTime);
      Serial.println(" ms");

      Serial.print("  → SNR recibido: ");
      Serial.println(remoteSNR);

      Serial.print("  → RSSI recibido: ");
      Serial.println(remoteRSSI);

      // VALIDACIÓN SEGÚN EL ENUNCIADO
      bool valid = true;

      if (remoteSNR < MIN_SNR) valid = false;
      if (remoteRSSI < MIN_RSSI) valid = false;
      if (remoteSNR < -150) valid = false;   // no llegó respuesta

      if (!valid) {
        Serial.println("  → ❌ Config descartada por no cumplir SNR/RSSI.");
        continue;
      }

      // SELECCIÓN POR MENOR TIEMPO (OBJETIVO DEL ENUNCIADO)
      if (txTime < best.txTime) {
        best.sf   = sf;
        best.bw   = bw;
        best.txTime = txTime;
        best.snr  = remoteSNR;
        best.rssi = remoteRSSI;
        best.valid = true;

        Serial.println("  → ✔ Nueva mejor configuración (tiempo más bajo).");
      }
    }
  }

  // APLICAR CONFIGURACIÓN ÓPTIMA
  if (!best.valid) {
    Serial.println("\n⚠ No se encontró ninguna configuración válida. Manteniendo valores actuales.");
    configSyncDone = true;
    return;
  }

  thisNodeConf.spreadingFactor = best.sf;
  thisNodeConf.bandwidth_index = best.bw;

  LoRa.setSpreadingFactor(best.sf);
  LoRa.setSignalBandwidth((long)bandwidth_kHz[best.bw]);
  delay(6);

  uint8_t finalPayload[2] = {best.sf, best.bw};
  sendMessageBlocking(finalPayload, 2, 1);
  LoRa.receive();

  Serial.println("\n=== CONFIGURACIÓN ÓPTIMA SELECCIONADA ===");
  Serial.print("SF = ");
  Serial.println(best.sf);
  Serial.print("BW = ");
  Serial.println(best.bw);
  Serial.print("Tiempo TX = ");
  Serial.print(best.txTime);
  Serial.println(" ms");
  Serial.print("SNR = ");
  Serial.println(best.snr);
  Serial.print("RSSI = ");
  Serial.println(best.rssi);
  Serial.println("=========================================\n");

  configSyncDone = true;
}