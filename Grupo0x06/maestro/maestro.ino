/* Maestro - Optimización dinámica de parámetros LoRa
   Basado en tu ejemplo original. */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          10000

// Umbrales (ajusta según entorno)
#define RSSI_MIN   -110
#define SNR_MIN     2.0

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0x06;     // Dirección de este maestro
uint8_t destination = 0x05;            // Dirección del esclavo

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión

// Contadores / estados
static uint16_t msgCount = 0;
String lastReceivedMessage = "";
uint32_t lastReceiveMillis = 0;

// Forward declarations
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCountRef);
void sendControlMessage(const String &msg);
bool waitForMessageStartsWith(const String &prefix, String &out, uint32_t timeout_ms);
bool waitForExactMessage(const String &exact, uint32_t timeout_ms);
String waitForMeasurement(uint32_t timeout_ms);
void parseMeasurement(const String &msg, int &rssi, float &snr);

// Optimización
void optimizeLoRaConfig();
void optimizeSpreadingFactor();
void optimizeBandwidth();
void optimizeCodingRate();
void optimizeTxPower();

void setup() 
{
  Serial.begin(9600);  
  while (!Serial);

  Serial.println("MASTER: LoRa Optimizer");

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  } else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  // Paràmetros radio iniciales
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(12);
  LoRa.setSyncWord(0x12);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);

  LoRa.onReceive(onReceive);
  LoRa.receive();
  LoRa.onTxDone(TxFinished);

  Serial.println("LoRa init succeeded.");

  // Pequeña espera para estabilizar
  delay(500);
}

static uint32_t lastSendTime_ms = 0;
static uint32_t txInterval_ms = TX_LAPSE_MS;
static uint32_t tx_begin_ms = 0;
static bool transmitting = false;

void loop() 
{
  // Ejecutar la optimización una sola vez al inicio (puedes repetirla si quieres)
  static bool optimized = false;
  if (!optimized) {
    optimizeLoRaConfig();
    optimized = true;
  }

  // --- Mantener la transmisión periódica del ejemplo original ---
  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {
    char message[50];
    snprintf(message, sizeof(message),"Message no. %03d from 0x%02X", msgCount, localAddress);

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(message, uint8_t(strlen(message)), msgCount);
    Serial.print("Sending '");
    Serial.print(message);
    Serial.print("' ");
  }

  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");

    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms; 
    float duty_cycle = (100.0f * TxTime_ms) / (lapse_ms ? lapse_ms : 1);

    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle,1);
    Serial.println(" %\n");

    if (duty_cycle <= 1.0f) {
      txInterval_ms = random(TX_LAPSE_MS) + 1000; 
    } else {
      txInterval_ms = TxTime_ms * 100;
    }

    transmitting = false;
    LoRa.receive();
  }

  // Procesar mensajes de control entrantes en background (opcional)
  // Si el esclavo tarda mucho en responder, la optimización gestionará timeouts.
}

// --------------------------------------------------------------------
// Sending message function (tal y como en tu original)
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCountRef) 
{
  while(!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(msgCountRef >> 7));
  LoRa.write((uint8_t)(msgCountRef & 0xFF));
  LoRa.write(msgLength);
  LoRa.print(outgoing);
  LoRa.endPacket(true);
  msgCountRef++;
}

// Envío de control (usa sendMessage)
void sendControlMessage(const String &msg) {
  char buffer[80];
  msg.toCharArray(buffer, sizeof(buffer));
  // Usamos msgCount para todos los mensajes
  sendMessage(buffer, strlen(buffer), msgCount);
  // Dejamos un pequeño margen para que el receptor procese
  delay(200);
}

// --------------------------------------------------------------------
// Receiving message function (modificado para guardar lastReceivedMessage)
void onReceive(int packetSize) 
{
  if (packetSize == 0) return;

  char buffer[100];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer)-1))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  buffer[receivedBytes] = '\0';

  if (incomingLength != receivedBytes) {
    Serial.print("Receiving error: declared message length " + String(incomingLength));
    Serial.println(" does not match length " + String(receivedBytes));
    return;
  }

  if ((recipient & localAddress) != localAddress ) {
    // No es para mí
    return;
  }

  // Guardar el último mensaje recibido para que otras rutinas lo consulten
  lastReceivedMessage = String(buffer);
  lastReceiveMillis = millis();

  // Imprimimos info en serial (útil para debug)
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + String(buffer));
  Serial.print("RSSI: " + String(LoRa.packetRssi()));
  Serial.println(" dBm\nSNR: " + String(LoRa.packetSnr()));
  Serial.println();
}

// Callback Tx finished
void TxFinished() {
  txDoneFlag = true;
}

// --------------------------------------------------------------------
// Funciones de espera y parseo de mensajes
bool waitForExactMessage(const String &exact, uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (lastReceivedMessage == exact) return true;
    // Optionally yield
  }
  return false;
}

bool waitForMessageStartsWith(const String &prefix, String &out, uint32_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (lastReceivedMessage.length() > 0 && lastReceivedMessage.startsWith(prefix)) {
      out = lastReceivedMessage;
      return true;
    }
  }
  return false;
}

String waitForMeasurement(uint32_t timeout_ms) {
  String out;
  if (waitForMessageStartsWith("MEASURE:", out, timeout_ms)) {
    return out;
  }
  return String("MEASURE:0,0");
}

void parseMeasurement(const String &msg, int &rssi, float &snr) {
  int p1 = msg.indexOf(':');
  int p2 = msg.indexOf(',');
  if (p1 < 0 || p2 < 0) {
    rssi = 0; snr = 0.0;
    return;
  }
  rssi = msg.substring(p1+1, p2).toInt();
  snr = msg.substring(p2+1).toFloat();
}

// --------------------------------------------------------------------
// Rutinas de optimización (un ejemplo – ajusta orden/estrategia si quieres)
void optimizeLoRaConfig() {
  Serial.println("\n[MASTER] ==== Iniciando optimización LoRa ====");
  optimizeSpreadingFactor();
  optimizeBandwidth();
  optimizeCodingRate();
  optimizeTxPower();
  Serial.println("[MASTER] ==== Optimización finalizada ====\n");
}

// Optimiza SF disminuyendo (12 -> 7) intentando mantener RSSI/SNR
void optimizeSpreadingFactor() {
  Serial.println("[MASTER] Optimizando Spreading Factor...");
  for (int sf = 12; sf >= 7; sf--) {
    Serial.println("[MASTER] Proponiendo SF " + String(sf));

    // 1) Indicar petición
    sendControlMessage("REQ_PARAM");
    if (!waitForExactMessage("ACK_PARAM", 2000)) {
      Serial.println("[MASTER] ERROR: No ACK_PARAM");
      continue;
    }

    // 2) Indicar nuevo parámetro
    sendControlMessage(String("PARAM_SET:SF=") + String(sf));
    if (!waitForExactMessage("PARAM_APPLIED", 2000)) {
      Serial.println("[MASTER] ERROR: No PARAM_APPLIED");
      continue;
    }

    // 3) Pedir medición
    sendControlMessage("MEASURE_REQ");
    String meas = waitForMeasurement(2000);
    int rssi;
    float snr;
    parseMeasurement(meas, rssi, snr);

    Serial.println("[MASTER] Resultado SF=" + String(sf) + " RSSI=" + String(rssi) + " SNR=" + String(snr));

    if (rssi < RSSI_MIN || snr < SNR_MIN) {
      Serial.println("[MASTER] Calidad insuficiente para SF " + String(sf) + ". Revertir a SF " + String(sf+1));
      LoRa.setSpreadingFactor(sf+1); // revertir en maestro
      // Notificar al esclavo para que vuelva atrás
      sendControlMessage("REQ_PARAM");
      waitForExactMessage("ACK_PARAM", 1000);
      sendControlMessage(String("PARAM_SET:SF=") + String(sf+1));
      waitForExactMessage("PARAM_APPLIED", 1000);
      return;
    }

    // Si OK, aplicar en maestro (ya debería haberse aplicado por la orden)
    LoRa.setSpreadingFactor(sf);
    // seguir probando siguiente SF más pequeño (más rápido)
  }
}

// Optimizar Bandwidth: intentar 250k y luego 500k (si hw lo permite)
void optimizeBandwidth() {
  Serial.println("[MASTER] Optimizando Bandwidth...");
  long bws[] = {250000L, 500000L};
  for (int i = 0; i < 2; i++) {
    long bw = bws[i];
    Serial.println("[MASTER] Proponiendo BW " + String(bw));
    sendControlMessage("REQ_PARAM");
    if (!waitForExactMessage("ACK_PARAM", 2000)) continue;
    sendControlMessage(String("PARAM_SET:BW=") + String(bw));
    if (!waitForExactMessage("PARAM_APPLIED", 2000)) continue;
    sendControlMessage("MEASURE_REQ");
    String meas = waitForMeasurement(2000);
    int rssi; float snr;
    parseMeasurement(meas, rssi, snr);
    Serial.println("[MASTER] Resultado BW=" + String(bw) + " RSSI=" + String(rssi) + " SNR=" + String(snr));
    if (rssi < RSSI_MIN || snr < SNR_MIN) {
      // revertir a 125k
      LoRa.setSignalBandwidth(125E3);
      sendControlMessage("REQ_PARAM");
      waitForExactMessage("ACK_PARAM", 1000);
      sendControlMessage(String("PARAM_SET:BW=") + String(125000L));
      waitForExactMessage("PARAM_APPLIED", 1000);
      return;
    }
    LoRa.setSignalBandwidth((double)bw);
  }
}

// Optimizar coding rate: intentar 4/5 (codingRate4 = 5 -> 4/5)
void optimizeCodingRate() {
  Serial.println("[MASTER] Optimizando Coding Rate...");
  int crCandidates[] = {5, 6, 7, 8}; // 4/5 .. 4/8 (5 es menos redundancia)
  for (int i = 0; i < 4; i++) {
    int cr = crCandidates[i];
    Serial.println("[MASTER] Proponiendo CR " + String(cr));
    sendControlMessage("REQ_PARAM");
    if (!waitForExactMessage("ACK_PARAM", 2000)) continue;
    sendControlMessage(String("PARAM_SET:CR=") + String(cr));
    if (!waitForExactMessage("PARAM_APPLIED", 2000)) continue;
    sendControlMessage("MEASURE_REQ");
    String meas = waitForMeasurement(2000);
    int rssi; float snr;
    parseMeasurement(meas, rssi, snr);
    Serial.println("[MASTER] Resultado CR=" + String(cr) + " RSSI=" + String(rssi) + " SNR=" + String(snr));
    if (rssi < RSSI_MIN || snr < SNR_MIN) {
      // reponer al valor anterior (que asumimos era 5)
      LoRa.setCodingRate4(5);
      sendControlMessage("REQ_PARAM");
      waitForExactMessage("ACK_PARAM", 1000);
      sendControlMessage(String("PARAM_SET:CR=") + String(5));
      waitForExactMessage("PARAM_APPLIED", 1000);
      return;
    }
    LoRa.setCodingRate4(cr);
  }
}

// Optimizar potencia Tx: bajar si sigue OK
void optimizeTxPower() {
  Serial.println("[MASTER] Optimizando Tx Power...");
  for (int p = 3; p <= 14; p++) { // probar subidas si quieres, aquí intentamos mantener baja potencia mínima 3
    Serial.println("[MASTER] Proponiendo POWER " + String(p));
    sendControlMessage("REQ_PARAM");
    if (!waitForExactMessage("ACK_PARAM", 2000)) continue;
    sendControlMessage(String("PARAM_SET:POWER=") + String(p));
    if (!waitForExactMessage("PARAM_APPLIED", 2000)) continue;
    sendControlMessage("MEASURE_REQ");
    String meas = waitForMeasurement(2000);
    int rssi; float snr;
    parseMeasurement(meas, rssi, snr);
    Serial.println("[MASTER] Resultado POWER=" + String(p) + " RSSI=" + String(rssi) + " SNR=" + String(snr));
    if (rssi < RSSI_MIN || snr < SNR_MIN) {
      // revertir al valor anterior p-1
      int prev = max(3, p-1);
      LoRa.setTxPower(prev, PA_OUTPUT_PA_BOOST_PIN);
      sendControlMessage("REQ_PARAM");
      waitForExactMessage("ACK_PARAM", 1000);
      sendControlMessage(String("PARAM_SET:POWER=") + String(prev));
      waitForExactMessage("PARAM_APPLIED", 1000);
      return;
    }
    LoRa.setTxPower(p, PA_OUTPUT_PA_BOOST_PIN);
    // si el objetivo es reducir potencia, en lugar de incrementar
    // empezaría por p = alto -> bajo; aquí queda como ejemplo.
  }
}
