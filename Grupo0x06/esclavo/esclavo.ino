/* Esclavo - Responde al maestro y aplica parámetros */
#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

const uint8_t localAddress = 0x05;  // Dirección del esclavo
uint8_t destination = 0x06;         // Dirección del maestro

volatile bool txDoneFlag = true;
static uint16_t msgCount = 0;

void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCountRef);
void sendControlMessage(const String &msg);
void applyParameter(const String &msg);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("SLAVE: LoRa responder");

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  } else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

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
}

void loop() {
  // El esclavo actúa reactivamente en onReceive()
  // Puedes añadir tareas periódicas aquí si necesitas
}

// --------------------------------------------------------------------
// Envío de mensajes (igual que maestro)
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

void sendControlMessage(const String &msg) {
  char buffer[80];
  msg.toCharArray(buffer, sizeof(buffer));
  sendMessage(buffer, strlen(buffer), msgCount);
  delay(100);
}

// --------------------------------------------------------------------
// Receiving: interpretar mensajes de control
void onReceive(int packetSize) {
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

  String msg = String(buffer);
  Serial.println("[SLAVE] Recibido: " + msg);

  if (msg == "REQ_PARAM") {
    sendControlMessage("ACK_PARAM");
  }
  else if (msg.startsWith("PARAM_SET:")) {
    applyParameter(msg);
    sendControlMessage("PARAM_APPLIED");
  }
  else if (msg == "MEASURE_REQ") {
    // Se devuelve la última RSSI/SNR observada.
    // Usamos packetRssi/packetSnr; aunque corresponden al último paquete recibido.
    char measure[40];
    snprintf(measure, sizeof(measure), "MEASURE:%d,%.2f", LoRa.packetRssi(), LoRa.packetSnr());
    sendControlMessage(String(measure));
  }
  else {
    // Mensaje normal: mostrarlo
    Serial.println("[SLAVE] Mensaje normal: " + msg);
  }
}

void TxFinished() {
  txDoneFlag = true;
}

// --------------------------------------------------------------------
// Aplicar parámetros pedidos por maestro
void applyParameter(const String &msg) {
  // FORMATO: PARAM_SET:SF=10  PARAM_SET:BW=250000  PARAM_SET:CR=5  PARAM_SET:POWER=3
  if (msg.indexOf("SF=") > 0) {
    int p = msg.indexOf("SF=");
    int sf = msg.substring(p+3).toInt();
    Serial.println("[SLAVE] Cambiando SF a " + String(sf));
    LoRa.setSpreadingFactor(sf);
  } else if (msg.indexOf("BW=") > 0) {
    int p = msg.indexOf("BW=");
    long bw = msg.substring(p+3).toInt();
    Serial.println("[SLAVE] Cambiando BW a " + String(bw));
    LoRa.setSignalBandwidth((double)bw);
  } else if (msg.indexOf("CR=") > 0) {
    int p = msg.indexOf("CR=");
    int cr = msg.substring(p+3).toInt();
    Serial.println("[SLAVE] Cambiando CR a " + String(cr));
    LoRa.setCodingRate4(cr);
  } else if (msg.indexOf("POWER=") > 0) {
    int p = msg.indexOf("POWER=");
    int pow = msg.substring(p+6).toInt();
    Serial.println("[SLAVE] Cambiando POWER a " + String(pow));
    LoRa.setTxPower(pow, PA_OUTPUT_PA_BOOST_PIN);
  } else {
    Serial.println("[SLAVE] PARAM_SET desconocido: " + msg);
  }
}
