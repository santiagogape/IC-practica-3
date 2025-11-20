/* ---------------------------------------------------------------------
 *  Esclavo LoRa compatible con el "maestro" autoAdjust (Arduino)
 * ---------------------------------------------------------------------
 *
 * Protocolo (igual que en el maestro):
 *  - Header:
 *      recipient (1 byte)
 *      sender    (1 byte)
 *      msgId     (2 bytes, high then low)
 *      length    (1 byte)
 *  - payload (length bytes)
 *
 * Comportamiento:
 *  - Si msgId == 0 : es una petición de prueba -> respondemos con un paquete
 *    de estado de 4 bytes con la configuración del esclavo + RSSI/SNR
 *    (formato que espera el maestro).
 *  - Si msgId == 1 : es la configuración final (payload[0]=SF, payload[1]=BW index)
 *    -> aplicamos y respondemos con ack (opcional).
 *
 * Ajusta las constantes (frecuencia, address) según tu montaje.
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h> // opcional según tu board

const uint8_t localAddress = 0x05;   // dirección de este esclavo
uint8_t masterAddress = 0x06;        // dirección del maestro (por defecto)
volatile bool transmitting = false;

typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

// Config actual del esclavo (valores por defecto; ajusta según necesites)
LoRaConfig_t thisNodeConf = { 7, 9, 5, 14 }; // ejemplo: BW idx=7 (125kHz), SF=9, CR=5, TxPwr=14dBm

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Enciende el LED integrado
  Serial.println("LoRa Slave for AutoAdjust - starting...");

  if (!init_PMIC()) {
    Serial.println("Init PMIC failed (or not present).");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (true);
  }

  // Config inicial en radio
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSyncWord(0x12);
  LoRa.setPreambleLength(8);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa.receive();

  Serial.println("LoRa slave ready.\n");
}

void loop() {
  // El esclavo funciona principalmente en modo recepción, el trabajo se hace en onReceive.
  // Aquí puedes añadir tareas periódicas si quieres (telemetría, etc.)
  delay(100);
}

// --------------------------------- helpers de envío ------------------------------
void sendMessageBlockingTo(uint8_t toAddr, uint8_t fromAddr, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) {
  // envío bloqueante (endPacket() sin true): retorna cuando la transmisión física termina
  LoRa.beginPacket();
  LoRa.write(toAddr);
  LoRa.write(fromAddr);
  LoRa.write((uint8_t)(msgCount >> 8));
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(payloadLength);
  LoRa.write(payload, (size_t)payloadLength);
  LoRa.endPacket(); // bloqueante
  // tras esto la transmisión ha terminado físicamente
}

// envío asíncrono (no usado mucho en esclavo para respuestas rápidas)
void sendMessageAsyncTo(uint8_t toAddr, uint8_t fromAddr, uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) {
  while (!LoRa.beginPacket()) {
    delay(5);
  }
  LoRa.write(toAddr);
  LoRa.write(fromAddr);
  LoRa.write((uint8_t)(msgCount >> 8));
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(payloadLength);
  LoRa.write(payload, (size_t)payloadLength);
  LoRa.endPacket(true); // asíncrono
}

// onTxDone
void onTxDone() {
  transmitting = false;
}

// --------------------------------- onReceive ------------------------------------
void onReceive(int packetSize) {
  if (packetSize == 0) return; // no hay nada

  // Leer cabecera
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  // leer payload
  uint8_t buffer[64];
  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < sizeof(buffer))) {
    buffer[receivedBytes++] = (uint8_t)LoRa.read();
  }

  // chequeo destinatario (aceptar broadcast 0xFF)
  if ((recipient != localAddress) && (recipient != 0xFF)) {
    // No es para mí
    return;
  }

  Serial.print("Received from 0x"); Serial.print(sender, HEX);
  Serial.print(" msgId: "); Serial.print(incomingMsgId);
  Serial.print(" len: "); Serial.println(incomingLength);

  // Obtener RSSI y SNR de la recepción que acaba de ocurrir:
  int rssi = LoRa.packetRssi();     // valor en dBm negativo
  float snr = LoRa.packetSnr();     // valor en dB (float)

  // ------------------ caso: petición de prueba (msgId == 0) -------------------
  if (incomingMsgId == 0) {
    // Maestro envía prueba con payload {sf, bw}
    // Respondemos con paquete de 4 bytes que el maestro espera:
    // formato:
    // byte0: (bandwidth_index <<4) | ((spreadingFactor - 6) << 1)
    // byte1: ((codingRate - 5) << 6) | ((txPower - 2) << 1)
    // byte2: uint8_t(-RSSI * 2)
    // byte3: uint8_t(148 + SNR)
    uint8_t reply[4];

    reply[0] = (thisNodeConf.bandwidth_index << 4) | ((thisNodeConf.spreadingFactor - 6) << 1);
    reply[1] = ((thisNodeConf.codingRate - 5) << 6) | ((thisNodeConf.txPower - 2) << 1);

    // convertimos RSSI y SNR al formato usado por el maestro
    int16_t encodedRSSI = int16_t(-rssi * 2); // ejemplo: maestro hace uint8_t(-LoRa.packetRssi()*2)
    if (encodedRSSI < 0) encodedRSSI = 0;
    if (encodedRSSI > 255) encodedRSSI = 255;
    reply[2] = (uint8_t)encodedRSSI;

    int16_t encodedSNR = int16_t(148 + round(snr)); // maestro usa uint8_t(148 + LoRa.packetSnr())
    if (encodedSNR < 0) encodedSNR = 0;
    if (encodedSNR > 255) encodedSNR = 255;
    reply[3] = (uint8_t)encodedSNR;

    Serial.print("Replying status -> ");
    Serial.print("BWidx:"); Serial.print(thisNodeConf.bandwidth_index);
    Serial.print(" SF:"); Serial.print(thisNodeConf.spreadingFactor);
    Serial.print(" EncRSSI:"); Serial.print(reply[2]);
    Serial.print(" EncSNR:"); Serial.println(reply[3]);

    // Enviar respuesta al sender (bloqueante para garantizar que maestro la reciba durante la ventana)
    sendMessageBlockingTo(sender, localAddress, reply, 4, 0x0000); // msgId del reply puede ser 0 (o el que consideres)
    // Asegurar volver a modo receive
    LoRa.receive();
    return;
  }

  // ------------------ caso: mensaje de configuración final (msgId == 1) -----------
  if (incomingMsgId == 1) {
    if (incomingLength >= 2) {
      uint8_t newSF = buffer[0];
      uint8_t newBWidx = buffer[1];
      Serial.print("Applying config from master: SF="); Serial.print(newSF);
      Serial.print(" BWidx="); Serial.println(newBWidx);

      // Validaciones simples
      if (newSF >= 6 && newSF <= 12 && newBWidx < 10) {
        thisNodeConf.spreadingFactor = newSF;
        thisNodeConf.bandwidth_index = newBWidx;
        LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
        LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
        delay(5);

        // Enviar ACK opcional al maestro (payload 1 byte = 0 OK)
        uint8_t ackPayload[1] = {0x00};
        sendMessageBlockingTo(sender, localAddress, ackPayload, 1, 0x0001);
        LoRa.receive();
        Serial.println("Config applied and ACK sent.");
      } else {
        Serial.println("Config invalid, ignoring.");
      }
    } else {
      Serial.println("Config message too short.");
    }
    return;
  }

  // ------------------ otros msgId -> posibilidad de procesarlos -----------------
  Serial.println("Unhandled msgId.");
}

// -----------------------------------------------------------------------------
// Fin del sketch
// -----------------------------------------------------------------------------
