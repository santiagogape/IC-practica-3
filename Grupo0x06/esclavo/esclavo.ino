#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

const uint8_t localAddress  = 0x05;   // Dirección esclavo
const uint8_t masterAddress = 0x06;   // Dirección maestro
const uint8_t SYNC_WORD     = 0x12;
#define TIMEOUT_MS 20000

uint32_t lastPacketTime = 0;
volatile uint8_t  spreadingFactor = 7;
volatile long     bandwidth       = 7800;
uint8_t  prev_spreadingFactor;
long     prev_bandwidth;
uint32_t messageCount = 0;

enum SlaveState {
  WAIT_SYNC,     // esperando "SI"
  WAIT_CONFIG,   // esperando "SXXYY"
  WAIT_SYNCEND,  // esperando "SE"
  WAIT_MSG       // esperando "M"
};
SlaveState state = WAIT_SYNC;

void updateRadio(uint8_t sf, long bw) {
  if ((sf < 6 || sf > 12) || (bw < 7800 || bw > 500000)){
    Serial.print("Configuración errónea");
    sf = 12;
    bw = 125000;
  } else {
    LoRa.setSpreadingFactor(sf);
    LoRa.setSignalBandwidth(bw);
    spreadingFactor = sf;
    bandwidth = bw;
    Serial.print("Nueva config (SF, BW): ");
    Serial.print(sf);
    Serial.print(", ");
    Serial.println(bw);
    LoRa.receive();
  }
}

void restorePrevConfig() {
  updateRadio(prev_spreadingFactor, prev_bandwidth);
}

void sendACK(const char* outgoing) {
  uint8_t msgLength = (uint8_t)strlen(outgoing);
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(masterAddress);                  // Añadimos el ID del destinatario
  LoRa.write(localAddress);                   // Añadimos el ID del remitente
  LoRa.write((uint8_t)(messageCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(messageCount & 0xFF)); 
  LoRa.write(msgLength);                      // Añadimos la longitud en bytes del mensaje
  LoRa.print(outgoing);                       // Añadimos el mensaje/payload
  LoRa.endPacket();                           // Finalizamos el paquete, pero no esperamos a su transmisión
  messageCount++;                             // Incrementamos el contador de mensajes

  Serial.print("ACK enviado: ");
  Serial.println(outgoing);
  LoRa.receive();
}

bool parseXBWYSF(const String cmd, long* bw, uint8_t* sf) {
  if (cmd.length() < 7) return false;  //minimo X7800Y6
  if (cmd[0] != 'X') return false;
  int idxY = cmd.indexOf('Y');
  if (idxY <= 4 || idxY >= cmd.length() - 1) return false;

  String bwStr = cmd.substring(1, idxY);     // entre X e Y
  String sfStr = cmd.substring(idxY + 1);    // tras Y

  long bwVal = bwStr.toInt();
  int  sfVal = sfStr.toInt();
  if (bwVal < 7800 || bwVal > 500000)  return false;
  if (sfVal < 6 || sfVal > 12) return false;

  *bw = bwVal;
  *sf = (uint8_t)sfVal;
  return true;
}

// Callback
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t recipient = LoRa.read();
  uint8_t sender    = LoRa.read();
  uint16_t msgID    = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t msgLen    = LoRa.read();

  if (recipient != localAddress && recipient != 0xFF) {
    Serial.print("Mensaje no es para mí. Destino: 0x");
    Serial.println(recipient, HEX);
    while (LoRa.available()) LoRa.read();
    return;
  }

  char payload[32];
  uint8_t i = 0;
  while (LoRa.available() && i < sizeof(payload) - 1 && i < msgLen) {
    payload[i++] = (char)LoRa.read();
  }
  payload[i] = '\0';

  String cmd = String(payload);
  lastPacketTime = millis();

  Serial.println("---- Paquete recibido ----");
  Serial.print("De: 0x");   Serial.println(sender, HEX);
  Serial.print("Para: 0x"); Serial.println(recipient, HEX);
  Serial.print("ID: ");     Serial.println(msgID);
  Serial.print("Len: ");    Serial.println(msgLen);
  Serial.print("CMD: ");    Serial.println(cmd);
  Serial.print("RSSI: ");   Serial.println(LoRa.packetRssi());
  Serial.print("SNR: ");    Serial.println(LoRa.packetSnr());
  Serial.println("--------------------------");

  switch (state) {

    case WAIT_SYNC:
      if (cmd == "SI") {
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    case WAIT_CONFIG:
      if (cmd.startsWith("X")) {
        prev_spreadingFactor = spreadingFactor;
        prev_bandwidth       = bandwidth;

        uint8_t tmp_sf;
        long    tmp_bw;
        if (parseXBWYSF(cmd, &tmp_bw, &tmp_sf)) {
          sendACK("SA");
          updateRadio(tmp_sf, tmp_bw);
          state = WAIT_SYNCEND;
        } else {
          Serial.println("Formato XBWYSF inválido, ignorando.");
        }
      }
      else if (cmd.startsWith("M")) {
        sendACK("MA");
        Serial.print("Mensaje recibido: ");
        Serial.println(cmd.substring(1));
        state = WAIT_MSG;
      }
      break;

    case WAIT_SYNCEND:
      if (cmd == "SE"){
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    case WAIT_MSG:
      if (cmd.startsWith("M")) {
        sendACK("MA");
        Serial.print("Mensaje recibido: ");
        Serial.println(cmd.substring(1));
      } else if (cmd == "SI") {
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    default:
      Serial.println("Error de protocolo: estado desconocido.");
      break;
  }
}

void checkTimeout() {
  if ((millis() - lastPacketTime) > TIMEOUT_MS) {
    Serial.println("Timeout detectado. Restableciendo estado/configuración.");
    if (state == WAIT_CONFIG || state == WAIT_MSG) {
      restorePrevConfig();
    }
    state = WAIT_SYNC;
    LoRa.receive();
    lastPacketTime = millis();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setPreambleLength(8);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.enableCrc();  // CRC activado

  updateRadio(spreadingFactor, bandwidth);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  lastPacketTime = millis();
  Serial.println("Esclavo LoRa listo.");
}

void loop() {
  checkTimeout();
}
