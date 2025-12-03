/* ---------------------------------------------------------------------
 *  ESCLAVO LoRa - Protocolo de Sincronización CORREGIDO
 *  
 *  Protocolo:
 *  1. Recibe "SI" → Responde "SA"
 *  2. Recibe "XBWYSF" → Responde "SA" y CAMBIA configuración
 *  3. Recibe "SE" (ya en nueva config) → Responde "SA"
 *  4. Espera siguiente "XBWYSF" o nuevo "SI"
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

const uint8_t localAddress  = 0x05;
const uint8_t masterAddress = 0x06;
const uint8_t SYNC_WORD     = 0x12;
#define TIMEOUT_MS 20000

uint32_t lastPacketTime = 0;
uint8_t spreadingFactor = 7;
long bandwidth = 125000;
uint8_t prev_spreadingFactor = 7;
long prev_bandwidth = 125000;
uint32_t messageCount = 0;

enum SlaveState {
  WAIT_SYNC,
  WAIT_CONFIG,
  WAIT_SYNCEND,
  READY
};
SlaveState state = WAIT_SYNC;

void updateRadio(uint8_t sf, long bw) {
  if ((sf < 6 || sf > 12) || (bw < 7800 || bw > 500000)) {
    Serial.println("!!! Configuración inválida, usando defaults !!!");
    sf = 7;
    bw = 125000;
  }
  
  LoRa.idle();
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth(bw);
  spreadingFactor = sf;
  bandwidth = bw;
  
  Serial.print("   Config aplicada - BW: ");
  Serial.print(bw);
  Serial.print(" Hz, SF: ");
  Serial.println(sf);
  
  delay(50);
  LoRa.receive();
}

void restorePrevConfig() {
  Serial.println("   Restaurando configuración anterior");
  updateRadio(prev_spreadingFactor, prev_bandwidth);
}

void sendACK(const char* outgoing) {
  uint8_t msgLength = (uint8_t)strlen(outgoing);
  
  LoRa.idle();
  delay(10);
  
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  
  LoRa.write(masterAddress);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(messageCount >> 7));
  LoRa.write((uint8_t)(messageCount & 0xFF));
  LoRa.write(msgLength);
  LoRa.print(outgoing);
  LoRa.endPacket();
  messageCount++;

  Serial.print(">> ACK enviado: '");
  Serial.print(outgoing);
  Serial.println("'");
  
  delay(50);
  LoRa.receive();
}

bool parseXBWYSF(const String cmd, long* bw, uint8_t* sf) {
  if (cmd.length() < 5) return false;
  if (cmd[0] != 'X') return false;
  
  int idxY = cmd.indexOf('Y');
  if (idxY <= 1 || idxY >= cmd.length() - 1) return false;

  String bwStr = cmd.substring(1, idxY);
  String sfStr = cmd.substring(idxY + 1);

  long bwVal = bwStr.toInt();
  int sfVal = sfStr.toInt();
  
  if (bwVal < 7800 || bwVal > 500000) return false;
  if (sfVal < 6 || sfVal > 12) return false;

  *bw = bwVal;
  *sf = (uint8_t)sfVal;
  return true;
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t msgID = ((uint16_t)LoRa.read() << 7) | (uint16_t)LoRa.read();
  uint8_t msgLen = LoRa.read();

  if (recipient != localAddress && recipient != 0xFF) {
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

  Serial.print("<< Recibido: '");
  Serial.print(cmd);
  Serial.print("' | RSSI: ");
  Serial.print(LoRa.packetRssi());
  Serial.print(" dBm | SNR: ");
  Serial.print(LoRa.packetSnr());
  Serial.print(" | Estado: ");

  switch (state) {
    case WAIT_SYNC:
      Serial.println("WAIT_SYNC");
      if (cmd == "SI") {
        Serial.println("   Iniciando sincronización");
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    case WAIT_CONFIG:
      Serial.println("WAIT_CONFIG");
      if (cmd.startsWith("X")) {
        prev_spreadingFactor = spreadingFactor;
        prev_bandwidth = bandwidth;

        uint8_t tmp_sf;
        long tmp_bw;
        
        if (parseXBWYSF(cmd, &tmp_bw, &tmp_sf)) {
          Serial.print("   Nueva config solicitada - BW: ");
          Serial.print(tmp_bw);
          Serial.print(", SF: ");
          Serial.println(tmp_sf);
          
          sendACK("SA");
          updateRadio(tmp_sf, tmp_bw);
          state = WAIT_SYNCEND;
        } else {
          Serial.println("   Error: formato XBWYSF inválido");
        }
      }
      else if (cmd == "SI") {
        Serial.println("   Re-iniciando sincronización");
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    case WAIT_SYNCEND:
      Serial.println("WAIT_SYNCEND");
      if (cmd == "SE") {
        Serial.println("   Ciclo de sincronización completado");
        sendACK("SA");
        state = WAIT_CONFIG; // Listo para siguiente config
      }
      else if (cmd == "SI") {
        Serial.println("   Re-iniciando desde SI");
        restorePrevConfig();
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    case READY:
      Serial.println("READY");
      if (cmd.startsWith("M")) {
        sendACK("MA");
        Serial.print("   Mensaje: ");
        Serial.println(cmd.substring(1));
      }
      else if (cmd == "SI") {
        Serial.println("   Nueva sincronización");
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;

    default:
      Serial.println("ERROR - Estado desconocido");
      state = WAIT_SYNC;
      break;
  }
  
  Serial.println();
}

void checkTimeout() {
  if ((millis() - lastPacketTime) > TIMEOUT_MS) {
    Serial.println("\n!!! TIMEOUT detectado !!!");
    Serial.println("Restaurando estado y configuración inicial\n");
    
    restorePrevConfig();
    state = WAIT_SYNC;
    LoRa.receive();
    lastPacketTime = millis();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== ESCLAVO LoRa - Protocolo Sincronización ===");

  if (!init_PMIC()) {
    Serial.println("Error: Inicialización BQ24195L fallida");
  } else {
    Serial.println("OK: BQ24195L inicializado");
  }

  if (!LoRa.begin(868E6)) {
    Serial.println("Error: LoRa init failed");
    while (true);
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setPreambleLength(8);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.enableCrc();

  updateRadio(spreadingFactor, bandwidth);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  lastPacketTime = millis();
  
  Serial.print("Config inicial - BW: ");
  Serial.print(bandwidth);
  Serial.print(" Hz, SF: ");
  Serial.println(spreadingFactor);
  Serial.println("Esperando sincronización...");
  Serial.println("===========================================\n");
}

void loop() {
  checkTimeout();
  delay(10);
}