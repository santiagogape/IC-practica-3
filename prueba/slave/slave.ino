#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

// Direcciones
const uint8_t LOCAL_ADDR  = 0x05;
const uint8_t MASTER_ADDR = 0x06;
const uint8_t SYNC_WORD   = 0x12;

// Config LoRa base (se irá cambiando)
const long   LORA_FREQ = 868E6;
const long   BW_TABLE[] = {125000, 250000, 500000, 62500, 41700, 31250};
const uint8_t BW_COUNT = sizeof(BW_TABLE) / sizeof(BW_TABLE[0]);

// Tipos de mensaje
const uint8_t CMD_TEST_TX   = 0x01;
const uint8_t CMD_ACK_TX    = 0x02;
const uint8_t CMD_TEST_CFG  = 0x03;
const uint8_t CMD_ACK_CFG   = 0x04;
const uint8_t CMD_SET_FINAL = 0x05;
const uint8_t CMD_ACK_FINAL = 0x06;
const uint8_t CMD_DATA      = 0x10;
const uint8_t CMD_ACK_DATA  = 0x11;

// Estado actual de radio (espejo local)
long   currentBW = BW_TABLE[0];
uint8_t currentSF = 7;
uint8_t currentTX = 5;

// Para datos
uint8_t lastDataId = 0;

// Prototipos
void onReceive(int packetSize);
void applyRadio(long bw, uint8_t sf, uint8_t tx);
void sendPacket(uint8_t dest, uint8_t type, uint8_t id, const uint8_t* payload, uint8_t len);
void sendAckSimple(uint8_t dest, uint8_t type, uint8_t id);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== ESCLAVO LoRa SYNC BIN ===");

  if (!init_PMIC()) {
    Serial.println("Aviso: BQ24195L no inicializado");
  } else {
    Serial.println("OK: BQ24195L inicializado");
  }

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Error: LoRa init failed");
    while (true);
  }

  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setPreambleLength(8);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  applyRadio(currentBW, currentSF, currentTX);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.print("Config inicial esclavo - BW=");
  Serial.print(currentBW);
  Serial.print(" SF=");
  Serial.print(currentSF);
  Serial.print(" TX=");
  Serial.println(currentTX);
}

void loop() {
  // Todo va por callback
  delay(10);
}

void applyRadio(long bw, uint8_t sf, uint8_t tx) {
  LoRa.idle();
  LoRa.setSignalBandwidth(bw);
  LoRa.setSpreadingFactor(sf);
  LoRa.setTxPower(tx, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.receive();
  currentBW = bw;
  currentSF = sf;
  currentTX = tx;
  Serial.print("-> Radio esclavo: BW=");
  Serial.print(bw);
  Serial.print(" SF=");
  Serial.print(sf);
  Serial.print(" TX=");
  Serial.println(tx);
}

void sendPacket(uint8_t dest, uint8_t type, uint8_t id, const uint8_t* payload, uint8_t len) {
  LoRa.idle();
  while (!LoRa.beginPacket()) { delay(1); }

  LoRa.write(dest);
  LoRa.write(LOCAL_ADDR);
  LoRa.write(type);
  LoRa.write(id);
  for (uint8_t i = 0; i < len; i++) LoRa.write(payload[i]);

  LoRa.endPacket();
  LoRa.receive();
}

void sendAckSimple(uint8_t dest, uint8_t type, uint8_t id) {
  sendPacket(dest, type, id, nullptr, 0);
}

void onReceive(int packetSize) {
  if (packetSize < 4) return;

  uint8_t dest  = LoRa.read();
  uint8_t src   = LoRa.read();
  uint8_t type  = LoRa.read();
  uint8_t id    = LoRa.read();

  uint8_t payload[16];
  uint8_t pLen = 0;
  while (LoRa.available() && pLen < sizeof(payload)) {
    payload[pLen++] = (uint8_t)LoRa.read();
  }

  if (dest != LOCAL_ADDR && dest != 0xFF) {
    return;
  }

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  Serial.print("RX type=0x"); Serial.print(type, HEX);
  Serial.print(" id="); Serial.print(id);
  Serial.print(" de 0x"); Serial.print(src, HEX);
  Serial.print(" RSSI="); Serial.print(rssi);
  Serial.print(" SNR="); Serial.println(snr);

  switch (type) {
    case CMD_TEST_TX: {
      // payload[0] = nuevo TX propuesto
      if (pLen >= 1) {
        uint8_t newTX = payload[0];
        Serial.print("  CMD_TEST_TX -> TX=");
        Serial.println(newTX);
        applyRadio(currentBW, currentSF, newTX);
      }
      sendAckSimple(src, CMD_ACK_TX, id);
      break;
    }

    case CMD_TEST_CFG: {
      // payload[0] = bwIndex, payload[1] = sf
      if (pLen >= 2) {
        uint8_t bwIndex = payload[0];
        uint8_t sf      = payload[1];
        if (bwIndex < BW_COUNT && sf >= 6 && sf <= 12) {
          long bw = BW_TABLE[bwIndex];
          Serial.print("  CMD_TEST_CFG -> BW=");
          Serial.print(bw);
          Serial.print(" SF=");
          Serial.println(sf);
          applyRadio(bw, sf, currentTX);
        }
      }
      sendAckSimple(src, CMD_ACK_CFG, id);
      break;
    }

    case CMD_SET_FINAL: {
      if (pLen >= 3) {
        uint8_t tx   = payload[0];
        uint8_t idx  = payload[1];
        uint8_t sf   = payload[2];
        if (idx < BW_COUNT && sf >= 6 && sf <= 12) {
          long bw = BW_TABLE[idx];
          Serial.println("  CMD_SET_FINAL recibido, aplicando config final");
          applyRadio(bw, sf, tx);
        }
      }
      sendAckSimple(src, CMD_ACK_FINAL, id);
      break;
    }

    case CMD_DATA: {
      // Guarda id y responde ACK_DATA con ese id en payload[0]
      lastDataId = id;
      Serial.print("  CMD_DATA recibido, len=");
      Serial.println(pLen);
      if (pLen > 0) {
        Serial.print("  Datos: ");
        for (uint8_t i = 0; i < pLen; i++) {
          Serial.write(payload[i]);
        }
        Serial.println();
      }
      uint8_t pay[1] = { id };
      sendPacket(src, CMD_ACK_DATA, id, pay, 1);
      break;
    }

    default:
      Serial.println("  Tipo desconocido");
      break;
  }
}
