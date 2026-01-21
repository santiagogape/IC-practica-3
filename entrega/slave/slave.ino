/* ------------------------------------------------------------
 * ESCLAVO LoRa - Protocolo de Sincronización Mejorado
 * Dirección local: 0x05, Maestro: 0x06, SyncWord: 0x12
 * - FSM robusta para TX y BW/SF
 * - Respuestas a QS (SSX...Y...T...) y RS (resync)
 * - ACK "MA" ante mensajes "MXXXXXXXXX"
 * - Mejor manejo de timeouts y rollback
 * ------------------------------------------------------------ */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

const uint8_t localAddress  = 0x05;
const uint8_t masterAddress = 0x06;
const uint8_t SYNC_WORD     = 0x12;

#define TIMEOUT_MS 2500

// Config original segura
#define ORIGINAL_BW 125000L
#define ORIGINAL_SF 7
#define ORIGINAL_TX 3

// Estado y configuración
uint32_t lastPacketTime = 0;
uint8_t  spreadingFactor = ORIGINAL_SF;
long     bandwidth = ORIGINAL_BW;
uint8_t  txPower = ORIGINAL_TX;

// Copias previas para rollback
uint8_t  prev_spreadingFactor = ORIGINAL_SF;
long     prev_bandwidth = ORIGINAL_BW;
uint8_t  prev_tx = ORIGINAL_TX;

// Mejor configuración conocida
uint8_t  best_tx = ORIGINAL_TX;
long     best_bw = ORIGINAL_BW;
uint8_t  best_sf = ORIGINAL_SF;

uint16_t messageCount = 0;

// ----------------- Helpers -----------------
bool isValidBandwidth(long bw) {
  const long allowed[] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};
  for (unsigned i = 0; i < sizeof(allowed)/sizeof(allowed[0]); ++i) {
    if (bw == allowed[i]) return true;
  }
  return false;
}

void applyRadioBW_SF(uint8_t sf, long bw) {
  if (!isValidBandwidth(bw) || sf < 6 || sf > 12) {
    Serial.println("!!! Config inválida, usando defaults !!!");
    sf = ORIGINAL_SF; bw = ORIGINAL_BW;
  }
  LoRa.idle();
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth(bw);
  spreadingFactor = sf;
  bandwidth       = bw;
  Serial.print(" Config aplicada - BW: "); Serial.print(bw);
  Serial.print(" Hz, SF: "); Serial.println(sf);
  delay(10);
  LoRa.receive();
}

void applyRadioTX(uint8_t tx) {
  if (tx < 2 || tx > 20) {
    Serial.println("!!! TX inválido, usando default !!!");
    tx = ORIGINAL_TX;
  }
  LoRa.idle();
  LoRa.setTxPower(tx, PA_OUTPUT_PA_BOOST_PIN);
  txPower = tx;
  Serial.print(" TX aplicado: "); Serial.println(tx);
  delay(10);
  LoRa.receive();
}

void restorePrevConfig() {
  Serial.println(" Restaurando BW/SF anterior");
  applyRadioBW_SF(ORIGINAL_SF, ORIGINAL_BW);
}

void restorePrevTX() {
  Serial.println(" Restaurando TX anterior");
  applyRadioTX(prev_tx);
}

void restoreBestKnown() {
  Serial.println(" Restaurando mejor config conocida");
  applyRadioBW_SF(best_sf, best_bw);
  applyRadioTX(best_tx);
}

void sendACK(const char* outgoing) {
  uint8_t msgLength = (uint8_t)strlen(outgoing);
  LoRa.idle();
  delay(5);
  while (!LoRa.beginPacket()) { delay(5); }
  LoRa.write(masterAddress);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)(messageCount >> 8));
  LoRa.write((uint8_t)(messageCount & 0xFF));
  LoRa.write(msgLength);
  LoRa.print(outgoing);
  LoRa.endPacket();
  messageCount++;
  Serial.print(">> ACK enviado: '"); Serial.print(outgoing); Serial.println("'");
  delay(10);
  LoRa.receive();
}

bool parseXBWYSF(const String& cmd, long* bw, uint8_t* sf) {
  if (cmd.length() < 5 || cmd[0] != 'X') return false;
  int idxY = cmd.indexOf('Y');
  if (idxY <= 1 || idxY >= cmd.length()-1) return false;
  long bwVal = cmd.substring(1, idxY).toInt();
  int  sfVal = cmd.substring(idxY + 1).toInt();
  if (!isValidBandwidth(bwVal) || sfVal < 6 || sfVal > 12) return false;
  *bw = bwVal; *sf = (uint8_t)sfVal;
  return true;
}

bool parseTX(const String& cmd, uint8_t* tx) {
  if (cmd.length() < 2 || cmd[0] != 'T') return false;
  int txVal = cmd.substring(1).toInt();
  if (txVal < 2 || txVal > 20) return false;
  *tx = (uint8_t)txVal;
  return true;
}

// ----------------- FSM -----------------
enum SlaveState {
  WAIT_SYNC,
  WAIT_CONFIG,
  WAIT_SYNCEND,
  WAIT_TX,
  READY
};
SlaveState state = WAIT_SYNC;

// ----------------- onReceive -----------------
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t recipient = LoRa.read();
  uint8_t sender    = LoRa.read();
  uint16_t msgID    = ((uint16_t)LoRa.read() << 8) | (uint16_t)LoRa.read();
  uint8_t  msgLen   = LoRa.read();

  if (recipient != localAddress && recipient != 0xFF) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  char payload[60];
  uint8_t i = 0;
  while (LoRa.available() && i < sizeof(payload)-1 && i < msgLen) {
    payload[i++] = (char)LoRa.read();
  }
  payload[i] = '\0';
  String cmd = String(payload);

  lastPacketTime = millis();
  Serial.print("<< Recibido: '"); Serial.print(cmd);
  Serial.print("'  RSSI: "); Serial.print(LoRa.packetRssi());
  Serial.print(" dBm  SNR: "); Serial.println(LoRa.packetSnr());

  // ----- Control inmediato: RS y QS -----
  if (cmd == "RS") {
    Serial.println("** RESYNC (RS) **");
    applyRadioBW_SF(ORIGINAL_SF, ORIGINAL_BW);
    applyRadioTX(ORIGINAL_TX);
    sendACK("SA");
    state = WAIT_SYNC;
    delay(10); LoRa.receive();
    return;
  }
  if (cmd == "QS") {
    char buf[48];
    snprintf(buf, sizeof(buf), "SSX%ldY%uT%u", bandwidth, spreadingFactor, txPower);
    sendACK(buf);
    delay(10); LoRa.receive();
    return;
  }

  // ----- ACK "MA" para mensajes "M..." -----
  if (cmd.length() >= 1 && cmd[0] == 'M') {
    sendACK("MA");
    Serial.print(" Mensaje M: "); Serial.println(cmd.substring(1));
    // No cambiar estado si estamos en READY
    if (state != READY) state = READY;
    return;
  }

  // ----- Protocolo principal -----
  switch (state) {
    case WAIT_SYNC:
      if (cmd == "SI") {
        Serial.println(" -> WAIT_CONFIG (BW/SF)");
        sendACK("SA"); 
        state = WAIT_CONFIG;
      } else if (cmd == "ST") {
        Serial.println(" -> WAIT_TX");
        sendACK("SA"); 
        state = WAIT_TX;
      }
      break;

    case WAIT_CONFIG:
      if (cmd.startsWith("X")) {
        prev_spreadingFactor = spreadingFactor;
        prev_bandwidth       = bandwidth;
        uint8_t tmp_sf; long tmp_bw;
        if (parseXBWYSF(cmd, &tmp_bw, &tmp_sf)) {
          sendACK("SA");
          applyRadioBW_SF(tmp_sf, tmp_bw);
          Serial.println(" -> WAIT_SYNCEND");
          state = WAIT_SYNCEND;
        } else {
          Serial.println(" Error: formato XBWYSF inválido");
        }
      } else if (cmd == "SI") {
        Serial.println(" Re-inicio SI en WAIT_CONFIG");
        sendACK("SA");
      } else if (cmd == "ST") {
        Serial.println(" Cambio a TX desde WAIT_CONFIG");
        sendACK("SA"); 
        state = WAIT_TX;
      }
      break;

    case WAIT_SYNCEND:
      if (cmd == "SE") {
        sendACK("SA");
        // Guardar como mejor configuración conocida
        best_tx = txPower;
        best_sf = spreadingFactor;
        best_bw = bandwidth;
        Serial.println(" Ciclo cerrado con SE");
        Serial.println(" -> WAIT_SYNC");
        state = WAIT_SYNC;
      }
      else if (cmd == "SI") {
        Serial.println(" SI en WAIT_SYNCEND: rollback BW/SF");
        restorePrevConfig();
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      else if (cmd == "ST") {
        Serial.println(" ST en WAIT_SYNCEND: rollback TX");
        restorePrevTX();
        sendACK("SA");
        state = WAIT_TX;
      }
      else if (cmd.startsWith("M")) {
        sendACK("MA");
        state = READY;
      }
      break;

    case WAIT_TX:
      if (cmd.startsWith("T")) {
        prev_tx = txPower;
        uint8_t tmp_tx;
        if (parseTX(cmd, &tmp_tx)) {
          sendACK("SA");
          applyRadioTX(tmp_tx);
          Serial.println(" -> WAIT_SYNCEND");
          state = WAIT_SYNCEND;
        } else {
          Serial.println(" Error: formato TX inválido");
        }
      } else if (cmd == "SI") {
        Serial.println(" Cambio a BW/SF desde WAIT_TX");
        sendACK("SA"); 
        state = WAIT_CONFIG;
      } else if (cmd == "ST") {
        Serial.println(" Re-inicio ST en WAIT_TX");
        sendACK("SA");
      } else if (cmd.startsWith("M")) {
        sendACK("MA");
        Serial.print(" Mensaje: "); Serial.println(cmd.substring(1));
      }
      break;

    case READY:
      if (cmd.startsWith("M")) {
        sendACK("MA"); 
        Serial.print(" Mensaje: "); Serial.println(cmd.substring(1));
      } else if (cmd == "SI") {
        Serial.println(" Nueva sincronización BW/SF desde READY");
        sendACK("SA"); 
        state = WAIT_CONFIG;
      } else if (cmd == "ST") {
        Serial.println(" Nueva sincronización TX desde READY");
        sendACK("SA"); 
        state = WAIT_TX;
      }
      break;

    default:
      Serial.println("ERROR - Estado desconocido");
      state = WAIT_SYNC;
      break;
  }

  Serial.println();
}

// ----------------- Timeout & mantenimiento -----------------
void checkTimeout() {
  if (state == READY) {
    esclavoMaintenanceTick();
  } else if ((millis() - lastPacketTime) > TIMEOUT_MS) {
    Serial.println("\n!!! TIMEOUT detectado !!!");
    Serial.println("Restaurando mejor configuración conocida\n");
    restoreBestKnown();
    state = WAIT_SYNC;
    LoRa.receive();
    lastPacketTime = millis();
  }
}

void esclavoMaintenanceTick() {
  static uint32_t lastTick = 0;
  if (millis() - lastTick > 500) {
    LoRa.receive();
    lastTick = millis();
  }
}

// ----------------- Setup & Loop -----------------
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("=== ESCLAVO LoRa - Protocolo Sincronización Mejorado ===");

  if (!init_PMIC()) Serial.println("Aviso: BQ24195L no inicializado");
  else              Serial.println("OK: BQ24195L inicializado");

  if (!LoRa.begin(868E6)) {
    Serial.println("Error: LoRa init failed");
    while (true);
  }
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setPreambleLength(8);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  applyRadioBW_SF(spreadingFactor, bandwidth);
  applyRadioTX(txPower);

  LoRa.onReceive(onReceive);
  LoRa.receive();
  lastPacketTime = millis();

  Serial.print("Config inicial - BW: "); Serial.print(bandwidth);
  Serial.print(" Hz, SF: "); Serial.print(spreadingFactor);
  Serial.print(", TX: "); Serial.println(txPower);
  Serial.println("Esperando sincronización...");
  Serial.println("===========================================\n");
}

void loop() {
  checkTimeout();
  esclavoMaintenanceTick();
  delay(10);
}