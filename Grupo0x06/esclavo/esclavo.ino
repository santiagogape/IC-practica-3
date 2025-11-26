#include <SPI.h>
#include <LoRa.h>
#include <ArduinoPMIC.h>

// Parámetros radio iniciales y direcciones
const uint8_t localAddress = 0x05;      // Dirección de este esclavo
const uint8_t masterAddress = 0x06;     // Dirección maestro
const uint8_t SYNC_WORD = 0x12;
#define TIMEOUT_MS 10000

// Variables para configuración y respaldo
uint32_t lastPacketTime = 0;
uint8_t spreadingFactor = 7;
long bandwidth = 125E3;
uint8_t prev_spreadingFactor;
long prev_bandwidth;

// Estados de la máquina de sincronía
enum SlaveState {
  WAIT_SYNC,
  WAIT_CONFIG,
  WAIT_SYNCEND,
  WAIT_MSG
};
SlaveState state = WAIT_SYNC;

// Inicialización básica (igual que el original)
bool init_PMIC()
{
  bool error = false;
  if (!PMIC.begin()) {
    Serial.println("ERROR: Failed to initialize PMIC!");
    return false;
  }
  if (!PMIC.setInputCurrentLimit(2.0)) error = true;
  if (!PMIC.setInputVoltageLimit(3.88)) error = true;
  if (!PMIC.setMinimumSystemVoltage(3.5)) error = true;
  if (!PMIC.setChargeVoltage(4.2)) error = true;
  if (!PMIC.setChargeCurrent(0.375)) error = true;
  if (!PMIC.enableCharge()) error = true;
  delay(2000);
  return !error;
}

// Cambia la configuración LoRa
void updateRadio(uint8_t sf, long bw) {
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth(bw);
  spreadingFactor = sf;
  bandwidth = bw;
  Serial.print("Nueva config (SF, BW): "); Serial.print(sf); Serial.print(", "); Serial.println(bw);
}

// Reestablece la config anterior si hubo fallo
void restorePrevConfig() {
  updateRadio(prev_spreadingFactor, prev_bandwidth);
}

// Serializa y parsea los paquetes de tipo SX/XY
void parseConfigMsg(uint8_t *buf, uint8_t &sf, long &bw) {
  sf = buf[0]; // p.ej., SF=7
  // BW codificado como uint8_t (0=7.8,1=10.4,...,6=125,7=250,8=500 kHz, ver asignación si necesario)
  switch(buf[1]) {
    case 0: bw=7.8E3; break;   case 1: bw=10.4E3; break;  case 2: bw=15.6E3; break;
    case 3: bw=20.8E3; break;  case 4: bw=31.25E3; break; case 5: bw=62.5E3; break;
    case 6: bw=125E3; break;   case 7: bw=250E3; break;   case 8: bw=500E3; break;
    default: bw=125E3; break;
  }
}

// Envía tipo 'ACK' (confirmación genérica)
void sendACK(const char* ack_type) {
  LoRa.beginPacket();
  LoRa.write(masterAddress);
  LoRa.write(localAddress);
  LoRa.write((uint8_t)strlen(ack_type));
  LoRa.print(ack_type);
  LoRa.endPacket();
  Serial.print("Enviado ACK: "); Serial.println(ack_type);
}

// Callback recepción de paquete
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t recipient = LoRa.read();  // Destino
  uint8_t sender = LoRa.read();     // Remitente
  uint8_t msgLen = LoRa.read();

  uint8_t payload[32];
  for (int i = 0; i < msgLen && i < 32; i++)
    payload[i] = LoRa.read();

  String cmd = "";
  for (int i=0; i < msgLen; i++) cmd += (char)payload[i];
  lastPacketTime = millis();

  Serial.print("RX cmd: "); Serial.println(cmd);

  switch(state){
    case WAIT_SYNC:  // Maestro inicia con 'SI'
      if (cmd == "SI") {
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;
    case WAIT_CONFIG: // Esperar SXYY
      if (cmd.startsWith("S")) {
        // Guardar config anterior antes de aplicar
        prev_spreadingFactor = spreadingFactor;
        prev_bandwidth = bandwidth;
        uint8_t tmp_sf; long tmp_bw;
        parseConfigMsg(payload+1, tmp_sf, tmp_bw); // salta la 'S'
        updateRadio(tmp_sf, tmp_bw);
        sendACK("SA");
        state = WAIT_SYNCEND;
      } else if (cmd.startsWith("M")) {
          sendACK("MA");
          Serial.print("Mensaje recibido: ");
          Serial.println(cmd.substring(1));
          state = WAIT_MSG;
      }
      break;
    case WAIT_SYNCEND: // Espera SE
      if (cmd == "SE") {
        sendACK("SA");
        state = WAIT_CONFIG;
      }
      break;
    case WAIT_MSG:
      if (cmd.startsWith("M")) {
          sendACK("MA");
          Serial.print("Mensaje recibido: ");
          Serial.println(cmd.substring(1));
      } else if (cmd == "SI"){
          sendACK("SA");
          state = WAIT_CONFIG;
      }
      break;
    default:
      Serial.print("Error de protocolo");
      break;
  }
}

// Verifica timeout y restablece si necesario
void checkTimeout() {
  if ((millis() - lastPacketTime) > TIMEOUT_MS) {
    Serial.println("Timeout detectado. Restableciendo estado/configuración.");
    if (state == WAIT_SYNCEND) {
      // Si ya cambiamos config, volver a anterior y cero
      restorePrevConfig();
    }
    state = WAIT_SYNC; // Volver al principio del protocolo
    sendACK("SA"); // Intentar resposición con el maestro
    lastPacketTime = millis();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  if (!init_PMIC()) Serial.println("Initilization of BQ24195L failed!");
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa init failed. Check your connections."); while (true);
  }
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setPreambleLength(8);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);
  updateRadio(spreadingFactor, bandwidth);
  LoRa.onReceive(onReceive);
  LoRa.receive();
  lastPacketTime = millis();
  Serial.println("Esclavo LoRa listo.");
}

void loop() {
  checkTimeout();
  // La recepción es gestionada por interrupción/callback
}
