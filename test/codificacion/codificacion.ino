
// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
    /* [6, 12] Aumentar el spreading factor incrementa 
    de forma significativa el tiempo de Tx
    SPF = 6 es un valor especial
    Ver tabla 12 del manual del SEMTECH SX1276 */
  uint8_t codingRate;
  // [5, 8] 5 da un tiempo de Tx menor
  uint8_t txPower; 
    /*
    Rango [2, 20] en dBm
    Importante seleccionar un valor bajo para pruebas
    a corta distancia y evitar saturar al receptor
    */
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 6, 10, 5, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = 0;


// RSSI puede estar en un rango de [0, -127] dBm
int randomRSSI() {return -random(0, 128);}
uint8_t inRangeRSSI(int value){
  if (value > -50) return 2; //excelente
  else if(value < -80) return 1; //peor que bueno
  else return 0; // bueno
}

// SNR puede estar en un rango de [20, -148] dBm
float randomSNR() {return 20.0 - float(random(0, 101) / 100.0) * 168.0;}
uint8_t inRangeRSSI(float value){
  if (value > 10) return 2; //excelente
  else if(value < 5) return 1; //peor que muy bueno
  else return 0; // muy bueno
}

uint8_t package[50];



uint8_t encode(LoRaConfig_t * node, int rssi, float snr, uint8_t *package, LoRaConfig_t * next_config){
  uint8_t length = 0;
  package[length]    = (node->bandwidth_index << 4);
  package[length++] |= ((node->spreadingFactor - 6) << 1);
  package[length]    = ((node->codingRate - 5) << 6);
  package[length++] |= ((node->txPower - 2) << 1);

  // Incluimos el RSSI y el SNR del último paquete recibido
  // RSSI puede estar en un rango de [0, -127] dBm
  package[length++] = uint8_t(-rssi * 2);
  // SNR puede estar en un rango de [20, -148] dBm
  package[length++] = uint8_t(148 + snr);

  package[length]    = (next_config->bandwidth_index << 4);
  package[length++] |= ((next_config->spreadingFactor - 6) << 1);
  package[length]    = ((next_config->codingRate - 5) << 6);
  package[length++] |= ((next_config->txPower - 2) << 1);
}



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

