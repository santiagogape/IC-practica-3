#define TX_LAPSE_MS          10000

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0x06;     // Dirección de este dispositivo
uint8_t destination = 0x05;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 

} LoRaConfig_t;

inline bool EqualConfig(LoRaConfig_t * a, LoRaConfig_t * b){
  return a->bandwidth_index == b->bandwidth_index &&
         a->spreadingFactor  == b->spreadingFactor  &&
         a->codingRate       == b->codingRate       &&
         a->txPower          == b->txPower;

}

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 6, 10, 5, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = 0;


/*
    casos:

   maestro envia por primera vez
*   esclavo responde
    --- ciclo ---{
        maestro recibe respuesta y ajusta
        maestro envia con nueva config
*       esclavo responde
    }
*/


void decode_from_master(const uint8_t *package, LoRaConfig_t * node);
uint8_t encode_from_slave(LoRaConfig_t * config, int rssi, float snr, uint8_t *package);

void configureLoRa(LoRaConfig_t * config);