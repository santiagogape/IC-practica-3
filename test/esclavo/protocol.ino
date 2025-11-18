
// RSSI puede estar en un rango de [0, -127] dBm
int randomRSSI() {return -random(0, 128);}
uint8_t inRangeRSSI(int value){
  if (value > -50) return 2; //excelente
  else if(value < -80) return 1; //peor que bueno
  else return 0; // bueno
}

// SNR puede estar en un rango de [20, -148] dBm
float randomSNR() {return 20.0 - float(random(0, 101) / 100.0) * 168.0;}
uint8_t inRangeSNR(float value){
  if (value > 10) return 2; //excelente
  else if(value < 5) return 1; //peor que muy bueno
  else return 0; // muy bueno
}

void increaseBandwidth(LoRaConfig_t * config){
  if (config->bandwidth_index < 9)
    config->bandwidth_index += 1;
}

void decreaseBandwidth(LoRaConfig_t * config){
  if (config->bandwidth_index > 0)
    config->bandwidth_index -= 1;
}

void increaseSpreadingFactor(LoRaConfig_t * config){
  if (config->spreadingFactor < 12)
    config->spreadingFactor += 1;
}

void decreaseSpreadingFactor(LoRaConfig_t * config){
  if (config->spreadingFactor > 6)
    config->spreadingFactor -= 1;
}

void increaseCodingRate(LoRaConfig_t * config){
  if (config->codingRate < 8)
    config->codingRate += 1;
}
void decreaseCodingRate(LoRaConfig_t * config){
  if (config->codingRate > 5)
    config->codingRate -= 1;
}

void increaseTxPower(LoRaConfig_t * config){
  if (config->txPower < 20)
    config->txPower += 1;
}

void decreaseTxPower(LoRaConfig_t * config){
  if (config->txPower > 2)
    config->txPower -= 1;
}

/*
    Calcula la siguiente configuración a partir de la
    configuración actual del nodo esclavo y los parámetros
    de calidad de la señal recibida (RSSI y SNR)
*/
void nextConfigFromSlave(const LoRaConfig_t *current,
                         int rssi, float snr,
                         LoRaConfig_t *next)
{
    *next = *current;

    // --- Rescate del enlace (señal mala) ---
    if (snr < -5) {
        increaseCodingRate(next);     // lo más fuerte primero
        return;
    }
    if (snr < 0) {
        increaseSpreadingFactor(next);
        return;
    }
    if (rssi < -80) {
        increaseTxPower(next);
        return;
    }

    // --- Mejora de velocidad (señal excelente) ---
    if (snr > 10 && rssi > -60) {
        decreaseSpreadingFactor(next);
        return;
    }

    if (snr > 7 && rssi > -55) {
        decreaseCodingRate(next);
        return;
    }

    if (snr > 12 && rssi > -50) {
        increaseBandwidth(next);      // más velocidad
        return;
    }

    if (rssi > -40) {
        decreaseTxPower(next);        // evitar saturación
        return;
    }
}

/*
    Decodifica el paquete recibido desde el esclavo
    y serializa la configuración del nodo, el RSSI y el SNR
*/
void decode_from_slave(const uint8_t *package, LoRaConfig_t * node, int *rssi, float *snr){
    node->bandwidth_index = package[0] >> 4;
    node->spreadingFactor = 6 + ((package[0] & 0x0F) >> 1);
    node->codingRate = 5 + (package[1] >> 6);
    node->txPower = 2 + ((package[1] & 0x3F) >> 1);
    *rssi = -int(package[2]) / 2.0f;
    *snr  =  float(package[3]) - 148;
}

/*
    Codifica el paquete a enviar al esclavo
    a partir de la configuración actual del nodo maestro
    y la configuración sugerida para el siguiente estado.
    Input: previous_config, rssi, snr
    Output: next_config, package
    Returns: length of the package
*/
uint8_t encode_from_master(LoRaConfig_t * previous_config, int rssi, float snr, uint8_t *package, LoRaConfig_t * next_config){

  nextConfigFromSlave(previous_config, rssi, snr, next_config);
  uint8_t length = 0;

  package[length]    = (next_config->bandwidth_index << 4);
  package[length++] |= ((next_config->spreadingFactor - 6) << 1);
  package[length]    = ((next_config->codingRate - 5) << 6);
  package[length++] |= ((next_config->txPower - 2) << 1);
    
  return length;
}

