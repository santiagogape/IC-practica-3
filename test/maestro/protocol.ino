
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
  return encode_config_to_package(next_config, package);
}

/*
    Codifica la configuración inicial del nodo
    en un paquete para su envío.
    se usa esta directamente para el primer mensaje, luego se usa encode_from_master()
*/
uint8_t encode_config_to_package(LoRaConfig_t * initial_config, uint8_t *package){
  uint8_t payloadLength = 0;

    package[payloadLength]    = (initial_config->bandwidth_index << 4);
    package[payloadLength++] |= ((initial_config->spreadingFactor - 6) << 1);
    package[payloadLength]    = ((initial_config->codingRate - 5) << 6);
    package[payloadLength++] |= ((initial_config->txPower - 2) << 1);
    return payloadLength;
}


void configureLoRa(LoRaConfig_t * config){
    LoRa.idle();
    LoRa.setSignalBandwidth(long(bandwidth_kHz[config->bandwidth_index])); 
                                    // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                    // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                    // Multiplicar por dos el ancho de banda
                                    // supone dividir a la mitad el tiempo de Tx
                                    
    LoRa.setSpreadingFactor(config->spreadingFactor);     
                                    // [6, 12] Aumentar el spreading factor incrementa 
                                    // de forma significativa el tiempo de Tx
                                    // SPF = 6 es un valor especial
                                    // Ver tabla 12 del manual del SEMTECH SX1276
    
    LoRa.setCodingRate4(config->codingRate);         
                                    // [5, 8] 5 da un tiempo de Tx menor
                                    
    LoRa.setTxPower(config->txPower, PA_OUTPUT_PA_BOOST_PIN); 
                                    // Rango [2, 20] en dBm
                                    // Importante seleccionar un valor bajo para pruebas
                                    // a corta distancia y evitar saturar al receptor
    LoRa.receive();
}