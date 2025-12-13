/* ---------------------------------------------------------------------
 *  Ejemplo MKR1310_LoRa_SendReceive_WithCallbacks
 *  Práctica 3
 *  Asignatura (GII-IoT)
 *  
 *  Basado en el ejemplo MKR1310_LoRa_SendReceive_WithReceiveCallback,
 *  muestra cómo es posible resolver la transmisión  
 *  y recepción de mensajes de forma asíncrona.
 *  Adicionalmente, monitorea el duty-cycle e intenta
 *  ajustar el intervalo entre la transmisión de paquetes
 *  para mantener el duty cycle por debajo del 1%.
 *  
 *  
 *  Este ejemplo requiere de una versión modificada
 *  de la librería Arduino LoRa (descargable desde 
 *  CV de la asignatura.
 *  
 *  También usa la librería Arduino_BQ24195 
 *  https://github.com/arduino-libraries/Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          1000

// NOTA: Ajustar estas variables 
#define LOCALADDRESS 0x06
#define DESTINATION 0x05
#define START_TX 2
#define START_BW 125000
#define START_SF 7

const uint8_t sync_txpower[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
const uint8_t SYNC_TX_COUNT  = sizeof(sync_txpower) / sizeof(sync_txpower[0]);

const long    sync_bw[] = {125000, 250000, 500000, 62500, 41700, 31250, 20800, 15600, 10400, 7800};
const uint8_t sync_sf[] = {7,      7,      7,      8,     9,     10,    11,    12,    12,    12};
const uint8_t SYNC_TESTS_COUNT = sizeof(sync_bw) / sizeof(sync_bw[0]);


volatile uint8_t txIndex = 0;

volatile bool txDoneFlag = true;
char pendingMsg[50];
enum SyncState {
  IDLE,
  SYNC_START,
  WAITSA_AFTERST,
  SEND_T,
  WAITSA_AFTERT,
  SEND_SE,
  SENDING,
  WAIT_SA_AFTER_ST,
  WAIT_SA_AFTER_TX,
  WAIT_SA_AFTER_SE_TX,
  WAIT_SA_AFTER_SI,
  WAIT_SA_AFTER_CONFIG,
  WAIT_SA_AFTER_SE_BWSF
};

volatile SyncState syncState;


// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);  
  while (!Serial); 

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");

  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  // Configuramos algunos parámetros de la radio
  LoRa.setSignalBandwidth(START_BW); // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
  LoRa.setSpreadingFactor(START_SF);     // [6, 12] Aumentar el spreading factor incrementa 
  // original 7, nosotros 12
  /*
    //12
    Sending 'Message no. 000 from 0xBB' ----> TX completed in 1652 msecs
    Duty cycle: 16.5 %

    Received from: 0x13
    Sent to: 0xff
    Message ID: 2
    Message length: 15
    Message: Roberto trabaja
    RSSI: -85 dBm
    SNR: 8.00
  */
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  LoRa.setSyncWord(0x12);         // Palabra de sincronización privada por defecto para SX127X 
                                  // Usaremos la palabra de sincronización para crear diferentes
                                  // redes privadas por equipos
  LoRa.setCodingRate4(5);         // [5, 8] 5 da un tiempo de Tx menor
  LoRa.setPreambleLength(8);      // Número de símbolos a usar como preámbulo

  LoRa.setTxPower(START_TX, PA_OUTPUT_PA_BOOST_PIN); // Rango [2, 20] en dBm
                                  // Importante seleccionar un valor bajo para pruebas
                                  // a corta distancia y evitar saturar al receptor

  // Indicamos el callback para cuando se reciba un paquete
  LoRa.onReceive(onReceive);
  
  // Nótese que la recepción está activada a partir de este punto
  LoRa.receive();

  // Activamos el callback que nos indicará cuando ha finalizado la 
  // transmisión de un mensaje
  LoRa.onTxDone(TxFinished);

  Serial.println("LoRa init succeeded.");
  syncState = IDLE;
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() 
{
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
  static bool transmitting = false;
  if (!transmitting && syncState==SEND_T && ((millis() - lastSendTime_ms) > txInterval_ms)){
    Serial.print(sync_txpower[txIndex]);

  snprintf(pendingMsg, sizeof(pendingMsg),
          "T%u",
          sync_txpower[txIndex]);
  txIndex++;
  if (txIndex > SYNC_TX_COUNT){
    txIndex = 0;
  }
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
  
    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.print("Sending '");
    Serial.print(pendingMsg);
    Serial.print("' ");
    syncState = WAITSA_AFTERT;
  }
  
  if (syncState == IDLE && !transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {

    snprintf(pendingMsg, sizeof(pendingMsg),"ST", 
             msgCount, LOCALADDRESS);

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();
  
    sendMessage(pendingMsg, uint8_t(strlen(pendingMsg)), msgCount);
    Serial.print("Sending '");
    Serial.print(pendingMsg);
    Serial.print("' ");
    syncState = WAITSA_AFTERST;
  }                  
  
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");
    
    // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms; 
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;
    
    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle,1);
    Serial.println(" %\n");

    // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
    // Dejamos random() solo para introducir cierta variabilidad
    // y verificar que el mecanismo corrector funciona
    if (duty_cycle <= 1.0f) {
      txInterval_ms = random(TX_LAPSE_MS) + 1000; 
    } else {
      txInterval_ms = TxTime_ms * 100;
    }
    
    transmitting = false;
    
    // Reactivamos la recepción de mensajes, que se desactiva
    // en segundo plano mientras se transmite
    LoRa.receive();   
  }
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount) 
{
  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);
  }
  LoRa.write(DESTINATION);                // Añadimos el ID del destinatario
  LoRa.write(LOCALADDRESS);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF)); 
  LoRa.write(msgLength);                  // Añadimos la longitud en bytes del mensaje
  LoRa.print(outgoing);                   // Añadimos el mensaje/payload
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
                                          // finalice su transmisión
  msgCount++;                             // Incrementamos el contador de mensajes
}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) 
{
  if (packetSize == 0) return;          // Si no hay mensajes, retornamos

  // Leemos los primeros bytes del mensaje
  char buffer[50];                      // Buffer para almacenar el mensaje
  int recipient = LoRa.read();          // Dirección del destinatario
  uint8_t sender = LoRa.read();         // Dirección del remitente
                                        // msg ID (High Byte first)
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | 
                            (uint16_t)LoRa.read();
  
  uint8_t incomingLength = LoRa.read(); // Longitud en bytes del mensaje
  
  uint8_t receivedBytes = 0;            // Leemos el mensaje byte a byte
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer)-1))) {            
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  buffer[receivedBytes] = '\0';         // Terminamos la cadena

  if (incomingLength != receivedBytes) {// Verificamos la longitud del mensaje
    Serial.print("Receiving error: declared message length " + String(incomingLength));
    Serial.println(" does not match length " + String(receivedBytes));
    return;                             
  }

  // Verificamos si se trata de un mensaje en broadcast o es un mensaje
  // dirigido específicamente a este dispositivo.
  // Nótese que este mecanismo es complementario al uso de la misma
  // SyncWord y solo tiene sentido si hay más de dos receptores activos
  // compartiendo la misma palabra de sincronización
  if ((recipient & LOCALADDRESS) != LOCALADDRESS ) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }
  if (syncState==WAITSA_AFTERST && strcmp(buffer, "SA") == 0){
    syncState=SEND_T;
    Serial.println("SA recibido");
  }
  if (syncState==WAITSA_AFTERT && strcmp(buffer, "SA") == 0){
    syncState=SEND_SE;
    Serial.println("SA recibido");
  }

  // Imprimimos los detalles del mensaje recibido
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + String(buffer));
  Serial.print("RSSI: " + String(LoRa.packetRssi()));
  Serial.println(" dBm\nSNR: " + String(LoRa.packetSnr()));
  Serial.println();
}

void TxFinished()
{
  txDoneFlag = true;
}

void testTX(){
  
}
