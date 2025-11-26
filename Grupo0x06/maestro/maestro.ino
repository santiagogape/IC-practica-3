/* ---------------------------------------------------------------------
 *  MKR1310_LoRa_SendReceive_WithCallbacks - PROTOCOLO SINCRONIZACIÓN
 *  Modificado para:
 *   - enviar "SI", esperar "SA"
 *   - enviar un valor de sync_tests, esperar "SA"
 *   - cambiar BW y SF, enviar "SE", esperar "SA" o restaurar si timeout
 * ---------------------------------------------------------------------
 */

#include <SPI.h>
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS          10000

// NOTA: Ajustar estas variables
const uint8_t localAddress = 0x06;     // Dirección de este dispositivo
uint8_t destination = 0x05;            // Dirección de destino, 0xFF broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión

// --- sync tests (valores de bandwidth/similares a los comentados)
const uint32_t sync_tests[] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};
const uint8_t SYNC_TESTS_COUNT = sizeof(sync_tests) / sizeof(sync_tests[0]);

// ------------------ Máquina de estados del protocolo -------------------
enum SyncState {
  IDLE,
  SENDING,                 // estado temporal: estamos transmitiendo algo de protocolo
  WAIT_SA_AFTER_SI,        // esperamos SA tras enviar "SI"
  WAIT_SA_AFTER_TEST,      // esperamos SA tras enviar un test
  WAIT_SA_AFTER_CONFIG     // esperamos SA tras enviar "SE" (después de cambiar config)
};

volatile SyncState syncState = IDLE;

// Si después de una transmisión de protocolo debe ponerse un estado concreto,
// lo guardamos aquí para activarlo cuando termine la Tx (en el manejo de txDone)
volatile SyncState protocolPendingState = IDLE;

// Tiempo de espera para recibir el "SA"
const uint32_t WAIT_TIMEOUT_MS = 5000; // ajustar si se quiere mayor tolerancia
uint32_t waitStartTime = 0;

// Variables para manejar envíos desde loop (no desde callback)
char pendingMsg[50];
uint8_t pendingMsgLen = 0;
volatile bool pendingSend = false;

// Guardado/restauración de configuración LoRa
uint32_t currentBW = 125000;       // mantendremos manualmente (no todos los cores LoRa tienen getter)
int currentSF = 12;
uint32_t prevBW = 125000;
int prevSF = 12;

// índice actual en sync_tests
uint8_t syncIndex = 0;

// Variables del ejemplo original
static uint32_t lastSendTime_ms = 0;
static uint16_t msgCount = 0;
static uint32_t txInterval_ms = TX_LAPSE_MS;
static uint32_t tx_begin_ms = 0;
static bool transmitting = false;

// Prototipos
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount);
void onReceive(int packetSize);
void TxFinished();

// --------------------------------------------------------------------
// Setup
// --------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Duplex with TxDone and Receive callbacks - PROTOCOLO");

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  } else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  // Config inicial (igual que tu sketch)
  LoRa.setSignalBandwidth(125E3);
  currentBW = 125000;
  LoRa.setSpreadingFactor(12);
  currentSF = 12;
  LoRa.setSyncWord(0x12);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setTxPower(3, PA_OUTPUT_PA_BOOST_PIN);

  LoRa.onReceive(onReceive);
  LoRa.receive();

  LoRa.onTxDone(TxFinished);

  Serial.println("LoRa init succeeded.");
}

// --------------------------------------------------------------------
// Loop: maneja envíos pendientes y la máquina de estados / timeouts
// --------------------------------------------------------------------
void loop()
{
  // Mecanismo original para medir duty-cycle cuando hacemos otros envíos
  static bool idlePrinted = false;

  // Si hay un envío pendiente solicitado por el protocolo y no estamos transmitiendo,
  // iniciamos la transmisión desde aquí (no desde el callback).
  if (pendingSend && !transmitting) {
    // preparar para enviar
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    // realizamos el envío
    sendMessage(pendingMsg, pendingMsgLen, msgCount);

    Serial.print("Protocol sending '");
    Serial.print(pendingMsg);
    Serial.println("'");

    // después de llamar a sendMessage dejamos que el flujo normal gestione el fin de TX
    pendingSend = false;
    // marcar estado temporal SENDING (para que al terminar el tx sepa que era protocolo)
    syncState = SENDING;
    idlePrinted = false;
  }

  // Manejo cuando transmisión finaliza (señalizada por TxFinished())
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");

    // Ajuste duty-cycle (tu lógica original)
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;
    float duty_cycle = (100.0f * TxTime_ms) / (lapse_ms ? lapse_ms : 1);

    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    if (duty_cycle <= 1.0f) {
      txInterval_ms = random(TX_LAPSE_MS) + 1000;
    } else {
      txInterval_ms = TxTime_ms * 100;
    }

    // Si este envío formaba parte del protocolo, activamos el estado pendiente
    if (protocolPendingState != IDLE) {
      syncState = protocolPendingState;
      protocolPendingState = IDLE;
      waitStartTime = millis(); // iniciamos timeout de espera del SA
      Serial.print("Protocol moved to state: ");
      switch (syncState) {
        case WAIT_SA_AFTER_SI: Serial.println("WAIT_SA_AFTER_SI"); break;
        case WAIT_SA_AFTER_TEST: Serial.println("WAIT_SA_AFTER_TEST"); break;
        case WAIT_SA_AFTER_CONFIG: Serial.println("WAIT_SA_AFTER_CONFIG"); break;
        default: Serial.println("OTHER"); break;
      }
    } else {
      // Si no era protocolo, volvemos a recepción normal
      LoRa.receive();
    }

    transmitting = false;
  }

  // Timeout handling: si estamos esperando un "SA" y pasa el tiempo, hacemos recuperación
  if ((syncState == WAIT_SA_AFTER_SI || syncState == WAIT_SA_AFTER_TEST || syncState == WAIT_SA_AFTER_CONFIG)
      && (millis() - waitStartTime > WAIT_TIMEOUT_MS)) {
    Serial.println("Timeout esperando SA. Accion de recuperacion.");

    if (syncState == WAIT_SA_AFTER_CONFIG) {
      // restaurar configuración previa
      Serial.println("Restaurando configuración previa LoRa");
      LoRa.setSignalBandwidth(prevBW);
      currentBW = prevBW;
      LoRa.setSpreadingFactor(prevSF);
      currentSF = prevSF;
    }

    // reiniciamos el protocolo
    syncState = IDLE;
    protocolPendingState = IDLE;
    pendingSend = false;
    Serial.println("Volviendo a IDLE.");
    LoRa.receive();
  }

  // En modo IDLE podemos, si queremos, lanzar el SI automáticamente:
  // Si quieres que SI se genere automáticamente al inicio, descomenta lo siguiente
  static bool initialSiSent = false;
  if (!initialSiSent && syncState == IDLE && !transmitting) {
    // Enviamos "SI"
    strcpy(pendingMsg, "SI");
    pendingMsgLen = 2;
    pendingSend = true;
    // Cuando acabe el Tx, queremos entrar en WAIT_SA_AFTER_SI
    protocolPendingState = WAIT_SA_AFTER_SI;
    initialSiSent = true;
  }

  // pequeña espera para liberar CPU y evitar bucle muy apretado
  delay(10);
}

// --------------------------------------------------------------------
// Sending message function (se mantiene tu implementación)
// --------------------------------------------------------------------
void sendMessage(char* outgoing, uint8_t msgLength, uint16_t &msgCount)
{
  while (!LoRa.beginPacket()) {
    delay(10);
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(msgLength);                  // Añadimos la longitud en bytes del mensaje
  LoRa.print(outgoing);                   // Añadimos el mensaje/payload
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a su transmisión
  msgCount++;                             // Incrementamos el contador de mensajes
}

// --------------------------------------------------------------------
// Receiving message function (callback)
//  - NO enviamos directamente desde aquí; preparamos mensajes para loop()
// --------------------------------------------------------------------
void onReceive(int packetSize)
{
  if (packetSize == 0) return;

  char buffer[50];
  int recipient = LoRa.read();
  uint8_t sender = LoRa.read();
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) |
                            (uint16_t)LoRa.read();
  uint8_t incomingLength = LoRa.read();

  uint8_t receivedBytes = 0;
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer) - 1))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  buffer[receivedBytes] = '\0';

  if (incomingLength != receivedBytes) {
    Serial.print("Receiving error: declared message length " + String(incomingLength));
    Serial.println(" does not match length " + String(receivedBytes));
    return;
  }

  // Filtro por destinatario (mecanismo complementario)
  if ((recipient & localAddress) != localAddress) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }

  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + String(buffer));
  Serial.print("RSSI: " + String(LoRa.packetRssi()));
  Serial.println(" dBm\nSNR: " + String(LoRa.packetSnr()));
  Serial.println();

  // Interpretamos mensajes de sincronización: "SI", "SA", "SE" y números
  if (receivedBytes >= 2 && buffer[0] == 'S') {
    if (buffer[1] == 'A') {
      Serial.println("Sync Acknowledge (SA) recibido.");

      // Reaccionamos según el estado actual esperado
      if (syncState == WAIT_SA_AFTER_SI) {
        // Debemos enviar un sync_test (valor) y despues esperar otro SA
        // Preparamos mensaje en pendingMsg para enviarlo desde loop()
        uint32_t val = sync_tests[syncIndex % SYNC_TESTS_COUNT];
        // convertir a cadena
        snprintf(pendingMsg, sizeof(pendingMsg), "%lu", (unsigned long)val);
        pendingMsgLen = strlen(pendingMsg);
        pendingSend = true;
        // cuando acabe el Tx, pasamos a WAIT_SA_AFTER_TEST
        protocolPendingState = WAIT_SA_AFTER_TEST;
        Serial.print("Preparado para enviar sync_test: ");
        Serial.println(pendingMsg);

        // incrementamos índice para la próxima vez si se quiere
        syncIndex = (syncIndex + 1) % SYNC_TESTS_COUNT;
      }
      else if (syncState == WAIT_SA_AFTER_TEST) {
        // Cuando recibimos SA tras el test, cambiamos configuración y enviamos "SE"
        // Guardamos configuración previa
        prevBW = currentBW;
        prevSF = currentSF;

        // Aplicamos nueva configuración (ejemplo: cambiar BW y SF)
        // Ajusta estos valores a lo que quieras probar:
        uint32_t newBW = 250000; // ejemplo: 250 kHz
        int newSF = 9;           // ejemplo: SF9

        Serial.print("Cambiando configuración LoRa. BW ");
        Serial.print(currentBW);
        Serial.print(" -> ");
        Serial.print(newBW);
        Serial.print(", SF ");
        Serial.print(currentSF);
        Serial.print(" -> ");
        Serial.println(newSF);

        LoRa.setSignalBandwidth((double)newBW);
        currentBW = newBW;
        LoRa.setSpreadingFactor(newSF);
        currentSF = newSF;

        // Preparamos "SE" y tras su TX esperamos otro SA (WAIT_SA_AFTER_CONFIG)
        strcpy(pendingMsg, "SE");
        pendingMsgLen = 2;
        pendingSend = true;
        protocolPendingState = WAIT_SA_AFTER_CONFIG;
        Serial.println("Preparado para enviar SE tras cambiar config.");
      }
      else if (syncState == WAIT_SA_AFTER_CONFIG) {
        // Éxito: SA final recibido confirmando el SE
        Serial.println("SA final recibido: configuración confirmada.");
        // Volver al estado IDLE (o al comportamiento normal que prefieras)
        syncState = IDLE;
        protocolPendingState = IDLE;
        pendingSend = false;
        LoRa.receive();
      }
      else {
        // SA recibido en un estado inesperado -> lo ignoramos o registramos
        Serial.println("SA recibido pero estado no esperado. Ignorando.");
      }
    }
    else if (buffer[1] == 'I') {
      Serial.println("SI recibido (otro dispositivo inicia sincronización).");
      // podrías responder SA si el protocolo lo requiere; aquí no hacemos nada.
    }
    else if (buffer[1] == 'E') {
      Serial.println("SE recibido (sync end de otro dispositivo).");
    }
  }
}

// --------------------------------------------------------------------
// Tx finished callback
// --------------------------------------------------------------------
void TxFinished()
{
  txDoneFlag = true;
}
