# PRACTICA 3 - LORA
+ Alberto Martel Rodríguez
+ Jose
+ Nicolás Rey Alfonso
+ Santiago Galindo Peralta

---
# Protocolo LoRa V3: Sincronización, Calibración, Configuración Óptima y Estabilidad

Proyecto dividido en:

```
/master/master.ino
/master/master.h
/slave/slave.ino
/slave/slave.h
/shared/lora_protocol_shared.h
/shared/BQ24195L_PMIC.ino
```

---

# 1.  Arquitectura General del Protocolo

El sistema está compuesto por dos nodos LoRa:

* **Master**
* **Slave**

Ambos deben intercambiar configuraciones LoRa (SF, BW, CR, TxPower) y mantener sincronización permanente durante todo el tiempo de operación.

El protocolo completo se divide en tres fases:

1. **SYNC** – Sincronización rápida inicial
2. **CALIBRATION** – Prueba sistemática de configuraciones
3. **STABLE** – Estado operativo donde se intercambian datos
4. **RECOVERY** – Estado adicional para recuperación ante fallos o pérdida de sincronización

---

## 1.1 Estados del Master

```
MasterMode:
  SYNC
  CALIBRATION
  STABLE
  RECOVERY
```

## 1.2 Estados del Slave

```
SlaveMode:
  SCAN_FOR_MASTER
  SYNCED
  STABLE
```

---

# 2.  Tipos de Mensaje (MessageType)

Definidos en `lora_protocol_shared.h`.

## 2.1 Protocolos de sincronización

| Código (hex) | Nombre           | Descripción                                |
| ------------ | ---------------- | ------------------------------------------ |
| 0x00         | MSG_SYNC_START   | Inicia la sincronización                   |
| 0x01         | MSG_SYNC_REPLY   | El esclavo confirma que oyó el SYNC        |
| 0x02         | MSG_SYNC_CONFIRM | El maestro confirma que detectó al esclavo |

## 2.2 Búsqueda de configuración óptima

| Código (hex) | Nombre             | Descripción                          |
| ------------ | ------------------ | ------------------------------------ |
| 0x10         | MSG_CONFIG_TEST    | Paquete de prueba con SF/BW/CR       |
| 0x11         | MSG_CONFIG_RESULT  | Respuesta del esclavo con RSSI/SNR   |
| 0x12         | MSG_CONFIG_FINAL   | Master informa configuración óptima  |
| 0x13         | MSG_CONFIG_CONFIRM | Esclavo confirma configuración final |

## 2.3 Mensajes operacionales

| Código | Nombre   | Descripción                    |
| ------ | -------- | ------------------------------ |
| 0x20   | MSG_DATA | Datos normales en fase estable |
| 0x21   | MSG_ACK  | Reconocimiento genérico        |

## 2.4 Mensajes de recuperación

| Código | Nombre               | Descripción                            |
| ------ | -------------------- | -------------------------------------- |
| 0x30   | MSG_ROLLBACK         | Master ordena volver a config anterior |
| 0x31   | MSG_ROLLBACK_CONFIRM | Esclavo confirma rollback              |

---

# 3. Formato del Paquete LoRa

Todos los mensajes intercambiados entre nodos tienen formato fijo:

```
  Byte 0:  DESTINATION
  Byte 1:  SENDER
  Byte 2:  TYPE
  Byte 3:  MSG_ID_HIGH
  Byte 4:  MSG_ID_LOW
  Byte 5:  PAYLOAD_LENGTH
  Byte 6..N-2: Payload
  Byte N-1: CRC (XOR simple)
```

CRC = XOR de todos los bytes del payload.

---

# 4.  Fase SYNC

## 4.1 Descripción

El master inicia la comunicación intentando primero la configuración “segura”:

* SF = 10
* BW = 125 kHz
* CR = 4/5
* TxPower = 10–14 dBm

El slave se encuentra rotando configuraciones predefinidas hasta oír un `MSG_SYNC_START`.

## 4.2 Flujo (Mermaid)

```
sequenceDiagram
    Master->>Slave: MSG_SYNC_START
    Slave->>Master: MSG_SYNC_REPLY
    Master->>Slave: MSG_SYNC_CONFIRM
    Slave->>Master: MSG_ACK
```

Cuando esta secuencia se completa, ambos avanzan a la fase de **calibración**.

---

# 5.  Fase CALIBRATION (Búsqueda de Configuración Óptima)

El master evalúa múltiples configuraciones LoRa:

* BW: 500, 250, 125, 62.5, 41.7 kHz
* SF: 7 a 12
* CR: 4/5 a 4/8

Proceso:

1. El master envía `MSG_CONFIG_TEST(sf, bw, cr)`
2. El esclavo adapta temporalmente SF/BW
3. El esclavo responde `MSG_CONFIG_RESULT(rssi, snr)`
4. El master evalúa:

   * rssi >= threshold
   * snr >= threshold
   * estabilidad (respuesta en < 10 s)
5. Si una config falla → rollback
6. Si obtiene estabilidad perfecta → marca como candidata

Cuando termina la exploración se envía:

```
MSG_CONFIG_FINAL(sf, bw, cr)
```

y el esclavo responde:

```
MSG_CONFIG_CONFIRM
```

Ambos entran a **STABLE**.

---

# 6.  Fase STABLE

En esta fase se intercambia `MSG_DATA`.

El mensaje incluye:

* Config LoRa del nodo remoto
* RSSI
* SNR

En caso de pérdida de comunicación por más de 10 s:

* El master entra a **RECOVERY**
* El slave espera un `MSG_ROLLBACK` o un nuevo `MSG_SYNC_START`

---

# 7.  Fase RECOVERY

Si falla una configuración durante calibración o en STABLE:

```
Master → Slave : MSG_ROLLBACK
Slave  → Master: MSG_ROLLBACK_CONFIRM
```

Después:

1. Ambos vuelven a última configuración estable
2. El master verifica conectividad mediante `MSG_DATA`
3. Si vuelve a fallar → retorna a fase SYNC

---

# 8.  RFC-Style Specification (Versión Formal)

## 8.1 Abstract

Este documento especifica el Protocolo LoRa V3 para sincronización, negociación de parámetros físicos y comunicación estable entre un nodo Master y un nodo Slave. El protocolo proporciona sincronización robusta, exploración de parámetros, recuperación ante fallos y mensajería confiable mediante CRC y ACK opcional.

---

## 8.2 Terminología

* **MUST**: Requisito obligatorio
* **SHOULD**: Requisito recomendado
* **MAY**: Opcional
* **Master**: Nodo iniciador
* **Slave**: Nodo receptor pasivo
* **SYNC**: Fase inicial obligatoria
* **CALIBRATION**: Pruebas activas de parámetros
* **STABLE**: Estado operativo normal
* **RECOVERY**: Estado para restaurar comunicación
* **Safe Config**: Parámetros LoRa que garantizan alta probabilidad de inicialización

---

## 8.3 Reglas del Protocolo

### 8.3.1 Reglas del Master

1. El Master **MUST** iniciar enviar `MSG_SYNC_START` inmediatamente tras configurar LoRa en modo seguro.
2. El Master **MUST** esperar `MSG_SYNC_REPLY` antes de avanzar.
3. El Master **MUST** explorar configuraciones en orden definido (BW descendente, SF ascendente).
4. El Master **MUST** descartar cualquier respuesta cuyo CRC sea inválido.
5. El Master **MUST** aplicar rollback si no recibe `MSG_CONFIG_RESULT` en menos de 10 s.
6. El Master **MUST** enviar `MSG_CONFIG_FINAL` y esperar `MSG_CONFIG_CONFIRM`.

---

### 8.3.2 Reglas del Slave

1. El Slave **MUST** rotar configuraciones hasta recibir `MSG_SYNC_START`.
2. El Slave **MUST** responder siempre `MSG_SYNC_REPLY`.
3. Durante calibración, el Slave **MUST** adaptar temporalmente SF/BW según el comando recibido.
4. El Slave **MUST** devolver mediciones RSSI/SNR usando `MSG_CONFIG_RESULT`.
5. Si recibe `MSG_ROLLBACK`, **MUST** revertir su configuración.

---

### 8.3.3 Timeouts

1. Todos los mensajes durante SYNC y CALIBRATION **MUST** usar timeout de 10 s.
2. Mensajes en STABLE **SHOULD** usar timeout de 10 s.
3. Si expira un timeout, el Master **MUST** entrar en RECOVERY.

---

### 8.3.4 Estructura del Mensaje

Definida en la sección 3. Todos los mensajes **MUST** incluir un byte de CRC válido.

---

## 8.4 Secuencia de Intercambio

### 8.4.1 SYNC

```
MASTER → SLAVE: MSG_SYNC_START
SLAVE  → MASTER: MSG_SYNC_REPLY
MASTER → SLAVE: MSG_SYNC_CONFIRM
SLAVE  → MASTER: MSG_ACK
```

### 8.4.2 Calibration

```
MASTER → SLAVE: MSG_CONFIG_TEST(sf,bw,cr)
SLAVE  → MASTER: MSG_CONFIG_RESULT(rssi,snr)
```

### 8.4.3 Final Selection

```
MASTER → SLAVE: MSG_CONFIG_FINAL
SLAVE  → MASTER: MSG_CONFIG_CONFIRM
```

### 8.4.4 Recovery

```
MASTER → SLAVE: MSG_ROLLBACK
SLAVE  → MASTER: MSG_ROLLBACK_CONFIRM
```

---

# 9.  Consideraciones de Implementación

* Toda la lógica compartida reside en `lora_protocol_shared.h`.
* `init_PMIC()` debe ejecutarse obligatoriamente tanto en el master como en el slave.
* Toda transición de configuración SF/BW/CR requiere un `delay(10)` para que la radio se estabilice.
* Cuando LoRa transmite, la recepción se pausa; siempre llamar `LoRa.receive()` al terminar.

---

# 10. Conclusión

Este protocolo proporciona robustez frente a:

* Configuraciones erróneas
* Distancias variables
* Ruido en el canal
* Cambios dinámicos de parámetros

Y permite:

* Sincronización automática
* Autoajuste de configuración
* Recuperación ante fallos
* Comunicación estable a largo plazo

---

Si quieres, ahora puedo generar:

1. Un PDF con el README
2. Diagrama extendido con estados internos
3. Versión en inglés
4. Versión estilo RFC puro (encabezados y formato oficial)

¿Deseas alguno de estos?
