
# Ejercicio propuesto
El ejemplo anterior puede servir de base para plantear un esquema que permita optimizar, entiéndase minimizar, `los tiempos de transmisión entre dos nodos de una red privada LoRa`. La idea es ir ajustando progresivamente los parámetros de configuración de las radios con el objetivo de reducir el tiempo de transmisión, pero garantizando que el `SNR y RSSI` de los paquetes recibidos se mantiene por encima de ciertos umbrales. Nótese que será necesario definir un pequeño` protocolo de comunicación` entre los nodos para poder coordinar los ajustes en la configuración de las radios. Uno de los nodos, el `nodo maestro`, deberá llevar la iniciativa a la hora de `activar cambios en la configuración`. Estos cambios deberán notificarse al otro nodo, o `nodo esclavo`, `antes de cambiar` efectivamente la configuración de cualquiera de las radios, para que los nodos puedan seguir comunicando por radio.

# nosotros
+ alberto y santiago: 0x06
+ nico y jose: 0x05

# requisitos

| RSSI (dBm)  | Calidad                             |
| ----------- | ----------------------------------- |
| -30 a -50   | Excelente (mismo cuarto o edificio) |
| -50 a -80   | Muy bueno / bueno                   |
| -80 a -100  | Aceptable                           |
| -100 a -115 | Débil pero usable                   |
| -115 a -125 | Muy débil (probable pérdida)        |
| < -125      | No decodifica                       |

| SNR (dB)  | Calidad                                      |
| --------- | -------------------------------------------- |
| +10 a +20 | Excelente                                    |
| +5 a +10  | Muy bueno                                    |
| 0 a +5    | Bueno                                        |
| -5 a 0    | Aceptable                                    |
| -10 a -5  | Pobre                                        |
| -7 a -20  | LoRa todavía puede decodificar gracias al SF |

> salidas en `MKR1310_LoRa_SendReceive_WithReceiveCallback`
> -56 -> -74
> 9.25 -> 9.75

`mantener SRRI  en [-80,-50] minimo y SNR en [5,10]`



# base
### Envío y recepción de mensajes binarios 
Habida cuenta de cómo se incrementa el tiempo de transmisión con la longitud de los paquetes, en redes LoRa resulta de capital importancia tratar de minimizar el volumen de datos a transmitir. Por ejemplo, con paquetes de datos que incluyan datos reales resulta ventajoso incluir estos datos, no como cadenas de caracteres ASCII, sino en formato binario. Sobre esta idea básica, de conformar el payload en un formato binario, es posible aplicar otras que reduzcan aún más la longitud del paquete, pero la estrategia óptima dependerá de las condiciones que se deriven de cada escenario concreto de aplicación. El portal de LoRa para desarrolladores incluye este interesante enlace [5], donde se revisan varias ideas para reducir el tamaño del payload. CayenneLPP [6] es una librería que permite empaquetar y desempaquetar datos estereotipados (temperatura, humedad, latitud, longitud, …) de forma cómoda y segura en paquetes binarios compactos. Un aspecto interesante de esta librería es que es posible conformar el contenido de un paquete de datos de forma dinámica. Los tipos de datos que incluyen siguen el IPSO Alliance Smart Objects Guidelines, que identifica cada tipo de dato con un “Object ID” limitado a un único octeto. El uso de esta librería no logra una compactación máxima del payload, pero sí facilita la integración de una red de sensores que comuniquen paquetes binarios con redes LoRaWAN En este apartado exploraremos el uso de paquetes binarios para transmitir datos sobre la configuración de las radios y sobre los niveles de RSSI y SNR de los paquetes recibidos. Estos datos podrían servir como base para trazar una táctica de minimización de la duración de las transmisiones. Veamos primero cómo podemos compactar la configuración de la radio en sus parámetros fundamentales:
+ Ancho de banda (A): los diferentes anchos de banda 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, 500E3 se pueden asociar con un identificador [0, 9] (4 bits), cero para 7.8E3, 9 para 500E3. 
+ Factor de dispersión (F): [6, 12] → [0, 6] (3 bits) 
+  Tasa de codificación (C): [5, 8] → [0, 3] (2 bits) 
+ Potencia de transmisión (T): [2, 20 dBm] → [0, 18] (5 bits)
+ Preámbulo: se supondrá fija a 8 símbolos

Es fácil ver que podemos transmitir estos datos de forma compacta, empleando solo 14 bits, mediante dos octetos:

Primer byte: **`A3 A2 A1 A0 F2 F1 F0 X`**
Segundo byte: **`C1 C0 T4 T3 T2 T1 T0 X`**

donde el ancho de banda ocuparía los 4 bits altos del primer octeto, de A3 a A0, el factor de dispersión los tres siguiente y así sucesivamente. X indica un bit sin contenido (nulo). El ejemplo siguiente envía esta información y añade también el RSSI y el SNR del último paquete recibido, cada uno de ellos en un octeto.

+ RSSI puede estar en un rango de [0, -127] dBm
+ SNR puede estar en un rango de [20, -148] dBm
