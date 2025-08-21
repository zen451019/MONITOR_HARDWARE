# Modulo de monitoreo de variables electromecánicas MONITOR

Placa electrónica basada en modulo [TTGO LoRa32 V2.1](controlador_central.md) (ESP32 + LoRa) para medir parámetros eléctricos de sistemas electromecánicos y transmitirlos de forma inalámbrica. El sistema integra sensado de tensión y corriente AC trifásica, monitoreo de nivel batería y una entrada digital para la detección de eventos.
#### Especificaciones
- Alimentación: LDO a 3.3 V/800 mA para lógica.
- Entradas dedicadas para sensores de corriente externos.
- Conectividad LoRa integrada en el TTGO LoRa32 V2.1.
- Bornas de tornillo para conexión de red AC, sensores.
- Conectores JST para conexiones de lógica y alimentación.

### [Arquitectura de hardware](arquitectura.md)

## Interfaces y pines

Medición de voltaje : borneras para conexión de señal de voltaje de corriente alterna trifásica mediante módulos [ZMPT101B](sensor_de_voltaje.md), alimentación y salida analógica hacia ADC ESP32 mediante conector JST de 5 pines.

| ENTRADA | DESCRIPCIÓN | SALIDA  |
| ------- | ----------- | ------- |
| 1       | NEUTRO      |         |
| 2       | LINEA 3     | GPIO 36 |
| 3       | LINEA 2     | GPIO 39 |
| 4       | LINEA 1     | GPIO 34 |

Medición de corriente: borneras para la conexión de sensores [SCT-013-000](sensor_de_corriente.md) (transformador de corriente), circuito de acondicionamiento y salida hacia ADC de ESP32.

| ENTRADA | DESCRIPCIÓN              | SALIDA  |
| ------- | ------------------------ | ------- |
| 8       | TERMINAL BLANCO SENSOR 3 |         |
| 9       | TERMINAL ROJO SENSOR 3   | GPIO 13 |
| 10      | TERMINAL BLANCO SENSOR 2 |         |
| 11      | TERMINAL ROJO SENSOR 2   | GPIO 04 |
| 12      | TERMINAL BLANCO SENSOR 1 |         |
| 13      | TERMINAL ROJO SENSOR 1   | GPIO 25 |

Batería: bornera de conexión de [voltaje de batería](nivel_bateria.md), divisor resistivo salida hacia ADC ESP32.

Señal de encendido y apagado: bornera de conexión hacia circuito [level shifter](deteccion_de_encendido.md) con optoacoplador PC817, salida a pin configuración pull-up ESP32.

| ENTRADA | DESCRIPCIÓN           | SALIDA  |
| ------- | --------------------- | ------- |
| 5       | NEGATIVO BATERIA      |         |
| 6       | SEÑAL DE ENCENDIDO    | GPIO 35 |
| 7       | POSITIVO DE LA BAERIA | GPIO 14 |

[Alimentación](alimentacion.md): Entrada de 3.3 Voltios mediante conector JST de dos 2 pines ( conector JST de 4 pines auxiliar, no soldado), alimentación de ESP32 Y demás componentes. DESCRIPCION

#### Requisitos eléctricos

- Entrada principal: 3.3 V DC (capacidad según consumo del TTGO + sensores).
- Señales analógicas: adecuadas al rango ADC del ESP32 (0–3.3 V). Se recomienda verificar que los divisores/condicionamiento mantengan amplitud y offset correctos.

### [Esquema eléctrico de la placa Monitor](esquematico) 
— Diagrama detallado de los componentes y conexiones eléctricas.
### [Diseño de la PCB de la placa Monitor](placa) 
— Layout y distribución física de los componentes en la placa de circuito impreso.
###  [Modelo 3D del Monitor](3D)
— Representación tridimensional del ensamblaje final para visualización y verificación mecánica.