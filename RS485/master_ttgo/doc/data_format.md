@file data_format.md
@defgroup group_data_format Data Format & Payload

@section group_data_format_overview Resumen
Este módulo se encarga de:
- Parsear respuestas crudas Modbus provenientes de `queueRespuestas`.
- Normalizar datos por sensor y generar `SensorDataPayload`.
- Agregar múltiples payloads en un mensaje compacto con Activate Byte y metadatos.
- Entregar fragmentos listos para transmisión a la capa externa (LMIC) vía `queueFragmentos`.

@section Estructura_y_componentes Estructura y componentes

- Transporte intermedio
  - ResponseFormat: datos crudos desde `handleData()` (bytes, longitud, deviceId, order).

- Datos normalizados
  - SensorDataPayload: paquete por sensor con registros procesados y metadatos (sensorID, slaveID, longitud útil).

- Tareas/Flujo
  - `DataFormatter`: decide si la respuesta es de descubrimiento o muestreo, y produce `SensorDataPayload`.
  - `DataAggregatorTask`: consume payloads, arma Activate Byte, añade cabeceras y construye el mensaje final.

@section group_data_format_api API principal
- Entrada
  - `queueRespuestas` (ResponseFormat).
- Salida
  - `queueSensorDataPayload` (SensorDataPayload).
  - `queueFragmentos` (mensaje agregado listo para transmisión).
- Utilidades
  - `formatAndEnqueueSensorData()` (Extraccion y formateo).
  - `parseAndStoreDiscoveryResponse()` (descubrimiento; actualiza estructuras).
  - `construirPayloadUnificado()` (agregación).
  - `BitPacker` compresión opcional por `compressedBytes`.

@section group_data_format_usage Uso
Resumen del flujo :

1) Formateo de respuestas
   - `DataFormatter` lee `ResponseFormat` de `queueRespuestas`.
- Descubrimiento: valida estructura y delega la actualización de parámetros del sensor (ver grupo \ref group_modbus_discovery "Modbus").
   - Muestreo: extrae registros , aplica `dataType`, `scale` y compresión si `compressedBytes > 0`.
   - Genera `SensorDataPayload` y lo publica en `queueSensorDataPayload`.

2) Agregación de payloads
   - `DataAggregatorTask` consume `SensorDataPayload` dentro de un intervalo fijo.
   - Construye Activate Byte por sensor presente y calcula longitudes por canal.
   - Llama `construirPayloadUnificado()` y publica el resultado en `queueFragmentos`.

3) Entrega a transmisión
   - El fragmento agregado queda listo para ser tomado por la tarea de transmisión externa.


@section Estructura_y_codificación Estructura y codificación del payload

El firmware usa un esquema binario optimizado para maximizar la eficiencia de transmisión sobre LoRaWAN. Esta sección describe la estructura del payload para facilitar la implementación del decodificador.

Componentes del payload:

- Encabezado (1 byte): Identificador de mensaje para mantener el orden de recepción.
- Timestamp (4 bytes): Marca de tiempo desde el arranque del sistema en formato UNIX.
- Activate byte (1 byte): Banderas que indican operación por tipo de sensor, codificadas bit a bit (Bit 0 = batería, Bit 1 = voltaje, Bit 2 = corriente; el resto reservado).
- Bytes de longitud de datos (dependen del Activate byte): Definen cantidad y formato de codificación de cada tipo de sensor activo. El número de estos bytes es igual al número de sensores activos indicados en el Activate byte.

Estructura de cada byte de longitud (8 bits):

```
Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
------|-------|-------|-------|-------|-------|-------|-------
PKD   | 2BIT  |               DATA_LENGTH (5 bits)
```

Descripción de campos:

- Bits 0-4 (DATA_LENGTH): Número de lecturas del sensor (0–31 muestras).
- Bit 6 (2BIT):
  - 0: Codificación estándar (1 bit por muestra).
  - 1: Codificación de 2 bits por muestra.
- Bit 7 (PKD):
  - 0: Formato alineado a byte.
  - 1: Formato empaquetado a nivel de bit.

Orden de bytes de longitud:

Los bytes de longitud siguen el mismo orden que las banderas del Activate byte (LSB → MSB):

- Bit 0: Batería (si activo)
- Bit 1: Voltaje (si activo)
- Bit 2: Corriente (si activo)

Ejemplo: Si Activate byte = 0x06 (voltaje + corriente activos), habrá 2 bytes de longitud: primero para voltaje, segundo para corriente.

- Data bytes: Bytes con datos de sensores; tanto el Activate byte como los bytes de longitud son necesarios para decodificar estos datos.