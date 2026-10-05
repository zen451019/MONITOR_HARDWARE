# Itinerario Modbus — `TTGO_MASTER_LORA_630` (copia congelada)

Master Modbus RTU sobre LoRaWAN (ESP32 / TTGO LoRa32 v2.1).
Arquitectura **table-driven**: una tabla estática de lecturas y una sola tarea de polling.

- **Snapshot congelado** del commit **`96ad3a6`** (rama `backup-sdm630`).
- **Dispositivo**: analizador Eastron **SDM630MCT** (`DEV_SDM`, slave **10**, float IEEE-754 32-bit).
- Reprogramable tal cual: es un proyecto PlatformIO completo e independiente.

---

## 1. Configuración del bus RS485 (`kBusCfg`)

| Parámetro | Valor |
|---|---|
| Baud rate | `9600` |
| UART config | `SERIAL_8N1` |
| RX pin | `13` |
| TX pin | `12` |
| Timeout por defecto | `2000 ms` |

Archivo: `include/ModbusConfig.h`.

## 2. Configuración por dispositivo (`kDeviceCfg`)

| Slave | Function code | `swapWords` | Timeout |
|---|---|---|---|
| `DEV_SDM` = **10** | `0x04` (input registers) | `false` (byte alto primero / MSB register first) | `2000 ms` |

Los helpers `lookupFunctionCode()`, `lookupTimeout()` y `lookupSwapWords()` resuelven
estos valores por `slaveID`; si no hay entrada, se usa `0x03`, `kBusCfg.defaultTimeoutMs` y `false`.

## 3. Itinerario de instrucciones (`kRequests[]`)

Orden exacto en que se ejecutan las lecturas (todas float 32-bit = 2 registros):

| # | sensorType (bit) | Dirección | #regs | canal | `swapWordOrder` | Registro SDM630 | Descripción |
|---|---|---|---|---|---|---|---|
| 1 | BATERIA (0) | `0x0156` | 2 | 0 | `false` | 30343 | Energía activa total (kWh) |
| 2 | VOLTAJE (1) | `0x00C8` | 2 | 0 | `false` | 30201 | V L1-L2 |
| 3 | VOLTAJE (1) | `0x00CA` | 2 | 1 | `false` | 30203 | V L2-L3 |
| 4 | VOLTAJE (1) | `0x00CC` | 2 | 2 | `false` | 30205 | V L3-L1 |
| 5 | CORRIENTE (2) | `0x0006` | 2 | 0 | `false` | 30007 | I L1 |
| 6 | CORRIENTE (2) | `0x0008` | 2 | 1 | `false` | 30009 | I L2 |
| 7 | CORRIENTE (2) | `0x000A` | 2 | 2 | `false` | 30011 | I L3 |
| 8 | EXT+0 (3) | `0x0034` | 2 | 0 | `false` | 30053 | Potencia activa total (P) |
| 9 | EXT+1 (4) | `0x0038` | 2 | 0 | `false` | 30057 | Potencia aparente total (S) |
| 10 | EXT+2 (5) | `0x003C` | 2 | 0 | `false` | 30061 | Potencia reactiva total (Q) |
| 11 | EXT+3 (6) | `0x003E` | 2 | 0 | `false` | 30063 | Factor de potencia total (PF) |
| 12 | EXT+4 (7) | `0x0046` | 2 | 0 | `false` | 30071 | Frecuencia |

Total: **11 requests** (comparten `SENSOR_ID_EXT_START + n` para P/S/Q/PF/Frecuencia).

## 4. Cómo las toma (`mainPollingTask`, `src/main.cpp`)

Cada `POLL_INTERVAL_MS` (30 s) se ejecuta un ciclo completo:

1. **Resolver parámetros**: `fnCode`, `timeout` y `swapWords` desde `kDeviceCfg`.
2. **Lectura**: `modbus_api_read_registers(slaveID, fnCode, startAddr, numRegs, timeout)`.
3. **Orden de bytes** del registro: si `swapWords` (aquí `false`), emite `hi,lo`.
4. **Padding** de cada request a múltiplo de 4 bytes (se anteponen ceros).
5. **`swapWordOrder`**: si aplica (aquí `false`), intercambia palabra alta/baja.
6. **Agrupación** por `sensorType` concatenando los bytes en el orden de la tabla.
7. **Fallo parcial**: si un request falla, se marca `failedTypes[sensorType]` y ese tipo
   completo se descarta del payload.
8. **Cierre de grupo**: se rellena a múltiplo de 4; `regsPerChannel = size / 4`.
9. **Payload**: `construirPayloadUnificado(msgId, payloads)`.
10. **Envío**: `xQueueSend(queueFragmentos, ...)` → tarea LoRa (`LMIC_setTxData2`).

> Nota: en este commit **no** existe el retardo inter-frame de 10 ms entre lecturas
> (se añadió después en `3439a67`) ni auto-reinicio por cantidad de mensajes.

## 5. Formato del payload LoRa

```
[ msgId(1B) ][ timestamp(4B BE / UNIX) ][ activateByte(1B) ][ lenBytes(1B c/u) ][ dataBlocks ]
```

- `activateByte`: un bit por `sensorType` (definidos en `include/SensorRegistry.h`).
  - bit 0 = BATERIA, bit 1 = VOLTAJE, bit 2 = CORRIENTE, bits 3..7 = EXT+0..EXT+4.
- `lenBytes`: un byte por bit activo, en orden LSB→MSB; valor = registros por canal (`& 0x1F`).
- `dataBlocks`: bloques de datos en el mismo orden; cada canal ocupa 32 bits (4 bytes).

## 6. Build (`platformio.ini`)

```
[env:ttgo-lora32-v21]
platform = espressif32
board = ttgo-lora32-v21
framework = arduino
monitor_speed = 921600
lib_deps =
  ModbusClient=https://github.com/eModbus/eModbus.git
  mcci-catena/MCCI LoRaWAN LMIC library@^5.0.1
  adafruit/Adafruit SSD1306@^2.5.15
build_flags =
  -D ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS
  -D CFG_us915
  -D CFG_sx1276_radio
```

Sin `SINGLE_CHANNEL_MODE` y sin `apply_lmic_patch.py` (pasó a subbanda 7 estándar).
Para reprogramar un gateway de canal único habría que añadir esos dos elementos.

## 7. Requisito local NO versionado: `include/loraconfig.h`

`include/loraconfig.h` está en `.gitignore` porque contiene las credenciales LoRaWAN
(`DEVADDR`, `NWKSKEY`, `APPSKEY`, etc.). **No se incluye en los snapshots de git**, así que
hay que colocarlo manualmente para poder compilar (error típico:
`fatal error: loraconfig.h: No such file or directory`).

- En este snapshot ya se copió desde `TTGO_MASTER_LORA/include/loraconfig.h`.
- Para el dispositivo 630 debes ajustar ahí su `DEVADDR` / `NWKSKEY` / `APPSKEY` propios.

## 8. Diferencias con la versión última (3 fases)

| Aspecto | Esta versión (630) | `TTGO_MASTER_LORA` |
|---|---|---|
| Slave ID | 10 | 1 |
| `swapWords` | `false` (byte alto primero) | `true` (byte bajo primero) |
| Formato | float IEEE-754 32-bit (MSB primero) | 16-bit (V/I) + int32 LOW-word-first |
| Requests | 11 | 12 |
| Tensión | F-F L1L2/L2L3/L3L1 | F-N L1/L2/L3 |
| Energía | total `0x0156` | total `0x003A` |
| Macros LMIC | no | `SINGLE_CHANNEL_MODE` + parche |
