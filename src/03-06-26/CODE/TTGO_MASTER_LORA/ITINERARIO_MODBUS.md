# Itinerario Modbus — `TTGO_MASTER_LORA` (versión ÚLTIMA)

Master Modbus RTU sobre LoRaWAN (ESP32 / TTGO LoRa32 v2.1).
Arquitectura **table-driven**: una tabla estática de lecturas y una sola tarea de polling.

- **Dispositivo actual**: medidor trifásico `DEV_TRIFASICO_NUEVO` (slave **1**).
- **Base commiteada de referencia**: `origin/main` → `4a03fc7`
  (`DEV_TRIFASICO_NUEVO`, polling cada 30 s, `-D SINGLE_CHANNEL_MODE` + `pre:apply_lmic_patch.py`).

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
| `DEV_TRIFASICO_NUEVO` = **1** | `0x04` (input registers) | `true` (byte bajo primero / little-endian por registro) | `2000 ms` |

Los helpers `lookupFunctionCode()`, `lookupTimeout()` y `lookupSwapWords()` resuelven
estos valores por `slaveID`; si no hay entrada, se usa `0x03`, `kBusCfg.defaultTimeoutMs` y `false`.

## 3. Itinerario de instrucciones (`kRequests[]`)

Orden exacto en que se ejecutan las lecturas (el `channelIndex` documenta el orden dentro
de cada `sensorType`; la agrupación real sigue el orden de la tabla):

| # | sensorType (bit) | Dirección | #regs | canal | `swapWordOrder` | Descripción |
|---|---|---|---|---|---|---|
| 1 | BATERIA (0) | `0x003A` | 2 | 0 | `true` | Energía activa total (32-bit, LOW word first) |
| 2 | VOLTAJE (1) | `0x0000` | 1 | 0 | `false` | V Fase A (16-bit, 0.1 V) |
| 3 | VOLTAJE (1) | `0x0001` | 1 | 1 | `false` | V Fase B |
| 4 | VOLTAJE (1) | `0x0002` | 1 | 2 | `false` | V Fase C |
| 5 | CORRIENTE (2) | `0x0003` | 1 | 0 | `false` | I Fase A (16-bit, 0.01 A) |
| 6 | CORRIENTE (2) | `0x0004` | 1 | 1 | `false` | I Fase B |
| 7 | CORRIENTE (2) | `0x0005` | 1 | 2 | `false` | I Fase C |
| 8 | EXT+0 (3) | `0x0020` | 2 | 0 | `true` | Potencia activa total (int32, LOW word first) |
| 9 | EXT+1 (4) | `0x0024` | 2 | 0 | `true` | Potencia aparente total (int32 signed) |
| 10 | EXT+2 (5) | `0x0022` | 2 | 0 | `true` | Potencia reactiva total (int32 signed) |
| 11 | EXT+3 (6) | `0x0027` | 1 | 0 | `false` | Factor de potencia combinado |
| 12 | EXT+4 (7) | `0x0006` | 1 | 0 | `false` | Frecuencia fase A (0.01 Hz) |

Total: **12 requests**.

## 4. Cómo las toma (`mainPollingTask`, `src/main.cpp`)

Cada `POLL_INTERVAL_MS` se ejecuta un ciclo completo:

1. **Resolver parámetros**: `fnCode`, `timeout` y `swapWords` desde `kDeviceCfg`.
2. **Lectura**: `modbus_api_read_registers(slaveID, fnCode, startAddr, numRegs, timeout)`.
3. **Orden de bytes** del registro: si `swapWords`, se emite `lo,hi`; si no, `hi,lo`.
4. **Padding** de cada request a múltiplo de 4 bytes (se anteponen ceros).
5. **`swapWordOrder`**: si aplica, intercambia palabra alta/baja (`bytes[0]↔[2]`, `bytes[1]↔[3]`).
6. **Agrupación** por `sensorType` concatenando los bytes en el orden de la tabla.
7. **Fallo parcial**: si un request falla, se marca `failedTypes[sensorType]` y ese tipo
   completo se descarta del payload.
8. **Cierre de grupo**: se rellena a múltiplo de 4; `regsPerChannel = size / 4`.
9. **Payload**: `construirPayloadUnificado(msgId, payloads)`.
10. **Envío**: `xQueueSend(queueFragmentos, ...)` → tarea LoRa (`LMIC_setTxData2`).

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
  -D SINGLE_CHANNEL_MODE
extra_scripts =
  pre:apply_lmic_patch.py
```

`SINGLE_CHANNEL_MODE` (en `initLoRa()`): `LMIC_selectSubBand(0)`, deshabilita canales
1..7 y 64 → modo gateway de canal único. Sin el flag, usa `LMIC_selectSubBand(7)`.
`apply_lmic_patch.py` parchea la librería LMIC durante el build.

## 7. Estado de esta carpeta

- Código local basado en `main` (`3439a67`) con el intervalo de polling extendido a
  `POLL_INTERVAL_MS = 600000` (10 min). **Sin auto-restart.**
- `origin/main` (`4a03fc7`) añade `-D SINGLE_CHANNEL_MODE` + `pre:apply_lmic_patch.py`
  (gateway de canal único); ese commit todavía no está en este checkout local.
- La tabla `kRequests[]`, el flujo y el payload son idénticos entre ambas.

### Requisito local NO versionado: `include/loraconfig.h`

Contiene las credenciales LoRaWAN y está en `.gitignore` (no viaja en git). Debe existir
para compilar; sin él aparece `fatal error: loraconfig.h: No such file or directory`.

## 8. Diferencias con la versión 630

| Aspecto | Esta versión (3 fases) | `TTGO_MASTER_LORA_630` |
|---|---|---|
| Slave ID | 1 | 10 |
| `swapWords` | `true` (byte bajo primero) | `false` (byte alto primero) |
| Formato | 16-bit (V/I) + int32 LOW-word-first | float IEEE-754 32-bit (MSB primero) |
| Requests | 12 | 11 |
| Tensión | F-N L1/L2/L3 | F-F L1L2/L2L3/L3L1 |
| Energía | total `0x003A` | total `0x0156` |
| Macros LMIC | `SINGLE_CHANNEL_MODE` + parche | no |
