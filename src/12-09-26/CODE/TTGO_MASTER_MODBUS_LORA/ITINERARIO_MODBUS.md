# Itinerario Modbus — `TTGO_MASTER_MODBUS_LORA`

Master Modbus RTU sobre LoRaWAN (ESP32 / TTGO LoRa32 v2.1).
Arquitectura **table-driven**: una tabla estática de lecturas y una sola tarea de polling.

- **Dispositivo**: medidor trifásico **`DEV_TRIFASICO_NUEVO`** (slave **1**, `u16`/`int32` little-endian por registro, ver `manual_modbus_rtu_clean.txt`).
- Reprogramable tal cual: es un proyecto PlatformIO completo e independiente.
- **Stack LoRaWAN**: **RadioLib** (migrado desde MCCI LMIC). Activación **OTAA**, **Class A**.
  Sesión y nonces persistidos en NVS (`Preferences`).

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

Cada `POLL_INTERVAL_MS` (30 s) se ejecuta un ciclo completo:

1. **Resolver parámetros**: `fnCode`, `timeout` y `swapWords` desde `kDeviceCfg`.
2. **Lectura**: `modbus_api_read_registers(slaveID, fnCode, startAddr, numRegs, timeout)`.
3. **Orden de bytes** del registro: si `swapWords` (aquí `true`), emite `lo,hi`
   (endereza el little-endian del dispositivo a big-endian).
4. **Padding** de cada request a múltiplo de 4 bytes (se anteponen ceros).
5. **`swapWordOrder`**: si aplica (aquí `true` en los 32-bit), intercambia palabra alta/baja.
6. **Retardo inter-frame**: `vTaskDelay(10 ms)` entre lecturas (margen RS485).
7. **Agrupación** por `sensorType` concatenando los bytes en el orden de la tabla.
8. **Fallo parcial**: si un request falla, se marca `failedTypes[sensorType]` y ese tipo
   completo se descarta del payload.
9. **Cierre de grupo**: se rellena a múltiplo de 4; `regsPerChannel = size / 4`.
10. **Payload**: `construirPayloadUnificado(msgId, payloads)`.
11. **Envío**: `xQueueSend(queueFragmentos, ...)` → tarea LoRa
    (`node.sendReceive(..., FPORT_DATA, ...)` de RadioLib).
12. **Downlink**: si `sendReceive` devuelve ventana > 0, se procesan los comandos en
    `FPORT_CMD` (ver sección 9).
13. **Espera**: `ulTaskNotifyTake` con `g_pollIntervalMs` (mutable por downlink); un
    comando `FORCE_POLL` despierta el ciclo de inmediato.

> Nota: el retardo inter-frame de 10 ms entre lecturas (commit `3439a67`) **sí** está
> presente. No hay auto-reinicio por cantidad de mensajes.

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
  jgromes/RadioLib@^7.0.0
```

Sin `build_flags` de LMIC y sin `SINGLE_CHANNEL_MODE`/`apply_lmic_patch.py`.

### Pinout de radio (TTGO LoRa32 v2.1)

| Señal | GPIO |
|---|---|
| NSS/CS | 18 |
| DIO0/IRQ | 26 |
| RST | 23 |
| DIO1 | 33 |

Definido en `src/main.cpp`: `SX1276 radio = new Module(18, 26, 23, 33);`.

### Subbanda US915

`LoRaWANNode node(&radio, &US915, LORAWAN_SUB_BAND);` con `LORAWAN_SUB_BAND = 2`.
RadioLib indexa las subbandas **desde 1**: subbanda **2** = **FSB 2** (canales 8-15),
que es la que tiene configurada el gateway. Debe coincidir con el `FSB` del gateway.

## 7. Requisito local NO versionado: `include/loraconfig.h`

`include/loraconfig.h` está en `.gitignore` porque contiene las credenciales LoRaWAN.
**No se incluye en los snapshots de git**, así que hay que colocarlo manualmente para poder
compilar (error típico: `fatal error: loraconfig.h: No such file or directory`).

Ahora usa **OTAA** (placeholders `0x00...` por defecto):

| Símbolo | Bytes | Descripción |
|---|---|---|
| `DEVEUI[8]` | 8 | Device EUI (MSB primero) |
| `JOINEUI[8]` | 8 | Join/App EUI (todo ceros es válido en desarrollo) |
| `APPKEY[16]` | 16 | Application root key |
| `NWKKEY[16]` | 16 | Network root key (LoRaWAN 1.0.x: igual a `APPKEY`) |

El helper `lorawan_eui_to_uint64()` empaqueta los EUI big-endian al `uint64_t` de RadioLib.

## 8. Diferencias con la versión SDM630

| Aspecto | Esta versión (trifásico) | `TTGO_MASTER_LORA_630` (SDM630MCT) |
|---|---|---|
| Slave ID | 1 | 10 |
| `swapWords` | `true` (byte bajo primero) | `false` (byte alto primero) |
| Formato | 16-bit (V/I) + int32 LOW-word-first | float IEEE-754 32-bit (MSB primero) |
| Requests | 12 | 11 |
| Tensión | F-N A/B/C | F-F L1L2/L2L3/L3L1 |
| Energía | total `0x003A` | total `0x0156` |
| Stack LoRaWAN | RadioLib (OTAA/Class A) | LMIC (o RadioLib tras la migración) |

## 9. Downlink / control (`FPORT_CMD` = 2, Class A)

Cada uplink abre las ventanas RX1/RX2; si el servidor encola un downlink, RadioLib lo
entrega en `sendReceive()` y `handleDownlink()` lo procesa:

| Comando | Bytes | Acción |
|---|---|---|
| `0x01` SET_POLL_INTERVAL | `[1..4]` uint32 BE (ms) | Cambia `g_pollIntervalMs` (mín. 1000) |
| `0x02` FORCE_POLL | — | Despierta el ciclo de polling ya (`xTaskNotifyGive`) |
| `0x03` SET_TX_POWER | `[1]` int8 (dBm) | `node.setTxPower()` |
| `0x04` SET_DATARATE | `[1]` uint8 DR | `node.setDatarate()` (ADR debe estar off) |
| `0x05` REBOOT | — | `ESP.restart()` |
| `0x07` SET_ADR | `[1]` 0/1 | `node.setADR()` |

Uplinks en `FPORT_DATA` = 1. La sesión y los nonces se guardan en NVS con
`saveSession()` tras cada `sendReceive` (y se restauran con `loadSession()` al arrancar)
para evitar desajustes de contador de frames entre reinicios.

> En Class A el downlink solo puede llegar tras un uplink. Para control en cualquier
> momento habría que pasar a Class C (`node.setClass(RADIOLIB_LORAWAN_CLASS_C)` +
> `getDownlinkClassC()`).
