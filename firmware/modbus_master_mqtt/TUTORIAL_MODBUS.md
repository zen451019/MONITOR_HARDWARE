# Tutorial: como funciona y como se extiende el Modbus de `modbus_master_mqtt`

> **Arquitectura actual:** Modbus RTU -> `MeasureFrame` -> cola -> Sparkplug B por MQTT.
> La configuracion declarativa esta en `include/DevicesConfig.h`.
> Los helpers de decodificacion estan en `include/ModbusDecode.h`.
> Las tareas estan en `src/main.cpp` y el payload Sparkplug en `src/SparkplugB.cpp`.
> No hay JSON ni payload LoRa en este firmware.

---

## 1. Idea general

```
mainPollingTask (core 0)                         tareaRed (core 1)
  1. por cada Device:                              WiFi / MQTT / Sparkplug
     por cada bloque: syncRequest()                 consume la cola
     por cada senal: decode()                       DDEATH / DBIRTH
     armar MeasureFrame  --xQueueSend--> [queueMqtt] --xQueueReceive--> DDATA / NDATA
```

- `mainPollingTask` lee el bus y produce **un `MeasureFrame` por Device y por ciclo**.
- `tareaRed` consume los frames y publica Sparkplug B.
- La cola desacopla Modbus de la red. Tiene `2 * kMaxDevices` = 16 frames.
- Si la cola se llena, los frames normales se descartan; un frame `cycleEnd` descarta el mas viejo y entra.
- El ciclo de polling es de 30 s (`POLL_INTERVAL_MS`).

---

## 2. Modelo de datos (`include/DevicesConfig.h`)

### 2.1 Bloque (`ModbusBlock`)

Una peticion Modbus = un bloque de registros contiguos.

```cpp
struct ModbusBlock {
    uint8_t  functionCode;  // 0x03 = holding, 0x04 = input
    uint16_t startAddr;
    uint16_t numRegs;
};
```

El slave ID **no** va en el bloque: va en `DeviceConfig`.

### 2.2 Senal (`ModbusSignal`)

Cada valor que quieres publicar. Apunta a un bloque y tiene su propia funcion `decode`.

```cpp
struct ModbusSignal {
    uint8_t sensorType;                                  // SENSOR_ID_*
    uint8_t channel;                                     // 0..3
    uint8_t blockIndex;                                  // indice en dev.blocks
    float (*decode)(const uint8_t* data, size_t len);
};
```

### 2.3 Metrica Sparkplug (`SpbMetricDef`)

Mapea una senal a una metrica publicada.

```cpp
struct SpbMetricDef {
    uint8_t     sensorType;
    uint8_t     channel;
    uint32_t    datatype;    // SPB_DT_*
    uint16_t    offset;      // alias = device.aliasBase + offset
    const char* name;
};
```

### 2.4 Device (`DeviceConfig`)

```cpp
struct DeviceConfig {
    const char* deviceId;    // Sparkplug Device ID
    uint8_t     slaveId;     // direccion Modbus
    uint16_t    aliasBase;   // primer alias del Device
    uint32_t    timeoutMs;   // timeout por bloque

    const ModbusBlock*  blocks;   size_t blockCount;
    const ModbusSignal* signals;  size_t signalCount;
    const SpbMetricDef* metrics;  size_t metricCount;
};
```

La tabla es `kDevices[]`. Agregar un medidor = agregar una entrada con
`deviceId`, `slaveId`, `aliasBase` unico y sus arrays.

El medidor actual es `medidor_trifasico_1`:

- slave `1`, alias base `10`, timeout `500 ms`
- 4 bloques, 17 senales, 17 metricas
- `kMeterBlocks`, `kMeterSignals`, `kMeterMetrics`

---

## 3. Helpers de decodificacion (`include/ModbusDecode.h`)

```cpp
uint16_t mb_u16(const uint8_t* d, size_t len, uint16_t reg);
int32_t  mb_i32(const uint8_t* d, size_t len, uint16_t reg);
uint8_t  mb_byte(const uint8_t* d, size_t len, uint16_t i);
```

- `reg` es el offset **en registros dentro del bloque** (0 = primer registro).
- Tienen guardas de limites: fuera de rango devuelven `0`.
- Son C++ puro y se prueban en el host con Unity:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" test -e native
```

---

## 4. Orden de bytes (importante)

El medidor manda **little-endian puro**:

- Dentro de cada registro: byte bajo primero.
- En valores de 32 bits: palabra baja primero.

Por eso los helpers decodifican directo. Si un dispositivo nuevo usa otro orden,
agrega un helper nuevo (por ejemplo `mb_u16_be`) y sus tests; no cambies los actuales.

---

## 5. Como agregar una lectura

### Caso A: extender un bloque existente

1. Sube `numRegs` del bloque.
2. Agrega la senal con el `blockIndex` correcto y el offset dentro del bloque.

Ejemplo: frecuencia B/C en el bloque 0 (`0x0007`, `0x0008`).

```cpp
{SENSOR_ID_FRECUENCIA, 1, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 7) * 0.01f; }},
{SENSOR_ID_FRECUENCIA, 2, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 8) * 0.01f; }},
```

### Caso B: bloque nuevo

1. Agrega el bloque a `dev.blocks` (o a `kMeterBlocks`).
2. Agrega las senales con el nuevo `blockIndex`.
3. Ajusta `blockCount` (con `arrayCount` no hace falta si usas el array completo).

```cpp
{0x04, 0x000E, 6},   // potencias por fase, 32-bit
```

### Caso C: publicar una metrica nueva

Agrega una entrada a `kMeterMetrics` con:

- `sensorType` y `channel` de la senal.
- `datatype` Sparkplug (`SPB_DT_FLOAT`, `SPB_DT_INT32`, etc.).
- `offset` unico dentro del Device.
- `name` unico.

El alias se calcula como:

```text
alias = device.aliasBase + offset
```

Los alias deben ser unicos en **todo el Edge Node**. El nodo usa `1..6`; el
primer Device empieza en `10`.

---

## 6. Ejemplo real: factor de potencia empaquetado

El medidor empaqueta dos valores por registro (cada byte x0.01):

| Registro | Byte bajo | Byte alto |
|---|---|---|
| `0x0026` | Fase B | Fase A |
| `0x0027` | Combinado | Fase C |

Como el dispositivo manda byte bajo primero:

- `0x0026` = `[B, A]` -> `d[0]=B`, `d[1]=A`
- `0x0027` = `[Combinado, C]` -> `d[2]=Combinado`, `d[3]=C`

Se lee con `mb_byte`:

```cpp
{SENSOR_ID_FACTOR_POTENCIA, 0, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 1) * 0.01f; }}, // A
{SENSOR_ID_FACTOR_POTENCIA, 1, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 0) * 0.01f; }}, // B
{SENSOR_ID_FACTOR_POTENCIA, 2, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 3) * 0.01f; }}, // C
{SENSOR_ID_FACTOR_POTENCIA, 3, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 2) * 0.01f; }}, // Combinado
```

---

## 7. Como agregar un tipo de sensor nuevo

Hoy hay 8 tipos logicos (`SENSOR_ID_*` 0..7) y:

- `MQTT_MAX_SENSORS = 8`
- `MQTT_MAX_CHANNELS = 4`

Estan en `include/MeasureFrame.h`.

Pasos:

1. Define el ID en `include/SensorRegistry.h`.
2. Si superas 8 tipos, sube `MQTT_MAX_SENSORS` en `MeasureFrame.h`.
3. Si superas 4 canales por tipo, sube `MQTT_MAX_CHANNELS`.
4. Agrega bloque/senal en `DevicesConfig.h`.
5. Agrega la metrica en `kMeterMetrics` con `offset` unico.

Los IDs de `SensorRegistry.h` son **indices logicos de este firmware**; no son
un formato de wire LoRa ni un Activate Byte.

---

## 8. Como llega la data a MQTT

En `mainPollingTask`:

1. Por cada Device y bloque: `modbus_api_read_registers()` (sincrona).
2. Si el bloque responde, por cada senal de ese bloque:
   `a.value[channel] = s.decode(r.data, r.data_len);`
3. Se arma un `MeasureFrame` por Device.
4. Se envia a `queueMqtt` sin bloquear.

En `tareaRed`:

1. Consume el frame.
2. Si el Device esta online: DDATA.
3. En `cycleEnd`: NDATA con la salud del nodo.
4. Tras 3 ciclos sin respuesta: DDEATH.
5. Al recuperarse: DBIRTH y luego DDATA.

Reglas Sparkplug relevantes:

- NBIRTH: una vez por sesion MQTT; `seq = 0`; incluye `bdSeq`.
- DBIRTH: uno por Device, con nombre + alias + datatype + timestamp + valor.
- NDATA/DDATA: solo alias + timestamp + valor.
- NDEATH: LWT, solo `bdSeq`, sin `seq`.
- DDEATH: sin metricas, con `seq`.
- QoS: NBIRTH/DBIRTH/NDEATH en 1; NDATA/DDATA/DDEATH en 0.
- Sin NTP valido no se publica birth.

---

## 9. Compilar, flashear y verificar

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"

& $pio run                 # build
& $pio run -t upload       # flash
& $pio device monitor      # monitor 115200
& $pio test -e native      # tests de host
```

Antes de compilar, copia `include/netconfig.example.h` a `include/netconfig.h`
y completa WiFi/MQTT. `netconfig.h` es local y gitignored.

### Verificacion MQTT rapida con MQTT Viewer

- Suscribete a `spBv1.0/#` con **QoS 1**.
- Revisa topics, QoS y retain:
  - NBIRTH / DBIRTH / NDEATH: QoS 1, retain 0.
  - NDATA / DDATA / DDEATH: QoS 0, retain 0.
- Si te suscribes con QoS 0, MQTT Viewer mostrara los QoS 1 degradados a 0.

### Verificacion de conformidad Sparkplug

El decoder/validador revisa secuencia, timestamps, alias y nombres:

```powershell
python tools\sparkplug\sparkplug_decode.py --host <IP_BROKER>
```

Un ciclo birth/data no debe producir `[WARN]`.

---

## 10. Errores comunes

| Sintoma | Causa probable |
|---|---|
| Valor negativo raro | Usaste `mb_u16` para algo con signo (potencias -> `mb_i32`) |
| Valor x10 / /10 | Escala equivocada (V x0.1, I x0.01, P/Q/S x0.1, Freq x0.01) |
| PF absurdo | Leiste el registro completo en vez de un byte (`mb_byte`) |
| `err=1 exc=0x00` | Timeout: el esclavo no respondio |
| `err=3 exc=0x02` | Excepcion Modbus: direccion ilegal |
| Frecuencia no cambia | No extendiste `numRegs` del bloque 0 |
| Todo 0 | Revision de cableado A/B, alimentacion, slave ID |
| No aparece en MQTT | NTP no sincronizado, MQTT no conectado, o birth no publicado |

---

## 11. Resumen para agregar algo

1. Ver el manual del medidor: direccion, tipo y escala.
2. Decidir si va en un bloque existente o en uno nuevo.
3. Agregar/extender el bloque en `DevicesConfig.h`.
4. Agregar la senal con su `decode` usando `mb_*`.
5. Agregar la metrica en `kMeterMetrics` con `offset` unico.
6. Si es un tipo nuevo, actualizar `SensorRegistry.h` y los topes de `MeasureFrame.h`.
7. Compilar, flashear y validar con MQTT Viewer y el decoder.
