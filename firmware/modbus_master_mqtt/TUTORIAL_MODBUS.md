# Tutorial: como funciona y como se extiende el Modbus de `modbus_master_mqtt`

Guia practica de la variante MQTT del master Modbus. Explica la arquitectura, como
agregar lecturas (bloques y senales), como escribir funciones de decodificacion y como
llega la data a la tarea MQTT.

---

## 1. Idea general

El firmware lee el medidor por **bloques** (varios registros contiguos en una sola
peticion Modbus), decodifica cada valor con **su propia funcion**, y entrega el
resultado a la tarea MQTT por una **cola**.

```
mainPollingTask (core 0)                       tareaMqtt (core 1)
  1. por cada bloque: leer registros
  2. por cada senal: llamar su decode()
  3. armar MeasureFrame  ---- xQueueSend ---->  [ queueMqtt ] --- xQueueReceive ---> (JSON + publish)
```

Piezas clave (en `include/ModbusConfig.h`):

| Pieza | Que es |
|---|---|
| `kBlocks[]` | Que rangos de registros se leen y como (1 peticion por bloque) |
| `kSignals[]` | Cada valor: a que sensor/canal pertenece y **como decodificarlo** |
| `mb_u16/mb_i32/mb_byte` | Helpers de decodificacion (little-endian) |
| `SensorRegistry.h` | IDs logicos de sensor (`SENSOR_ID_*`) |

En `src/main.cpp`:

| Pieza | Que es |
|---|---|
| `MeasureFrame` / `SensorValues` | Estructura que viaja a la tarea MQTT (valores ya escalados) |
| `mainPollingTask` | Lee bloques, decodifica y encola |
| `tareaMqtt` | Recibe el frame y (a futuro) publica JSON |

---

## 2. El modelo de datos

### 2.1 Bloque (`ModbusBlock`)
Una peticion Modbus = un bloque.

```cpp
struct ModbusBlock {
    uint8_t  slaveID;       // esclavo (1)
    uint8_t  functionCode;  // 0x04 = input registers
    uint16_t startAddr;     // direccion inicial
    uint16_t numRegs;       // cuantos registros leer
};
```

```cpp
const ModbusBlock kBlocks[] = {
    {DEV_TRIFASICO_NUEVO, 0x04, 0x0000, 9},  // V A/B/C + I A/B/C + Freq A/B/C
    {DEV_TRIFASICO_NUEVO, 0x04, 0x0020, 6},  // P/Q/S totales
    {DEV_TRIFASICO_NUEVO, 0x04, 0x0026, 2},  // Factor de potencia
    {DEV_TRIFASICO_NUEVO, 0x04, 0x003A, 2},  // Energia activa total
};
```

### 2.2 Senal (`ModbusSignal`)
Cada valor que quieres publicar. Lleva **su funcion** `decode`.

```cpp
struct ModbusSignal {
    uint8_t sensorType;                                  // SENSOR_ID_*
    uint8_t channel;                                     // 0,1,2,3...
    uint8_t blockIndex;                                  // indice en kBlocks
    float (*decode)(const uint8_t* data, size_t len);    // como decodificar
};
```

### 2.3 Funcion de decodificacion
Recibe los **bytes crudos del bloque** (`data`, `len`) y devuelve el valor final.
Es el equivalente al `lambda` por sensor de ESPHome: tu decides como acomodar los datos.

```cpp
{SENSOR_ID_VOLTAJE, 0, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 0) * 0.1f; }},
```

---

## 3. Orden de bytes (importante)

El medidor **no** usa el orden Modbus estandar. Manda:

- **Dentro de cada registro:** byte bajo primero (little-endian de 16 bits).
- **En valores de 32 bits:** palabra baja primero (little-endian de 32 bits).

Resultado: en el aire, los valores son **little-endian puro**. Por eso los helpers
decodifican directo:

```cpp
uint16_t mb_u16(const uint8_t* d, size_t len, uint16_t reg);  // d[2r] | d[2r+1]<<8
int32_t  mb_i32(const uint8_t* d, size_t len, uint16_t reg);  // 4 bytes little-endian
uint8_t  mb_byte(const uint8_t* d, size_t len, uint16_t i);   // byte crudo i
```

`reg` es el **offset en registros dentro del bloque** (0 = primer registro del bloque).

> Los helpers tienen guardas de limites: si pides fuera de rango devuelven 0.

---

## 4. Como agregar una lectura

### Caso A: extender un bloque existente (registros contiguos)
Ejemplo ya aplicado: **Frecuencia B/C** (`0x0007`, `0x0008`) son contiguos al bloque 0.

1. Subir `numRegs` del bloque 0 de `7` a `9`:
   ```cpp
   {DEV_TRIFASICO_NUEVO, 0x04, 0x0000, 9},
   ```
2. Agregar las senales (mismo `sensorType`, nuevos `channel`):
   ```cpp
   {SENSOR_ID_FRECUENCIA, 1, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 7) * 0.01f; }},
   {SENSOR_ID_FRECUENCIA, 2, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 8) * 0.01f; }},
   ```
   El offset (7 y 8) es el registro **dentro del bloque** (0x0007 - 0x0000 = 7).

### Caso B: bloque nuevo (registros en otra zona)
1. Agregar el bloque:
   ```cpp
   // Potencia activa por fase A/B/C (32-bit) @0x000E..0x0013
   {DEV_TRIFASICO_NUEVO, 0x04, 0x000E, 6},
   ```
   (recuerda: es un indice nuevo, p. ej. 4)
2. Agregar las senales apuntando a ese `blockIndex`:
   ```cpp
   {SENSOR_ID_POTENCIA_ACTIVA, 0, 4, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 0) * 0.1f; }}, // Fase A
   {SENSOR_ID_POTENCIA_ACTIVA, 1, 4, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 2) * 0.1f; }}, // Fase B
   {SENSOR_ID_POTENCIA_ACTIVA, 2, 4, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 4) * 0.1f; }}, // Fase C
   ```

> Regla: `channel` debe ser < `MQTT_MAX_CHANNELS` (hoy 4). Si necesitas mas, sube esa constante en `main.cpp`.

---

## 5. Ejemplo real: Factor de Potencia empaquetado

El medidor empaqueta **dos valores por registro** (cada byte ×0.01):

| Registro | Byte bajo | Byte alto |
|---|---|---|
| `0x0026` | Fase B | Fase A |
| `0x0027` | Combinado | Fase C |

Como el dispositivo manda **byte bajo primero**, en el aire:
- `0x0026` = `[B, A]`  -> `d[0]=B`, `d[1]=A`
- `0x0027` = `[Combinado, C]` -> `d[2]=Combinado`, `d[3]=C`

Se lee con `mb_byte`:

```cpp
{SENSOR_ID_FACTOR_POTENCIA, 0, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 1) * 0.01f; }}, // A
{SENSOR_ID_FACTOR_POTENCIA, 1, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 0) * 0.01f; }}, // B
{SENSOR_ID_FACTOR_POTENCIA, 2, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 3) * 0.01f; }}, // C
{SENSOR_ID_FACTOR_POTENCIA, 3, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 2) * 0.01f; }}, // Combinado
```

Esto es exactamente lo que pediste: **una funcion por llamada** para separar los 4 datos.

---

## 6. Como agregar un TIPO de sensor nuevo

Hoy hay 8 tipos (IDs 0..7) y `main.cpp` usa `acc[8]` / `frame.sensors[8]`.

1. Definir el ID en `include/SensorRegistry.h`:
   ```cpp
   constexpr uint8_t SENSOR_ID_MI_SENSOR = 8;
   ```
2. Subir los topes en `src/main.cpp`:
   ```cpp
   #define MQTT_MAX_SENSORS  9   // o mas
   ```
   y en `mainPollingTask` el arreglo `SensorAcc acc[8];` -> `acc[MQTT_MAX_SENSORS];`
   y el bucle `for (uint8_t st = 0; st < 8; ++st)` -> `< MQTT_MAX_SENSORS`.
3. Agregar su bloque/senal en `kBlocks`/`kSignals`.
4. (Cuando exista el paso MQTT) mapear el ID -> nombre de topic.

> Nota: los IDs 0..7 coinciden con los bits del `activateByte` del firmware LoRaWAN y
> con `codec.js`. A partir de 8 ya no existen en ese formato; son **solo para MQTT**.

---

## 7. Como llega la data a la tarea MQTT

En `mainPollingTask`:

1. Por cada bloque: `modbus_api_read_registers(...)` (sincrona).
2. Si el bloque responde, por cada senal de ese bloque: `a.value[channel] = s.decode(r.data, r.data_len);`
3. Se arma `MeasureFrame` (con `sensorId`, `channels` y los `float value[]`).
4. `xQueueSend(queueMqtt, &frame, ...)`.

En `tareaMqtt`:

```cpp
if (xQueueReceive(queueMqtt, &frame, portMAX_DELAY) == pdTRUE) {
    // el frame llego -> aqui se empaqueta JSON y se publica
}
```

- `xQueueReceive` **duerme** hasta que hay un frame; el `pdTRUE` es el "ya hay que empaquetar".
- La cola lleva una **copia** del struct: no hay memoria compartida ni carreras.

---

## 8. Como compilar y flashear

```powershell
# Compilar
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run

# Flashear (con el TTGO por USB)
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -t upload

# Monitor serie
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor
```

Credenciales WiFi/MQTT: `include/netconfig.h` (gitignored).

---

## 9. Errores comunes

| Sintoma | Causa probable |
|---|---|
| Valor negativo raro | Usaste `mb_u16` para algo con signo (potencias -> `mb_i32`) |
| Valor x10 / /10 | Escala equivocada (V x0.1, I x0.01, P/Q/S x0.1, Freq x0.01) |
| PF absurdo | Leiste el registro completo en vez de un byte (`mb_byte`) |
| "Bloque fallo: err=..." | El medidor no acepta ese rango -> divide el bloque en sub-bloques |
| Frecuencia no cambia | No extendiste `numRegs` del bloque 0 |
| Todo 0 | Revision de cableado A/B, alimentacion, direccion de esclavo |

---

## 10. Resumen del flujo para agregar algo
1. Ver el manual del medidor: direccion y escala.
2. Decidir si va en un bloque existente (contiguo) o bloque nuevo.
3. Agregar/extender `kBlocks`.
4. Agregar la senal en `kSignals` con su `decode` (usando los helpers).
5. Si es tipo nuevo: ID + subir topes en `main.cpp`.
6. Compilar y validar en el banco.
