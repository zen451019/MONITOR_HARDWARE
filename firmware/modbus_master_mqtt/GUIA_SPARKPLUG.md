# Guia de prueba - Sparkplug B

Esta guia explica como verificar el firmware `modbus_master_mqtt` con Sparkplug B.
El firmware publica:

- `NBIRTH` / `NDATA`: metricas del Edge Node (ESP32).
- `DBIRTH` / `DDATA`: metricas del Device (medidor Modbus).
- `NDEATH`: Will MQTT del Edge Node.
- `DDEATH`: perdida de un Device.
- `NCMD`: comandos del host (Rebirth).

## 1. Compilar y flashear

```powershell
cd firmware\modbus_master_mqtt

& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -t upload
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor
```

El monitor serie muestra el arranque, la conexion WiFi/MQTT, el `bdSeq`, el birth y los errores Modbus.

## 2. Topics que debe crear

El script Python y MQTT Explorer corren en la misma PC que Mosquitto, por eso usan
`127.0.0.1`. El ESP32 **no** puede usar `127.0.0.1`: en `include/netconfig.h`,
`MQTT_HOST` debe ser la IP LAN de la PC (por ejemplo `192.168.1.10`).

Con el broker y la configuracion actual:

```
spBv1.0/EMASA/NBIRTH/nemo_medidor1
spBv1.0/EMASA/DBIRTH/nemo_medidor1/medidor_trifasico_1
spBv1.0/EMASA/NDATA/nemo_medidor1
spBv1.0/EMASA/DDATA/nemo_medidor1/medidor_trifasico_1
spBv1.0/EMASA/NDEATH/nemo_medidor1
spBv1.0/EMASA/DDEATH/nemo_medidor1/medidor_trifasico_1
spBv1.0/EMASA/NCMD/nemo_medidor1      (solo lo publica el host)
```

## 3. Ver los topics en MQTT Explorer

1. Abre MQTT Explorer.
2. Conexion al broker: `127.0.0.1`, puerto `1883`, sin usuario/password.
3. En el arbol, suscribete a `spBv1.0/#`.
4. Reinicia el ESP32 o envia un Rebirth para que aparezcan los birth.

Que vas a ver:

- `NBIRTH` y `DBIRTH` aparecen al conectar.
- `NDATA` y `DDATA` aparecen cada 30 s.
- El payload de Sparkplug es **Protobuf binario**. MQTT Explorer puede mostrarlo como hexadecimal o como bytes raros; **no lo decodifica como tags**.
- Para ver nombres y valores, usa el script Python de la seccion 4 o un cliente Sparkplug (Ignition, MQTT Viewer, EMQX, HiveMQ).
- `NBIRTH`, `DBIRTH`, `NDATA` y `DDATA` **no son retained**. Si MQTT Explorer se conecta despues del birth, solo vera `DDATA` con alias. En ese caso envia un Rebirth.

En MQTT Explorer revisa tambien:

- `retain = false` en birth/data.
- `NBIRTH`/`DBIRTH` con QoS 0.
- `NDEATH` con QoS 1 cuando el broker lo publica.

Tambien puedes verificar QoS/retain por consola:

```powershell
& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -v `
  -F "%q %r %t" -t "spBv1.0/#"
```

Formato de salida: `QoS retain topic`. Debe verse `0 0` en birth/data y `1 0` en NDEATH.

## 4. Decodificar la data con Python

El proyecto incluye:

```
tools/sparkplug/sparkplug_decode.py
tools/sparkplug/requirements.txt
tools/sparkplug/python/sparkplug_b_pb2.py
```

### Instalacion

```powershell
cd firmware\modbus_master_mqtt
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r tools\sparkplug\requirements.txt
```

### Ejecutar el consumidor

```powershell
python tools\sparkplug\sparkplug_decode.py --host 127.0.0.1
```

El birth se publica **solo cuando NTP ya sincronizo**. Si no hay hora UTC, el firmware
espera y no publica NBIRTH; revisa el monitor serie.

Salida esperada al reiniciar el ESP32:

```text
EMASA/nemo_medidor1  NBIRTH  seq=0  ts=...  qos=0 retain=False
  + bdSeq                    alias=   5 type=Int64      value=0
  + Node Control/Rebirth     alias=   6 type=Boolean    value=False
  + Uptime_ms                alias=   1 type=UInt64     value=...
  + RSSI_dBm                 alias=   2 type=Int32      value=-67
  + FreeHeap_bytes           alias=   3 type=UInt32     value=...
  + Modbus_ScanOK            alias=   4 type=Boolean    value=True

EMASA/nemo_medidor1/medidor_trifasico_1  DBIRTH  seq=1  ts=...  qos=0 retain=False
  + Voltage_L1               alias=  10 type=Float      value=...
  + Voltage_L2               alias=  11 type=Float      value=...
  ...
  + ActiveEnergy_Total       alias=  26 type=Float      value=...

EMASA/nemo_medidor1/medidor_trifasico_1  DDATA  seq=2  ts=...  qos=0 retain=False
  - Voltage_L1               alias=  10 type=Float      value=...
  ...
```

En el cable, `DDATA`/`NDATA` llevan **solo `alias`**, sin `name` ni `datatype`, como
exige Sparkplug B cuando se usan alias. El script resuelve los nombres con el birth.
Si lo arrancas despues del birth, envia un Rebirth.

El script imprime `[WARN]` si detecta:
secuencia con huecos, birth sin alias/datatype/timestamp, data con `name`/`datatype`,
NDEATH sin `bdSeq`, o `bdSeq` de NDEATH que no coincide con el ultimo NBIRTH.

### Enviar un Rebirth

En otra terminal:

```powershell
python tools\sparkplug\sparkplug_decode.py --host 127.0.0.1 `
  --send-rebirth --group EMASA --node nemo_medidor1
```

El script envia un NCMD conforme: **QoS 0, `retain=false`, sin `seq` y usando solo
`alias=6`**. El ESP32 debe responder con un nuevo `NBIRTH` y un nuevo `DBIRTH`, con
`seq=0`. Para probar un host que envie por nombre:

```powershell
python tools\sparkplug\sparkplug_decode.py --host 127.0.0.1 `
  --send-rebirth --rebirth-by-name --group EMASA --node nemo_medidor1
```

## 5. Pruebas recomendadas

### 5.1 Arranque normal

1. Arranca el consumidor Python.
2. Reinicia el ESP32.
3. Verifica `NBIRTH`, `DBIRTH`, y luego `NDATA`/`DDATA` cada 30 s.
4. Comprueba que `seq` incrementa: `0` NBIRTH, `1` DBIRTH, `2...` data.
5. Comprueba que los valores del medidor coinciden con el display/banco.

### 5.2 NDEATH

1. Con el consumidor Python activo, corta la alimentacion del ESP32 o apaga el WiFi.
2. Espera el keepalive del broker (15 s x 1.5, aproximadamente 22 s).
3. Debe aparecer:

```text
EMASA/nemo_medidor1  NDEATH  bdSeq=0
```

Si haces un disconnect limpio, el broker no publica el Will; el firmware no publica NDEATH manualmente en un apagado limpio.

### 5.3 Reconexion y bdSeq

1. Vuelve a encender el ESP32.
2. Debe aparecer un nuevo `NBIRTH` con `bdSeq` incrementado (`1`, `2`, ...).
3. El valor se guarda en NVS, por lo que sobrevive a un reinicio.

### 5.4 Rebirth

1. Ejecuta el comando `--send-rebirth`.
2. Debe aparecer un nuevo `NBIRTH` y `DBIRTH`.
3. `seq` vuelve a `0`.
4. El `bdSeq` no cambia en un Rebirth (es la misma sesion MQTT).

### 5.5 DDEATH y recuperacion

1. Desconecta el medidor Modbus (A/B o alimentacion) o el RS485.
2. Durante los primeros 2 ciclos veras `DDATA` con `value=null` en las metricas del Device.
3. Al tercer ciclo fallido debe aparecer:

```text
EMASA/nemo_medidor1/medidor_trifasico_1  DDEATH
```

4. Vuelve a conectar el medidor.
5. Debe aparecer un nuevo `DBIRTH` y luego `DDATA` con valores.

El umbral de DDEATH esta en `include/SparkplugConfig.h`:

```cpp
constexpr uint8_t SPB_DEVICE_DEATH_CYCLES = 3;
```

### 5.6 Fallo parcial de un bloque

Si falla un solo bloque, el Device sigue online. Las metricas de ese bloque salen con `is_null=true` y las demas siguen con valores.
Para probarlo sin hardware especial, cambia temporalmente una direccion de `kMeterBlocks` en `include/DevicesConfig.h` y observa el `DDATA`.

## 6. Limpieza de topics viejos

El firmware anterior publicaba un JSON tipo ChirpStack y un `status` retained. Si ese topic sigue en el broker, borralo:

```powershell
mosquitto_pub -h 127.0.0.1 -r -n `
  -t "application/4567890123456781/device/3d17012a16db3d97/event/status"
```

El antiguo topic `application/.../event/up` no era retained, pero puede quedar en el historial del cliente.

## 7. Troubleshooting

| Sintoma | Causa probable |
|---|---|
| No aparece ningun topic | WiFi/MQTT no conectado; revisa monitor serie, IP del broker, firewall |
| Solo aparecen DDATA con `alias:10` | El consumidor no vio el birth; envia Rebirth |
| No aparece NBIRTH | NTP sin sincronizar (el birth se bloquea), fallo de publicacion o `SUBACK` de NCMD |
| Timestamps en 1970 o `-` | NTP no sincronizo; revisa DNS/internet o usa un NTP local |
| Dos equipos se pisan | `SPB_EDGE_NODE` debe ser unico por unidad; el client ID ya incluye la MAC |
| DDEATH inmediato | El medidor no responde; revisa slave ID, cableado A/B y alimentacion |
| `publish fallo` | Heap bajo o MQTT desconectado; revisa `FreeHeap_bytes` y logs |
| El birth aparece y desaparece | MQTT Explorer no retiene birth; es normal, envia Rebirth |
| NDEATH no aparece al apagar | El broker tarda el keepalive; espera ~22 s |

## 8. Configuracion

`include/netconfig.h`:

```c
#define SPB_GROUP_ID   "EMASA"
#define SPB_EDGE_NODE  "nemo_medidor1"
#define SPB_NTP1       "pool.ntp.org"
#define SPB_NTP2       "time.nist.gov"
```

`include/DevicesConfig.h`:

```cpp
{ "medidor_trifasico_1", 1, 10, 2000, ... }
```

- `deviceId`: nombre del Device en Sparkplug.
- `slaveId`: direccion Modbus.
- `aliasBase`: primer alias del Device (10 para el primero, 30 para el segundo, etc.).

Alias reservados del nodo:

| Metrica | Alias |
|---|---|
| `Uptime_ms` | 1 |
| `RSSI_dBm` | 2 |
| `FreeHeap_bytes` | 3 |
| `Modbus_ScanOK` | 4 |
| `bdSeq` | 5 |
| `Node Control/Rebirth` | 6 |

Notas:
- El **client ID MQTT** ahora incluye los ultimos 24 bits de la MAC (`nemo-medidor1-xxxxxx`).
- `SPB_EDGE_NODE` sigue siendo configurable: si instalas varias unidades, cada una debe tener un edge node ID distinto.
- El firmware **no publica NBIRTH hasta tener hora UTC valida**. Si el equipo no tiene NTP, no aparecera online en Sparkplug. Usa un NTP local en la LAN si no hay internet.
- El script Python es un consumidor y verificador basico, no reemplaza el Eclipse Sparkplug TCK.

## 9. Agregar un segundo medidor

Edita `include/DevicesConfig.h`:

```cpp
constexpr DeviceConfig kDevices[] = {
    { "medidor_trifasico_1", 1, 10, 2000, kMeterBlocks, ..., kMeterMetrics, ... },
    { "medidor_trifasico_2", 2, 30, 2000, kMeterBlocks, ..., kMeterMetrics, ... },
};
```

Si es otro modelo de medidor, define sus propios `kOtherBlocks`, `kOtherSignals` y `kOtherMetrics` y apunta esa entrada a esos arrays.
No hay que tocar `SparkplugB.cpp` ni el loop de polling.
