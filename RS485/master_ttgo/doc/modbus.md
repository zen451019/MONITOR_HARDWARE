@file modbus.md
@defgroup group_modbus_discovery Modbus Core

@section group_modbus_overview Resumen
Este módulo implementa el cliente Modbus RTU (ESP32/TTGO) y el flujo completo de:
- Descubrimiento inicial de sensores en esclavos RS485.
- Planificación de lecturas periódicas (Scheduler) en función de parámetros de cada sensor.
- Gestión de respuestas y errores con correlación por token.
- Formateo de datos en payloads por sensor y agregación en un payload LoRaWAN.

@section Estructura_y_componentes Estructura y componentes

- Datos y configuración
  - ModbusSensorParam, ModbusSlaveParam: definición de sensores y esclavos.
  - SensorSchedule: entradas del planificador de muestreo.
  - ResponseFormat: transporte interno de respuestas desde callbacks.

- Solicitudes y correlación
  - ModbusRequestInfo: rastreo de solicitudes asíncronas por token.
  - API: addRequest(), findRequestByToken(), DiscoveryOrder, RequestType.

- Tareas principales
  - `initialDiscoveryTask`: descubrimiento de sensores.
  - `EventManager`: traduce eventos en transacciones Modbus y registra tokens.
  - `DataRequestScheduler`: genera eventos de muestreo según scheduleList.
  - Callbacks: `handleData` y `handleError`.

@section group_modbus_api API principal
- Datos/Config: `ModbusSensorParam`, `ModbusSlaveParam`, `SensorSchedule`.
- Transporte: `ResponseFormat`.
- Solicitudes: `ModbusRequestInfo`, `DiscoveryOrder`, `RequestType`, `addRequest()`, `findRequestByToken()`.
- Callbacks/Tareas: `initialDiscoveryTask`, `handleData`, `handleError`, `EventManager`, `DataRequestScheduler`.

@section group_modbus_usage Uso
Resumen del flujo:

1) Arranque Modbus
   - Inicializa UART RS485 y ModbusClientRTU (timeout/begin).
   - Registra callbacks: handleData(), handleError().
   - Crea tareas: EventManager y DataRequestScheduler.
   - Lanza initialDiscoveryTask con dispositivosAConsultar.

2) Descubrimiento
   - initialDiscoveryTask publica order=DISCOVERY_READ_SENSOR_PARAM.
   - EventManager envía solicitud de lectura (addr=0, qty=8) y registra token.
   - handleData() parsea y actualiza slaveList con parseAndStoreDiscoveryResponse().
   - initScheduler() calcula scheduleList.

3) Muestreo periódico
   - DataRequestScheduler dispara REQUEST_SAMPLING según nextSampleTime.
   - EventManager consulta getSensorParams() y envía solicitud de lectura con startAddress/numRegs.
   - handleData() entrega registros al siguiente subsistema.

4) Errores Modbus
   - handleError(): gestiona TIMEOUT, aumenta consecutiveFails, elimina esclavos inactivos y recalcula scheduler.
   - Los tokens se invalidan tras procesarse.

Advertencias y buenas prácticas:
- Asegurar layout consistente de 8 registros de parámetros para descubrimiento.
- Respetar el offset 3 para datos de registros en respuestas Modbus RTU.
- Proteger acceso a scheduleList con `schedulerMutex`.
- Revisar límites `MAX_*` para evitar overflow en colas y tokens.