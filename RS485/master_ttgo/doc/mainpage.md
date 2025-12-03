@mainpage Modbus RTU Master Monitor

@section summary Resumen
Firmware para un Gateway maestro Modbus RTU en ESP32 TTGO LORA 32 V2.1 basado en RS485, planifica lecturas periódicas, normaliza los datos y los transmite por LoRaWAN (LMIC)dentro de un entorno FreeRTOS.

@section deps Dependencias
 * FreeRTOS: tareas, colas, semáforos y sincronización.
 * Arduino core ESP32 + SPI/UART + ModbusClientRTU.
 * [LMIC (LoRaWAN)](https://github.com/mcci-catena/arduino-lmic) y pila HAL asociada.
 * PlatformIO como toolchain recomendado.

@section groups Grupos
 * `group_modbus_discovery`: Modbus puro (descubrimiento, scheduler, EventManager, callbacks, tokens).
 * `group_data_format`: parsing y normalización de respuestas.
 * `group_lorawan`: LMIC y transmisión.
 * `group_tasks_utils`: tareas auxiliares y utilitarios (DataPrinter, helpers, defines comunes).
 
@section flow Flujo general
  1. **Arranque**: `setup()` inicializa Serial/SPI, ModbusClientRTU, colas y tareas.
  2. **Descubrimiento**: `initialDiscoveryTask` emite órdenes FC03→EventManager, que delega en MB; `parseAndStoreDiscoveryResponse` llena `slaveList`.
  3. **Planificación**: `initScheduler` deriva `scheduleList`; `DataRequestScheduler` publica eventos de muestreo en `queueEventos_Scheduler`.
  4. **Gestión de eventos**: `EventManager` prioriza scheduler vs periférico, crea tokens, invoca `MB.addRequest` y registra contexto en `requestBuffer`.
  5. **Callbacks Modbus**: `handleData`/`handleError` encapsulan respuestas y las envían a `queueRespuestas`, actualizando fallos y limpiando scheduler según sea necesario.
  6. **Formateo**: `DataFormatter` decide si la respuesta es descubrimiento o muestreo; usa `BitPacker` y `formatAndEnqueueSensorData` para generar `SensorDataPayload` y los envía a `queueSensorDataPayload`.
  7. **Agregación**: `DataAggregatorTask` consume payloads, arma un Activate Byte y construye el mensaje final con `construirPayloadUnificado`, que deposita en `queueFragmentos`.
  8. **Transmisión LoRaWAN**: `tareaLoRa` toma fragmentos, espera `semaforoEnvioCompleto` y llama a `LMIC_setTxData2`; `tareaRunLoop` mantiene `os_runloop_once`.
 
@section data_paths Colas y sincronización
* `queueEventos_Peripheral` y `queueEventos_Scheduler`: entrada al EventManager.
* `queueRespuestas`: puente callbacks → formatter.
* `queueSensorDataPayload`: datos normalizados listos para agregación.
* `queueFragmentos`: paquetes listos para LMIC.
* `schedulerMutex` y `semaforoEnvioCompleto`: sincronización crítica.
