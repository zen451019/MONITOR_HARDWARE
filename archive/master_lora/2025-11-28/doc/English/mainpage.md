@mainpage Modbus RTU Master Monitor

@section summary Summary
Firmware for a Modbus RTU master gateway on ESP32 TTGO LORA 32 V2.1 based on RS485, schedules periodic readings, normalizes data and transmits it via LoRaWAN (LMIC) within a FreeRTOS environment.

@section deps Dependencies
 * FreeRTOS: tasks, queues, semaphores and synchronization.
 * Arduino core ESP32 + SPI/UART + ModbusClientRTU.
 * [LMIC (LoRaWAN)](https://github.com/mcci-catena/arduino-lmic) and associated HAL stack.
 * PlatformIO as recommended toolchain.

@section groups Groups
 * `group_modbus_discovery`: Pure Modbus (discovery, scheduler, EventManager, callbacks, tokens).
 * `group_data_format`: parsing and response normalization.
 * `group_lorawan`: LMIC and transmission.
 
@section flow General flow
  1. **Startup**: `setup()` initializes Serial/SPI, ModbusClientRTU, queues and tasks.
  2. **Discovery**: `initialDiscoveryTask` issues FC03→EventManager orders, which delegates to MB; `parseAndStoreDiscoveryResponse` fills `slaveList`.
  3. **Scheduling**: `initScheduler` derives `scheduleList`; `DataRequestScheduler` publishes sampling events to `queueEventos_Scheduler`.
  4. **Event management**: `EventManager` prioritizes scheduler vs peripheral, creates tokens, invokes `MB.addRequest` and registers context in `requestBuffer`.
  5. **Modbus callbacks**: `handleData`/`handleError` encapsulate responses and send them to `queueRespuestas`, updating failures and cleaning scheduler as needed.
  6. **Formatting**: `DataFormatter` decides if response is discovery or sampling; uses `BitPacker` and `formatAndEnqueueSensorData` to generate `SensorDataPayload` and sends them to `queueSensorDataPayload`.
  7. **Aggregation**: `DataAggregatorTask` consumes payloads, builds an Activate Byte and constructs the final message with `construirPayloadUnificado`, which deposits to `queueFragmentos`.
  8. **LoRaWAN transmission**: `tareaLoRa` takes fragments, waits for `semaforoEnvioCompleto` and calls `LMIC_setTxData2`; `tareaRunLoop` maintains `os_runloop_once`.
 
@section data_paths Queues and synchronization
* `queueEventos_Peripheral` and `queueEventos_Scheduler`: input to EventManager.
* `queueRespuestas`: bridge callbacks → formatter.
* `queueSensorDataPayload`: normalized data ready for aggregation.
* `queueFragmentos`: packets ready for LMIC.
* `schedulerMutex` and `semaforoEnvioCompleto`: critical synchronization.
