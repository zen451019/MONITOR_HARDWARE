@file modbus.md
@defgroup group_modbus_discovery Modbus Core

@section group_modbus_overview Overview
This module implements the Modbus RTU client (ESP32/TTGO) and the complete flow of:
- Initial sensor discovery on RS485 slaves.
- Periodic reading scheduling (Scheduler) based on each sensor's parameters.
- Response and error management with token correlation.
- Data formatting in payloads per sensor and aggregation in a LoRaWAN payload.

@section Structure_and_components Structure and components

- Data and configuration
  - ModbusSensorParam, ModbusSlaveParam: sensor and slave definitions.
  - SensorSchedule: sampling scheduler entries.
  - ResponseFormat: internal response transport from callbacks.

- Requests and correlation
  - ModbusRequestInfo: asynchronous request tracking by token.
  - API: addRequest(), findRequestByToken(), DiscoveryOrder, RequestType.

- Main tasks
  - `initialDiscoveryTask`: sensor discovery.
  - `EventManager`: translates events into Modbus transactions and registers tokens.
  - `DataRequestScheduler`: generates sampling events according to scheduleList.
  - Callbacks: `handleData` and `handleError`.

@section group_modbus_api Main API
- Data/Config: `ModbusSensorParam`, `ModbusSlaveParam`, `SensorSchedule`.
- Transport: `ResponseFormat`.
- Requests: `ModbusRequestInfo`, `DiscoveryOrder`, `RequestType`, `addRequest()`, `findRequestByToken()`.
- Callbacks/Tasks: `initialDiscoveryTask`, `handleData`, `handleError`, `EventManager`, `DataRequestScheduler`.

@section group_modbus_usage Usage
Flow summary:

1) Modbus startup
   - Initializes UART RS485 and ModbusClientRTU (timeout/begin).
   - Registers callbacks: handleData(), handleError().
   - Creates tasks: EventManager and DataRequestScheduler.
   - Launches initialDiscoveryTask with devicesToQuery.

2) Discovery
   - initialDiscoveryTask publishes order=DISCOVERY_READ_SENSOR_PARAM.
   - EventManager sends read request (addr=0, qty=8) and registers token.
   - handleData() parses and updates slaveList with parseAndStoreDiscoveryResponse().
   - initScheduler() calculates scheduleList.

3) Periodic sampling
   - DataRequestScheduler triggers REQUEST_SAMPLING according to nextSampleTime.
   - EventManager queries getSensorParams() and sends read request with startAddress/numRegs.
   - handleData() delivers registers to the next subsystem.

4) Modbus errors
   - handleError(): handles TIMEOUT, increases consecutiveFails, removes inactive slaves and recalculates scheduler.
   - Tokens are invalidated after processing.

Warnings and best practices:
- Ensure consistent layout of 8 parameter registers for discovery.
- Respect offset 3 for register data in Modbus RTU responses.
- Protect scheduleList access with `schedulerMutex`.
- Review `MAX_*` limits to avoid overflow in queues and tokens.