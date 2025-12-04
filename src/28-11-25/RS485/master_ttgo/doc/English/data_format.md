@file data_format.md
@defgroup group_data_format Data Format & Payload

@section group_data_format_overview Overview
This module is responsible for:
- Parsing raw Modbus responses from `queueRespuestas`.
- Normalizing data per sensor and generating `SensorDataPayload`.
- Aggregating multiple payloads into a compact message with Activate Byte and metadata.
- Delivering ready-for-transmission fragments to the external layer (LMIC) via `queueFragmentos`.

@section Structure_and_components Structure and components

- Intermediate transport
  - ResponseFormat: raw data from `handleData()` (bytes, length, deviceId, order).

- Normalized data
  - SensorDataPayload: per-sensor packet with processed registers and metadata (sensorID, slaveID, useful length).

- Tasks/Flow
  - `DataFormatter`: decides if response is discovery or sampling, and produces `SensorDataPayload`.
  - `DataAggregatorTask`: consumes payloads, builds Activate Byte, adds headers and constructs final message.

@section group_data_format_api Main API
- Input
  - `queueRespuestas` (ResponseFormat).
- Output
  - `queueSensorDataPayload` (SensorDataPayload).
  - `queueFragmentos` (aggregated message ready for transmission).
- Utilities
  - `formatAndEnqueueSensorData()` (extraction and formatting).
  - `parseAndStoreDiscoveryResponse()` (discovery; updates structures).
  - `construirPayloadUnificado()` (aggregation).
  - `BitPacker` optional compression by `compressedBytes`.

@section group_data_format_usage Usage
Flow summary:

1) Response formatting
   - `DataFormatter` reads `ResponseFormat` from `queueRespuestas`.
   - Discovery: validates structure and delegates sensor parameter update (see group \ref group_modbus_discovery "Modbus").
   - Sampling: extracts registers, applies `dataType`, `scale` and compression if `compressedBytes > 0`.
   - Generates `SensorDataPayload` and publishes it to `queueSensorDataPayload`.

2) Payload aggregation
   - `DataAggregatorTask` consumes `SensorDataPayload` within a fixed interval.
   - Builds Activate Byte per present sensor and calculates lengths per channel.
   - Calls `construirPayloadUnificado()` and publishes result to `queueFragmentos`.

3) Delivery to transmission
   - The aggregated fragment is ready to be taken by the external transmission task.


@section Structure_and_encoding Structure and payload encoding

The firmware uses an optimized binary scheme to maximize transmission efficiency over LoRaWAN. This section describes the payload structure to facilitate decoder implementation.

Payload components:

- Header (1 byte): Message identifier to maintain reception order.
- Timestamp (4 bytes): Time mark since system startup in UNIX format.
- Activate byte (1 byte): Flags indicating operation by sensor type, encoded bit by bit (Bit 0 = battery, Bit 1 = voltage, Bit 2 = current; rest reserved).
- Data length bytes (depend on Activate byte): Define quantity and encoding format for each active sensor type. The number of these bytes equals the number of active sensors indicated in the Activate byte.

Structure of each length byte (8 bits):

```
Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
------|-------|-------|-------|-------|-------|-------|-------
PKD   | 2BIT  |               DATA_LENGTH (5 bits)
```

Field descriptions:

- Bits 0-4 (DATA_LENGTH): Number of sensor readings (0–31 samples).
- Bit 6 (2BIT):
  - 0: Standard encoding (1 bit per sample).
  - 1: 2-bit per sample encoding.
- Bit 7 (PKD):
  - 0: Byte-aligned format.
  - 1: Bit-level packed format.

Length byte order:

Length bytes follow the same order as the Activate byte flags (LSB → MSB):

- Bit 0: Battery (if active)
- Bit 1: Voltage (if active)
- Bit 2: Current (if active)

Example: If Activate byte = 0x06 (voltage + current active), there will be 2 length bytes: first for voltage, second for current.

- Data bytes: Bytes with sensor data; both the Activate byte and length bytes are necessary to decode this data.