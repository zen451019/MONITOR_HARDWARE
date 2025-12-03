@file lorawan.md
@defgroup group_lorawan LoRaWAN

@section group_lorawan_overview Overview
This module manages LoRaWAN transmission using LMIC:
- Initializes the LMIC stack and device HAL (TTGO LORA 32 V2.1).
- Manages frame construction and transmission from `queueFragmentos`.
- Monitors LMIC events (EV_TXCOMPLETE, join, duty cycle) and synchronizes via `semaforoEnvioCompleto`.

@section Structure_and_components Structure and components

- Configuration
  - Network parameters (OTAA/ABP), channels and region.
  - HAL and SPI/Radio pinmap.

- Transport to LMIC
  - Fragment: binary block ready for transmission (produced by aggregation in Data Format).

- Tasks/Callbacks
  - `tareaLoRa`: takes fragments from `queueFragmentos` and calls `LMIC_setTxData2`.
  - `tareaRunLoop`: executes `os_runloop_once` to process LMIC events.
  - `onEvent()`: event callback (EV_TXCOMPLETE, EV_JOINED, EV_LINK_DEAD, etc.).

@section group_lorawan_api Main API
- Input
  - `queueFragmentos` (payload ready for TX).
- Output
  - Signaling via `semaforoEnvioCompleto` after EV_TXCOMPLETE.
- Functions/Callbacks
  - `LMIC_setTxData2(port, data, len, confirmed)`.
  - `onEvent(ev)` state and error handling.
  - `tareaLoRa`, `tareaRunLoop`.
- Configuration
  - `LMIC_reset()`, `LMIC_setTxData2()`, `LMIC_setClockError()`, channel/region setup.

@section group_lorawan_usage Usage
Flow summary:

1) LoRaWAN startup
   - Initializes HAL/SPI/Radio and resets LMIC (`LMIC_reset()`).
   - Configures OTAA/ABP, region and channels.
   - Creates `tareaRunLoop` and `tareaLoRa`.

2) Data transmission
   - `tareaLoRa` blocks on `queueFragmentos`, builds frame and calls `LMIC_setTxData2`.
   - Avoids collisions by checking `LMIC.opmode & OP_TXRXPEND`.

3) LMIC events
   - `onEvent(EV_TXCOMPLETE)`: releases `semaforoEnvioCompleto` and optionally schedules next transmission.
   - `onEvent(EV_JOINED/EV_JOINING)`: controls join state.
   - Handles duty cycle and retries according to region.

Best practices (LoRaWAN)
- Check `OP_TXRXPEND` before sending to avoid breaking duty cycle.
- Use consistent application ports and payload sizes within regional limits.
- Minimize work inside `onEvent`; delegate to tasks when possible.
- Adjust `LMIC_setClockError()` if crystal has notable deviation.
- Ensure join retry with backoff and log states for diagnostics.