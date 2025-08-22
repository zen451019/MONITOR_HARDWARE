RMS_Voltage (ESP32) - Project README

Overview
--------
This project runs on an ESP32 and performs multichannel RMS measurement for voltage and current sensors. It:

- Samples analog signals from configured ADC pins using a high-frequency ISR-driven sampler.
- Computes RMS per channel from a FIFO of raw ADC samples.
- Applies an adaptive EMA (exponential moving average) filter to stabilize readings.
- Aggregates measurements into blocks and encodes them into compact binary fragments.
- Sends fragments over LoRaWAN (LMIC library) to a gateway.
- Periodically measures battery level and transmits it.
- Detects system enable/disable via a monitor pin (MONITOR_PIN) and shows recent events on an SSD1306 OLED.

Key features
------------
- Multichannel acquisition using a timer-driven ISR that rotates through enabled ADC pins.
- RMS calculation using incremental sums (sum_x, sum_x2) stored in a per-pin FIFO for efficiency.
- Compact binary encoding and fragmenting for LoRa payload limits.
- FreeRTOS-based architecture: separate tasks for processing, storage/encoding, LoRa transmission, battery monitoring and display.

Hardware
--------
- ESP32 (board compatible with PlatformIO project configuration)
- ADC sensors connected to pins declared in `pin_configs` (see `src/main.cpp`)
- OLED SSD1306 connected via I2C (address 0x3C)
- LoRa radio pins mapped in `lmic_pins` (adjust for your board)
- Monitor pin (`MONITOR_PIN`) to enable/disable the system

Build & Run
-----------
This project uses PlatformIO. From the project root (where `platformio.ini` is located):

```powershell
platformio run
platformio run -t upload
```

Adjust board and upload options in `platformio.ini` if needed.

Code structure
--------------
- `src/main.cpp` - Main application. Contains:
  - ISR: `onADCTimer` (ADC sampling)
  - Tasks: `TaskProcesamiento`, `TaskRegistroResultados`, `TaskBatteryLevel`, `TaskDisplay`, `TaskMonitorPin`, `tareaLoRa`
  - Encoding: `codificarYFragmentar`, `codificarYFragmentarBattery`
  - Helpers: `applyEMA*`, `calculateRMS_fifo`

- `include/`, `lib/`, and `backups/` - supporting headers, libraries and older snapshots.

Important notes
---------------
- The LMIC keys in `src/main.cpp` are placeholders; replace with your network credentials or use OTAA as needed.
- Keep EMA parameters and PROCESS_PERIOD_MS tuned to match sampling frequency (FS_HZ) and desired responsiveness.
- Verify LoRa region settings (sub-band, DR) to match your gateway/provider.

Next steps / Improvements
------------------------
- Split `main.cpp` into smaller modules: `adc.cpp/h`, `lora.cpp/h`, `display.cpp/h` for maintainability.
- Add unit tests for encoding/decoding routines or a small host-side script to validate fragments.


Contact
-------
Manuel Felipe Ospina
