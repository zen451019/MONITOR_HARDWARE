
# NEMO: Electromechanical Monitoring Firmware

This repository contains the complete source code for the firmware for NEMO (Node for Electromechanical Monitoring & Operation), a high-performance IoT node designed for real-time monitoring of analog signals.

The system is optimized to capture, process, and transmit voltage and current data from sensors, making it an ideal solution for predictive maintenance, energy consumption monitoring, and industrial machinery diagnostics applications.

## Project Philosophy

The NEMO firmware is built on three fundamental pillars:

1. Accuracy in Acquisition: The basis of any monitoring system is data quality. A hardware timer interrupt (ISR) is used to sample signals at a high and constant frequency, ensuring an accurate representation of waveforms.

2. Edge Computing: Sending raw data over low-bandwidth networks such as LoRaWAN is inefficient. This firmware performs all heavy processing (RMS calculation, filtering) directly on the ESP32, sending only valuable, pre-processed information.

3. Robust and Efficient Operation: Thanks to its FreeRTOS-based architecture, the system is inherently multitasking. Critical operations such as sampling, processing, and communication occur concurrently without interfering with each other, ensuring stable, low-power operation.

## System Architecture

The firmware operates as a data pipeline managed by several concurrent FreeRTOS tasks.

### 1. Sampling Stage (ISR - `onADCTimer`)

- **Function:** This is the heart of the system. A timer interrupt is triggered at `FS_HZ` (960 Hz by default).

- **Operation:** On each shot, the ISR reads the raw value from an ADC pin and stores it in a dedicated circular FIFO buffer (`RMS_FIFO`) for that pin. The ISR is minimal to ensure ultra-fast execution and not to disturb the sampling accuracy. The system rotates between the enabled ADC pins on each call.

### 2. Processing Stage (`TaskProcesamiento`)

- **Function:** Convert raw data into useful metrics.

- **Operation:** This task wakes up periodically (every `PROCESS_PERIOD_MS`). For each enabled channel, it calculates the RMS value from the data in its FIFO buffer. It then applies an **adaptive EMA (Exponential Moving Average) filter** that intelligently smooths the reading: it applies more filtering when the signal is stable and less when it detects rapid changes.

### 3. Storage and Encoding Stage (`TaskRegistroResultados`)

- **Function:** Acts as the main orchestrator. Collects the results, packages them, and prepares them for shipment.

- **Operation:** 
	 1. Receives the filtered RMS results from the `TaskProcesamiento`.
	
	2. Stores them in a `RESULTADOS_POR_BLOQUE` block (20 by default).
	
	3. Once the block is full, invokes the `codificarUnificado` function.
	
	4. This function creates a highly compact binary payload using a `BitPacker` that packs the current values (10 bits per sample) to save space. It also integrates the latest battery level reading.
	
	5. The final payload is placed in a queue (`queueFragmentos`) to be sent by LoRaWAN, and a summary is sent to the display task.

#### Payload Structure and Encoding

The NEMO firmware uses a highly optimized binary encoding scheme to maximize data transmission efficiency over LoRaWAN. This section describes the payload structure to facilitate decoder implementation.

**Payload Components:**

- **Header (1 byte):** Contains the message identifier, allowing the system to keep a clear order of the messages received.
- **Timestamp (4 bytes):** System startup timestamp in UNIX format.
- **Activate byte (1byte):** Flags indicating sensor operation encoded in each bit of the byte (0 = battery, 1 = voltage, 2 = current), the rest reserved for future sensors.
- **Data length bytes (dependent on Activate byte):** These bytes define the length and encoding format for each active sensor type. The number of Data Length bytes equals the number of active sensors indicated in the Activate byte.

	##### Byte Structure (8 bits total):
	
	```
	Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
	------|-------|-------|-------|-------|-------|-------|-------
	PKD   | 2BIT  |       |        DATA_LENGTH (5 bits)         
	```    
	
	##### Bit Field Descriptions:
	
	**Bits 0-4 (DATA_LENGTH):** Number of sensor readings (0-31 samples)
	
	**Bit 6 (2BIT Flag):**
	
	- `0`: Standard encoding (1-bit per sample)
	- `1`: 2-bit encoding per sample
	
	**Bit 7 (PKD - Packed Flag):**
	
	- `0`: Byte-aligned format
	-  `1`: Bit-packed format
	
	##### Byte Order:
	
	Data Length bytes follow the same order as the Activate byte flags (LSB to MSB):
	
	- **Bit 0:** Battery (if active)
	- **Bit 1:** Voltage (if active)
	- **Bit 2:** Current (if active)
	
	**Example:** If Activate byte = `0x06` (voltage + current active), you'll have 2 Data Length bytes: first for voltage, second for current.

- **Data bytes:** bytes with sensor data; both the active byte and data length bytes are necessary to decode this data.

#### Bit Packing Process
When the PKD (Packed) flag is set, NEMO uses a custom bit-packing algorithm to maximize data efficiency. The process packs 10-bit sensor values sequentially into a continuous bit stream, eliminating the typical byte-alignment padding.

**Packing Method:**
- Each sensor reading is stored as exactly 10 bits (range 0-1023)
- Values are packed sequentially without byte boundaries
- A custom `BitPacker` class handles the bit-level operations
- Result: ~25% space savings compared to standard 16-bit storage

**Example:** 3 samples with values [512, 1023, 256]

```c++
Standard:  [0x02,0x00] [0x03,0xFF] [0x01,0x00]  = 6 bytes
Bit-packed: [0x80,0x0F,0xFF,0x00,0x10]        = 4 bytes (30 bits)
```

This efficient packing allows more sensor data per LoRaWAN frame while maintaining full 10-bit precision.
### 4. Support Tasks

- `tareaLoRa`: Task dedicated to managing the LoRaWAN communication stack (LMIC). It is a consumer that waits patiently in the `queueFragmentos` queue to send data as soon as it is ready.

- `TaskDisplay`: Manages the OLED display. It shows a startup screen with the logo and then enters an “event history” mode, displaying a summary of the latest transmissions sent. It is efficient, as it only redraws the screen when it receives new information through its queue (`queueDisplayInfo`).

- `TaskMonitorPin`: Allows you to enable or disable the system via a physical pin. When disabled, sampling and processing tasks are paused to save power, but the system continues to report the battery level periodically.

- `TaskBatteryLevel`: Low-priority task that runs every 60 seconds to measure battery voltage, ensuring that the device's status is always known.

## Getting Started

### Hardware Requirements

- **Plate**: Electromechanical variable monitoring module board MONITOR for NEMO
### Configuration and Compilation

This project is designed to be used with PlatformIO and Visual Studio Code.

1. **Clone this repository.**

2. **Open the project with PlatformIO** in VS Code. Library dependencies (`LMIC`, `Adafruit GFX`, etc.) will be downloaded automatically.

3. **Configure your LoRaWAN credentials:**

	- Open `src/main.cpp`.
	
	- Find the following variables and replace the example values with your own LoRaWAN network keys (the code uses **ABP** by default):
	
		```c++
		static u1_t NWKSKEY[16] = { 0x49, ... };
		static u1_t APPSKEY[16] = { 0x53, ... };
		static const u4_t DEVADDR = 0x260CB229;
		```

4. **Adjust the LoRaWAN region:**

- The firmware is configured for `US915`. If you are in another region (e.g., `EU868`), find the `initLoRa()` function and adjust the `LMIC_selectSubBand()` and `LMIC_setDrTxpow()` settings accordingly.
5. **Compile and Upload the Firmware:**

- Use the PlatformIO buttons in the VS Code status bar or run the following commands in the terminal:
	```bash
	platformio run
	platformio run -t upload
	```

## Roadmap and Contributions

- [ ] **Modularize the Code:** Separate the logic of `main.cpp` into smaller, more manageable files (e.g., `lora.cpp`, `display.cpp`, `adc.cpp`).

- [ ] **Implement OTAA:** Add support for Over-the-Air Activation (OTAA) in LoRaWAN, which is a more secure and flexible method than ABP.
  
- [ ] **Configuration Management:** Move all key definitions and parameters to a single `config.h` file to facilitate adjustments.