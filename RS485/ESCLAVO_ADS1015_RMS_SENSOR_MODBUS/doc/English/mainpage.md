@mainpage Modbus RTU Slave Monitor with RMS Sensor

@section summary Summary
Firmware for Modbus RTU slave device on ESP32 with ADS1015 ADC, calculates real-time RMS values using circular FIFOs, maintains measurement history and responds to Modbus RTU queries over RS485 within a FreeRTOS environment.

@section deps Dependencies
 * FreeRTOS: tasks, queues, semaphores and synchronization.
 * Arduino core ESP32 + Wire/I2C + ModbusServerRTU.
 * [Adafruit_ADS1X15](https://github.com/adafruit/Adafruit_ADS1X15) for ADC communication.
 * PlatformIO as recommended toolchain.

@section groups Groups
 * `group_hardware`: pin configuration, peripherals and system initialization.
 * `group_adc_rms`: ADC acquisition, signal processing and RMS calculation with circular FIFOs.
 * `group_modbus`: Modbus RTU server, registers and RS485 communication.
 
@section flow General Flow
  1. **Startup**: `setup()` initializes Serial/I2C, ADS1015, ModbusServerRTU, queues and tasks.
  2. **ADC Configuration**: sets gain, sampling rate and interrupt mode for ADS1015.
  3. **Acquisition**: `task_adquisicion` manages round-robin conversions between channels using interrupts; queues samples in `queue_adc_samples`.
  4. **Processing**: `task_procesamiento` consumes samples, updates circular FIFOs with cumulative sums and calculates RMS periodically using incremental algorithms.
  5. **History**: RMS values are stored in thread-safe circular buffers protected by `rms_history_mutex`.
  6. **Modbus Update**: `dataUpdateTask` extracts data from history, applies conversion factor and updates `holdingRegisters`.
  7. **Modbus Responses**: `readHoldingRegistersWorker` handles read requests for configuration parameters (registers 0-7) and RMS data (registers 10+).
  8. **Debug**: `loop()` displays RMS values periodically for system verification.
 
@section data_paths Queues and Synchronization
* `queue_adc_samples`: bridge acquisition → processing of ADC samples.
* `fifos[NUM_CHANNELS]`: circular buffers for efficient RMS calculation.
* `rms_history_ch0/ch1/ch2`: independent histories for each channel.
* `holdingRegisters`: Modbus registers updated periodically.
* `rms_history_mutex` and `dataMutex`: critical synchronization for concurrent access.

@section modbus_interface Modbus Interface
* **Registers 0-7**: sensor configuration parameters (sensorID, numberOfChannels, startAddress, etc.).
* **Registers 10+**: historical RMS data organized by channel with applied conversion factor.
* **Function 03**: holding register read with address validation and error handling.
* **Discovery**: responds with `SensorData` structure for automatic identification by masters.

@section rms_algorithm RMS Algorithm
* **Circular FIFO**: 320-sample buffer with cumulative sums for efficiency.
* **Incremental calculation**: mean and variance calculated without traversing entire buffer.
* **Periodic update**: RMS values calculated every 1000ms.
* **Thread-safe**: protected access through semaphores for safe concurrency.