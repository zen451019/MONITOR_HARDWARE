@file modbus.md
@defgroup group_modbus Modbus Communication

@section group_modbus_overview Overview
This module implements the Modbus RTU server (ESP32) and the complete flow of:
- Response to sensor discovery requests from RS485 masters.
- Holding register management for RMS data and configuration parameters.
- Periodic data updates from the RMS processing subsystem.
- Error handling and address validation in the Modbus protocol.

@section Structure_and_components Structure and Components

- Data and configuration
  - SensorData: sensor parameter definition for discovery.
  - holdingRegisters[]: Modbus register array with RMS data.
  - sensor: global instance with device configuration.

- Communication and protocol
  - ModbusServerRTU: RTU server with configurable timeout.
  - HardwareSerial: RS485 serial port for physical communication.
  - readHoldingRegistersWorker(): main worker for function 03.

- Main tasks
  - `dataUpdateTask`: periodic register update from RMS history.
  - Callbacks: registered worker for READ_HOLD_REGISTER.

@section group_modbus_api Main API
- Configuration: `SLAVE_ID`, `NUM_REGISTERS`, `MODBUS_UPDATE_INTERVAL_MS`.
- Data: `SensorData`, `holdingRegisters[]`, `sensor`.
- Communication: `ModbusSerial`, `MBserver`, `dataMutex`.
- Functions: `dataUpdateTask()`, `readHoldingRegistersWorker()`.

@section group_modbus_usage Usage
Flow summary:

1) Modbus initialization
   - RS485 UART configuration (19200 bps, 8N1) on pins RX=20, TX=21.
   - ModbusServerRTU initialization with SLAVE_ID=1 and timeout=2000ms.
   - Worker registration for function 03 (holding register read).
   - Creation of `dataMutex` for register protection.

2) Sensor discovery
   - readHoldingRegistersWorker() responds to requests for addresses 0-7.
   - Returns SensorData structure with: sensorID=1, numberOfChannels=3, startAddress=10.
   - Includes parameters: maxRegisters=18, samplingInterval=1000ms, dataType=1, scale=1.
   - Enables automatic identification by Modbus masters.

3) Data update
   - `dataUpdateTask` runs every 300ms on core 0 with priority 1.
   - Extracts RMS history using get_rms_history() for each channel.
   - Applies CONVERSION_FACTOR=0.618 to convert to physical units.
   - Updates holdingRegisters[] thread-safely with dataMutex.

4) Data read response
   - readHoldingRegistersWorker() handles requests for addresses 10+.
   - Validates address range and number of requested registers.
   - Protects access to holdingRegisters[] with dataMutex (100ms timeout).
   - Returns RMS data organized by channel or ILLEGAL_DATA_ADDRESS error.

5) Modbus error handling
   - Validation of out-of-range addresses (>= NUM_REGISTERS).
   - SERVER_DEVICE_BUSY response if dataMutex cannot be obtained.
   - Debug logs for received ServerID and FunctionCode.

Warnings and best practices:
- Ensure consistent layout of 8 parameter registers for discovery.
- Respect data offset in Modbus RTU registers according to specification.
- Protect access to holdingRegisters[] with dataMutex in all operations.
- Verify NUM_REGISTERS limits to prevent buffer overflow.
- Maintain consistency between numberOfChannels and data organization in registers.
- Configure appropriate server timeout according to expected RS485 bus latency.