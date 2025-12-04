@file hardware.md
@defgroup group_hardware Hardware and System

@section group_hardware_overview Overview
This module implements the ESP32 hardware configuration and complete flow of:
- I2C peripheral configuration for ADS1015 communication.
- RS485 UART configuration for Modbus RTU communication.
- ADC conversion interrupt system initialization.
- FreeRTOS configuration with tasks distributed across cores.

@section Structure_and_components Structure and Components

- Pin and peripheral configuration
  - I2C_SDA_PIN, I2C_SCL_PIN: I2C bus for ADS1015.
  - RX_PIN, TX_PIN: RS485 serial communication.
  - ADS_ALERT_PIN: conversion complete interrupt.

- Hardware objects
  - Adafruit_ADS1015: analog-to-digital converter instance.
  - HardwareSerial: serial port for Modbus RTU.
  - ModbusServerRTU: Modbus protocol server.

- System configuration
  - NUM_CHANNELS: configured ADC channels (3 channels).
  - Speed parameters: 19200 bps UART, 400kHz I2C, 3300 SPS ADC.
  - GAIN_TWOTHIRDS: ADS1015 gain for ±6.144V range.

@section group_hardware_api Main API
- Configuration: `I2C_SDA_PIN`, `I2C_SCL_PIN`, `ADS_ALERT_PIN`, `RX_PIN`, `TX_PIN`, `NUM_CHANNELS`.
- Objects: `ads`.
- Functions: `setup()`, `loop()`, `on_adc_data_ready()`.

@section group_hardware_usage Usage
Flow summary:

1) Hardware initialization
   - I2C bus configuration (SDA=9, SCL=8) at 400kHz.
   - ADS1015 initialization with TWOTHIRDS gain and 3300 SPS speed.
   - UART configuration (RX=20, TX=21) at 19200 bps, 8N1 for RS485.
   - Interrupt registration on pin 10 for ADS1015 ALERT signal.

2) FreeRTOS task configuration
   - `task_adquisicion`: core 0, priority 5, ADC conversion management.
   - `task_procesamiento`: core 0, priority 3, RMS calculation with FIFOs.
   - `dataUpdateTask`: core 0, priority 1, Modbus register updates.
   - Modbus server: core 0, RTU request handling.

3) ADC configuration
   - Single-ended mode on 3 channels (A0, A1, A2).
   - Interrupt-driven conversions using ALERT signal.
   - Round-robin sequence between channels managed by ISR.

4) Modbus configuration
   - RTU server with ID=1, timeout=2000ms.
   - Worker registration for function 03 (holding register read).
   - Discovery response (registers 0-7) and RMS data (registers 10+).

Warnings and best practices:
- Verify I2C connections before `ads.begin()` to avoid hangs.
- Use external pullups on SDA/SCL lines if communication issues occur.
- Properly configure ADS1015 ALERT signal for interrupts.
- Verify I2C bus speed according to cable length.
- Protect concurrent hardware access using appropriate semaphores.
- Ensure stable power supply for accurate ADC conversions.