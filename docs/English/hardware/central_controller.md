# Central Controller

A microcontroller with high computing power, multiple input/output interfaces, integrated analog-to-digital conversion, and robust communication capabilities is required as the central control device. For these reasons, the ESP32 family was selected, which offers an appropriate balance between performance, wireless connectivity, development ecosystem, and cost, enabling local processing, communications management, and real-time control within a single SoC.

## Module Selection

To accelerate development and consolidate computing, radio, and power functions onto a single board, the TTGO LoRa32 v2.1 module/board is selected, based on the ESP32‑PICO‑D4 SoC and integrated with an SX127x family LoRa transceiver. This choice provides a ready-to-prototype and ready-to-operate environment, with wireless connectivity and sufficient I/O resources for system data acquisition and transmission.


<p style="text-align:center;"> <img src="ttgo_lora32.png" style="width:50%;"> </p>

### SoC ESP32 PICO D4 Capabilities

* CPU: dual-core Xtensa LX6 up to 240 MHz, with ULP coprocessor for ultra-low power tasks.
* Memory: integrated SRAM and support for external Flash; DMA for high-performance peripherals.
* ADC/DAC: 12-bit ADC with multiple channels and 2 8-bit DACs
* GPIO and interfaces: UART, I2C, SPI, I2S, PWM (ledc), RMT, SD/SDIO; Ethernet MAC and CAN (TWAI).
* Security: AES encryption, SHA, RSA/ECC, HMAC; Secure Boot and Flash Encryption.
* Industrial temperature range: −40 °C to +85 °C.

### Integrated LoRa Radio

The TTGO LoRa32 v2.1 incorporates a LoRaSX1276 transceiver compatible with LoRaWAN in the target regional band (868/915 MHz), with configurable dispersion factors and bandwidths, and adjustable transmission power for long-range links.

### Relevant Features of the TTGO LoRa32 v2.1

* Power supply: integrated regulator, USB input and 3V3 pin; charger for Li‑ion/Li‑Po battery with JST connector.
* User interface: integrated OLED display, buttons, and status LEDs for local diagnostics.
* Connectivity and programming: USB‑UART port for programming/debugging and log support; LoRa antenna included.