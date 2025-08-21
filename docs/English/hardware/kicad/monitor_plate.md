# Electromechanical variable monitoring module MONITOR

Electronic board based on the [TTGO LoRa32 V2.1](../central_controller.md) module (ESP32 + LoRa) for measuring electrical parameters of electromechanical systems and transmitting them wirelessly. The system integrates three-phase AC voltage and current sensing, battery level monitoring, and a digital input for event detection.
#### Specifications
- Power supply: 3.3 V / 800 mA LDO for logic.
- Dedicated inputs for external current sensors.
- Integrated LoRa connectivity in the TTGO LoRa32 V2.1.
- Screw terminals for AC mains and sensor connections.
### [Hardware Architecture](../architecture.md)

## Interfaces and Pins

Voltage measurement: terminal blocks for connecting the three-phase AC voltage signal using [ZMPT101B](../voltage_sensor.md) modules, with power supply and analog output to the ESP32 ADC through a 5-pin JST connector.

| INPUT | DESCRIPTION | OUTPUT  |
| ----- | ----------- | ------- |
| 1     | NEUTRAL     |         |
| 2     | LINE 3      | GPIO 36 |
| 3     | LINE 2      | GPIO 39 |
| 4     | LINE 1      | GPIO 34 |

Current measurement: terminal blocks for connecting [SCT-013-000](../current_sensor.md) sensors (current transformers), conditioning circuit, and output to the ESP32 ADC.

| INPUT | DESCRIPTION             | OUTPUT  |
| ----- | ----------------------- | ------- |
| 8     | SENSOR 3 WHITE TERMINAL |         |
| 9     | SENSOR 3 RED TERMINAL   | GPIO 13 |
| 10    | SENSOR 2 WHITE TERMINAL |         |
| 11    | SENSOR 2 RED TERMINAL   | GPIO 04 |
| 12    | SENSOR 1 WHITE TERMINAL |         |
| 13    | SENSOR 1 RED TERMINAL   | GPIO 25 |
Battery: terminal block connection for [battery voltage](../battery_level.md), resistive divider output to ESP32 ADC.

Power on/off signal: terminal block connection to [level shifter](../ignition_detection.md) circuit with PC817 optocoupler, output to ESP32 pull-up configured pin.

| INPUT | DESCRIPTION      | OUTPUT  |
| ----- | ---------------- | ------- |
| 5     | BATTERY NEGATIVE |         |
| 6     | POWER ON SIGNAL  | GPIO 35 |
| 7     | BATTERY POSITIVE | GPIO 14 |

[Power Supply](../power_supply.md): 3.3 Volt input through a 2-pin JST connector (auxiliary 4-pin JST connector, not soldered), powering the ESP32 and other components.
#### Electrical Requirements

- Main input: 3.3 V DC (capacity according to TTGO + sensor consumption).
- Analog signals: matched to the ESP32 ADC range (0–3.3 V). It is recommended to verify that dividers/conditioning circuits maintain proper amplitude and offset.
### [Electrical Schematic of the Monitor Board](./Schema.md)
— Detailed diagram of components and electrical connections.

### [PCB Design of the Monitor Board](./plate.md)
— Layout and physical distribution of components on the printed circuit board.

### [3D Model of the Monitor](./3D_E.md)
— Three-dimensional representation of the final assembly for visualization and mechanical verification.