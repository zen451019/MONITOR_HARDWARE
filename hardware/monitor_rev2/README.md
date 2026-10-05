# MODBUS MASTER MONITOR

## Description
This PCB uses an ESP32 TTGO LoRa V2.1 that functions as a Modbus master and acts as a gateway for Modbus slaves with various integrated sensors. The design includes a UART to RS485 transceiver for network communication using the Modbus protocol. Additionally, it integrates a voltage detector for a 12-volt battery, a multipurpose digital input, and an 8-position DIP switch for Modbus address selection.

## Repository Contents
- `monitor_REV2/` folder: KiCad files for the schematic and PCB.
- `docs/` folder: Diagrams, images, and PDFs of the design.

## Schematic
![Schematic](docs/schematic.pdf)

## PCB
![PCB](docs/pcb.pdf)

## Manufacturing
### Gerber Files
The Gerber files are in `monitor_REV2/output`.

### Bill of Materials (BOM)
Included as `monitor_REV2/monitor REV2.csv`.


