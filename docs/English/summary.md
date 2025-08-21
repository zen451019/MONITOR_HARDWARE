# SUMMARY

## Objective

Monitor operating variables in electromechanical systems using dedicated hardware: precise measurement of operating voltage, operating current, battery level, and other critical parameters through sensors and data acquisition modules, with wireless data transmission to a LoRaWAN Gateway.
## Key Metrics

- **System voltage:** ability to accurately measure voltage in single-phase, two-phase, and three-phase AC systems, within a range of 0 to 220 V AC.
- **System current:** ability to accurately measure alternating current in single-phase, two-phase, and three-phase systems, within a range of 0 to 100 A AC.
- **Battery voltage:** ability to accurately measure the battery voltage of the electromechanical system (when applicable) and estimate its state of charge (SoC).
- **Event detection:** ability to detect power-on and power-off events in the electromechanical system.
## Requirements

In addition to measuring current and voltage in three-phase systems, the module must wirelessly transmit the information using LoRaWAN technology, sending it to a gateway located a few meters or even tens of meters away from the acquisition module. It must also guarantee uninterrupted transmission, requiring autonomous operation capability when the main power supply is deactivated.

### [Electromechanical Variables Monitoring Board — Monitor](monitor_plate)