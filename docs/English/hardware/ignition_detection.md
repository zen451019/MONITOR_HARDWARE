# Ignition Detection

The module must have a mechanism to detect when the electromechanical system is turned on and off, as certain variables depend directly on it being active in order to be processed correctly. To this end, a digital input is designed to allow the microcontroller to capture external signals.

The signal to be monitored will be the presence or absence of DC voltage from the battery, the controller, or any other device in the electromechanical system. This offers flexibility in both hardware (using a relay, the controller itself, etc., to generate the power-on signal) and software for encoding the logic.

## Signal Conditioning

In order for the microcontroller to safely capture these signals, given the differences in voltage levels, an opto-coupler level shifter is used, providing level adaptation and galvanic isolation.

<p style="text-align:center;"> <img src="level_shifter.drawio.svg" style="width:50%;"> </p>

Each time a digital signal arrives, the optocoupler activates its diode, which in turn turns on the phototransistor. This allows the pin on the [ESP32](central_controller) to drop to GND. For correct detection, the ESP32 pin must be configured as an input with pull-up.