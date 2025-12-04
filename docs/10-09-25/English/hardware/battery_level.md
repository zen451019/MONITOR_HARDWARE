## Battery Level

The battery in an electromechanical system acts as a primary or backup power source, ensuring startup, continuous operation in the event of power outages, and system stability. It also dampens transients, supplies peak currents to actuators, and keeps control and sensing circuits powered for reliable operation.

## ## Signal Conditioning

The battery signal is direct current (DC) and, due to its voltage level, the [ESP32](./central_controller.md) cannot monitor it directly. Therefore, a voltage divider is implemented that reduces the signal to a safe range for the microcontroller, while maintaining its representativeness with respect to the actual battery level.

<p style="text-align:center;"> <img src="../../res/img/hardware/divisor.drawio.svg" style="width:50%;"> </p>

$$
V_{out} = V_{in} \frac{ R_2 }{R_1 + R_2}
$$
With Vin max at 3.3 volts and Vout max at 15 volts for a safe range, and defining R2 = 3.3 kΩ, we find R1.


$$
R_1 = R_2 \frac{ V_{in}-V_{out}}{V_{out}}
$$
$$
R_1 = 3.3 kΩ \frac{ 15V-3.3V}{3.3V} = 11700Ω
$$
Approximating the value of R1 to a commercial resistor, we have that R1 = 12 kΩ, with this the maximum Vin is

$$
V_{in} = 3.3V \frac{ 12 kΩ + 3.3 kΩ }{3.3 kΩ} = 15.3 V
$$

This voltage divider adapts the range from 0 V to 15.3 V to a safe range from 0 V to 3.3 V for microcontroller reading. The actual value is then scaled using software.

<p style="text-align:center;"> <img src="../../res/img/hardware/divisor_bateria.drawio.svg" style="width:50%;"> </p>

