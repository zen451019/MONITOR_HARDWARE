## Nivel de batería

La batería de un sistema electromecánico cumple el papel de fuente de energía primaria o de respaldo, asegurando el arranque, la continuidad de operación ante cortes de suministro y la estabilidad del sistema. Además, amortigua transitorios, suministra corrientes de pico a actuadores, y mantiene alimentados los circuitos de control y sensado para un funcionamiento confiable.

## Acondicionar señal

La señal de una batería es de corriente continua (DC) y, debido a su nivel de voltaje, el [ESP32](controlador_central) no puede monitorearla directamente. Por ello, se implementa un divisor de voltaje que reduce la señal a un rango seguro para el microcontrolador, manteniendo su representatividad respecto al nivel real de la batería.
<p style="text-align:center;"> <img src="divisor.drawio.svg" style="width:50%;"> </p>

$$
V_{out} = V_{in} \frac{ R_2 }{R_1 + R_2}
$$
Siendo el Vin máximo 3.3 Volts y el Vout máximo Y un Vout máximo de 15 volts para tener un rango seguro y definiendo R2 = 3.3 kΩ se halla R1


$$
R_1 = R_2 \frac{ V_{in}-V_{out}}{V_{out}}
$$
$$
R_1 = 3.3 kΩ \frac{ 15V-3.3V}{3.3V} = 11700Ω
$$
Aproximando el valor de R1 a una resistencia comercial tenemos que R1 = 12 kΩ, con esto el Vin máximo queda

$$
V_{in} = 3.3V \frac{ 12 kΩ + 3.3 kΩ }{3.3 kΩ} = 15.3 V
$$

Este divisor de voltaje adapta el rango de 0 V a 15.3 V a un rango seguro de 0 V a 3.3 V para la lectura del microcontrolador. Posteriormente, el valor real se escala mediante software.

<p style="text-align:center;"> <img src="divisor_bateria.drawio.svg" style="width:50%;"> </p>

