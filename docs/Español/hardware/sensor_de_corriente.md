# Transductor de corriente trifásico

El transductor de corriente es el elemento responsable de captar la corriente eléctrica presente en el sistema electromecánico y de transformarla en una señal adecuada para la etapa de adquisición del microcontrolador, mediante el acondicionamiento correspondiente. Existen múltiples variantes de transductores de corriente, basadas en principios como el efecto Hall, la resistencia shunt o el transformador de corriente, y disponibles con salidas analógicas o digitales, de acuerdo con las necesidades de interfaz y las especificaciones del sistema.

## Sensor SCT-013-000

El SCT-013-000  es un transductor de corriente alterna no invasivo de núcleo partido que emplea un transformador de corriente para medir AC con aislamiento galvánico. En su variante (100 A : 50 mA) entrega una señal de corriente proporcional que requiere una resistencia de carga (burden) externa para convertirla en una tensión apta para lectura por microcontroladores.

<p style="text-align:center;"> <img src="SCT-013-000.png" style="width:45%;"> </p>

### Características
- Núcleo partido tipo clip, instalación sin interrumpir el circuito y con aislamiento galvánico seguro.
- Relación de transformación 100 A : 50 mA (modelo SCT-013-000, sin resistencia de carga integrada).
- Requiere resistencia burden externa dimensionada según el rango del ADC y el nivel de corriente a medir.
- Salida analógica proporcional a la corriente primaria; precisa sesgo de media (bias) para lectura AC con ADC de 3.3 V o 5 V.
- Frecuencia nominal 50/60 Hz; buena linealidad en aplicaciones residenciales e industriales.

## Acondicionamiento de señal

El SCT-013-000 entrega una señal de corriente alterna en su secundario proporcional a la corriente que circula por el primario, al operar como un transformador de corriente. En esta versión (100 A : 50 mA), la relación es 2000:1, de modo que, por ejemplo, si por el primario circulan 50 A RMS, por el secundario circularán 25 mA RMS. Para que el microcontrolador pueda registrar esta magnitud, es necesario convertir la corriente secundaria en una tensión mediante una resistencia de carga (**burden**) conectada en derivación entre los terminales del sensor. Por ley de Ohm, la tensión resultante es :  $$V = I_{sec} × R_{burden}$$
Para dimensionar la resistencia de **burden** debe considerarse el nivel de tensión objetivo en la salida del sensor, de modo que no se exceda el rango de entrada del convertidor analógico-digital (ADC) del [ESP32](controlador_central), típicamente 0 a 3.3 V. En consecuencia, la tensión generada por la corriente secundaria a través de la **burden** debe ser, como máximo, igual o inferior al límite del ADC. En aplicaciones de corriente alterna, es habitual añadir un sesgo de corriente continua (bias) a mitad de escala del ADC para centrar la señal dentro del rango 0–3.3 V y evitar recortes.

![[señal_acodiciona.drawio.svg]]

Se ha considerado un rango de tensión de 2.828 V, el cual se ubica cómodamente dentro del rango del ADC. Al no operar cerca de sus límites, se dispone de un margen de seguridad suficiente que reduce el riesgo de saturación y recorte de la señal. 

Suponiendo un voltaje objetivo de

$$
2.828 V_{pico a pico} = 1.414 V_{pico}
$$
Con una corriente máxima de
$$
I_{RMS(sec)} = 50 mA
$$
Se despeja R
$$
I_{RMS(sec)} × R_{burden} = V_{RMS}   
$$
$$
50mA × R_{burden} =  \frac{ V_{pico} }{\sqrt{2}} 
$$
$$
50mA × R_{burden} =  \frac{ 1.414 V_{pico} }{\sqrt{2}} = 1 V_{RMS}
$$
$$
R_{burden} \approx 20 \Omega
$$

Como resultado, la resistencia de carga (burden) necesaria es de aproximadamente 20 Ω. Aún es necesario añadir una tensión de polarización (bias) a la señal, ya que de lo contrario se perdería la mitad de la amplitud debido a la incapacidad del microcontrolador para medir tensiones negativas. Para aprovechar el mayor rango dinámico posible, se fija dicha polarización en la mitad del rango de medida, para lo cual se emplea un divisor de tensión, la siguiente ecuación representa el voltaje de salida en función del voltaje de entrada al divisor de tensión.

<p style="text-align:center;"> <img src="divisor.drawio.svg" style="width:50%;"> </p>

$$
V_{out} = V_{in} \frac{ R_2 }{R_1 + R_2}
$$
Si R2 = R1
$$
V_{out} = V_{in} \frac{ R_2 }{2 R_2}
$$
$$
V_{out} =  \frac{ V_{in} }{2}
$$

Para limitar la corriente, se selecciona R = 10 kΩ. Con este valor, se propone el siguiente circuito de acondicionamiento para el sensor de corriente alterna SCT-013-000 (100 A / 50 mA).


![[Diagrama_sensor_corriente.drawio.svg]]

Se agrega un capacitor de 100 nF como filtro antialias y otro de 10 µF para mantener estable la tensión de bias. Al igual que el [sensor de voltaje](sensor_de_voltaje), el sensor y su circuito de acondicionamiento solo pueden medir una señal monofásica. Para compensarlo y obtener mediciones bifásicas o trifásicas, se propone el uso de tres sensores con sus respectivos acondicionamientos; de esta forma, cada uno se encarga de una línea de voltaje y se obtienen mediciones completas.