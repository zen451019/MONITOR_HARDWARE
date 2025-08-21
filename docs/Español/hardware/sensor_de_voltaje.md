# Transductor del voltaje trifásico

El transductor de voltaje es el dispositivo encargado de medir la señal de tensión generada por el sistema electromecánico y convertirla en una señal compatible con la etapa de adquisición del microcontrolador, mediante el correspondiente acondicionamiento de señal. Existen diversos tipos de transductores de tensión, con salidas analógicas o digitales, según su principio de medición y los requisitos de interfaz del sistema.


<p style="text-align:center;"> <img src="trasnductor_voltaje.png" style="width:40%;"> </p>

## Sensor zmpt101b

El ZMPT101B es un transductor de tensión alterna monofásica que utiliza un microtransformador para medir voltajes AC con aislamiento galvánico. El módulo integra un amplificador y un potenciómetro para calibrar la ganancia, entregando una señal analógica proporcional apta para lectura con microcontroladores.
<p style="text-align:center;"> <img src="ZMPT101B.png" style="width:40%;"> </p>
### Características

* Aislamiento galvánico mediante microtransformador, seguro para mediciones de red.
* Módulo con etapa de acondicionamiento: amplificador operacional y trimmer de ajuste.
* Salida analógica proporcional, compatible con ADC de 3.3 V y 5 V.
* Frecuencia nominal: 50/60 Hz; buena linealidad en aplicaciones residenciales e industriales.

## Acondicionamiento de señal

El módulo ZMPT101B entrega una señal analógica proporcional al voltaje AC de entrada. Cuando se alimenta con 3,3 V, la salida es una señal alterna superpuesta a un nivel de referencia de 1,65 V, de modo que las variaciones alrededor de ese punto son proporcionales a la tensión medida y pueden muestrearse con un conversor análogo digital del [ESP32](controlador_central).

<p style="text-align:center;"> <img src="acondicionamiento_V_1.drawio.svg" style="width:70%;"> </p>

Gracias a este módulo es posible registrar una señal representativa de la tensión en corriente alterna con poco más que una conexión directa entre el módulo ZMPT101B y el microcontrolador. De este modo, el acondicionamiento de la señal se reduce a la inclusión de un condensador destinado a filtrar componentes de alta frecuencia y a mejorar el desempeño del convertidor analógico-digital (ADC) del microcontrolador durante el muestreo.


<p style="text-align:center;"> <img src="acondicionamiento_V_3.drawio.svg" style="width:70%;"> </p>

Dado que el sensor ZMPT101B es un dispositivo monofásico, su aplicación en la medición de tensión trifásica se encuentra limitada. Para compensar esta restricción, se propone emplear tres sensores ZMPT101B, cada uno dedicado a una fase del sistema, obteniendo así una medición trifásica efectiva.
