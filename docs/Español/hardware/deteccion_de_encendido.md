# Detección De Encendido

El modulo debe contar con un mecanismo para detectar el encendido y apagado del sistema electromecánico, ya que ciertas variables dependen directamente de que esté activo para procesarse correctamente. Para ello, se diseña una entrada digital que permita al microcontrolador captar señales externas.

La señal a monitorear será la presencia o ausencia de voltaje DC, proveniente de la batería, el controlador o cualquier otro dispositivo del sistema electromecánico. Esto ofrece flexibilidad tanto en hardware (usando un relé, el propio controlador, etc., para generar la señal de encendido) como en software para codificar la lógica.

## Acondicionamiento de señal

Para que el microcontrolador capte estas señales de forma segura, dadas las diferencias de nivel de voltaje, se utiliza un level shifter con optoacoplador, proporcionando adaptación de niveles y aislamiento galvánico.

<p style="text-align:center;"> <img src="level_shifter.drawio.svg" style="width:50%;"> </p>

Cada vez que llega una señal digital, el optoacoplador activa su diodo, lo que a su vez enciende el fototransistor. Esto permite que el pin del [ESP32](controlador_central) caiga a GND. Para una detección correcta, el pin del ESP32 debe configurarse como una entrada con pull-up.