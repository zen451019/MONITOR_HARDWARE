# Alimentación del modulo

La alimentación del módulo debe ser confiable, estable e ininterrumpida. Como los datos se transmiten 24/7, es imprescindible evitar apagados ante cualquier eventualidad. Para ello, se requiere un sistema de alimentación que contemple posibles fallas y, en lo posible, las mitigue o contrarreste.
## Requerimientos

- **Punto de Alimentación:** El principal punto de alimentación es la batería del sistema electromecánico, por lo que el módulo debe ser capaz de tomar energía de esta fuente.
- **Alimentación ininterrumpida:** El módulo debe seguir operando aun si se interrumpe la fuente principal, por lo que debe incluir un sistema que garantice un suministro continuo de energía.
- **Nivel de voltaje estable:** Tanto el [ESP32](controlador_central) como los demás componentes del módulo requieren una alimentación de 3.3 Voltios. Por lo tanto, el sistema de alimentación debe garantizar una fuente estable para el módulo.

Para cumplir estos requisitos, se seleccionan dos módulos para conformar el sistema de alimentación.

## Conversor reductor DC-DC

- Entrada: 12–24 VDC. Salida: 5 VDC.
- Potencia máxima: 50 W. Eficiencia: 95%.
- Protecciones: sobrecorriente, sobretensión, sobretemperatura, apagado y cortocircuito.
- Rango de temperatura: −40 a 85 °C

<p style="text-align:center;"> <img src="DC-DC.png" style="width:50%;"> </p>


###  Módulo UPS de batería 18650

- Entrada: 5 VDC. Salida: 3.3 VDC (800mA), 3.7VDC.
- Con interruptor on/off.
- Botón de activación de protección contra baja tensión y sobredescarga.
- Límite máximo de corriente de protección de la batería: 2A.

<p style="text-align:center;"> <img src="UPS.png" style="width:40%;"> </p>

Con este arreglo, el convertidor DC‑DC se conecta a la batería del sistema electromecánico y entrega 5 V fijos. Ese voltaje alimenta el módulo UPS, que carga su celda 18650 y, mediante su regulador de 3.3 V, suministra una tensión estable al módulo.

<p style="text-align:center;"> <img src="alimetacion.drawio.svg" style="width:90%;"> </p>

De este modo, el sistema de alimentación cuenta con protecciones ante eventualidades de la batería (caídas de tensión, cortocircuitos, etc.). Además, si se interrumpe la alimentación, la UPS entra en acción de forma instantánea, garantizando la continuidad y evitando que el módulo se apague.