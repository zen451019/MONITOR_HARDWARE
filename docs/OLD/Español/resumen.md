# RESUMEN

## Objetivo 

Monitorear variables de operación en sistemas electromecánicos mediante hardware dedicado: medición precisa de voltaje de funcionamiento, corriente de operación, nivel de batería y otros parámetros críticos usando sensores y módulos de adquisición de datos, con transmisión inalámbrica de datos hacia un Gateway Lora WAN.
## Métricas clave

* Tensión del sistema: capacidad para medir con precisión el voltaje en sistemas monofásicos, bifásicos y trifásicos de corriente alterna, en un rango de 0 a 220 V AC.
* Corriente del sistema: capacidad para medir con precisión corriente alterna en sistemas monofásicos, bifásicos y trifásicos, en un rango de 0 a 100 A AC.
* Tensión de batería: capacidad para medir con precisión el voltaje de la batería del sistema electromecánico (cuando aplique) y estimar su estado de carga (SoC).
* Detección de eventos: Capacidad para detectar eventos de encendido y apagado en el sistema electromecánico. 
## Requerimientos

El módulo, además de medir corriente y tensión en sistemas trifásicos, debe transmitir la información de forma inalámbrica mediante tecnología LoRaWAN, enviándola a un gateway ubicado a pocos metros o a decenas de metros del módulo de adquisición. Asimismo, debe garantizar transmisión ininterrumpida, por lo que requiere capacidad de operación autónoma cuando la fuente de alimentación principal esté desactivada.

### [Placa de monitoreo de variables electromecánicas Monitor](./hardware/kicad/Placa_Monitor.md)