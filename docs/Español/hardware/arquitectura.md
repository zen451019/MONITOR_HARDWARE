# Arquitectura del Sistema

El sistema se compone de un controlador central responsable de la adquisición, procesamiento y transmisión de datos, apoyado por múltiples transductores y módulos de acondicionamiento que adaptan las distintas señales para su correcta lectura e interpretación por el controlador.

![[Arquitecruta_Sistema.drawio.svg]]

Las señales crudas pasan por acondicionamiento, son digitalizadas por el controlador, procesadas y clasificadas, y finalmente serializadas en vectores de datos para su envío por LoRaWAN. El gateway recibe y reenvía los datos a la plataforma de backend donde se almacenan y explotan.