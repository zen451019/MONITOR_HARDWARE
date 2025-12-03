@file lorawan.md
@defgroup group_lorawan LoRaWAN

@section group_lorawan_overview Resumen
Este módulo gestiona la transmisión LoRaWAN usando LMIC:
- Inicializa la pila LMIC y la HAL del dispositivo (TTGO LORA 32 V2.1).
- Administra la construcción y envío de frames desde `queueFragmentos`.
- Supervisa eventos LMIC (EV_TXCOMPLETE, join, duty cycle) y sincroniza mediante `semaforoEnvioCompleto`.

@section Estructura_y_componentes Estructura y componentes

- Configuración
  - Parámetros de red (OTAA/ABP), canales y región.
  - HAL y pinmap SPI/Radio.

- Transporte hacia LMIC
  - Fragmento: bloque binario listo para transmisión (producido por agregación en Data Format).

- Tareas/Callbacks
  - `tareaLoRa`: toma fragmentos de `queueFragmentos` y llama `LMIC_setTxData2`.
  - `tareaRunLoop`: ejecuta `os_runloop_once` para procesar eventos LMIC.
  - `onEvent()`: callback de eventos (EV_TXCOMPLETE, EV_JOINED, EV_LINK_DEAD, etc.).

@section group_lorawan_api API principal
- Entrada
  - `queueFragmentos` (payload listo para TX).
- Salida
  - Señalización por `semaforoEnvioCompleto` tras EV_TXCOMPLETE.
- Funciones/Callbacks
  - `LMIC_setTxData2(port, data, len, confirmed)`.
  - `onEvent(ev)` manejo de estados y errores.
  - `tareaLoRa`, `tareaRunLoop`.
- Configuración
  - `LMIC_reset()`, `LMIC_setTxData2()`, `LMIC_setClockError()`, seteo de canales/región.

@section group_lorawan_usage Uso
Resumen del flujo:

1) Arranque LoRaWAN
   - Inicializa HAL/SPI/Radio y resetea LMIC (`LMIC_reset()`).
   - Configura OTAA/ABP, región y canales.
   - Crea `tareaRunLoop` y `tareaLoRa`.

2) Envío de datos
   - `tareaLoRa` bloquea en `queueFragmentos`, arma el frame y llama `LMIC_setTxData2`.
   - Evita colisiones verificando `LMIC.opmode & OP_TXRXPEND`.

3) Eventos LMIC
   - `onEvent(EV_TXCOMPLETE)`: libera `semaforoEnvioCompleto` y opcionalmente reprograma próximo envío.
   - `onEvent(EV_JOINED/EV_JOINING)`: controla el estado de join.
   - Maneja duty cycle y reintentos según la región.

Buenas prácticas (LoRaWAN)
- Verificar `OP_TXRXPEND` antes de enviar para no romper duty cycle.
- Usar puertos de aplicación coherentes y tamaños de payload dentro del límite regional.
- Minimizar trabajo dentro de `onEvent`; delegar a tareas cuando sea posible.
- Ajustar `LMIC_setClockError()` si el cristal tiene desviación notable.
- Asegurar reintento de join con backoff y registrar estados para diagnóstico.