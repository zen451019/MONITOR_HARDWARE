# Mejoras para Medición RMS Estable

## Problemas Identificados

### 1. **Interferencia de Red Eléctrica**
- Otros dispositivos conectados introducen ruido y armónicos
- Variaciones en la impedancia de la red
- Switching de fuentes de alimentación

### 2. **Limitaciones del Código Original**
- Frecuencia de muestreo baja (1kHz)
- Buffer pequeño (256 muestras)
- Falta de filtrado anti-aliasing
- Precisión limitada en cálculos

## Mejoras Implementadas en el Código

### 1. **Parámetros Mejorados**
- Frecuencia de muestreo: 1kHz → 2kHz
- Tamaño de buffer: 256 → 512 muestras
- Tiempo de captura: ~0.25s (más ciclos para promedio estable)

### 2. **Filtrado Digital**
- Filtro Butterworth pasa-bajos de 4to orden
- Frecuencia de corte: 500Hz
- Elimina ruido de alta frecuencia y aliasing

### 3. **Mayor Precisión**
- Cálculos intermedios en double
- Validación de muestras ADC
- Filtro EMA más conservador (α=0.1)

### 4. **Configuración ADC Mejorada**
- Resolución 12 bits
- Atenuación 11dB para rango completo
- Mejor aprovechamiento del rango dinámico

## Recomendaciones de Hardware

### 1. **Filtro Anti-Aliasing Analógico**
Agregar antes del ADC:
```
Señal → R=1kΩ → C=330nF → ADC
                    |
                   GND
```
Frecuencia de corte: ~480Hz

### 2. **Acondicionamiento de Señal**
- **Divisor de voltaje preciso** con resistencias de 1%
- **Amplificador operacional** para buffering
- **Protección contra sobrevoltaje**

### 3. **Aislamiento**
- **Transformador de aislamiento** para medición de voltaje AC
- **Sensor de corriente tipo pinza** para medición no invasiva
- **Optoacopladores** para aislamiento digital

### 4. **Alimentación Limpia**
- **Fuente lineal** en lugar de switching para el ESP32
- **Filtros LC** en la alimentación
- **Condensadores de desacoplo** cerca del ADC

### 5. **PCB y Cableado**
- **Plano de tierra sólido**
- **Separación analógica/digital**
- **Cable blindado** para señales de entrada
- **Ferrites** en cables de alimentación

## Calibración y Verificación

### 1. **Calibración de Ganancia**
```cpp
// Medir voltaje conocido y ajustar ganancia
float voltaje_referencia = 230.0f;  // Voltaje RMS conocido
float lectura_sistema = medicion_rms;
float factor_correccion = voltaje_referencia / lectura_sistema;
// Aplicar factor_correccion a la ganancia
```

### 2. **Verificación con Multímetro**
- Usar multímetro True RMS como referencia
- Comparar en diferentes condiciones de carga
- Documentar desviaciones y ajustar

### 3. **Pruebas de Estabilidad**
- Medición continua durante 24 horas
- Variación de temperatura
- Diferentes cargas en la red

## Configuración Recomendada

### Para Voltaje AC (230V):
```cpp
// Divisor 230V → 3.3V (ratio ~70:1)
{pin, 70.0f, ESP32AnalogRead(), buffer, 0.0f, 0, true, 0.0f}
```

### Para Corriente AC (con sensor 30A/1V):
```cpp
// Sensor 30A → 1V, luego a 3.3V ADC
{pin, 30.0f, ESP32AnalogRead(), buffer, 0.0f, 0, true, 0.0f}
```

## Monitoreo y Debug

El código incluye salida de debug cada 5 segundos:
```
Pin 36: Raw RMS=2.145, Filtered=2.138, Gain=386.0
```

Esto permite verificar:
- Valor RMS sin filtrar
- Valor RMS filtrado
- Ganancia aplicada

## Próximos Pasos

1. **Implementar filtro analógico** antes del ADC
2. **Calibrar ganancias** con voltímetro de referencia
3. **Agregar aislamiento** para seguridad
4. **Optimizar frecuencia de muestreo** según necesidades
5. **Implementar detección de armónicos** si es necesario