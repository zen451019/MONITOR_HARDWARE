#pragma once

#include <stdint.h>

// Filtro biquad en forma directa II transpuesta
// Proporciona diseñadores de paso bajo y paso alto (RBJ cookbook) y proceso por muestra.

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} Biquad;

// Reinicia los estados internos
void biquad_reset(Biquad* s);

// Diseña como paso bajo (LPF)
void biquad_set_lowpass(Biquad* s, float fs_hz, float f0_hz, float Q);

// Diseña como paso alto (HPF)
void biquad_set_highpass(Biquad* s, float fs_hz, float f0_hz, float Q);

// Procesa una muestra
float biquad_process(Biquad* s, float x);
