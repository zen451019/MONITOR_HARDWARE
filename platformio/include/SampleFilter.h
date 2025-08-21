#pragma once

// FIR de 67 taps para filtrar antes del cálculo RMS
#define SAMPLEFILTER_TAP_NUM 67

typedef struct {
  float history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, float input);
float SampleFilter_get(SampleFilter* f);
