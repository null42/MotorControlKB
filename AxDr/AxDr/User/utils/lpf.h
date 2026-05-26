#ifndef __LPF_H__
#define __LPF_H__

#include <stdint.h>

typedef struct
{
    float in;
    float out;
    float coeff;
} lpf_t;

typedef struct
{
    uint32_t cnt;
    uint32_t num;
    float in;
    float out;
    float sum;
} mean_t;

typedef struct
{
    uint32_t cnt;
    uint32_t num;
    float in;
    float out;
    float max_val;
} max_filter_t;

void lpf_init(lpf_t *lpf, float init_val, float cutoff_freq, float sample_freq);
float lpf_calc(lpf_t *lpf, float input);

void mean_init(mean_t *mean, uint32_t num);
void mean_calc(mean_t *mean, float input);

void max_filter_init(max_filter_t *max, uint32_t num);
void max_filter_calc(max_filter_t *max, float input);

#endif
