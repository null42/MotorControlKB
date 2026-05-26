#include "lpf.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void lpf_init(lpf_t *lpf, float init_val, float cutoff_freq, float sample_freq)
{
    lpf->out = init_val;
    float tau = 1.0f / (2.0f * M_PI * cutoff_freq);
    lpf->coeff = sample_freq / (sample_freq + tau);
}

float lpf_calc(lpf_t *lpf, float input)
{
    lpf->in = input;
    lpf->out += lpf->coeff * (lpf->in - lpf->out);
    return lpf->out;
}

void mean_init(mean_t *mean, uint32_t num)
{
    mean->cnt = 0;
    mean->num = num;
    mean->sum = 0.0f;
    mean->out = 0.0f;
}

void mean_calc(mean_t *mean, float input)
{
    mean->in = input;
    mean->sum += input;
    mean->cnt++;
    
    if (mean->cnt >= mean->num)
    {
        mean->out = mean->sum / (float)mean->num;
        mean->cnt = 0;
        mean->sum = 0.0f;
    }
}

void max_filter_init(max_filter_t *max, uint32_t num)
{
    max->cnt = 0;
    max->num = num;
    max->max_val = 0.0f;
    max->out = 0.0f;
}

void max_filter_calc(max_filter_t *max, float input)
{
    max->in = input;
    max->cnt++;
    
    if (input >= max->max_val)
    {
        max->max_val = input;
    }
    
    if (max->cnt >= max->num)
    {
        max->out = max->max_val;
        max->cnt = 0;
        max->max_val = 0.0f;
    }
}
