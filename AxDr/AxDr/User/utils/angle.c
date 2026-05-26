#include "angle.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif

#define SINE_TABLE_SIZE 512

static const float sin_table[SINE_TABLE_SIZE + 1] = {
    0.0000f, 0.0123f, 0.0245f, 0.0368f, 0.0491f, 0.0614f, 0.0737f, 0.0859f,
    0.0982f, 0.1104f, 0.1227f, 0.1349f, 0.1471f, 0.1593f, 0.1715f, 0.1837f,
    0.1959f, 0.2081f, 0.2203f, 0.2324f, 0.2446f, 0.2567f, 0.2688f, 0.2809f,
    0.2930f, 0.3050f, 0.3171f, 0.3291f, 0.3411f, 0.3530f, 0.3650f, 0.3769f,
    0.3888f, 0.4007f, 0.4125f, 0.4243f, 0.4361f, 0.4478f, 0.4595f, 0.4712f,
    0.4829f, 0.4945f, 0.5061f, 0.5177f, 0.5292f, 0.5407f, 0.5522f, 0.5636f,
    0.5750f, 0.5864f, 0.5977f, 0.6090f, 0.6203f, 0.6315f, 0.6427f, 0.6539f,
    0.6650f, 0.6761f, 0.6872f, 0.6982f, 0.7092f, 0.7201f, 0.7310f, 0.7419f,
    0.7527f, 0.7635f, 0.7742f, 0.7849f, 0.7955f, 0.8061f, 0.8167f, 0.8272f,
    0.8377f, 0.8481f, 0.8585f, 0.8688f, 0.8791f, 0.8894f, 0.8996f, 0.9097f,
    0.9198f, 0.9298f, 0.9398f, 0.9497f, 0.9596f, 0.9694f, 0.9792f, 0.9889f,
    0.9986f, 1.0000f
};

void angle_normalize(float *angle)
{
    while (*angle >= 1.0f)
    {
        *angle -= 1.0f;
    }
    while (*angle < 0.0f)
    {
        *angle += 1.0f;
    }
}

void sincos_calc(trig_t *trig)
{
    float input = trig->angle;
    angle_normalize(&input);
    
    float findex = input * (float)SINE_TABLE_SIZE;
    uint32_t index = (uint32_t)findex;
    float fract = findex - (float)index;
    
    float m = sin_table[index];
    float n = sin_table[index + 1];
    trig->sin_val = m + fract * (n - m);
    
    float cos_input = input + 0.25f;
    if (cos_input >= 1.0f)
    {
        cos_input -= 1.0f;
    }
    
    findex = cos_input * (float)SINE_TABLE_SIZE;
    index = (uint32_t)findex;
    fract = findex - (float)index;
    
    m = sin_table[index];
    n = sin_table[index + 1];
    trig->cos_val = m + fract * (n - m);
}

float angle_diff(float angle1, float angle2)
{
    float diff = angle1 - angle2;
    
    if (diff > 0.5f)
    {
        diff -= 1.0f;
    }
    else if (diff < -0.5f)
    {
        diff += 1.0f;
    }
    
    return diff;
}
