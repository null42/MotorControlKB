#ifndef __ANGLE_H__
#define __ANGLE_H__

#include <stdint.h>

typedef struct
{
    float angle;
    float sin_val;
    float cos_val;
} trig_t;

void angle_normalize(float *angle);
void sincos_calc(trig_t *trig);
float angle_diff(float angle1, float angle2);

#endif
