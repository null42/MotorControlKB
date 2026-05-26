#ifndef __TABLE_H__
#define __TABLE_H__

#include <stdint.h>

typedef struct
{
    float *x;
    float *y;
    uint32_t n;
} table_1d_t;

typedef struct
{
    float *x;
    float *y;
    float *z;
    int32_t nx;
    int32_t ny;
} table_2d_t;

float table_1d_interp(const table_1d_t *table, float x);
float table_2d_interp(const table_2d_t *table, float x, float y);

#endif
