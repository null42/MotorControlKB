#include "table.h"

static uint32_t find_index(float *x, uint32_t n, float value)
{
    uint32_t low = 0;
    uint32_t high = n - 1;
    uint32_t mid;
    
    while (low < high)
    {
        mid = (low + high + 1) / 2;
        if (x[mid] <= value)
        {
            low = mid;
        }
        else
        {
            high = mid - 1;
        }
    }
    
    return low;
}

float table_1d_interp(const table_1d_t *table, float x)
{
    if (table->n == 0)
    {
        return 0.0f;
    }
    
    if (table->n == 1)
    {
        return table->y[0];
    }
    
    if (x <= table->x[0])
    {
        return table->y[0];
    }
    
    if (x >= table->x[table->n - 1])
    {
        return table->y[table->n - 1];
    }
    
    uint32_t i = find_index(table->x, table->n, x);
    
    float x0 = table->x[i];
    float x1 = table->x[i + 1];
    float y0 = table->y[i];
    float y1 = table->y[i + 1];
    
    if (x1 == x0)
    {
        return y0;
    }
    
    float t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}

float table_2d_interp(const table_2d_t *table, float x, float y)
{
    if (table->nx == 0 || table->ny == 0)
    {
        return 0.0f;
    }
    
    uint32_t ix = find_index(table->x, (uint32_t)table->nx, x);
    uint32_t iy = find_index(table->y, (uint32_t)table->ny, y);
    
    if (ix >= (uint32_t)(table->nx - 1))
    {
        ix = (uint32_t)(table->nx - 2);
    }
    if (iy >= (uint32_t)(table->ny - 1))
    {
        iy = (uint32_t)(table->ny - 2);
    }
    
    float x0 = table->x[ix];
    float x1 = table->x[ix + 1];
    float y0 = table->y[iy];
    float y1 = table->y[iy + 1];
    
    float z00 = table->z[iy * table->nx + ix];
    float z01 = table->z[iy * table->nx + ix + 1];
    float z10 = table->z[(iy + 1) * table->nx + ix];
    float z11 = table->z[(iy + 1) * table->nx + ix + 1];
    
    float tx = (x - x0) / (x1 - x0);
    float ty = (y - y0) / (y1 - y0);
    
    float z0 = z00 + tx * (z01 - z00);
    float z1 = z10 + tx * (z11 - z10);
    
    return z0 + ty * (z1 - z0);
}
