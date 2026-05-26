#ifndef __STARTUP_H__
#define __STARTUP_H__

#include <stdint.h>
#include "observer.h"

typedef enum
{
    STARTUP_STATE_IDLE = 0,
    STARTUP_STATE_ALIGN = 1,
    STARTUP_STATE_RAMP = 2,
    STARTUP_STATE_SWITCH = 3,
    STARTUP_STATE_DONE = 4,
    STARTUP_STATE_ERROR = 5
} startup_state_t;

typedef struct
{
    float align_current;
    float align_time_ms;
    uint32_t align_cnt;
    uint32_t align_cnt_max;
} startup_align_t;

typedef struct
{
    float start_freq;
    float end_freq;
    float ramp_rate;
    float current_freq;
    float current_angle;
    uint32_t ramp_cnt;
} startup_ramp_t;

typedef struct
{
    float switch_time_ms;
    uint32_t switch_cnt;
    uint32_t switch_cnt_max;
    float blend_weight;
} startup_switch_t;

typedef struct
{
    startup_state_t state;
    startup_align_t align;
    startup_ramp_t ramp;
    startup_switch_t sw;
    
    float Ts;
    float target_speed;
    
    uint8_t enable;
    uint8_t observer_ready;
    
    float theta_openloop;
    float speed_openloop;
    
} startup_t;

void startup_init(startup_t *startup, float sample_time);
void startup_config(startup_t *startup, 
                    float align_current, float align_time_ms,
                    float start_freq, float end_freq, float ramp_rate,
                    float switch_time_ms);
void startup_start(startup_t *startup, float target_speed);
void startup_update(startup_t *startup, observer_t *obs);
void startup_stop(startup_t *startup);

startup_state_t startup_get_state(startup_t *startup);
float startup_get_angle(startup_t *startup);
float startup_get_speed(startup_t *startup);
uint8_t startup_is_done(startup_t *startup);

#endif
