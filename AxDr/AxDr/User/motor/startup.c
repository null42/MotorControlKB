#include "startup.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif

void startup_init(startup_t *startup, float sample_time)
{
    startup->state = STARTUP_STATE_IDLE;
    startup->Ts = sample_time;
    startup->enable = 0;
    startup->observer_ready = 0;
    startup->theta_openloop = 0.0f;
    startup->speed_openloop = 0.0f;
    
    startup->align.align_current = 0.5f;
    startup->align.align_time_ms = 500.0f;
    startup->align.align_cnt = 0;
    startup->align.align_cnt_max = 0;
    
    startup->ramp.start_freq = 5.0f;
    startup->ramp.end_freq = 50.0f;
    startup->ramp.ramp_rate = 10.0f;
    startup->ramp.current_freq = 0.0f;
    startup->ramp.current_angle = 0.0f;
    startup->ramp.ramp_cnt = 0;
    
    startup->sw.switch_time_ms = 100.0f;
    startup->sw.switch_cnt = 0;
    startup->sw.switch_cnt_max = 0;
    startup->sw.blend_weight = 0.0f;
}

void startup_config(startup_t *startup,
                    float align_current, float align_time_ms,
                    float start_freq, float end_freq, float ramp_rate,
                    float switch_time_ms)
{
    startup->align.align_current = align_current;
    startup->align.align_time_ms = align_time_ms;
    startup->align.align_cnt_max = (uint32_t)(align_time_ms / 1000.0f / startup->Ts);
    
    startup->ramp.start_freq = start_freq;
    startup->ramp.end_freq = end_freq;
    startup->ramp.ramp_rate = ramp_rate;
    
    startup->sw.switch_time_ms = switch_time_ms;
    startup->sw.switch_cnt_max = (uint32_t)(switch_time_ms / 1000.0f / startup->Ts);
}

void startup_start(startup_t *startup, float target_speed)
{
    startup->target_speed = target_speed;
    startup->enable = 1;
    startup->state = STARTUP_STATE_ALIGN;
    startup->align.align_cnt = 0;
    startup->ramp.current_freq = startup->ramp.start_freq;
    startup->ramp.current_angle = 0.0f;
    startup->ramp.ramp_cnt = 0;
    startup->sw.switch_cnt = 0;
    startup->sw.blend_weight = 0.0f;
    startup->theta_openloop = 0.0f;
    startup->speed_openloop = 0.0f;
}

void startup_update(startup_t *startup, observer_t *obs)
{
    if (!startup->enable)
    {
        return;
    }
    
    switch (startup->state)
    {
    case STARTUP_STATE_ALIGN:
        startup->align.align_cnt++;
        startup->theta_openloop = 0.0f;
        startup->speed_openloop = 0.0f;
        
        if (startup->align.align_cnt >= startup->align.align_cnt_max)
        {
            startup->state = STARTUP_STATE_RAMP;
            startup->ramp.current_freq = startup->ramp.start_freq;
        }
        break;
        
    case STARTUP_STATE_RAMP:
        startup->ramp.current_freq += startup->ramp.ramp_rate * startup->Ts;
        
        if (startup->ramp.current_freq >= startup->ramp.end_freq)
        {
            startup->ramp.current_freq = startup->ramp.end_freq;
        }
        
        startup->speed_openloop = startup->ramp.current_freq;
        startup->theta_openloop += startup->ramp.current_freq * startup->Ts;
        
        while (startup->theta_openloop >= 1.0f)
        {
            startup->theta_openloop -= 1.0f;
        }
        
        startup->ramp.ramp_cnt++;
        
        float freq_threshold = startup->ramp.start_freq + 
                              (startup->ramp.end_freq - startup->ramp.start_freq) * 0.5f;
        
        if (startup->ramp.current_freq >= freq_threshold)
        {
            if (observer_is_converged(obs))
            {
                startup->state = STARTUP_STATE_SWITCH;
                startup->sw.switch_cnt = 0;
                startup->sw.blend_weight = 0.0f;
            }
        }
        
        if (startup->ramp.ramp_cnt > 10000)
        {
            startup->state = STARTUP_STATE_ERROR;
        }
        break;
        
    case STARTUP_STATE_SWITCH:
        startup->sw.switch_cnt++;
        startup->sw.blend_weight = (float)startup->sw.switch_cnt / 
                                   (float)startup->sw.switch_cnt_max;
        
        if (startup->sw.blend_weight > 1.0f)
        {
            startup->sw.blend_weight = 1.0f;
        }
        
        float theta_obs = observer_get_angle(obs);
        startup->theta_openloop = startup->theta_openloop * (1.0f - startup->sw.blend_weight) +
                                  theta_obs * startup->sw.blend_weight;
        
        startup->speed_openloop = startup->speed_openloop * (1.0f - startup->sw.blend_weight) +
                                  observer_get_speed(obs) * startup->sw.blend_weight;
        
        if (startup->sw.switch_cnt >= startup->sw.switch_cnt_max)
        {
            startup->state = STARTUP_STATE_DONE;
        }
        break;
        
    case STARTUP_STATE_DONE:
        startup->theta_openloop = observer_get_angle(obs);
        startup->speed_openloop = observer_get_speed(obs);
        break;
        
    case STARTUP_STATE_ERROR:
        break;
        
    default:
        break;
    }
}

void startup_stop(startup_t *startup)
{
    startup->enable = 0;
    startup->state = STARTUP_STATE_IDLE;
}

startup_state_t startup_get_state(startup_t *startup)
{
    return startup->state;
}

float startup_get_angle(startup_t *startup)
{
    return startup->theta_openloop;
}

float startup_get_speed(startup_t *startup)
{
    return startup->speed_openloop;
}

uint8_t startup_is_done(startup_t *startup)
{
    return (startup->state == STARTUP_STATE_DONE);
}
