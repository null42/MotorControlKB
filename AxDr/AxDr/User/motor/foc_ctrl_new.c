#include "foc_ctrl_new.h"
#include "observer_config.h"
#include "encoder.h"
#include <math.h>

observer_t g_observer;
startup_t g_startup;
angle_switch_t g_angle_switch;

static float theta_encoder = 0.0f;
static float theta_observer = 0.0f;
static float speed_encoder = 0.0f;
static float speed_observer = 0.0f;

void foc_ctrl_init(void)
{
    observer_motor_para_t motor_para = {
        .Rs = OBS_MOTOR_RS,
        .Ld = OBS_MOTOR_LD,
        .Lq = OBS_MOTOR_LQ,
        .flux = OBS_MOTOR_FLUX,
        .Ts = OBS_MOTOR_TS
    };
    
    observer_init(&g_observer, OBSERVER_TYPE, &motor_para);
    
    startup_init(&g_startup, OBS_MOTOR_TS);
    startup_config(&g_startup,
                   STARTUP_ALIGN_CURRENT, STARTUP_ALIGN_TIME_MS,
                   STARTUP_START_FREQ, STARTUP_END_FREQ, STARTUP_RAMP_RATE,
                   STARTUP_SWITCH_TIME_MS);
    
    g_angle_switch.current_src = ANGLE_SRC_ENCODER;
    g_angle_switch.target_src = ANGLE_SRC_ENCODER;
    g_angle_switch.transition_weight = 0.0f;
    g_angle_switch.transition_active = 0;
}

void angle_source_switch(angle_src_t target_src)
{
    if (g_angle_switch.current_src != target_src)
    {
        g_angle_switch.target_src = target_src;
        g_angle_switch.transition_active = 1;
        g_angle_switch.transition_weight = 0.0f;
    }
}

void angle_fusion_update(void)
{
    if (g_angle_switch.transition_active)
    {
        g_angle_switch.transition_weight += 0.005f;
        
        if (g_angle_switch.transition_weight >= 1.0f)
        {
            g_angle_switch.transition_weight = 1.0f;
            g_angle_switch.current_src = g_angle_switch.target_src;
            g_angle_switch.transition_active = 0;
        }
    }
    
    switch (g_angle_switch.current_src)
    {
    case ANGLE_SRC_ENCODER:
        foc.theta = theta_encoder;
        break;
        
    case ANGLE_SRC_OBSERVER:
        foc.theta = theta_observer;
        break;
        
    case ANGLE_SRC_HYBRID:
        foc.theta = theta_encoder * (1.0f - g_angle_switch.transition_weight) +
                    theta_observer * g_angle_switch.transition_weight;
        break;
    }
}

void foc_ctrl_update(void)
{
    MT6816_Calc_Elec_Angle(mt.para.pairs);
    theta_encoder = mt6816.elec_angle;
    speed_encoder = mt6816.speed;
    
    float i_alpha = foc.i_a;
    float i_beta = (foc.i_b - foc.i_c) * 0.57735026919f;
    float v_alpha = foc.v_alpha;
    float v_beta = foc.v_beta;
    float dir = 1.0f;
    
    observer_update(&g_observer, i_alpha, i_beta, v_alpha, v_beta, dir);
    
    theta_observer = observer_get_angle(&g_observer) * 6.28318530718f;
    speed_observer = observer_get_speed(&g_observer);
    
    if (mt.ctrl_mode == dragif_mode || mt.ctrl_mode == dragvf_mode)
    {
        startup_update(&g_startup, &g_observer);
        
        if (startup_is_done(&g_startup))
        {
            angle_source_switch(ANGLE_SRC_OBSERVER);
        }
        else
        {
            foc.theta = startup_get_angle(&g_startup) * 6.28318530718f;
        }
    }
    else if (mt.ctrl_mode == spd_curr || mt.ctrl_mode == curr_mode)
    {
        angle_fusion_update();
    }
    
    if (observer_is_converged(&g_observer))
    {
        observer_adapt(&g_observer, speed_observer, foc.i_q);
    }
}
