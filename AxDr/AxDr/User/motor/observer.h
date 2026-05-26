#ifndef __OBSERVER_H__
#define __OBSERVER_H__

#include <stdint.h>
#include "lpf.h"
#include "angle.h"
#include "table.h"
#include "pid.h"

typedef enum
{
    OBS_TYPE_NONE = 0,
    OBS_TYPE_SMO = 1,
    OBS_TYPE_FLUX = 2,
    OBS_TYPE_EMF = 3
} observer_type_t;

typedef enum
{
    OBS_STATE_IDLE = 0,
    OBS_STATE_INIT = 1,
    OBS_STATE_RUNNING = 2,
    OBS_STATE_CONVERGED = 3,
    OBS_STATE_ERROR = 4
} observer_state_t;

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float Ls;
    float flux;
    float Ts;
} observer_motor_para_t;

typedef struct
{
    lpf_t lpf_i_alpha_err;
    lpf_t lpf_i_beta_err;
    
    float i_alpha_last;
    float i_beta_last;
    
    float emf_alpha;
    float emf_beta;
    
    float Rs;
    float Ls_over_Ts;
} obs_emf_t;

typedef struct
{
    pid_t pid_pll;
    lpf_t lpf_freq;
    trig_t trig;
    trig_t trig_comp;
    
    float a_alpha;
    float a_beta;
    float e_alpha;
    float e_beta;
    
    float h1_set;
    float Ts;
    float one_over_Ld;
    float Rs_over_Ld;
    float Ld_Lq_over_Ld;
    
    float h1_base;
    float pll_kp_base;
    float pll_ki_base;
    
    table_1d_t tab_h1_coeff;
    table_1d_t tab_kp_coeff;
    table_1d_t tab_ki_coeff;
    table_2d_t tab_angle_comp;
} obs_smo_t;

typedef struct
{
    pid_t pid_pll;
    lpf_t lpf_freq;
    trig_t trig;
    trig_t trig_comp;
    
    float x_alpha;
    float x_beta;
    
    float gamma_set;
    float Ts;
    float Ld;
    float Lq;
    float flux;
    
    float gamma_base;
    float pll_kp_base;
    float pll_ki_base;
    
    table_1d_t tab_gamma_coeff;
    table_1d_t tab_kp_coeff;
    table_1d_t tab_ki_coeff;
    table_2d_t tab_angle_comp;
} obs_flux_t;

typedef struct
{
    observer_type_t type;
    observer_state_t state;
    
    obs_emf_t emf;
    obs_smo_t smo;
    obs_flux_t flux;
    
    observer_motor_para_t motor_para;
    
    float theta_est;
    float speed_est;
    float sin_val;
    float cos_val;
    
    uint8_t converged;
    uint8_t valid;
    uint32_t converge_cnt;
    
} observer_t;

void observer_init(observer_t *obs, observer_type_t type, observer_motor_para_t *para);
void observer_set_para(observer_t *obs, float Rs, float Ld, float Lq, float flux, float Ts);
void observer_update(observer_t *obs, float i_alpha, float i_beta, float v_alpha, float v_beta, float dir);
void observer_adapt(observer_t *obs, float speed, float current);

float observer_get_angle(observer_t *obs);
float observer_get_speed(observer_t *obs);
float observer_get_sin(observer_t *obs);
float observer_get_cos(observer_t *obs);

uint8_t observer_is_converged(observer_t *obs);
uint8_t observer_is_valid(observer_t *obs);
void observer_reset(observer_t *obs);

#endif
