#include "observer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void observer_emf_init(obs_emf_t *emf, observer_motor_para_t *para)
{
    lpf_init(&emf->lpf_i_alpha_err, 0.0f, 500.0f, 1.0f / para->Ts);
    lpf_init(&emf->lpf_i_beta_err, 0.0f, 500.0f, 1.0f / para->Ts);
    
    emf->i_alpha_last = 0.0f;
    emf->i_beta_last = 0.0f;
    emf->emf_alpha = 0.0f;
    emf->emf_beta = 0.0f;
    
    emf->Rs = para->Rs;
    emf->Ls_over_Ts = para->Ls / para->Ts;
}

static void observer_emf_update(obs_emf_t *emf, float i_alpha, float i_beta, float v_alpha, float v_beta)
{
    float di_alpha = i_alpha - emf->i_alpha_last;
    float di_beta = i_beta - emf->i_beta_last;
    
    lpf_calc(&emf->lpf_i_alpha_err, di_alpha);
    lpf_calc(&emf->lpf_i_beta_err, di_beta);
    
    emf->emf_alpha = v_alpha - emf->Rs * i_alpha - emf->Ls_over_Ts * emf->lpf_i_alpha_err.out;
    emf->emf_beta = v_beta - emf->Rs * i_beta - emf->Ls_over_Ts * emf->lpf_i_beta_err.out;
    
    emf->i_alpha_last = i_alpha;
    emf->i_beta_last = i_beta;
}

static void observer_smo_init(obs_smo_t *smo, observer_motor_para_t *para)
{
    pid_init(&smo->pid_pll, 0.0f, 0.0f, 0.0f, 1000.0f, -1000.0f);
    lpf_init(&smo->lpf_freq, 0.0f, 100.0f, 1.0f / para->Ts);
    
    smo->trig.angle = 0.0f;
    smo->trig.sin_val = 0.0f;
    smo->trig.cos_val = 1.0f;
    
    smo->a_alpha = 0.0f;
    smo->a_beta = 0.0f;
    smo->e_alpha = 0.0f;
    smo->e_beta = 0.0f;
    
    smo->Ts = para->Ts;
    smo->one_over_Ld = 1.0f / para->Ld;
    smo->Rs_over_Ld = para->Rs / para->Ld;
    smo->Ld_Lq_over_Ld = (para->Ld - para->Lq) / para->Ld;
}

static void observer_smo_adapt(obs_smo_t *smo, float speed, float current)
{
    float speed_abs = fabsf(speed);
    float current_abs = fabsf(current);
    
    smo->h1_set = smo->h1_base * table_1d_interp(&smo->tab_h1_coeff, speed_abs);
    smo->pid_pll.kp = smo->pll_kp_base * table_1d_interp(&smo->tab_kp_coeff, speed_abs);
    smo->pid_pll.ki = smo->pll_ki_base * table_1d_interp(&smo->tab_ki_coeff, speed_abs);
    
    smo->trig_comp.angle = table_2d_interp(&smo->tab_angle_comp, speed_abs, current_abs);
    angle_normalize(&smo->trig_comp.angle);
    sincos_calc(&smo->trig_comp);
}

static void observer_smo_update(obs_smo_t *smo, float i_alpha, float i_beta, float v_alpha, float v_beta, float dir)
{
    smo->a_alpha += smo->Ts * (
        -smo->Rs_over_Ld * smo->a_alpha
        - smo->Ld_Lq_over_Ld * smo->lpf_freq.out * smo->a_beta
        + smo->one_over_Ld * v_alpha
        - smo->one_over_Ld * smo->e_alpha
    );
    
    smo->a_beta += smo->Ts * (
        -smo->Rs_over_Ld * smo->a_beta
        + smo->Ld_Lq_over_Ld * smo->lpf_freq.out * smo->a_alpha
        + smo->one_over_Ld * v_beta
        - smo->one_over_Ld * smo->e_beta
    );
    
    smo->e_alpha = smo->h1_set * (smo->a_alpha - i_alpha);
    smo->e_beta = smo->h1_set * (smo->a_beta - i_beta);
    
    smo->pid_pll.ref = -dir * smo->e_alpha * smo->trig.cos_val;
    smo->pid_pll.fdb = dir * smo->e_beta * smo->trig.sin_val;
    pid_calc(&smo->pid_pll);
    
    lpf_calc(&smo->lpf_freq, smo->pid_pll.out);
    
    smo->trig.angle += smo->Ts * smo->lpf_freq.out;
    angle_normalize(&smo->trig.angle);
    sincos_calc(&smo->trig);
}

static void observer_flux_init(obs_flux_t *flux, observer_motor_para_t *para)
{
    pid_init(&flux->pid_pll, 0.0f, 0.0f, 0.0f, 1000.0f, -1000.0f);
    lpf_init(&flux->lpf_freq, 0.0f, 100.0f, 1.0f / para->Ts);
    
    flux->trig.angle = 0.0f;
    flux->trig.sin_val = 0.0f;
    flux->trig.cos_val = 1.0f;
    
    flux->x_alpha = 0.0f;
    flux->x_beta = 0.0f;
    
    flux->Ts = para->Ts;
    flux->Ld = para->Ld;
    flux->Lq = para->Lq;
    flux->flux = para->flux;
}

static void observer_flux_adapt(obs_flux_t *flux, float speed, float current)
{
    float speed_abs = fabsf(speed);
    float current_abs = fabsf(current);
    
    flux->gamma_set = flux->gamma_base * table_1d_interp(&flux->tab_gamma_coeff, speed_abs);
    flux->pid_pll.kp = flux->pll_kp_base * table_1d_interp(&flux->tab_kp_coeff, speed_abs);
    flux->pid_pll.ki = flux->pll_ki_base * table_1d_interp(&flux->tab_ki_coeff, speed_abs);
    
    flux->trig_comp.angle = table_2d_interp(&flux->tab_angle_comp, speed_abs, current_abs);
    angle_normalize(&flux->trig_comp.angle);
    sincos_calc(&flux->trig_comp);
}

static void observer_flux_update(obs_flux_t *flux, float i_alpha, float i_beta, float v_alpha, float v_beta)
{
    float id = i_alpha * flux->trig.cos_val + i_beta * flux->trig.sin_val;
    float iq = -i_alpha * flux->trig.sin_val + i_beta * flux->trig.cos_val;
    
    float psi_d = id * flux->Ld + flux->flux;
    float psi_q = iq * flux->Lq;
    
    float psi_alpha = psi_d * flux->trig.cos_val - psi_q * flux->trig.sin_val;
    float psi_beta = psi_d * flux->trig.sin_val + psi_q * flux->trig.cos_val;
    
    flux->x_alpha += flux->Ts * (v_alpha + flux->gamma_set * (psi_alpha - flux->x_alpha));
    flux->x_beta += flux->Ts * (v_beta + flux->gamma_set * (psi_beta - flux->x_beta));
    
    flux->pid_pll.ref = (flux->x_beta - flux->Lq * i_beta) * flux->trig.cos_val;
    flux->pid_pll.fdb = (flux->x_alpha - flux->Lq * i_alpha) * flux->trig.sin_val;
    pid_calc(&flux->pid_pll);
    
    lpf_calc(&flux->lpf_freq, flux->pid_pll.out);
    
    flux->trig.angle += flux->Ts * flux->lpf_freq.out;
    angle_normalize(&flux->trig.angle);
    sincos_calc(&flux->trig);
}

void observer_init(observer_t *obs, observer_type_t type, observer_motor_para_t *para)
{
    obs->type = type;
    obs->state = OBS_STATE_INIT;
    
    obs->motor_para = *para;
    
    obs->theta_est = 0.0f;
    obs->speed_est = 0.0f;
    obs->sin_val = 0.0f;
    obs->cos_val = 1.0f;
    
    obs->converged = 0;
    obs->valid = 0;
    obs->converge_cnt = 0;
    
    switch (type)
    {
    case OBS_TYPE_EMF:
        observer_emf_init(&obs->emf, para);
        break;
    case OBS_TYPE_SMO:
        observer_smo_init(&obs->smo, para);
        break;
    case OBS_TYPE_FLUX:
        observer_flux_init(&obs->flux, para);
        break;
    default:
        break;
    }
    
    obs->state = OBS_STATE_RUNNING;
}

void observer_set_para(observer_t *obs, float Rs, float Ld, float Lq, float flux, float Ts)
{
    obs->motor_para.Rs = Rs;
    obs->motor_para.Ld = Ld;
    obs->motor_para.Lq = Lq;
    obs->motor_para.Ls = (Ld + Lq) / 2.0f;
    obs->motor_para.flux = flux;
    obs->motor_para.Ts = Ts;
    
    if (obs->type == OBS_TYPE_EMF)
    {
        obs->emf.Rs = Rs;
        obs->emf.Ls_over_Ts = obs->motor_para.Ls / Ts;
    }
    else if (obs->type == OBS_TYPE_SMO)
    {
        obs->smo.Ts = Ts;
        obs->smo.one_over_Ld = 1.0f / Ld;
        obs->smo.Rs_over_Ld = Rs / Ld;
        obs->smo.Ld_Lq_over_Ld = (Ld - Lq) / Ld;
    }
    else if (obs->type == OBS_TYPE_FLUX)
    {
        obs->flux.Ts = Ts;
        obs->flux.Ld = Ld;
        obs->flux.Lq = Lq;
        obs->flux.flux = flux;
    }
}

void observer_update(observer_t *obs, float i_alpha, float i_beta, float v_alpha, float v_beta, float dir)
{
    if (obs->state != OBS_STATE_RUNNING && obs->state != OBS_STATE_CONVERGED)
    {
        return;
    }
    
    switch (obs->type)
    {
    case OBS_TYPE_EMF:
        observer_emf_update(&obs->emf, i_alpha, i_beta, v_alpha, v_beta);
        obs->theta_est = atan2f(-obs->emf.emf_alpha, obs->emf.emf_beta) / (2.0f * M_PI);
        if (obs->theta_est < 0.0f)
        {
            obs->theta_est += 1.0f;
        }
        obs->speed_est = sqrtf(obs->emf.emf_alpha * obs->emf.emf_alpha + obs->emf.emf_beta * obs->emf.emf_beta);
        break;
        
    case OBS_TYPE_SMO:
        observer_smo_update(&obs->smo, i_alpha, i_beta, v_alpha, v_beta, dir);
        obs->theta_est = obs->smo.trig.angle;
        obs->speed_est = obs->smo.lpf_freq.out;
        obs->sin_val = obs->smo.trig.sin_val;
        obs->cos_val = obs->smo.trig.cos_val;
        break;
        
    case OBS_TYPE_FLUX:
        observer_flux_update(&obs->flux, i_alpha, i_beta, v_alpha, v_beta);
        obs->theta_est = obs->flux.trig.angle;
        obs->speed_est = obs->flux.lpf_freq.out;
        obs->sin_val = obs->flux.trig.sin_val;
        obs->cos_val = obs->flux.trig.cos_val;
        break;
        
    default:
        break;
    }
    
    float emf_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (emf_mag > 0.5f)
    {
        obs->converge_cnt++;
        if (obs->converge_cnt > 100)
        {
            obs->converged = 1;
            obs->valid = 1;
            obs->state = OBS_STATE_CONVERGED;
        }
    }
    else
    {
        obs->converge_cnt = 0;
    }
}

void observer_adapt(observer_t *obs, float speed, float current)
{
    if (obs->type == OBS_TYPE_SMO)
    {
        observer_smo_adapt(&obs->smo, speed, current);
    }
    else if (obs->type == OBS_TYPE_FLUX)
    {
        observer_flux_adapt(&obs->flux, speed, current);
    }
}

float observer_get_angle(observer_t *obs)
{
    return obs->theta_est;
}

float observer_get_speed(observer_t *obs)
{
    return obs->speed_est;
}

float observer_get_sin(observer_t *obs)
{
    return obs->sin_val;
}

float observer_get_cos(observer_t *obs)
{
    return obs->cos_val;
}

uint8_t observer_is_converged(observer_t *obs)
{
    return obs->converged;
}

uint8_t observer_is_valid(observer_t *obs)
{
    return obs->valid;
}

void observer_reset(observer_t *obs)
{
    obs->theta_est = 0.0f;
    obs->speed_est = 0.0f;
    obs->sin_val = 0.0f;
    obs->cos_val = 1.0f;
    obs->converged = 0;
    obs->valid = 0;
    obs->converge_cnt = 0;
    obs->state = OBS_STATE_INIT;
}
