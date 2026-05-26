/**************************************************************************************************
*     File Name :                        lxfoc_autotune.c
*     Library/Module Name :              LXFOC
*     Author :                           LXFOC
*     Create Date :                      2025/04/05
*     Abstract Description :             PI参数自整定源文件 - VESC与ODrive融合版
*                                   
*   Fusion Strategy:
*   1. Current Loop PI:
*      - ODrive: 延迟补偿(1.5Ts)，考虑数字控制延迟
*      - VESC: 简洁带宽设计，阻尼比可调
*      -> 融合: ODrive延迟补偿公式 + VESC带宽设计理念
*   
*   2. Speed Loop PI:
*      - ODrive: 精确物理模型，基于J和Kt
*      - VESC: 实用惯性估计，鲁棒性好
*      -> 融合: ODrive精确模型 + VESC惯性辨识方法
*   
*   3. Anti-windup:
*      - ODrive: Back-calculation方法
*      - VESC: Clamping方法
*      -> 融合: 提供多种方法供用户选择
*   
*   4. Inertia Identification:
*      - ODrive: 加速度积分法（精确但需运动）
*      - VESC: 阶跃响应法（简单实用）
*      -> 融合: 两种方法都提供
*   
*   Reference:
*   - ODrive: https://github.com/odriverobotics/ODrive (controller.cpp, motor.cpp)
*   - VESC: https://github.com/vedderb/bldc (mc_interface.c, mcpwm_foc.c)
**************************************************************************************************/

#include "lxfoc_autotune.h"
#include <math.h>

#define LXFOC_DELAY_CYCLES_DEFAULT    (1.5f)
#define LXFOC_MIN_INERTIA            (1e-10f)
#define LXFOC_MAX_INERTIA            (100.0f)

void LXFOC_PI_Init(LXFOC_PI_Controller_t* pi, float Kp, float Ki, float out_max, float out_min)
{
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->Kd = 0.0f;
    pi->out_max = out_max;
    pi->out_min = out_min;
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->prev_meas = 0.0f;
    pi->K_back_calc = 1.0f;
    pi->integral_limit = out_max;
    pi->anti_windup = LXFOC_ANTI_WINDUP_CLAMPING;
}

void LXFOC_SetAntiWindupMethod(LXFOC_PI_Controller_t* pi, LXFOC_AntiWindupMethod_e method)
{
    pi->anti_windup = method;
}

float LXFOC_PI_Update(LXFOC_PI_Controller_t* pi, float ref, float meas, float dt)
{
    float error = ref - meas;
    float output;
    
    pi->integral += error * dt;
    
    switch (pi->anti_windup) {
        case LXFOC_ANTI_WINDUP_CLAMPING:
            if (pi->Ki > 1e-10f) {
                float int_max = pi->out_max / pi->Ki;
                float int_min = pi->out_min / pi->Ki;
                if (pi->integral > int_max) pi->integral = int_max;
                else if (pi->integral < int_min) pi->integral = int_min;
            }
            break;
            
        case LXFOC_ANTI_WINDUP_BACK_CALC:
            {
                float output_raw = pi->Kp * error + pi->Ki * pi->integral;
                float saturated = output_raw;
                if (output_raw > pi->out_max) saturated = pi->out_max;
                else if (output_raw < pi->out_min) saturated = pi->out_min;
                float back_calc = (saturated - output_raw) * pi->K_back_calc;
                pi->integral += back_calc * dt;
            }
            break;
            
        case LXFOC_ANTI_WINDUP_CONDITIONAL:
            {
                float output_test = pi->Kp * error + pi->Ki * pi->integral;
                if ((output_test < pi->out_max && output_test > pi->out_min) ||
                    (output_test >= pi->out_max && error < 0.0f) ||
                    (output_test <= pi->out_min && error > 0.0f)) {
                    pi->integral += error * dt;
                }
            }
            break;
            
        default:
            break;
    }
    
    output = pi->Kp * error + pi->Ki * pi->integral;
    
    if (output > pi->out_max) output = pi->out_max;
    else if (output < pi->out_min) output = pi->out_min;
    
    pi->prev_error = error;
    pi->prev_meas = meas;
    
    return output;
}

float LXFOC_PI_Update_AntiWindup(LXFOC_PI_Controller_t* pi, float ref, float meas, float dt)
{
    float error = ref - meas;
    float P_term = pi->Kp * error;
    float I_term = pi->Ki * pi->integral;
    float output_unsat = P_term + I_term;
    float output_sat = output_unsat;
    
    if (output_sat > pi->out_max) output_sat = pi->out_max;
    else if (output_sat < pi->out_min) output_sat = pi->out_min;
    
    if (pi->Ki > 1e-10f) {
        float anti_windup_correction = (output_sat - output_unsat) / pi->Ki;
        pi->integral += (error + anti_windup_correction * pi->K_back_calc) * dt;
    }
    
    pi->prev_error = error;
    return output_sat;
}

void LXFOC_PI_Reset(LXFOC_PI_Controller_t* pi)
{
    pi->integral = 0.0f;
    pi->prev_error = 0.0f;
    pi->prev_meas = 0.0f;
}

float LXFOC_CalculateCurrentKp_ODrive(float Ls, float bw_rad, float Ts, float delay_cycles)
{
    float delay_sec = delay_cycles * Ts;
    float Kp = Ls * bw_rad / (1.0f + bw_rad * delay_sec);
    return Kp;
}

float LXFOC_CalculateCurrentKi_ODrive(float Rs, float bw_rad, float Ls, float Ts)
{
    float Ki = Rs * bw_rad;
    float Ki_max = 1.0f / (Ls * Ts * 0.5f);
    if (Ki > Ki_max) Ki = Ki_max;
    return Ki;
}

float LXFOC_CalculateCurrentKp_VESC(float Ls, float bw_rad)
{
    return Ls * bw_rad;
}

float LXFOC_CalculateCurrentKi_VESC(float Rs, float bw_rad)
{
    return Rs * bw_rad;
}

float LXFOC_CalculateSpeedKp(float J, float bw_rad, float Kt)
{
    if (Kt < 1e-6f) Kt = 1e-6f;
    if (J < LXFOC_MIN_INERTIA) J = LXFOC_MIN_INERTIA;
    return J * bw_rad / Kt;
}

float LXFOC_CalculateSpeedKi(float J, float bw_rad, float Kt)
{
    if (Kt < 1e-6f) Kt = 1e-6f;
    if (J < LXFOC_MIN_INERTIA) J = LXFOC_MIN_INERTIA;
    return J * bw_rad * bw_rad * 0.25f / Kt;
}

float LXFOC_EstimateInertia_VESC(float Flux, float Lq, float pole_pairs)
{
    float Kt = 1.5f * pole_pairs * Flux;
    float J_est = Kt * Kt * Lq * 0.1f;
    if (J_est < LXFOC_MIN_INERTIA) J_est = LXFOC_MIN_INERTIA;
    return J_est;
}

float LXFOC_EstimateInertia_ODrive(float Kt, float Lq, float Rs)
{
    float tau_e = Lq / Rs;
    float J_est = Kt * Kt * tau_e * 0.05f;
    if (J_est < LXFOC_MIN_INERTIA) J_est = LXFOC_MIN_INERTIA;
    return J_est;
}

void LXFOC_MeasureInertia_Init(LXFOC_InertiaMeas_t* meas)
{
    meas->torque_cmd = 0.0f;
    meas->speed_meas = 0.0f;
    meas->accel_sum = 0.0f;
    meas->torque_integral = 0.0f;
    meas->inertia_estimated = 0.0f;
    meas->sample_count = 0;
    meas->meas_time_ms = 2000;
    meas->speed_prev = 0.0f;
    meas->accel_threshold = 10.0f;
}

float LXFOC_MeasureInertia_Update(LXFOC_InertiaMeas_t* meas, float torque_cmd, float speed_meas, float dt)
{
    float accel = (speed_meas - meas->speed_prev) / dt;
    meas->speed_prev = speed_meas;
    
    if (LXFOC_ABS(accel) > meas->accel_threshold) {
        meas->accel_sum += accel * accel * dt;
        meas->torque_integral += torque_cmd * accel * dt;
        meas->sample_count++;
    }
    
    if (meas->accel_sum > 1e-6f) {
        meas->inertia_estimated = meas->torque_integral / meas->accel_sum;
    }
    
    if (meas->inertia_estimated < LXFOC_MIN_INERTIA) {
        meas->inertia_estimated = LXFOC_MIN_INERTIA;
    } else if (meas->inertia_estimated > LXFOC_MAX_INERTIA) {
        meas->inertia_estimated = LXFOC_MAX_INERTIA;
    }
    
    return meas->inertia_estimated;
}

float LXFOC_CalculateKt(float Flux, float pole_pairs)
{
    return 1.5f * pole_pairs * Flux;
}

float LXFOC_CalculateMaxSpeed(float Vbus, float Flux, float pole_pairs, float Ld, float Lq)
{
    float omega_max = Vbus / (pole_pairs * Flux);
    float speed_rad_s = omega_max;
    float speed_rpm = speed_rad_s * 60.0f / (2.0f * 3.14159265f);
    return speed_rpm;
}

void LXFOC_AutoTune_Init(LXFOC_AutoTune_t* tune, LXFOC_MotorConfig_t* config)
{
    uint16_t i;
    
    tune->state = LXFOC_AUTOTUNE_IDLE;
    tune->bandwidth_level = LXFOC_BANDWIDTH_MEDIUM;
    tune->inertia_method = LXFOC_INERTIA_METHOD_STEP;
    
    tune->current_bw_hz = 500.0f;
    tune->speed_bw_hz = 50.0f;
    tune->current_damping = 0.707f;
    tune->speed_damping = 1.0f;
    
    tune->current_Kp_calculated = 0.0f;
    tune->current_Ki_calculated = 0.0f;
    tune->speed_Kp_calculated = 0.0f;
    tune->speed_Ki_calculated = 0.0f;
    
    tune->delay_compensation = LXFOC_DELAY_CYCLES_DEFAULT;
    tune->Kt = LXFOC_CalculateKt(config->Flux, config->pole_pairs);
    tune->J_estimated = config->inertia_kg_m2;
    
    if (tune->J_estimated < LXFOC_MIN_INERTIA) {
        tune->J_estimated = LXFOC_EstimateInertia_VESC(config->Flux, config->Lq, config->pole_pairs);
    }
    
    LXFOC_PI_Init(&tune->Id_PI, 0.0f, 0.0f, config->Vbus * 0.9f, -config->Vbus * 0.9f);
    LXFOC_PI_Init(&tune->Iq_PI, 0.0f, 0.0f, config->Vbus * 0.9f, -config->Vbus * 0.9f);
    LXFOC_PI_Init(&tune->Speed_PI, 0.0f, 0.0f, config->max_current_A, -config->max_current_A);
    
    LXFOC_MeasureInertia_Init(&tune->inertia_meas);
    
    tune->verify_index = 0;
    tune->verify_complete = 0;
    tune->step_index = 0;
    tune->state_timer = 0;
    tune->timeout_ms = 10000;
    
    for (i = 0; i < 100; i++) {
        tune->verify_Id_response[i] = 0.0f;
        tune->verify_Iq_response[i] = 0.0f;
    }
    
    for (i = 0; i < 200; i++) {
        tune->step_response_data[i] = 0.0f;
    }
}

void LXFOC_AutoTune_SetBandwidth(LXFOC_AutoTune_t* tune, LXFOC_BandwidthLevel_e level)
{
    tune->bandwidth_level = level;
    
    switch (level) {
        case LXFOC_BANDWIDTH_LOW:
            tune->current_bw_hz = 200.0f;
            tune->speed_bw_hz = 20.0f;
            tune->current_damping = 1.0f;
            tune->speed_damping = 1.0f;
            break;
            
        case LXFOC_BANDWIDTH_MEDIUM:
            tune->current_bw_hz = 500.0f;
            tune->speed_bw_hz = 50.0f;
            tune->current_damping = 0.707f;
            tune->speed_damping = 1.0f;
            break;
            
        case LXFOC_BANDWIDTH_HIGH:
            tune->current_bw_hz = 1000.0f;
            tune->speed_bw_hz = 100.0f;
            tune->current_damping = 0.707f;
            tune->speed_damping = 0.707f;
            break;
            
        case LXFOC_BANDWIDTH_CUSTOM:
        default:
            break;
    }
}

void LXFOC_AutoTune_SetCurrentBandwidth(LXFOC_AutoTune_t* tune, float bw_hz)
{
    tune->current_bw_hz = bw_hz;
    tune->bandwidth_level = LXFOC_BANDWIDTH_CUSTOM;
}

void LXFOC_AutoTune_SetSpeedBandwidth(LXFOC_AutoTune_t* tune, float bw_hz)
{
    tune->speed_bw_hz = bw_hz;
    tune->bandwidth_level = LXFOC_BANDWIDTH_CUSTOM;
}

void LXFOC_AutoTune_SetInertiaMethod(LXFOC_AutoTune_t* tune, LXFOC_InertiaMethod_e method)
{
    tune->inertia_method = method;
}

void LXFOC_AutoTune_SetAntiWindupMethod(LXFOC_PI_Controller_t* pi, LXFOC_AntiWindupMethod_e method)
{
    pi->anti_windup = method;
}

void LXFOC_AutoTune_CalculateCurrentPI(LXFOC_AutoTune_t* tune, LXFOC_MotorConfig_t* config)
{
    float Ls = (config->Ld + config->Lq) / 2.0f;
    float bw_rad = LXFOC_2PI * tune->current_bw_hz;
    float Kp, Ki;
    
    Kp = LXFOC_CalculateCurrentKp_ODrive(Ls, bw_rad, config->Ts, tune->delay_compensation);
    Ki = LXFOC_CalculateCurrentKi_ODrive(config->Rs, bw_rad, Ls, config->Ts);
    
    Kp = Kp * tune->current_damping;
    Ki = Ki * tune->current_damping * tune->current_damping;
    
    tune->current_Kp_calculated = Kp;
    tune->current_Ki_calculated = Ki;
    
    LXFOC_PI_Init(&tune->Id_PI, Kp, Ki, config->Vbus * 0.95f, -config->Vbus * 0.95f);
    LXFOC_PI_Init(&tune->Iq_PI, Kp, Ki, config->Vbus * 0.95f, -config->Vbus * 0.95f);
    
    tune->Id_PI.anti_windup = LXFOC_ANTI_WINDUP_BACK_CALC;
    tune->Iq_PI.anti_windup = LXFOC_ANTI_WINDUP_BACK_CALC;
    tune->Id_PI.K_back_calc = 0.5f;
    tune->Iq_PI.K_back_calc = 0.5f;
}

void LXFOC_AutoTune_CalculateSpeedPI(LXFOC_AutoTune_t* tune, LXFOC_MotorConfig_t* config)
{
    float Kt = tune->Kt;
    float J = tune->J_estimated;
    float bw_rad = LXFOC_2PI * tune->speed_bw_hz;
    float Kp, Ki;
    
    Kp = LXFOC_CalculateSpeedKp(J, bw_rad, Kt);
    Ki = LXFOC_CalculateSpeedKi(J, bw_rad, Kt);
    
    Kp = Kp * tune->speed_damping;
    Ki = Ki * tune->speed_damping * tune->speed_damping;
    
    float Kp_max = config->max_current_A * 50.0f;
    float Ki_max = config->max_current_A * 500.0f;
    if (Kp > Kp_max) Kp = Kp_max;
    if (Ki > Ki_max) Ki = Ki_max;
    
    tune->speed_Kp_calculated = Kp;
    tune->speed_Ki_calculated = Ki;
    
    LXFOC_PI_Init(&tune->Speed_PI, Kp, Ki, config->max_current_A, -config->max_current_A);
    tune->Speed_PI.anti_windup = LXFOC_ANTI_WINDUP_CLAMPING;
}

void LXFOC_AutoTune_Start(LXFOC_AutoTune_t* tune)
{
    tune->state = LXFOC_AUTOTUNE_INERTIA_MEAS;
    tune->verify_index = 0;
    tune->verify_complete = 0;
    tune->step_index = 0;
    tune->state_timer = 0;
    
    LXFOC_PI_Reset(&tune->Id_PI);
    LXFOC_PI_Reset(&tune->Iq_PI);
    LXFOC_PI_Reset(&tune->Speed_PI);
    LXFOC_MeasureInertia_Init(&tune->inertia_meas);
}

void LXFOC_AutoTune_Stop(LXFOC_AutoTune_t* tune)
{
    tune->state = LXFOC_AUTOTUNE_IDLE;
}

LXFOC_AutoTuneState_e LXFOC_AutoTune_GetState(LXFOC_AutoTune_t* tune)
{
    return tune->state;
}

void LXFOC_AutoTune_Update(LXFOC_AutoTune_t* tune, 
                            float Id_ref, float Iq_ref,
                            float Id_meas, float Iq_meas,
                            float speed_ref, float speed_meas,
                            float* Vd_out, float* Vq_out)
{
    static float Iq_ref_prev = 0.0f;
    static uint32_t step_count = 0;
    static float torque_cmd = 0.0f;
    float dt = 1.0f / 20000.0f;
    
    tune->state_timer++;
    
    switch (tune->state) {
        case LXFOC_AUTOTUNE_IDLE:
            *Vd_out = 0.0f;
            *Vq_out = 0.0f;
            break;
            
        case LXFOC_AUTOTUNE_INERTIA_MEAS:
            {
                torque_cmd = tune->Kt * Iq_ref;
                LXFOC_MeasureInertia_Update(&tune->inertia_meas, torque_cmd, speed_meas, dt);
                
                *Vd_out = LXFOC_PI_Update(&tune->Id_PI, Id_ref, Id_meas, dt);
                *Vq_out = LXFOC_PI_Update(&tune->Iq_PI, Iq_ref, Iq_meas, dt);
                
                step_count++;
                if (step_count > tune->inertia_meas.meas_time_ms * 20) {
                    tune->J_estimated = tune->inertia_meas.inertia_estimated;
                    tune->state = LXFOC_AUTOTUNE_CURRENT_LOOP;
                    step_count = 0;
                }
            }
            break;
            
        case LXFOC_AUTOTUNE_CURRENT_LOOP:
            *Vd_out = LXFOC_PI_Update(&tune->Id_PI, Id_ref, Id_meas, dt);
            *Vq_out = LXFOC_PI_Update(&tune->Iq_PI, Iq_ref, Iq_meas, dt);
            
            if (tune->verify_index < 100) {
                tune->verify_Iq_response[tune->verify_index] = Iq_meas;
                tune->verify_index++;
            }
            
            if (LXFOC_ABS(Iq_ref - Iq_ref_prev) > 0.1f) {
                tune->verify_index = 0;
            }
            Iq_ref_prev = Iq_ref;
            
            step_count++;
            if (step_count > 5000) {
                tune->state = LXFOC_AUTOTUNE_SPEED_LOOP;
                step_count = 0;
            }
            break;
            
        case LXFOC_AUTOTUNE_SPEED_LOOP:
            {
                float Iq_cmd = LXFOC_PI_Update(&tune->Speed_PI, speed_ref, speed_meas, dt);
                *Vd_out = LXFOC_PI_Update(&tune->Id_PI, Id_ref, Id_meas, dt);
                *Vq_out = LXFOC_PI_Update(&tune->Iq_PI, Iq_cmd, Iq_meas, dt);
            }
            
            step_count++;
            if (step_count > 5000) {
                tune->state = LXFOC_AUTOTUNE_COMPLETE;
                step_count = 0;
            }
            break;
            
        case LXFOC_AUTOTUNE_VERIFY:
            *Vd_out = LXFOC_PI_Update(&tune->Id_PI, Id_ref, Id_meas, dt);
            *Vq_out = LXFOC_PI_Update(&tune->Iq_PI, Iq_ref, Iq_meas, dt);
            break;
            
        case LXFOC_AUTOTUNE_COMPLETE:
            *Vd_out = 0.0f;
            *Vq_out = 0.0f;
            break;
            
        case LXFOC_AUTOTUNE_ERROR:
            *Vd_out = 0.0f;
            *Vq_out = 0.0f;
            break;
            
        default:
            *Vd_out = 0.0f;
            *Vq_out = 0.0f;
            break;
    }
    
    if (tune->state_timer > tune->timeout_ms * 20) {
        tune->state = LXFOC_AUTOTUNE_ERROR;
    }
}

void LXFOC_AutoTune_GetCurrentPI(LXFOC_AutoTune_t* tune, float* Kp, float* Ki)
{
    if (Kp) *Kp = tune->current_Kp_calculated;
    if (Ki) *Ki = tune->current_Ki_calculated;
}

void LXFOC_AutoTune_GetSpeedPI(LXFOC_AutoTune_t* tune, float* Kp, float* Ki)
{
    if (Kp) *Kp = tune->speed_Kp_calculated;
    if (Ki) *Ki = tune->speed_Ki_calculated;
}

void LXFOC_AutoTune_FromIdentifiedParams(LXFOC_AutoTune_t* tune, 
                                          LXFOC_ParamIdentify_t* ident,
                                          float pole_pairs,
                                          float max_speed_rpm,
                                          float max_current_A)
{
    LXFOC_MotorConfig_t config;
    
    config.Rs = ident->params.Rs;
    config.Ld = ident->params.Ld;
    config.Lq = ident->params.Lq;
    config.Flux = ident->params.Flux;
    config.Ts = ident->params.Ts;
    config.Vbus = ident->params.Vbus;
    config.pole_pairs = pole_pairs;
    config.max_speed_rpm = max_speed_rpm;
    config.max_current_A = max_current_A;
    config.inertia_kg_m2 = 0.0f;
    
    LXFOC_AutoTune_Init(tune, &config);
    LXFOC_AutoTune_CalculateCurrentPI(tune, &config);
    LXFOC_AutoTune_CalculateSpeedPI(tune, &config);
}
