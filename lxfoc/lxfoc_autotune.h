/**************************************************************************************************
*     File Name :                        lxfoc_autotune.h
*     Library/Module Name :              LXFOC
*     Author :                           LXFOC
*     Create Date :                      2025/04/05
*     Abstract Description :             PI参数自整定头文件 - VESC与ODrive融合版
*                                   
*   Fusion Strategy:
*   - Current Loop: ODrive's delay compensation (1.5Ts) + VESC's bandwidth design
*   - Speed Loop: ODrive's precise model + VESC's inertia identification
*   - Anti-windup: VESC's clamping + ODrive's back-calculation
*   - Inertia: Step response method (VESC) + Acceleration integration (ODrive)
*   
*   Reference:
*   - ODrive: https://github.com/odriverobotics/ODrive (controller.cpp, motor.cpp)
*   - VESC: https://github.com/vedderb/bldc (mc_interface.c, mcpwm_foc.c)
**************************************************************************************************/

#ifndef LXFOC_AUTOTUNE_H
#define LXFOC_AUTOTUNE_H

#include "lxfoc_param_identify.h"

#define LXFOC_AUTOTUNE_VERSION      "2.0.0-fusion"

typedef enum {
    LXFOC_AUTOTUNE_IDLE = 0,
    LXFOC_AUTOTUNE_INERTIA_MEAS,
    LXFOC_AUTOTUNE_CURRENT_LOOP,
    LXFOC_AUTOTUNE_SPEED_LOOP,
    LXFOC_AUTOTUNE_VERIFY,
    LXFOC_AUTOTUNE_COMPLETE,
    LXFOC_AUTOTUNE_ERROR
} LXFOC_AutoTuneState_e;

typedef enum {
    LXFOC_BANDWIDTH_LOW = 0,
    LXFOC_BANDWIDTH_MEDIUM,
    LXFOC_BANDWIDTH_HIGH,
    LXFOC_BANDWIDTH_CUSTOM
} LXFOC_BandwidthLevel_e;

typedef enum {
    LXFOC_INERTIA_METHOD_STEP = 0,
    LXFOC_INERTIA_METHOD_ACCEL_INT
} LXFOC_InertiaMethod_e;

typedef enum {
    LXFOC_ANTI_WINDUP_CLAMPING = 0,
    LXFOC_ANTI_WINDUP_BACK_CALC,
    LXFOC_ANTI_WINDUP_CONDITIONAL
} LXFOC_AntiWindupMethod_e;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float out_max;
    float out_min;
    float integral;
    float prev_error;
    float prev_meas;
    float K_back_calc;
    float integral_limit;
    LXFOC_AntiWindupMethod_e anti_windup;
} LXFOC_PI_Controller_t;

typedef struct {
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float Ts;
    float Vbus;
    float pole_pairs;
    float max_speed_rpm;
    float max_current_A;
    float inertia_kg_m2;
} LXFOC_MotorConfig_t;

typedef struct {
    float torque_cmd;
    float speed_meas;
    float accel_sum;
    float torque_integral;
    float inertia_estimated;
    uint32_t sample_count;
    uint32_t meas_time_ms;
    float speed_prev;
    float accel_threshold;
} LXFOC_InertiaMeas_t;

typedef struct {
    LXFOC_PI_Controller_t Id_PI;
    LXFOC_PI_Controller_t Iq_PI;
    LXFOC_PI_Controller_t Speed_PI;
    
    float current_bw_hz;
    float speed_bw_hz;
    float current_damping;
    float speed_damping;
    
    float current_Kp_calculated;
    float current_Ki_calculated;
    float speed_Kp_calculated;
    float speed_Ki_calculated;
    
    float delay_compensation;
    float Kt;
    float J_estimated;
    
    LXFOC_AutoTuneState_e state;
    LXFOC_BandwidthLevel_e bandwidth_level;
    LXFOC_InertiaMethod_e inertia_method;
    
    LXFOC_InertiaMeas_t inertia_meas;
    
    float verify_Id_response[100];
    float verify_Iq_response[100];
    uint16_t verify_index;
    uint8_t verify_complete;
    
    float step_response_data[200];
    uint16_t step_index;
    
    uint32_t state_timer;
    uint32_t timeout_ms;
    
} LXFOC_AutoTune_t;

void LXFOC_AutoTune_Init(LXFOC_AutoTune_t* tune, LXFOC_MotorConfig_t* config);

void LXFOC_AutoTune_SetBandwidth(LXFOC_AutoTune_t* tune, LXFOC_BandwidthLevel_e level);
void LXFOC_AutoTune_SetCurrentBandwidth(LXFOC_AutoTune_t* tune, float bw_hz);
void LXFOC_AutoTune_SetSpeedBandwidth(LXFOC_AutoTune_t* tune, float bw_hz);
void LXFOC_AutoTune_SetInertiaMethod(LXFOC_AutoTune_t* tune, LXFOC_InertiaMethod_e method);
void LXFOC_AutoTune_SetAntiWindupMethod(LXFOC_PI_Controller_t* pi, LXFOC_AntiWindupMethod_e method);

void LXFOC_AutoTune_CalculateCurrentPI(LXFOC_AutoTune_t* tune, LXFOC_MotorConfig_t* config);
void LXFOC_AutoTune_CalculateSpeedPI(LXFOC_AutoTune_t* tune, LXFOC_MotorConfig_t* config);

void LXFOC_AutoTune_Start(LXFOC_AutoTune_t* tune);
void LXFOC_AutoTune_Stop(LXFOC_AutoTune_t* tune);
LXFOC_AutoTuneState_e LXFOC_AutoTune_GetState(LXFOC_AutoTune_t* tune);

void LXFOC_AutoTune_Update(LXFOC_AutoTune_t* tune, 
                            float Id_ref, float Iq_ref,
                            float Id_meas, float Iq_meas,
                            float speed_ref, float speed_meas,
                            float* Vd_out, float* Vq_out);

void LXFOC_AutoTune_GetCurrentPI(LXFOC_AutoTune_t* tune, float* Kp, float* Ki);
void LXFOC_AutoTune_GetSpeedPI(LXFOC_AutoTune_t* tune, float* Kp, float* Ki);

void LXFOC_PI_Init(LXFOC_PI_Controller_t* pi, float Kp, float Ki, float out_max, float out_min);
float LXFOC_PI_Update(LXFOC_PI_Controller_t* pi, float ref, float meas, float dt);
float LXFOC_PI_Update_AntiWindup(LXFOC_PI_Controller_t* pi, float ref, float meas, float dt);
void LXFOC_PI_Reset(LXFOC_PI_Controller_t* pi);

float LXFOC_CalculateCurrentKp_ODrive(float Ls, float bw_rad, float Ts, float delay_cycles);
float LXFOC_CalculateCurrentKi_ODrive(float Rs, float bw_rad, float Ls, float Ts);
float LXFOC_CalculateCurrentKp_VESC(float Ls, float bw_rad);
float LXFOC_CalculateCurrentKi_VESC(float Rs, float bw_rad);

float LXFOC_CalculateSpeedKp(float J, float bw_rad, float Kt);
float LXFOC_CalculateSpeedKi(float J, float bw_rad, float Kt);

float LXFOC_EstimateInertia_VESC(float Flux, float Lq, float pole_pairs);
float LXFOC_EstimateInertia_ODrive(float Kt, float Lq, float Rs);
void LXFOC_MeasureInertia_Init(LXFOC_InertiaMeas_t* meas);
float LXFOC_MeasureInertia_Update(LXFOC_InertiaMeas_t* meas, float torque_cmd, float speed_meas, float dt);

void LXFOC_AutoTune_FromIdentifiedParams(LXFOC_AutoTune_t* tune, 
                                          LXFOC_ParamIdentify_t* ident,
                                          float pole_pairs,
                                          float max_speed_rpm,
                                          float max_current_A);

float LXFOC_CalculateKt(float Flux, float pole_pairs);
float LXFOC_CalculateMaxSpeed(float Vbus, float Flux, float pole_pairs, float Ld, float Lq);

#endif
