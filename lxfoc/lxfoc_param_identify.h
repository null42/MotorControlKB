/**************************************************************************************************
*     File Name :                        lxfoc_param_identify.h
*     Library/Module Name :              LXFOC
*     Author :                           LXFOC
*     Create Date :                      2025/04/05
*     Abstract Description :             Motor Parameter Identification - Fusion Version
*                                   Combining the best of VESC and ODrive
*                                   
*   Key Features from ODrive:
*   - Integrator-based resistance measurement (more stable)
*   - Voltage toggling inductance measurement (more reliable)
*   - Phase balance check (I_beta monitoring)
*   
*   Key Features from VESC:
*   - Deadtime compensation
*   - Temperature compensation
*   - High-pass filter for flux integration
**************************************************************************************************/

#ifndef LXFOC_PARAM_IDENTIFY_H
#define LXFOC_PARAM_IDENTIFY_H

#include <math.h>
#include <stdint.h>

#define LXFOC_PI                (3.14159265358979f)
#define LXFOC_2PI               (6.28318530717959f)
#define LXFOC_SQRT3             (1.73205080756888f)
#define LXFOC_ONE_BY_SQRT3      (0.57735026918963f)

#define LXFOC_ABS(x)            ((x) > 0 ? (x) : -(x))
#define LXFOC_MIN(a, b)         ((a) < (b) ? (a) : (b))
#define LXFOC_MAX(a, b)         ((a) > (b) ? (a) : (b))
#define LXFOC_CLAMP(x, lo, hi)  (LXFOC_MIN(LXFOC_MAX(x, lo), hi))

typedef enum {
    LXFOC_IDENTIFY_IDLE = 0,
    LXFOC_IDENTIFY_RESISTANCE,
    LXFOC_IDENTIFY_INDUCTANCE,
    LXFOC_IDENTIFY_FLUX,
    LXFOC_IDENTIFY_COMPLETE,
    LXFOC_IDENTIFY_ERROR
} LXFOC_IdentifyState_e;

typedef enum {
    LXFOC_ERROR_NONE = 0,
    LXFOC_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE,
    LXFOC_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE,
    LXFOC_ERROR_UNBALANCED_PHASES,
    LXFOC_ERROR_UNKNOWN_VBUS,
    LXFOC_ERROR_CURRENT_SENSE_SATURATION,
    LXFOC_ERROR_TIMEOUT
} LXFOC_Error_e;

typedef struct {
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float Ts;
    float Vbus;
    float pole_pairs;
} LXFOC_MotorParams_t;

typedef struct {
    float target_current;
    float max_voltage;
    float kI;
    float kIBetaFilt;
    float test_voltage;
    float actual_current;
    float I_beta_filtered;
    float I_beta_sum;
    float I_beta_count;
    float Rs_sum;
    float Rs_count;
    uint32_t start_timestamp;
    uint32_t last_timestamp;
    uint8_t attached;
} LXFOC_ResistanceMeas_t;

typedef struct {
    float test_voltage;
    float delta_I;
    float last_Ialpha;
    uint32_t start_timestamp;
    uint32_t last_timestamp;
    float sign;
    uint8_t attached;
    uint8_t toggle_count;
} LXFOC_InductanceMeas_t;

typedef struct {
    float flux_alpha_int;
    float flux_beta_int;
    float flux_alpha_hpf;
    float flux_beta_hpf;
    float hpf_alpha;
    float flux_sum;
    float flux_count;
    uint32_t start_timestamp;
} LXFOC_FluxMeas_t;

typedef struct {
    LXFOC_IdentifyState_e state;
    LXFOC_Error_e error;
    LXFOC_MotorParams_t params;
    
    LXFOC_ResistanceMeas_t resistance;
    LXFOC_InductanceMeas_t inductance;
    LXFOC_FluxMeas_t flux;
    
    float I_calib;
    float V_calib_max;
    float deadtime_ns;
    float temperature_degC;
    float temp_coeff;
    
    float I_alpha_prev;
    float I_beta_prev;
    
    uint32_t state_timer;
    uint32_t timeout_ms;
    
} LXFOC_ParamIdentify_t;

typedef struct {
    float I_alpha;
    float I_beta;
    float V_alpha;
    float V_beta;
    float Vbus;
    uint32_t timestamp;
} LXFOC_Electrical_t;

void LXFOC_ParamIdentify_Init(LXFOC_ParamIdentify_t* meas, float Ts, float Vbus);

void LXFOC_ParamIdentify_SetCalibCurrent(LXFOC_ParamIdentify_t* meas, float I_calib);
void LXFOC_ParamIdentify_SetMaxVoltage(LXFOC_ParamIdentify_t* meas, float V_max);
void LXFOC_ParamIdentify_SetDeadtime(LXFOC_ParamIdentify_t* meas, float deadtime_ns);
void LXFOC_ParamIdentify_SetTemperature(LXFOC_ParamIdentify_t* meas, float temp_degC);

LXFOC_IdentifyState_e LXFOC_ParamIdentify_GetState(LXFOC_ParamIdentify_t* meas);
LXFOC_Error_e LXFOC_ParamIdentify_GetError(LXFOC_ParamIdentify_t* meas);

void LXFOC_ParamIdentify_Start(LXFOC_ParamIdentify_t* meas);
void LXFOC_ParamIdentify_Stop(LXFOC_ParamIdentify_t* meas);

void LXFOC_ParamIdentify_Update(LXFOC_ParamIdentify_t* meas,
                                 LXFOC_Electrical_t* elec,
                                 float* mod_alpha_out,
                                 float* mod_beta_out);

void LXFOC_GetIdentifiedParams(LXFOC_ParamIdentify_t* meas,
                                float* Rs, float* Ld, float* Lq, float* Flux);

float LXFOC_GetResistance(const LXFOC_ParamIdentify_t* meas);
float LXFOC_GetInductance(const LXFOC_ParamIdentify_t* meas);
float LXFOC_GetFluxLinkage(const LXFOC_ParamIdentify_t* meas);

uint8_t LXFOC_IsValidResistance(float Rs);
uint8_t LXFOC_IsValidInductance(float Ls);
uint8_t LXFOC_IsValidFlux(float Flux);

#endif
