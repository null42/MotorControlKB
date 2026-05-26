/**************************************************************************************************
*     File Name :                        lxfoc_param_identify.c
*     Library/Module Name :              LXFOC
*     Author :                           LXFOC
*     Create Date :                      2025/04/05
*     Abstract Description :             Motor Parameter Identification - Fusion Version
*                                   Combining the best of VESC and ODrive
*                                   
*   Reference:
*   - ODrive: https://github.com/odriverobotics/ODrive (motor.cpp)
*   - VESC: https://github.com/vedderb/bldc
**************************************************************************************************/

#include "lxfoc_param_identify.h"

#define LXFOC_RESISTANCE_VALID_MIN     (0.001f)
#define LXFOC_RESISTANCE_VALID_MAX     (100.0f)
#define LXFOC_INDUCTANCE_VALID_MIN     (2e-6f)
#define LXFOC_INDUCTANCE_VALID_MAX     (4000e-6f)
#define LXFOC_FLUX_VALID_MIN           (0.001f)
#define LXFOC_FLUX_VALID_MAX           (1.0f)

#define LXFOC_RESISTANCE_KI            (1.0f)
#define LXFOC_RESISTANCE_KI_BETA_FILT  (80.0f)
#define LXFOC_RESISTANCE_MEAS_TIME_MS  (3000)

#define LXFOC_INDUCTANCE_TOGGLE_COUNT  (1250)
#define LXFOC_INDUCTANCE_MIN_TOGGLE    (100)

#define LXFOC_FLUX_HPF_ALPHA           (0.995f)
#define LXFOC_FLUX_MEAS_TIME_MS        (2000)

void LXFOC_ParamIdentify_Init(LXFOC_ParamIdentify_t* meas, float Ts, float Vbus)
{
    meas->state = LXFOC_IDENTIFY_IDLE;
    meas->error = LXFOC_ERROR_NONE;
    
    meas->params.Ts = Ts;
    meas->params.Vbus = Vbus;
    meas->params.Rs = 0.0f;
    meas->params.Ld = 0.0f;
    meas->params.Lq = 0.0f;
    meas->params.Flux = 0.0f;
    meas->params.pole_pairs = 4.0f;
    
    meas->resistance.target_current = 5.0f;
    meas->resistance.max_voltage = 2.0f;
    meas->resistance.kI = LXFOC_RESISTANCE_KI;
    meas->resistance.kIBetaFilt = LXFOC_RESISTANCE_KI_BETA_FILT;
    meas->resistance.test_voltage = 0.0f;
    meas->resistance.actual_current = 0.0f;
    meas->resistance.I_beta_filtered = 0.0f;
    meas->resistance.I_beta_sum = 0.0f;
    meas->resistance.I_beta_count = 0.0f;
    meas->resistance.Rs_sum = 0.0f;
    meas->resistance.Rs_count = 0.0f;
    meas->resistance.start_timestamp = 0;
    meas->resistance.last_timestamp = 0;
    meas->resistance.attached = 0;
    
    meas->inductance.test_voltage = 2.0f;
    meas->inductance.delta_I = 0.0f;
    meas->inductance.last_Ialpha = 0.0f;
    meas->inductance.start_timestamp = 0;
    meas->inductance.last_timestamp = 0;
    meas->inductance.sign = 1.0f;
    meas->inductance.attached = 0;
    meas->inductance.toggle_count = 0;
    
    meas->flux.flux_alpha_int = 0.0f;
    meas->flux.flux_beta_int = 0.0f;
    meas->flux.flux_alpha_hpf = 0.0f;
    meas->flux.flux_beta_hpf = 0.0f;
    meas->flux.hpf_alpha = LXFOC_FLUX_HPF_ALPHA;
    meas->flux.flux_sum = 0.0f;
    meas->flux.flux_count = 0.0f;
    meas->flux.start_timestamp = 0;
    
    meas->I_calib = 5.0f;
    meas->V_calib_max = 2.0f;
    meas->deadtime_ns = 500.0f;
    meas->temperature_degC = 25.0f;
    meas->temp_coeff = 0.00386f;
    
    meas->I_alpha_prev = 0.0f;
    meas->I_beta_prev = 0.0f;
    
    meas->state_timer = 0;
    meas->timeout_ms = 5000;
}

void LXFOC_ParamIdentify_SetCalibCurrent(LXFOC_ParamIdentify_t* meas, float I_calib)
{
    meas->I_calib = I_calib;
    meas->resistance.target_current = I_calib;
}

void LXFOC_ParamIdentify_SetMaxVoltage(LXFOC_ParamIdentify_t* meas, float V_max)
{
    meas->V_calib_max = V_max;
    meas->resistance.max_voltage = V_max;
    meas->inductance.test_voltage = V_max;
}

void LXFOC_ParamIdentify_SetDeadtime(LXFOC_ParamIdentify_t* meas, float deadtime_ns)
{
    meas->deadtime_ns = deadtime_ns;
}

void LXFOC_ParamIdentify_SetTemperature(LXFOC_ParamIdentify_t* meas, float temp_degC)
{
    meas->temperature_degC = temp_degC;
}

LXFOC_IdentifyState_e LXFOC_ParamIdentify_GetState(LXFOC_ParamIdentify_t* meas)
{
    return meas->state;
}

LXFOC_Error_e LXFOC_ParamIdentify_GetError(LXFOC_ParamIdentify_t* meas)
{
    return meas->error;
}

void LXFOC_ParamIdentify_Start(LXFOC_ParamIdentify_t* meas)
{
    meas->state = LXFOC_IDENTIFY_RESISTANCE;
    meas->error = LXFOC_ERROR_NONE;
    meas->state_timer = 0;
    
    meas->resistance.test_voltage = 0.0f;
    meas->resistance.actual_current = 0.0f;
    meas->resistance.I_beta_filtered = 0.0f;
    meas->resistance.I_beta_sum = 0.0f;
    meas->resistance.I_beta_count = 0.0f;
    meas->resistance.Rs_sum = 0.0f;
    meas->resistance.Rs_count = 0.0f;
    meas->resistance.attached = 0;
    
    meas->inductance.delta_I = 0.0f;
    meas->inductance.last_Ialpha = 0.0f;
    meas->inductance.sign = 1.0f;
    meas->inductance.attached = 0;
    meas->inductance.toggle_count = 0;
    
    meas->flux.flux_alpha_int = 0.0f;
    meas->flux.flux_beta_int = 0.0f;
    meas->flux.flux_alpha_hpf = 0.0f;
    meas->flux.flux_beta_hpf = 0.0f;
    meas->flux.flux_sum = 0.0f;
    meas->flux.flux_count = 0.0f;
}

void LXFOC_ParamIdentify_Stop(LXFOC_ParamIdentify_t* meas)
{
    meas->state = LXFOC_IDENTIFY_IDLE;
}

uint8_t LXFOC_IsValidResistance(float Rs)
{
    return (Rs >= LXFOC_RESISTANCE_VALID_MIN && Rs <= LXFOC_RESISTANCE_VALID_MAX);
}

uint8_t LXFOC_IsValidInductance(float Ls)
{
    return (Ls >= LXFOC_INDUCTANCE_VALID_MIN && Ls <= LXFOC_INDUCTANCE_VALID_MAX);
}

uint8_t LXFOC_IsValidFlux(float Flux)
{
    return (Flux >= LXFOC_FLUX_VALID_MIN && Flux <= LXFOC_FLUX_VALID_MAX);
}

static float lxfoc_deadtime_comp_voltage(float I_measured, float deadtime_ns, float Ts)
{
    float V_comp = 0.0f;
    if (LXFOC_ABS(I_measured) > 0.1f) {
        float sign_I = (I_measured > 0.0f) ? 1.0f : -1.0f;
        V_comp = sign_I * deadtime_ns * 1e-9f / Ts;
    }
    return V_comp;
}

static float lxfoc_temp_comp_resistance(float Rs, float temp_degC, float temp_coeff, float temp_ref)
{
    float delta_temp = temp_degC - temp_ref;
    return Rs * (1.0f + temp_coeff * delta_temp);
}

static void lxfoc_hpf_update(float* state, float* output, float input, float alpha)
{
    float prev_state = *state;
    *state = input;
    *output = alpha * (*output + input - prev_state);
}

static void lxfoc_resistance_measurement(LXFOC_ParamIdentify_t* meas,
                                          LXFOC_Electrical_t* elec,
                                          float* mod_alpha_out,
                                          float* mod_beta_out)
{
    LXFOC_ResistanceMeas_t* r = &meas->resistance;
    float Ts = meas->params.Ts;
    float Vbus = meas->params.Vbus;
    
    float I_alpha = elec->I_alpha;
    float I_beta = elec->I_beta;
    float I_mag = sqrtf(I_alpha * I_alpha + I_beta * I_beta);
    
    if (!r->attached) {
        r->start_timestamp = elec->timestamp;
        r->last_timestamp = elec->timestamp;
        r->test_voltage = r->target_current * 0.1f;
        r->attached = 1;
        r->actual_current = I_mag;
        *mod_alpha_out = r->test_voltage / Vbus;
        *mod_beta_out = 0.0f;
        return;
    }
    
    r->actual_current = I_mag;
    r->test_voltage += (r->kI * Ts) * (r->target_current - r->actual_current);
    
    float V_deadtime = lxfoc_deadtime_comp_voltage(I_alpha, meas->deadtime_ns, Ts);
    r->test_voltage += V_deadtime * 0.5f;
    
    if (LXFOC_ABS(r->test_voltage) > r->max_voltage) {
        meas->error = LXFOC_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        meas->state = LXFOC_IDENTIFY_ERROR;
        *mod_alpha_out = 0.0f;
        *mod_beta_out = 0.0f;
        return;
    }
    
    r->I_beta_filtered += (r->kIBetaFilt * Ts) * (I_beta - r->I_beta_filtered);
    
    float dt = (float)(elec->timestamp - r->last_timestamp) * 1e-6f;
    if (dt > 0.0f && dt < 0.1f) {
        if (LXFOC_ABS(I_beta) > 0.01f) {
            r->I_beta_sum += LXFOC_ABS(I_beta);
            r->I_beta_count += 1.0f;
        }
        
        float Rs_sample = r->test_voltage / r->target_current;
        if (LXFOC_IsValidResistance(Rs_sample)) {
            r->Rs_sum += Rs_sample;
            r->Rs_count += 1.0f;
        }
    }
    r->last_timestamp = elec->timestamp;
    
    float elapsed_ms = (float)(elec->timestamp - r->start_timestamp) * 1e-3f;
    if (elapsed_ms > LXFOC_RESISTANCE_MEAS_TIME_MS && r->Rs_count > 100.0f) {
        float Rs_measured = r->Rs_sum / r->Rs_count;
        Rs_measured = lxfoc_temp_comp_resistance(Rs_measured, meas->temperature_degC, 
                                                  meas->temp_coeff, 25.0f);
        
        if (!LXFOC_IsValidResistance(Rs_measured)) {
            meas->error = LXFOC_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
            meas->state = LXFOC_IDENTIFY_ERROR;
        } else {
            float I_beta_avg = r->I_beta_sum / LXFOC_MAX(r->I_beta_count, 1.0f);
            if (LXFOC_ABS(I_beta_avg) / r->target_current > 0.2f) {
                meas->error = LXFOC_ERROR_UNBALANCED_PHASES;
                meas->state = LXFOC_IDENTIFY_ERROR;
            } else {
                meas->params.Rs = Rs_measured;
                meas->state = LXFOC_IDENTIFY_INDUCTANCE;
                meas->state_timer = 0;
                meas->inductance.attached = 0;
                meas->inductance.toggle_count = 0;
            }
        }
    }
    
    *mod_alpha_out = r->test_voltage / Vbus;
    *mod_beta_out = 0.0f;
}

static void lxfoc_inductance_measurement(LXFOC_ParamIdentify_t* meas,
                                          LXFOC_Electrical_t* elec,
                                          float* mod_alpha_out,
                                          float* mod_beta_out)
{
    LXFOC_InductanceMeas_t* l = &meas->inductance;
    float Vbus = meas->params.Vbus;
    float I_alpha = elec->I_alpha;
    
    if (!l->attached) {
        l->start_timestamp = elec->timestamp;
        l->last_timestamp = elec->timestamp;
        l->last_Ialpha = I_alpha;
        l->delta_I = 0.0f;
        l->sign = 1.0f;
        l->attached = 1;
        l->toggle_count = 0;
        *mod_alpha_out = l->test_voltage / Vbus;
        *mod_beta_out = 0.0f;
        return;
    }
    
    float sign = (l->test_voltage * l->sign >= 0.0f) ? 1.0f : -1.0f;
    l->delta_I += -sign * (I_alpha - l->last_Ialpha);
    l->last_Ialpha = I_alpha;
    
    l->sign *= -1.0f;
    l->toggle_count++;
    
    float V_out = l->test_voltage * l->sign;
    *mod_alpha_out = V_out / Vbus;
    *mod_beta_out = 0.0f;
    
    float elapsed_ms = (float)(elec->timestamp - l->start_timestamp) * 1e-3f;
    
    if (l->toggle_count >= LXFOC_INDUCTANCE_TOGGLE_COUNT) {
        float dt = (float)(elec->timestamp - l->start_timestamp) * 1e-6f;
        
        if (dt > 0.001f && l->delta_I > 0.01f) {
            float L_measured = LXFOC_ABS(l->test_voltage) / (l->delta_I / dt);
            
            if (!LXFOC_IsValidInductance(L_measured)) {
                meas->error = LXFOC_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
                meas->state = LXFOC_IDENTIFY_ERROR;
            } else {
                meas->params.Ld = L_measured;
                meas->params.Lq = L_measured * 1.2f;
                meas->state = LXFOC_IDENTIFY_FLUX;
                meas->state_timer = 0;
                meas->flux.start_timestamp = elec->timestamp;
            }
        } else {
            meas->error = LXFOC_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
            meas->state = LXFOC_IDENTIFY_ERROR;
        }
    }
    
    l->last_timestamp = elec->timestamp;
}

static void lxfoc_flux_measurement(LXFOC_ParamIdentify_t* meas,
                                    LXFOC_Electrical_t* elec,
                                    float* mod_alpha_out,
                                    float* mod_beta_out)
{
    LXFOC_FluxMeas_t* f = &meas->flux;
    float Ts = meas->params.Ts;
    float Rs = meas->params.Rs;
    float Ls = (meas->params.Ld + meas->params.Lq) / 2.0f;
    
    float V_alpha = elec->V_alpha;
    float V_beta = elec->V_beta;
    float I_alpha = elec->I_alpha;
    float I_beta = elec->I_beta;
    
    float V_alpha_comp = V_alpha - Rs * I_alpha;
    float V_beta_comp = V_beta - Rs * I_beta;
    
    f->flux_alpha_int += V_alpha_comp * Ts;
    f->flux_beta_int += V_beta_comp * Ts;
    
    lxfoc_hpf_update(&f->flux_alpha_hpf, &f->flux_alpha_hpf, f->flux_alpha_int, f->hpf_alpha);
    lxfoc_hpf_update(&f->flux_beta_hpf, &f->flux_beta_hpf, f->flux_beta_int, f->hpf_alpha);
    
    float flux_alpha = f->flux_alpha_hpf - Ls * I_alpha;
    float flux_beta = f->flux_beta_hpf - Ls * I_beta;
    float flux_mag = sqrtf(flux_alpha * flux_alpha + flux_beta * flux_beta);
    
    if (LXFOC_IsValidFlux(flux_mag)) {
        f->flux_sum += flux_mag;
        f->flux_count += 1.0f;
    }
    
    float elapsed_ms = (float)(elec->timestamp - f->start_timestamp) * 1e-3f;
    
    if (elapsed_ms > LXFOC_FLUX_MEAS_TIME_MS && f->flux_count > 100.0f) {
        float Flux_measured = f->flux_sum / f->flux_count;
        
        if (LXFOC_IsValidFlux(Flux_measured)) {
            meas->params.Flux = Flux_measured;
            meas->state = LXFOC_IDENTIFY_COMPLETE;
        } else {
            meas->params.Flux = 0.01f;
            meas->state = LXFOC_IDENTIFY_COMPLETE;
        }
    }
    
    float V_out = meas->I_calib * Rs * 0.5f;
    *mod_alpha_out = V_out / meas->params.Vbus;
    *mod_beta_out = 0.0f;
}

void LXFOC_ParamIdentify_Update(LXFOC_ParamIdentify_t* meas,
                                 LXFOC_Electrical_t* elec,
                                 float* mod_alpha_out,
                                 float* mod_beta_out)
{
    meas->state_timer++;
    
    *mod_alpha_out = 0.0f;
    *mod_beta_out = 0.0f;
    
    switch (meas->state) {
        case LXFOC_IDENTIFY_IDLE:
            break;
            
        case LXFOC_IDENTIFY_RESISTANCE:
            lxfoc_resistance_measurement(meas, elec, mod_alpha_out, mod_beta_out);
            break;
            
        case LXFOC_IDENTIFY_INDUCTANCE:
            lxfoc_inductance_measurement(meas, elec, mod_alpha_out, mod_beta_out);
            break;
            
        case LXFOC_IDENTIFY_FLUX:
            lxfoc_flux_measurement(meas, elec, mod_alpha_out, mod_beta_out);
            break;
            
        case LXFOC_IDENTIFY_COMPLETE:
        case LXFOC_IDENTIFY_ERROR:
            break;
    }
    
    if (meas->state_timer > meas->timeout_ms * 10) {
        meas->error = LXFOC_ERROR_TIMEOUT;
        meas->state = LXFOC_IDENTIFY_ERROR;
    }
    
    meas->I_alpha_prev = elec->I_alpha;
    meas->I_beta_prev = elec->I_beta;
}

void LXFOC_GetIdentifiedParams(LXFOC_ParamIdentify_t* meas,
                                float* Rs, float* Ld, float* Lq, float* Flux)
{
    if (Rs) *Rs = meas->params.Rs;
    if (Ld) *Ld = meas->params.Ld;
    if (Lq) *Lq = meas->params.Lq;
    if (Flux) *Flux = meas->params.Flux;
}

float LXFOC_GetResistance(const LXFOC_ParamIdentify_t* meas)
{
    return meas->params.Rs;
}

float LXFOC_GetInductance(const LXFOC_ParamIdentify_t* meas)
{
    return meas->params.Ld;
}

float LXFOC_GetFluxLinkage(const LXFOC_ParamIdentify_t* meas)
{
    return meas->params.Flux;
}
