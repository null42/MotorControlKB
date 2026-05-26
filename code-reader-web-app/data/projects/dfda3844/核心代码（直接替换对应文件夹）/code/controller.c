/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file        controller
 * @brief       控制器库 - 包含PID和滑模控制器(SMC)实现
 * @author      GLM-5
 * @date        2025
 ********************************************************************************************************************/

#include "controller.h"

/**
 * @brief  限幅函数
 * @param  val: 待限幅的值
 * @param  min: 最小值
 * @param  max: 最大值
 * @return 限幅后的值
 */
static float control_limit(float val, float min, float max) {
    return (val < min) ? min : ((val > max) ? max : val);
}

/**
 * @brief  SMC饱和函数(核心防抖逻辑)
 * @param  x: 输入值s
 * @param  th: 饱和层厚度
 * @return 饱和函数输出值 [-1, 1]
 */
static float smc_sat(float x, float th) {
    if (th <= 1e-6f) {
        return (x >= 0.0f) ? 1.0f : -1.0f;
    }
    if (x > th)
        return 1.0f;
    else if (x < -th)
        return -1.0f;
    else
        return x / th;
}

/**
 * @brief  SMC控制器初始化函数
 * @param  smc: SMC控制器上下文指针
 * @param  type: 控制器类型(SMC_POS/SMC_VEL)
 * @return 无
 */
void SMC_Init(SMC_Context *smc, SMC_Type type) {
    memset(smc, 0, sizeof(SMC_Context));
    smc->type = type;
    smc->first_run = 1;
    smc->sat_th = 1.0f;
    smc->out_max = 1000.0f;
}

/**
 * @brief  SMC控制器计算函数
 * @param  smc: SMC控制器上下文指针
 * @param  ref: 目标参考值
 * @param  meas: 当前测量值
 * @param  dt: 采样周期(秒)
 * @return 控制输出值
 * @note   调参指南:
 *         第一阶段: 调节k(开关增益),直到电机能跟上目标
 *         第二阶段: 调节sat_th(饱和层),消除抖动
 *         第三阶段: 调节c(滑模面系数),优化动态性能
 *         第四阶段: 调节q(线性增益),提高收敛速度
 *         第五阶段: 调节ki_s(积分趋近),消除静差
 */
float SMC_Calc(SMC_Context *smc, float ref, float meas, float dt) {
    float error, error_dot;
    float s = 0.0f;
    float u = 0.0f;

    error = ref - meas;

    if (smc->first_run) {
        error_dot = 0.0f;
        smc->first_run = 0;
    } else {
        error_dot = -(meas - smc->prev_meas) / dt;
    }
    smc->prev_meas = meas;

    if (smc->type == SMC_POS) {
        s = smc->c * error + error_dot;
    } else {
        smc->int_e += error * dt;
        smc->int_e = control_limit(smc->int_e, -smc->out_max, smc->out_max);
        s = error + smc->c * smc->int_e;
    }

    smc->int_s += s * dt;
    float s_int_limit = smc->out_max * 0.5f;
    smc->int_s = control_limit(smc->int_s, -s_int_limit, s_int_limit);

    float reaching_term = -smc->k * smc_sat(s, smc->sat_th)
                          - smc->q * s
                          - smc->ki_s * smc->int_s;

    u = reaching_term;

    return control_limit(u, -smc->out_max, smc->out_max);
}

/**
 * @brief  PID位置式控制器计算函数
 * @param  pid: PID控制器结构体指针
 * @param  setpoint: 目标设定值
 * @param  measure: 当前测量值
 * @return 控制输出值
 * @note   支持功能: 死区处理、积分抗饱和、微分先行、不完全微分、动态前馈
 */
float PID_Position_Calculate(PID_Positional *pid, float setpoint, float measure) {
    float error = setpoint - measure;

    if(fabs(error) < pid->deadband) error = 0;

    float P = pid->Kp * error;

    pid->integral += error * pid->T;
    pid->integral = control_limit(pid->integral, -pid->integral_max, pid->integral_max);
    float I = pid->Ki * pid->integral;

    float deriv = -(measure - pid->prev_measure) / pid->T;
    float D = pid->Kd * (deriv * pid->tau + pid->prev_deriv * (1 - pid->tau));
    pid->prev_deriv = deriv;

    float FF = pid->feedforward * (setpoint - pid->prev_setpoint) / pid->T;

    float output = P + I + D + FF;
    output = control_limit(output, -pid->out_max, pid->out_max);

    pid->prev_error = error;
    pid->prev_measure = measure;
    pid->prev_setpoint = setpoint;

    return output;
}
