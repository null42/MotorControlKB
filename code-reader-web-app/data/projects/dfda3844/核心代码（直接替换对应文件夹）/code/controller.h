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

#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "zf_common_headfile.h"

/* SMC控制器类型枚举 */
typedef enum {
    SMC_POS = 0,
    SMC_VEL = 1
} SMC_Type;

/* SMC控制器上下文结构体 */
typedef struct {
    float c;
    float k;
    float q;
    float ki_s;
    float sat_th;
    float out_max;
    float prev_meas;
    float int_e;
    float int_s;
    int first_run;
    SMC_Type type;
} SMC_Context;

/* PID位置式控制器结构体 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float T;
    float tau;
    float deadband;
    float out_max;
    float integral_max;
    float feedforward;
    float prev_error;
    float prev_measure;
    float prev_setpoint;
    float integral;
    float prev_deriv;
} PID_Positional;

#define PID_SAVE_LEN ( (size_t)&(((PID_Positional*)0)->prev_error) / sizeof(float) )

/**
 * @brief  SMC控制器初始化函数
 * @param  smc: SMC控制器上下文指针
 * @param  type: 控制器类型(SMC_POS/SMC_VEL)
 * @return 无
 */
void SMC_Init(SMC_Context *smc, SMC_Type type);

/**
 * @brief  SMC控制器计算函数
 * @param  smc: SMC控制器上下文指针
 * @param  ref: 目标参考值
 * @param  meas: 当前测量值
 * @param  dt: 采样周期(秒)
 * @return 控制输出值
 */
float SMC_Calc(SMC_Context *smc, float ref, float meas, float dt);

/**
 * @brief  PID位置式控制器计算函数
 * @param  pid: PID控制器结构体指针
 * @param  setpoint: 目标设定值
 * @param  measure: 当前测量值
 * @return 控制输出值
 */
float PID_Position_Calculate(PID_Positional *pid, float setpoint, float measure);

#endif
