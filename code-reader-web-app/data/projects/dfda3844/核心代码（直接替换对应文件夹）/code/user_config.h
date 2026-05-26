/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2026,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改时必须保留以上的版权声明，谢谢合作。
 *
 * 文件名      : user_config.h
 * 简介        : 用户配置文件，包含舵机等外设的配置宏定义
 * 作者        : GLM-5
 * 日期        : 2026年1月7日
 ********************************************************************************************************************/

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "zf_common_headfile.h"

#define ACCEL_GRAVITY       (9.8f)
#define DEG_TO_RAD          (3.14159265f / 180.0f)

#define Servo_PWM_PORT      (ATOM1_CH1_P33_9)
#define SERVO_N             (6)
#define SERVO_PWM_FREQ      (50 * SERVO_N)
#define Servo_Mid           (750)
#define Servo_L_Limit       (250 * SERVO_N)
#define Servo_R_Limit       (1250 * SERVO_N)

#endif
