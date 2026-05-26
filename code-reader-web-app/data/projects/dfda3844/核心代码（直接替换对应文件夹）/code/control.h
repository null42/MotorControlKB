/*********************************************************************
 * Copyright (C) 2024
 *
 * File Name    : control.h
 * Description  : 自平衡自行车控制模块头文件，包含三环串级PID控制
 *                - 外环：航向环 (Yaw)
 *                - 中环：平衡环 (Roll)
 *                - 内环：角速度环 (Gyro)
 * Author       : GLM-5 
 * Date         : 2024-xx-xx
 ********************************************************************/

#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "pose.h"
#include "controller.h"

#define YAW_PERIOD_MS   16
#define ROLL_PERIOD_MS  4
#define GYRO_PERIOD_MS  2

typedef struct {
    PID_Positional pid_yaw;
    PID_Positional pid_roll;
    PID_Positional pid_gyro;
    float gps_target_yaw_deg;
    float mechanical_zero;
    float target_roll_deg;
    float target_gyro_z;
    float final_servo_pwm;
    int is_balancing;
    uint8 balance_count;
} Bike_Ctrl_t;

extern Bike_Ctrl_t BikeCtrl;

void Bike_Ctrl_Init(void);
void Bike_Ctrl_Update(float dt_s);

#endif
