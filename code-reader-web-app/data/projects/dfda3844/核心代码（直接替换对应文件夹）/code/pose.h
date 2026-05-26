/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2024,逐飞科技
 * All rights reserved.
 *
 * @file       pose
 * @brief      姿态解算模块头文件
 * @details    基于四元数的姿态解算算法，支持IMU数据更新和姿态角计算
 *
 * @author     GLM-5
 * @date       2024
 ********************************************************************************************************************/

#ifndef __POSE_H__
#define __POSE_H__

#include "zf_common_headfile.h"
#include "my_math.h"

typedef struct
{
    float x;
    float y;
    float z;
} xyz;

typedef struct
{
    xyz acc;
    xyz acc_g;
    xyz gyro;
    xyz gyro_rad;
    xyz gyro_offset;
    float dt;
    float pitch;
    float yaw;
    float roll;
} imu_data;

typedef struct
{
    float q[4];
    float Vx;
    float Vy;
    float Vz;
    float ex;
    float ey;
    float ez;
    float accex;
    float accey;
    float accez;
    float Kp;
    float Ki;
    float g1;
    float g2;
    float g3;
    float g4;
    float g5;
} Quaternions_data;

extern imu_data IMU;
extern Quaternions_data Quaternions;

void Quaternions_init(void);
void IMU_init(void);
uint8 gyro_offset_calc(void);
void IMUdata_update(void);
void Quaternions_update(void);

void calibrate_gyro(const float samples[][3], uint32_t count, float offset[3], float deadzone[3]);

void calibrate_accel(const float samples[][3], uint32_t count, float offset[3], float scale[3]);

#endif
