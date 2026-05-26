/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2024,逐飞科技
 * All rights reserved.
 *
 * @file       pose
 * @brief      姿态解算模块源文件
 * @details    基于四元数的姿态解算算法实现，包含IMU数据更新、姿态角计算和校准功能
 *
 * @author      GLM-5
 * @date       2024
 ********************************************************************************************************************/

#include "pose.h"

Quaternions_data Quaternions;
imu_data IMU;

/**
 * @brief       快速平方根倒数计算
 * @param       x 输入值
 * @return      1/sqrt(x)的近似值
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief       陀螺仪死区滤波（作弊函数）
 * @details     对小于阈值的陀螺仪数据进行清零处理，减少静态漂移
 */
static void cheat(void)
{
    if(IMU.gyro.x < 0.5f && IMU.gyro.x > -0.5f)
    {
        IMU.gyro.x = 0;
    }
    if(IMU.gyro.y < 0.5f && IMU.gyro.y > -0.5f)
    {
        IMU.gyro.y = 0;
    }
    if(IMU.gyro.z < 0.9f && IMU.gyro.z > -0.9f)
    {
        IMU.gyro.z = 0;
    }
}

/**
 * @brief       四元数初始化
 * @details     初始化四元数参数，设置PI控制器增益
 */
void Quaternions_init(void)
{
    Quaternions.q[0] = 1.0f;
    Quaternions.q[1] = 0.0f;
    Quaternions.q[2] = 0.0f;
    Quaternions.q[3] = 0.0f;

    Quaternions.Kp = 4.5f;
    Quaternions.Ki = 0.01f;
}

/**
 * @brief       IMU数据结构初始化
 * @details     初始化IMU数据结构和陀螺仪传感器
 */
void IMU_init(void)
{
    imu660ra_init();
    IMU.acc.x = 0;
    IMU.acc.y = 0;
    IMU.acc.z = 0;

    IMU.gyro.x = 0;
    IMU.gyro.y = 0;
    IMU.gyro.z = 0;

    IMU.dt = 0.001f;

    IMU.gyro_offset.x = 0;
    IMU.gyro_offset.y = 0;
    IMU.gyro_offset.z = 0;

    IMU.pitch = 0;
    IMU.yaw = 0;
    IMU.roll = 0;
}

/**
 * @brief       陀螺仪零偏计算
 * @details     采集1000个样本计算陀螺仪零偏
 * @return      计算完成返回1
 */
uint8 gyro_offset_calc(void)
{
    for (int i = 0; i < 1000; i++)
    {
        imu660ra_get_gyro();
        IMU.gyro_offset.x += (float)imu660ra_gyro_x;
        IMU.gyro_offset.y += (float)imu660ra_gyro_y;
        IMU.gyro_offset.z += (float)imu660ra_gyro_z;
        system_delay_ms(1);
    }
    IMU.gyro_offset.x /= 1000.0f;
    IMU.gyro_offset.y /= 1000.0f;
    IMU.gyro_offset.z /= 1000.0f;

    return 1;
}

/**
 * @brief       IMU数据更新
 * @details     读取陀螺仪和加速度计数据，进行坐标转换和零偏补偿
 */
void IMUdata_update(void)
{
    imu660ra_get_gyro();
    imu660ra_get_acc();

    IMU.acc.x = imu660ra_acc_transition(imu660ra_acc_x);
    IMU.acc.y = imu660ra_acc_transition(imu660ra_acc_y);
    IMU.acc.z = imu660ra_acc_transition(imu660ra_acc_z);

    IMU.gyro.x = imu660ra_gyro_transition(imu660ra_gyro_x - (int16)IMU.gyro_offset.x);
    IMU.gyro.y = imu660ra_gyro_transition(imu660ra_gyro_y - (int16)IMU.gyro_offset.y);
    IMU.gyro.z = imu660ra_gyro_transition(imu660ra_gyro_z - (int16)IMU.gyro_offset.z);

    cheat();
}

/**
 * @brief       四元数姿态更新
 * @details     使用互补滤波算法更新四元数，计算姿态角
 */
void Quaternions_update(void)
{
    IMU.gyro_rad.x = IMU.gyro.x * 0.0174f;
    IMU.gyro_rad.y = IMU.gyro.y * 0.0174f;
    IMU.gyro_rad.z = IMU.gyro.z * 0.0174f;

    float acc_g = invSqrt(SQ(IMU.acc.x) + SQ(IMU.acc.y) + SQ(IMU.acc.z));
    IMU.acc_g.x = IMU.acc.x * acc_g;
    IMU.acc_g.y = IMU.acc.y * acc_g;
    IMU.acc_g.z = IMU.acc.z * acc_g;

    Quaternions.Vx = 2.0f * (Quaternions.q[1] * Quaternions.q[3] - Quaternions.q[0] * Quaternions.q[2]);
    Quaternions.Vy = 2.0f * (Quaternions.q[0] * Quaternions.q[1] + Quaternions.q[2] * Quaternions.q[3]);
    Quaternions.Vz = SQ(Quaternions.q[0]) - SQ(Quaternions.q[1]) - SQ(Quaternions.q[2]) + SQ(Quaternions.q[3]);

    Quaternions.ex = (IMU.acc_g.y * Quaternions.Vz - IMU.acc_g.z * Quaternions.Vy);
    Quaternions.ey = (IMU.acc_g.z * Quaternions.Vx - IMU.acc_g.x * Quaternions.Vz);
    Quaternions.ez = (IMU.acc_g.x * Quaternions.Vy - IMU.acc_g.y * Quaternions.Vx);

    Quaternions.accex += Quaternions.ex * Quaternions.Ki * IMU.dt;
    Quaternions.accey += Quaternions.ey * Quaternions.Ki * IMU.dt;
    Quaternions.accez += Quaternions.ez * Quaternions.Ki * IMU.dt;

    IMU.gyro_rad.x += Quaternions.Kp * Quaternions.ex + Quaternions.accex;
    IMU.gyro_rad.y += Quaternions.Kp * Quaternions.ey + Quaternions.accey;
    IMU.gyro_rad.z += Quaternions.Kp * Quaternions.ez + Quaternions.accez;

    IMU.gyro_rad.x *= 0.5f * IMU.dt;
    IMU.gyro_rad.y *= 0.5f * IMU.dt;
    IMU.gyro_rad.z *= 0.5f * IMU.dt;

    float q0 = Quaternions.q[0];
    float q1 = Quaternions.q[1];
    float q2 = Quaternions.q[2];
    float q3 = Quaternions.q[3];

    Quaternions.q[0] += -1.0f * q1 * IMU.gyro_rad.x - q2 * IMU.gyro_rad.y - q3 * IMU.gyro_rad.z;
    Quaternions.q[1] += q0 * IMU.gyro_rad.x + q2 * IMU.gyro_rad.z - q3 * IMU.gyro_rad.y;
    Quaternions.q[2] += q0 * IMU.gyro_rad.y - q1 * IMU.gyro_rad.z + q3 * IMU.gyro_rad.x;
    Quaternions.q[3] += q0 * IMU.gyro_rad.z + q1 * IMU.gyro_rad.y - q2 * IMU.gyro_rad.x;

    float q_g = invSqrt(SQ(Quaternions.q[0]) + SQ(Quaternions.q[1]) + SQ(Quaternions.q[2]) + SQ(Quaternions.q[3]));
    Quaternions.q[0] *= q_g;
    Quaternions.q[1] *= q_g;
    Quaternions.q[2] *= q_g;
    Quaternions.q[3] *= q_g;

    Quaternions.g1 = 2.0f * (Quaternions.q[1] * Quaternions.q[3] - Quaternions.q[0] * Quaternions.q[2]);
    Quaternions.g2 = 2.0f * (Quaternions.q[0] * Quaternions.q[1] + Quaternions.q[2] * Quaternions.q[3]);
    Quaternions.g3 = SQ(Quaternions.q[0]) - SQ(Quaternions.q[1]) - SQ(Quaternions.q[2]) + SQ(Quaternions.q[3]);
    Quaternions.g4 = 2.0f * (Quaternions.q[1] * Quaternions.q[2] + Quaternions.q[0] * Quaternions.q[3]);
    Quaternions.g5 = SQ(Quaternions.q[0]) + SQ(Quaternions.q[1]) - SQ(Quaternions.q[2]) - SQ(Quaternions.q[3]);

    IMU.pitch = -my_asinf(Quaternions.g1) * 57.3f;
    IMU.roll = fast_atan2f(Quaternions.g2, Quaternions.g3) * 57.3f;
    IMU.yaw = fast_atan2f(Quaternions.g4, Quaternions.g5) * 57.3f;
}

/**
 * @brief       陀螺仪校准 - 静态平均法
 * @details     计算均值作为零偏，计算方差后取3σ作为死区阈值
 * @param       samples 采样数据数组 [count][3]
 * @param       count 采样数量
 * @param       offset 输出零偏
 * @param       deadzone 输出死区阈值（3σ原则）
 */
void calibrate_gyro(const float samples[][3], uint32_t count, float offset[3], float deadzone[3])
{
    if (count == 0 || samples == NULL || offset == NULL || deadzone == NULL)
    {
        return;
    }

    float sum[3] = {0.0f, 0.0f, 0.0f};
    float sum_sq[3] = {0.0f, 0.0f, 0.0f};

    for (uint32_t i = 0; i < count; i++)
    {
        sum[0] += samples[i][0];
        sum[1] += samples[i][1];
        sum[2] += samples[i][2];
    }

    offset[0] = sum[0] / (float)count;
    offset[1] = sum[1] / (float)count;
    offset[2] = sum[2] / (float)count;

    for (uint32_t i = 0; i < count; i++)
    {
        float diff0 = samples[i][0] - offset[0];
        float diff1 = samples[i][1] - offset[1];
        float diff2 = samples[i][2] - offset[2];
        sum_sq[0] += diff0 * diff0;
        sum_sq[1] += diff1 * diff1;
        sum_sq[2] += diff2 * diff2;
    }

    float variance[3];
    variance[0] = sum_sq[0] / (float)count;
    variance[1] = sum_sq[1] / (float)count;
    variance[2] = sum_sq[2] / (float)count;

    deadzone[0] = 3.0f * sqrtf(variance[0]);
    deadzone[1] = 3.0f * sqrtf(variance[1]);
    deadzone[2] = 3.0f * sqrtf(variance[2]);
}

/**
 * @brief       加速度计校准 - 最大最小值法（6面校准简化版）
 * @details     找每个轴的最大最小值，零偏=(Max+Min)/2，比例=2g/(Max-Min)
 * @param       samples 采样数据数组 [count][3]
 * @param       count 采样数量
 * @param       offset 输出零偏
 * @param       scale 输出比例因子
 */
void calibrate_accel(const float samples[][3], uint32_t count, float offset[3], float scale[3])
{
    if (count == 0 || samples == NULL || offset == NULL || scale == NULL)
    {
        return;
    }

    float max_val[3] = {samples[0][0], samples[0][1], samples[0][2]};
    float min_val[3] = {samples[0][0], samples[0][1], samples[0][2]};

    for (uint32_t i = 1; i < count; i++)
    {
        if (samples[i][0] > max_val[0]) max_val[0] = samples[i][0];
        if (samples[i][0] < min_val[0]) min_val[0] = samples[i][0];
        if (samples[i][1] > max_val[1]) max_val[1] = samples[i][1];
        if (samples[i][1] < min_val[1]) min_val[1] = samples[i][1];
        if (samples[i][2] > max_val[2]) max_val[2] = samples[i][2];
        if (samples[i][2] < min_val[2]) min_val[2] = samples[i][2];
    }

    offset[0] = (max_val[0] + min_val[0]) / 2.0f;
    offset[1] = (max_val[1] + min_val[1]) / 2.0f;
    offset[2] = (max_val[2] + min_val[2]) / 2.0f;

    scale[0] = 2.0f * 9.8f / (max_val[0] - min_val[0]);
    scale[1] = 2.0f * 9.8f / (max_val[1] - min_val[1]);
    scale[2] = 2.0f * 9.8f / (max_val[2] - min_val[2]);
}
