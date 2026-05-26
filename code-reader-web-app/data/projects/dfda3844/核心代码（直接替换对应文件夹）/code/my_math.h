/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file        my_math
 * @brief       数学运算库，提供快速三角函数、向量运算、GPS计算等功能
 * @author      GLM-5
 * @date        2025
 ********************************************************************************************************************/

#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "zf_common_headfile.h"

#define SQ(x) (x * x)
#define MY_ABS(x) ((x) > 0 ? (x) : -(x))
#define FAST_ABS_INT(x) (((x) ^ ((x) >> 31)) - ((x) >> 31))
#define MY_ABS_float(x) ((x) > 0.0f ? (x) : -(x))
#define MY_MAX(x, y) ((x) > (y) ? (x) : (y))
#define MY_MIN(x, y) ((x) < (y) ? (x) : (y))
#define MY_MIN_FAST(x, y) ((x) ^ (((x)^(y)) & (((x) < (y)) - 1)))
#define MY_MAX_FAST(x, y) ((x) ^ (((x)^(y)) & (((x) > (y)) - 1)))

#define MY_PI   (3.14159265358979323846f)
#define MY_PI_2 (1.57079632679489661923f)
#define MY_PI_4 (0.785398163397448309616f)

#define DEG2RAD(x) ((x) * 0.01745329f)
#define RAD2DEG(x) ((x) * 57.2957795f)

typedef struct
{
    double x;
    double y;
    double z;
} Vec3;

/**
 * @brief  使用四元数旋转向量
 * @param  q: 四元数数组 [w, x, y, z]
 * @param  v: 待旋转向量
 * @return 旋转后的向量
 */
static inline Vec3 Quat_Rotate_Vector(const float q[4], Vec3 v)
{
    Vec3 out;
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    float tx = 2.0f * (qy * v.z - qz * v.y);
    float ty = 2.0f * (qz * v.x - qx * v.z);
    float tz = 2.0f * (qx * v.y - qy * v.x);

    out.x = v.x + qw * tx + (qy * tz - qz * ty);
    out.y = v.y + qw * ty + (qz * tx - qx * tz);
    out.z = v.z + qw * tz + (qx * ty - qy * tx);

    return out;
}

/**
 * @brief  将角度限制在 -180 到 +180 度之间
 * @param  angle_deg: 输入角度（度）
 * @return 限制后的角度（-180 ~ +180度）
 */
static inline float Constrain_Angle_180(float angle_deg)
{
    while(angle_deg > 180.0f) angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/**
 * @brief  快速反正切函数（float版本）
 * @param  dy: y坐标
 * @param  dx: x坐标
 * @return 反正切值（弧度）
 */
float fast_atan2f(float dy, float dx);

/**
 * @brief  快速正弦函数（float版本）
 * @param  x: 输入角度（弧度）
 * @return 正弦值
 */
float my_sinf(float x);

/**
 * @brief  快速余弦函数（float版本）
 * @param  x: 输入角度（弧度）
 * @return 余弦值
 */
float my_cosf(float x);

/**
 * @brief  快速平方根函数（float版本）
 * @param  x: 输入值
 * @return 平方根值
 */
float my_sqrtf(float x);

/**
 * @brief  快速反正弦函数（float版本）
 * @param  x: 输入值
 * @return 反正弦值（弧度）
 */
float my_asinf(float x);

/**
 * @brief  快速反正切函数（double版本）
 * @param  dy: y坐标
 * @param  dx: x坐标
 * @return 反正切值（弧度）
 */
double fast_atan2(double dy, double dx);

/**
 * @brief  高精度反正切函数（double版本）
 * @param  dy: y坐标
 * @param  dx: x坐标
 * @return 反正切值（弧度）
 */
double my_atan2(double dy, double dx);

/**
 * @brief  高精度正弦函数（double版本）
 * @param  x: 输入角度（弧度）
 * @return 正弦值
 */
double my_sin(double x);

/**
 * @brief  高精度余弦函数（double版本）
 * @param  x: 输入角度（弧度）
 * @return 余弦值
 */
double my_cos(double x);

/**
 * @brief  高精度平方根函数（double版本）
 * @param  x: 输入值
 * @return 平方根值
 */
double my_sqrt(double x);

/**
 * @brief  高精度反正弦函数（double版本）
 * @param  x: 输入值
 * @return 反正弦值（弧度）
 */
double my_asin(double x);

/**
 * @brief  计算两个GPS坐标点之间的距离
 * @param  lat1: 起点纬度
 * @param  lon1: 起点经度
 * @param  lat2: 终点纬度
 * @param  lon2: 终点经度
 * @return 距离（单位：米）
 */
double GPS_Get_Distance_Double(double lat1, double lon1, double lat2, double lon2);

/**
 * @brief  计算从点1指向点2的方位角
 * @param  lat1: 起点纬度
 * @param  lon1: 起点经度
 * @param  lat2: 终点纬度
 * @param  lon2: 终点经度
 * @return 方位角（0-360度，0为正北）
 */
double GPS_Get_Bearing_Double(double lat1, double lon1, double lat2, double lon2);

#endif
