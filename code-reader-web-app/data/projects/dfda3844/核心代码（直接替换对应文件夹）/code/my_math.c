/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file        my_math
 * @brief       数学运算库实现，提供快速三角函数、向量运算、GPS计算等功能
 * @author      GLM-5
 * @date        2025
 ********************************************************************************************************************/

#include "my_math.h"

#if 0
static void Cov_cala(int x[], int y[], int n, double *p)
{
  int i;
  double averx, avery, sumx = 0, sumy = 0;
  double sum = 0;
  for (i = 0; i < n; i++) {
    sumx += x[i];
    sumy += y[i];
  }
  averx = sumx / n;
  avery = sumy / n;
  for (i = 0; i < n; i++) {
    sum += (x[i] - averx) * (y[i] - avery);
  }
  *p = sum / n;
}

static double Variance_cala(int data[], int length) {
    double mean = 0.0, variance = 0.0;
    int i;
 
    for (i = 0; i < length; i++) {
        mean += data[i];
    }
    mean /= length;
 
    for (i = 0; i < length; i++) {
        variance += (data[i] - mean) * (data[i] - mean);
    }
    variance /= length;
 
    return variance;
}
#endif

/**
 * @brief  快速反正切函数（float版本）
 * @param  dy: y坐标
 * @param  dx: x坐标
 * @return 反正切值（弧度），误差约0.000191869665
 */
float fast_atan2f(float dy, float dx)
{   
    if(dy > 0.0f && dx == 0.0f)
    {
        return MY_PI/2;
    }
    if (dy < 0.0f && dx == 0.0f)
    {
        return -MY_PI/2;
    }
    if (dy == 0.0f && dx == 0.0f)
    {
    }
    
    float ax = MY_ABS_float(dx), ay = MY_ABS_float(dy);
    float temp1 = MY_MIN(ax, ay)/MY_MAX(ax, ay);
    float temp2 = temp1*temp1;
    float result = ((-0.0464964749 * temp2 + 0.15931422) * temp2 - 0.327622764) * temp2 * temp1 + temp1;

	if (ay > ax)
	{
		result = MY_PI_2 - result;
	}
	if (dx < 0.0f)
	{
		result = MY_PI - result;
	}
	if (dy < 0.0f)
	{
		result = (-result);
	}
    return result;
}

/**
 * @brief  快速反正切函数（double版本）
 * @param  dy: y坐标
 * @param  dx: x坐标
 * @return 反正切值（弧度）
 */
double fast_atan2(double dy, double dx)
{
    if(dy > 0.0f && dx == 0.0f)
    {
        return MY_PI/2;
    }
    if (dy < 0.0f && dx == 0.0f)
    {
        return -MY_PI/2;
    }
    if (dy == 0.0f && dx == 0.0f)
    {
    }

    double ax = MY_ABS_float(dx), ay = MY_ABS_float(dy);
    double temp1 = MY_MIN(ax, ay)/MY_MAX(ax, ay);
    double temp2 = temp1*temp1;
    double result = ((-0.0464964749 * temp2 + 0.15931422) * temp2 - 0.327622764) * temp2 * temp1 + temp1;

    if (ay > ax)
    {
        result = MY_PI_2 - result;
    }
    if (dx < 0.0f)
    {
        result = MY_PI - result;
    }
    if (dy < 0.0f)
    {
        result = (-result);
    }
    return result;
}

/**
 * @brief  高精度反正切函数（double版本）
 * @param  dy: y坐标
 * @param  dx: x坐标
 * @return 反正切值（弧度），精度接近1e-7 rad
 */
double my_atan2(double dy, double dx)
{
    if (dx == 0.0) {
        if (dy > 0.0) return MY_PI_2;
        if (dy < 0.0) return -MY_PI_2;
        return 0.0;
    }

    double abs_y = MY_ABS(dy);
    double abs_x = MY_ABS(dx);
    double r, angle;

    if (abs_x > abs_y) {
        r = abs_y / abs_x;
    } else {
        r = abs_x / abs_y;
    }

    double r2 = r * r;
    angle = (((( -0.01213232 * r2 + 0.05368137) * r2 - 0.12086509) * r2 + 0.19389249) * r2 - 0.333056) * r2 * r + r;

    if (abs_y > abs_x) {
        angle = MY_PI_2 - angle;
    }
    if (dx < 0.0) {
        angle = MY_PI - angle;
    }
    if (dy < 0.0) {
        angle = -angle;
    }

    return angle;
}

/**
 * @brief  快速正弦函数（float版本）
 * @param  x: 输入角度（弧度）
 * @return 正弦值，使用泰勒展开计算
 */
float my_sinf(float x)
{
    while (x >  MY_PI) x -= 2.0f * MY_PI;
    while (x < -MY_PI) x += 2.0f * MY_PI;

    float x2 = x * x;
    return x * (1.0f - x2 * (0.166666667f - x2 * (0.008333333f - x2 * 0.000198413f)));
}

/**
 * @brief  高精度正弦函数（double版本）
 * @param  x: 输入角度（弧度）
 * @return 正弦值，使用高精度多项式逼近
 */
double my_sin(double x)
{
    while (x >  MY_PI) x -= 2.0 * MY_PI;
    while (x < -MY_PI) x += 2.0 * MY_PI;

    double x2 = x * x;
    double res = x * (1.0 + x2 * (-0.16666666666666666 +
                 x2 * (0.008333333333333333 +
                 x2 * (-0.0001984126984126984 +
                 x2 * 0.0000027557319223985))));
    return res;
}

/**
 * @brief  快速余弦函数（float版本）
 * @param  x: 输入角度（弧度）
 * @return 余弦值
 */
float my_cosf(float x)
{
    return my_sinf(x + MY_PI_2);
}

/**
 * @brief  高精度余弦函数（double版本）
 * @param  x: 输入角度（弧度）
 * @return 余弦值
 */
double my_cos(double x)
{
    return my_sin(x + MY_PI_2);
}

/**
 * @brief  快速平方根函数（float版本）
 * @param  x: 输入值
 * @return 平方根值，使用牛顿迭代法
 */
float my_sqrtf(float x)
{
    if (x <= 0) return 0;
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    float y = *(float*)&i;
    y = y * (1.5f - xhalf * y * y);
    return 1.0f / y;
}

/**
 * @brief  高精度平方根函数（double版本）
 * @param  x: 输入值
 * @return 平方根值，使用牛顿迭代法
 */
double my_sqrt(double x)
{
    if (x <= 0) return 0;

    double res = x;
    for (int i = 0; i < 5; i++) {
        res = 0.5 * (res + x / res);
    }
    return res;
}

/**
 * @brief  快速反正弦函数（float版本）
 * @param  x: 输入值
 * @return 反正弦值（弧度），使用多项式拟合
 */
float my_asinf(float x)
{
    float x2 = x * x;
    return x * (1.0f + x2 * (0.166666667f + x2 * 0.075f));
}

/**
 * @brief  高精度反正弦函数（double版本）
 * @param  x: 输入值
 * @return 反正弦值（弧度），使用泰勒展开
 */
double my_asin(double x)
{
    if (x > 1.0) x = 1.0;
    if (x < -1.0) x = -1.0;

    double x2 = x * x;
    return x * (1.0 + x2 * (0.16666666666666666 +
                 x2 * (0.075 +
                 x2 * (0.04464285714285714 +
                 x2 * 0.03038194444444444))));
}

/**
 * @brief  将角度标准化到 0 ~ 360 度
 * @param  angle: 输入角度（度）
 * @return 标准化后的角度（0 ~ 360度）
 */
static float normalize_angle_360(float angle) {
    while (angle < 0) angle += 360.0;
    while (angle >= 360) angle -= 360.0;
    return angle;
}

/**
 * @brief  将角度标准化到 -180 ~ 180 度
 * @param  angle: 输入角度（度）
 * @return 标准化后的角度（-180 ~ 180度）
 */
static float normalize_angle_180(float angle) {
    while (angle <= -180) angle += 360.0;
    while (angle > 180) angle -= 360.0;
    return angle;
}

/**
 * @brief  计算两个GPS坐标点之间的距离
 * @param  lat1: 起点纬度
 * @param  lon1: 起点经度
 * @param  lat2: 终点纬度
 * @param  lon2: 终点经度
 * @return 距离（单位：米）
 */
double GPS_Get_Distance_Double(double lat1, double lon1, double lat2, double lon2)
{
    double radLat1 = lat1 * (MY_PI / 180.0);
    double radLat2 = lat2 * (MY_PI / 180.0);
    double a = radLat1 - radLat2;
    double b = (lon1 - lon2) * (MY_PI / 180.0);

    double s_a_2 = my_sin(a / 2.0);
    double s_b_2 = my_sin(b / 2.0);

    double step = s_a_2 * s_a_2 + my_cos(radLat1) * my_cos(radLat2) * s_b_2 * s_b_2;

    return 2.0 * 6371000.0 * my_asin(my_sqrt(step));
}

/**
 * @brief  计算从点1指向点2的方位角
 * @param  lat1: 起点纬度
 * @param  lon1: 起点经度
 * @param  lat2: 终点纬度
 * @param  lon2: 终点经度
 * @return 方位角（0-360度，0为正北）
 */
double GPS_Get_Bearing_Double(double lat1, double lon1, double lat2, double lon2)
{
    double radLat1 = lat1 * (MY_PI / 180.0);
    double radLat2 = lat2 * (MY_PI / 180.0);
    double dLon = (lon2 - lon1) * (MY_PI / 180.0);

    double y = my_sin(dLon) * my_cos(radLat2);
    double x = my_cos(radLat1) * my_sin(radLat2) -
               my_sin(radLat1) * my_cos(radLat2) * my_cos(dLon);

    double angle = my_atan2((float)y, (float)x) * (180.0 / MY_PI);

    if (angle < 0) angle += 360.0;
    return angle;
}
