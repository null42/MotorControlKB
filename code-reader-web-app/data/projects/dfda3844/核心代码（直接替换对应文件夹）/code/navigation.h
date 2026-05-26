/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       navigation
 * @brief      导航模块头文件，提供GPS坐标转换、航点记录、路径规划、航向融合等功能
 * @author     GLM-5
 * @date       2025-01-01
 * @version    2.0
 ********************************************************************************************************************/

#ifndef __NAVIGATION_H__
#define __NAVIGATION_H__

#include "zf_device_gnss.h"
#include <math.h>
#include "my_math.h"

#define EARTH_RADIUS            6378137.0
#define MAX_WAYPOINTS           124
#define SAMPLES_PER_POINT       20
#define WAYPOINT_REACH_DIST     1.5
#define ALIGN_THRESHOLD_DIST    2.0
#define MIN_SPEED_FOR_GPS_HEADING 1.0

#define SCALE_PIXELS_PER_METER  10
#define OFFSET_X                160
#define OFFSET_Y                120

typedef enum {
    RECORD_IDLE = 0,
    RECORD_SAMPLING,
    RECORD_DONE
} Record_Status_e;

typedef enum {
    NAV_FSM_IDLE = 0,
    NAV_FSM_ALIGNING,
    NAV_FSM_RUNNING,
    NAV_FSM_FINISHED
} Nav_FSM_e;

typedef enum {
    ALIGN_MODE_MANUAL_PUSH = 1,
    ALIGN_MODE_DYNAMIC_RUN = 2
} Align_Mode_e;

typedef struct {
    double y;
    double x;
} Local_Point;

typedef struct {
    uint8 count;
    uint8 current_target_idx;
    Local_Point points[MAX_WAYPOINTS];
} Waypoint_Map_t;

typedef struct {
    double heading;
    double velocity;
    Local_Point current_pos;
} Robot_State;

typedef struct {
    Nav_FSM_e state;
    Align_Mode_e align_mode;
    Local_Point start_pos;
    double current_yaw;
    uint8 is_yaw_initialized;
    double accum_dist;
} Nav_State_t;

extern Robot_State robot;
extern Waypoint_Map_t my_route;
extern Record_Status_e record_status;
extern Nav_State_t nav_state;
extern double latitude_Data[MAX_WAYPOINTS];
extern double longitude_Data[MAX_WAYPOINTS];
extern uint8 waypoint_count;

/**
 * @brief  初始化导航系统
 * @param  mode 对准模式选择
 * @return 无
 * @note   初始化GNSS模块，设置对准模式
 */
void Nav_Init(Align_Mode_e mode);

/**
 * @brief  获取GNSS数据
 * @param  无
 * @return uint8 返回状态值
 */
uint8 Get_Gnss_Data(void);

/**
 * @brief  将GPS经纬度转换为本地平面坐标
 * @param  lat 纬度值(度)
 * @param  lon 经度值(度)
 * @return Local_Point 转换后的本地平面坐标(单位:米)
 * @note   使用墨卡托投影简化算法，以初始化点为原点
 */
Local_Point GPS_To_Local(double lat, double lon);

/**
 * @brief  触发航点记录
 * @param  无
 * @return 无
 * @note   通常绑定在按键上，触发后开始采样
 */
void Waypoint_Record_Trigger(void);

/**
 * @brief  航点记录采样处理
 * @param  无
 * @return 无
 * @note   建议在gnss_data_parse()解析成功后立即调用
 */
void Waypoint_Record_Process(void);

/**
 * @brief  使用编码器和陀螺仪更新位置
 * @param  dt 时间间隔(秒)
 * @param  speed_m_s 当前速度(米/秒)
 * @param  gyro_rad_s 陀螺仪角速度(弧度/秒)
 * @return 无
 */
void Update_Pos_Odometry(double dt, double speed_m_s, double gyro_rad_s);

/**
 * @brief  计算导航控制命令
 * @param  out_steering_angle 输出参数，航向偏差(弧度)
 * @return double 距离当前目标点的距离(米)
 */
double Calculate_Nav_Command(double *out_steering_angle);

/**
 * @brief  导航状态机更新
 * @param  current_gps 当前GPS坐标
 * @param  gyro_z_dt 陀螺仪Z轴角速度乘以时间间隔(弧度)
 * @return 无
 * @note   周期性调用(如20ms)，处理状态转换和航向融合
 */
void Nav_State_Update(Local_Point current_gps, double gyro_z_dt);

/**
 * @brief  更新航向角(融合GPS和IMU)
 * @param  last_gps 上一次GPS位置
 * @param  current_gps 当前GPS位置
 * @param  gyro_z_dt 陀螺仪Z轴角速度乘以时间间隔(弧度)
 * @param  speed_m_s 当前速度(米/秒)
 * @return 无
 * @note   使用互补滤波融合GPS航向和IMU航向
 */
void Update_Heading(Local_Point last_gps, Local_Point current_gps, double gyro_z_dt, double speed_m_s);

/**
 * @brief  计算两点间的方位角
 * @param  p1 起点
 * @param  p2 终点
 * @return double 方位角(弧度，0为东，逆时针为正)
 */
double Get_Two_Points_Azimuth(Local_Point p1, Local_Point p2);

/**
 * @brief  计算两点间的距离
 * @param  p1 起点
 * @param  p2 终点
 * @return double 距离(米)
 */
double Get_Two_Points_Distance(Local_Point p1, Local_Point p2);

/**
 * @brief  在屏幕上绘制导航地图
 * @param  无
 * @return 无
 */
void Nav_Display_Map(void);

#endif
