/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2025,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       navigation
 * @brief      导航模块实现文件，提供GPS坐标转换、航点记录、路径规划、航向融合等功能
 * @author     GLM-5
 * @date       2025-01-01
 * @version    2.0
 ********************************************************************************************************************/

#include "navigation.h"

double latitude_Data[MAX_WAYPOINTS] = {0};
double longitude_Data[MAX_WAYPOINTS] = {0};
uint8 waypoint_count = 0;

static double origin_lat = 0;
static double origin_lon = 0;
static uint8 is_nav_initialized = 0;

Robot_State robot = {0, 0, {0, 0}};
Waypoint_Map_t my_route = {0, 0, {{0, 0}}};
Record_Status_e record_status = RECORD_IDLE;
Nav_State_t nav_state = {NAV_FSM_IDLE, ALIGN_MODE_DYNAMIC_RUN, {0, 0}, 0.0, 0, 0.0};

static uint8 sample_cnt = 0;
static double lat_sum = 0;
static double lon_sum = 0;

static Local_Point last_gps_pos = {0, 0};
static double complementary_filter_alpha = 0.98;

/**
 * @brief  初始化导航系统
 * @param  mode 对准模式选择
 * @return 无
 * @note   初始化GNSS模块，设置对准模式
 */
void Nav_Init(Align_Mode_e mode)
{
    gnss_init(TAU1201);
    
    nav_state.state = NAV_FSM_IDLE;
    nav_state.align_mode = mode;
    nav_state.is_yaw_initialized = 0;
    nav_state.current_yaw = 0.0;
    nav_state.accum_dist = 0.0;
}

/**
 * @brief  获取GNSS数据
 * @param  无
 * @return uint8 返回状态值
 * @note   在回调函数执行结束后解析数据
 */
uint8 Get_Gnss_Data(void)
{
    if(gnss_flag)
    {
        gnss_data_parse();
        gnss_flag = 0;
    }
    return 1;
}

/**
 * @brief  将GPS经纬度转换为本地平面坐标
 * @param  lat 纬度值(度)
 * @param  lon 经度值(度)
 * @return Local_Point 转换后的本地平面坐标(单位:米)
 * @note   使用墨卡托投影简化算法，以初始化点为原点
 */
Local_Point GPS_To_Local(double lat, double lon)
{
    Local_Point p = {0, 0};
    if (!is_nav_initialized)
    {
        origin_lat = lat;
        origin_lon = lon;
        is_nav_initialized = 1;
        return p;
    }

    double d_lat = lat - origin_lat;
    double d_lon = lon - origin_lon;

    double mean_lat_rad = (lat + origin_lat) / 2.0 * (PI / 180.0);

    p.y = d_lat * (PI / 180.0) * EARTH_RADIUS;
    p.x = d_lon * (PI / 180.0) * EARTH_RADIUS * cos(mean_lat_rad);

    return p;
}

/**
 * @brief  触发航点记录
 * @param  无
 * @return 无
 * @note   通常绑定在按键上，触发后开始采样
 */
void Waypoint_Record_Trigger(void)
{
    if (my_route.count >= MAX_WAYPOINTS) return;
    if (gnss.state == 0) return;

    lat_sum = 0;
    lon_sum = 0;
    sample_cnt = 0;
    record_status = RECORD_SAMPLING;
}

/**
 * @brief  航点记录采样处理
 * @param  无
 * @return 无
 * @note   建议在gnss_data_parse()解析成功后立即调用
 */
void Waypoint_Record_Process(void)
{
    if (record_status != RECORD_SAMPLING) return;

    if (gnss.state == 1 && gnss.satellite_used >= 8)
    {
        lat_sum += gnss.latitude;
        lon_sum += gnss.longitude;
        sample_cnt++;

        if (sample_cnt >= SAMPLES_PER_POINT)
        {
            double avg_lat = lat_sum / SAMPLES_PER_POINT;
            double avg_lon = lon_sum / SAMPLES_PER_POINT;

            latitude_Data[waypoint_count] = avg_lat;
            longitude_Data[waypoint_count] = avg_lon;
            my_route.points[my_route.count] = GPS_To_Local(avg_lat, avg_lon);
            my_route.count++;
            waypoint_count++;

            record_status = RECORD_DONE;
        }
    }
}

/**
 * @brief  使用编码器和陀螺仪更新位置
 * @param  dt 时间间隔(秒)
 * @param  speed_m_s 当前速度(米/秒)
 * @param  gyro_rad_s 陀螺仪角速度(弧度/秒)
 * @return 无
 */
void Update_Pos_Odometry(double dt, double speed_m_s, double gyro_rad_s)
{
    robot.heading += gyro_rad_s * dt;

    if (robot.heading > PI)  robot.heading -= 2 * PI;
    if (robot.heading < -PI) robot.heading += 2 * PI;

    double dist = speed_m_s * dt;
    robot.current_pos.x += dist * cos(robot.heading);
    robot.current_pos.y += dist * sin(robot.heading);
    robot.velocity = speed_m_s;
}

/**
 * @brief  计算导航控制命令
 * @param  out_steering_angle 输出参数，航向偏差(弧度)
 * @return double 距离当前目标点的距离(米)
 */
double Calculate_Nav_Command(double *out_steering_angle)
{
    if (my_route.count == 0) return 0;

    Local_Point target = my_route.points[my_route.current_target_idx];

    double dx = target.x - robot.current_pos.x;
    double dy = target.y - robot.current_pos.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < WAYPOINT_REACH_DIST)
    {
        if (my_route.current_target_idx < my_route.count - 1)
        {
            my_route.current_target_idx++;
        }
    }

    double target_heading = atan2(dy, dx);
    double angle_diff = target_heading - robot.heading;

    if (angle_diff > PI)  angle_diff -= 2 * PI;
    if (angle_diff < -PI) angle_diff += 2 * PI;

    *out_steering_angle = angle_diff;
    return distance;
}

/**
 * @brief  计算两点间的方位角
 * @param  p1 起点
 * @param  p2 终点
 * @return double 方位角(弧度，0为东，逆时针为正)
 */
double Get_Two_Points_Azimuth(Local_Point p1, Local_Point p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return atan2(dy, dx);
}

/**
 * @brief  计算两点间的距离
 * @param  p1 起点
 * @param  p2 终点
 * @return double 距离(米)
 */
double Get_Two_Points_Distance(Local_Point p1, Local_Point p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

/**
 * @brief  更新航向角(融合GPS和IMU)
 * @param  last_gps 上一次GPS位置
 * @param  current_gps 当前GPS位置
 * @param  gyro_z_dt 陀螺仪Z轴角速度乘以时间间隔(弧度)
 * @param  speed_m_s 当前速度(米/秒)
 * @return 无
 * @note   使用互补滤波融合GPS航向和IMU航向
 */
void Update_Heading(Local_Point last_gps, Local_Point current_gps, double gyro_z_dt, double speed_m_s)
{
    nav_state.current_yaw += gyro_z_dt;
    
    if (nav_state.current_yaw > PI)  nav_state.current_yaw -= 2 * PI;
    if (nav_state.current_yaw < -PI) nav_state.current_yaw += 2 * PI;

    if (speed_m_s > MIN_SPEED_FOR_GPS_HEADING)
    {
        double gps_heading = Get_Two_Points_Azimuth(last_gps, current_gps);
        
        nav_state.current_yaw = complementary_filter_alpha * nav_state.current_yaw 
                              + (1.0 - complementary_filter_alpha) * gps_heading;
    }
    
    robot.heading = nav_state.current_yaw;
}

/**
 * @brief  导航状态机更新
 * @param  current_gps 当前GPS坐标
 * @param  gyro_z_dt 陀螺仪Z轴角速度乘以时间间隔(弧度)
 * @return 无
 * @note   周期性调用(如20ms)，处理状态转换和航向融合
 */
void Nav_State_Update(Local_Point current_gps, double gyro_z_dt)
{
    double dist_from_start = Get_Two_Points_Distance(nav_state.start_pos, current_gps);
    
    switch (nav_state.state)
    {
        case NAV_FSM_IDLE:
            nav_state.start_pos = current_gps;
            nav_state.accum_dist = 0.0;
            nav_state.is_yaw_initialized = 0;
            nav_state.current_yaw = 0.0;
            last_gps_pos = current_gps;
            
            if (nav_state.align_mode == ALIGN_MODE_DYNAMIC_RUN)
            {
                nav_state.state = NAV_FSM_ALIGNING;
            }
            else if (nav_state.align_mode == ALIGN_MODE_MANUAL_PUSH)
            {
                nav_state.state = NAV_FSM_ALIGNING;
            }
            break;
            
        case NAV_FSM_ALIGNING:
            if (dist_from_start > ALIGN_THRESHOLD_DIST)
            {
                double moving_heading = Get_Two_Points_Azimuth(nav_state.start_pos, current_gps);
                nav_state.current_yaw = moving_heading;
                nav_state.is_yaw_initialized = 1;
                nav_state.state = NAV_FSM_RUNNING;
                robot.heading = moving_heading;
            }
            else
            {
                nav_state.current_yaw += gyro_z_dt;
                if (nav_state.current_yaw > PI)  nav_state.current_yaw -= 2 * PI;
                if (nav_state.current_yaw < -PI) nav_state.current_yaw += 2 * PI;
            }
            break;
            
        case NAV_FSM_RUNNING:
            Update_Heading(last_gps_pos, current_gps, gyro_z_dt, robot.velocity);
            last_gps_pos = current_gps;
            
            if (my_route.current_target_idx >= my_route.count && my_route.count > 0)
            {
                nav_state.state = NAV_FSM_FINISHED;
            }
            break;
            
        case NAV_FSM_FINISHED:
            break;
            
        default:
            nav_state.state = NAV_FSM_IDLE;
            break;
    }
}

/**
 * @brief  在屏幕上绘制导航地图
 * @param  无
 * @return 无
 */
void Nav_Display_Map(void)
{
    for (uint8 i = 0; i < my_route.count; i++)
    {
        int16 disp_x = OFFSET_X + (int16)(my_route.points[i].x * SCALE_PIXELS_PER_METER);
        int16 disp_y = OFFSET_Y - (int16)(my_route.points[i].y * SCALE_PIXELS_PER_METER);

        ips200_draw_point(disp_x - 2, disp_y - 2, RGB565_BLUE);

        if (i == my_route.current_target_idx)
        {
            ips200_draw_point(disp_x - 4, disp_y - 4, RGB565_RED);
        }
    }

    int16 car_x = OFFSET_X + (int16)(robot.current_pos.x * SCALE_PIXELS_PER_METER);
    int16 car_y = OFFSET_Y - (int16)(robot.current_pos.y * SCALE_PIXELS_PER_METER);

    ips200_draw_line(car_x - 5, car_y, car_x + 5, car_y, RGB565_RED);
    ips200_draw_line(car_x, car_y - 5, car_x, car_y + 5, RGB565_RED);

    if (record_status == RECORD_SAMPLING)
    {
        ips200_show_string(0, 0, "Sampling...");
    }
}
