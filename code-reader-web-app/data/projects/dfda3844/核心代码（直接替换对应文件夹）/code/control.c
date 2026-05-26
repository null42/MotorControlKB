/*********************************************************************
 * Copyright (C) 2024
 *
 * File Name    : control.c
 * Description  : 自平衡自行车控制模块实现，包含三环串级PID控制
 *                - 外环：航向环 (Yaw) - 计算目标侧倾角
 *                - 中环：平衡环 (Roll) - 维持目标侧倾角
 *                - 内环：角速度环 (Gyro) - 执行舵机动作
 * Author       : GLM-5 
 * Date         : 2024-xx-xx
 ********************************************************************/

#include "control.h"

Bike_Ctrl_t BikeCtrl;

/**
 * @brief  初始化自行车控制器
 * @param  无
 * @return 无
 */
void Bike_Ctrl_Init(void)
{
    BikeCtrl.pid_yaw.Kp = 0.0f;
    BikeCtrl.pid_yaw.Ki = 0.0f;
    BikeCtrl.pid_yaw.Kd = 0.0f;
    BikeCtrl.pid_yaw.T = YAW_PERIOD_MS / 1000.0f;
    BikeCtrl.pid_yaw.out_max = 15.0f;

    BikeCtrl.pid_roll.Kp = 0.0f;
    BikeCtrl.pid_roll.Ki = 0.0f;
    BikeCtrl.pid_roll.Kd = 0.0f;
    BikeCtrl.pid_roll.T = ROLL_PERIOD_MS / 1000.0f;
    BikeCtrl.pid_roll.out_max = 300.0f;

    BikeCtrl.pid_gyro.Kp = 0.0f;
    BikeCtrl.pid_gyro.Ki = 0.0f;
    BikeCtrl.pid_gyro.Kd = 0.0f;
    BikeCtrl.pid_gyro.T = GYRO_PERIOD_MS / 1000.0f;
    BikeCtrl.pid_gyro.out_max = 5000.0f;

    BikeCtrl.mechanical_zero = 0.0f;
    BikeCtrl.balance_count = 0;
}

/**
 * @brief  自平衡自行车控制更新函数
 * @param  dt_s: 控制周期，单位秒
 * @return 无
 * @note   在定时器中调用，实现三环串级控制
 *         X轴向前，Y轴向左，Z轴向上
 */
void Bike_Ctrl_Update(float dt_s)
{
    float q_curr[4] = {Quaternions.q[0], Quaternions.q[1], Quaternions.q[2],
                       Quaternions.q[3]};

    if (BikeCtrl.balance_count == YAW_PERIOD_MS) {
        Vec3 v_forward_body = {1.0f, 0.0f, 0.0f};
        Vec3 v_curr_heading = Quat_Rotate_Vector(q_curr, v_forward_body);

        float rad_target = DEG2RAD(BikeCtrl.gps_target_yaw_deg);
        Vec3 v_target_heading;
        v_target_heading.x = cosf(rad_target);
        v_target_heading.y = sinf(rad_target);
        v_target_heading.z = 0.0f;

        float yaw_error_rad = atan2f(v_target_heading.y, v_target_heading.x) -
                              atan2f(v_curr_heading.y, v_curr_heading.x);

        yaw_error_rad = atan2f(sinf(yaw_error_rad), cosf(yaw_error_rad));

        float yaw_error_deg = RAD2DEG(yaw_error_rad);

        BikeCtrl.target_roll_deg =
            PID_Position_Calculate(&BikeCtrl.pid_yaw, 0.0f, -yaw_error_deg);

        BikeCtrl.balance_count = 0;
    }

    if (BikeCtrl.balance_count % ROLL_PERIOD_MS == 0)
    {
        float g_body_y = Quaternions.g1;

        float target_g_y =
            sinf(DEG2RAD(BikeCtrl.target_roll_deg + BikeCtrl.mechanical_zero));

        float roll_error_val = target_g_y - g_body_y;

        BikeCtrl.target_gyro_z = PID_Position_Calculate(&BikeCtrl.pid_roll, 0.0f,
                                                        -roll_error_val * 100.0f);
    }

    if (BikeCtrl.balance_count % GYRO_PERIOD_MS == 0)
    {
        float gyro_z_meas = IMU.gyro.z;

        BikeCtrl.final_servo_pwm = PID_Position_Calculate(
            &BikeCtrl.pid_gyro, BikeCtrl.target_gyro_z, gyro_z_meas);
    }
}

/**
 * @brief  自平衡自行车错误保护函数
 * @param  无
 * @return 无
 * @note   当倾斜角度超过45度时停止输出，限制目标侧倾角范围
 */
static void Bike_error_protect(void)
{
    if (MY_ABS_float(Quaternions.g1) > 0.707f) {
        BikeCtrl.final_servo_pwm = Servo_Mid;
        return;
    }
    if (BikeCtrl.target_roll_deg > 20.0f)
        BikeCtrl.target_roll_deg = 20.0f;
    if (BikeCtrl.target_roll_deg < -20.0f)
        BikeCtrl.target_roll_deg = -20.0f;
}
