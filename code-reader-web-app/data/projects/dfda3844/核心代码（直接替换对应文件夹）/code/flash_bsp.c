/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2026,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       flash_bsp
 * @brief      Flash底层驱动板级支持包
 * @author     GLM-5
 * @version    查看 readme 文档
 * @date       2026年1月30日
 ********************************************************************************************************************/

#include "flash_bsp.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS经纬度数据组合
// 参数说明     y           int32类型的GPS数据
// 返回参数     double      转换后的double类型GPS数据
// 使用示例     double lat = Package_GPS(lat_int);
// 备注信息     将int32类型转换为double类型
//-------------------------------------------------------------------------------------------------------------------
static double Package_GPS(int32 y)
{
    double x = 0;
    x = y * 1.0 / 10000000.0;
    return x;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS经纬度数据拆分
// 参数说明     y           double类型的GPS数据
// 返回参数     int32       转换后的int32类型GPS数据
// 使用示例     int32 lat_int = Disassembly_GPS(latitude);
// 备注信息     将double类型转换为int32类型，采用四舍五入
//-------------------------------------------------------------------------------------------------------------------
static int32 Disassembly_GPS(double y)
{
    int32 x = 0;
    x = (int32)(y * 10000000.0 + (y >= 0 ? 0.5 : -0.5));
    return x;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将GPS数据写入Flash
// 参数说明     index       数据集序号 (1, 2, 3...)
// 返回参数     void
// 使用示例     Write_GPS_To_Flash_Group(1);
// 备注信息     经度存入奇数页，纬度存入偶数页
//-------------------------------------------------------------------------------------------------------------------
void Write_GPS_To_Flash_Group(uint8 index)
{
    uint8 lon_page = (index * 2) - 1;
    uint8 lat_page = (index * 2);

    if (waypoint_count == 0) return;

    flash_erase_page(0, lon_page);
    flash_buffer_clear();
    flash_union_buffer[0].int32_type = (int32)waypoint_count;
    for (int i = 0; i < waypoint_count; i++)
    {
        flash_union_buffer[i + 1].int32_type = Disassembly_GPS(longitude_Data[i]);
    }
    flash_write_page_from_buffer(0, lon_page);

    flash_erase_page(0, lat_page);
    flash_buffer_clear();
    flash_union_buffer[0].int32_type = (int32)waypoint_count;
    for (int i = 0; i < waypoint_count; i++)
    {
        flash_union_buffer[i + 1].int32_type = Disassembly_GPS(latitude_Data[i]);
    }
    flash_write_page_from_buffer(0, lat_page);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash读取GPS数据
// 参数说明     index       数据集序号 (1, 2, 3...)
// 返回参数     void
// 使用示例     Read_GPS_From_Flash_Group(1);
// 备注信息     从奇数页读取经度，从偶数页读取纬度
//-------------------------------------------------------------------------------------------------------------------
void Read_GPS_From_Flash_Group(uint8 index)
{
    uint8 lon_page = (index * 2) - 1;
    uint8 lat_page = (index * 2);

    waypoint_count = 0;

    if (flash_check(0, lon_page) == 1)
    {
        flash_read_page_to_buffer(0, lon_page);

        int32 saved_num = flash_union_buffer[0].int32_type;

        if (saved_num > 0 && saved_num <= MAX_WAYPOINTS)
        {
            waypoint_count = (uint16)saved_num;

            for (int i = 0; i < waypoint_count; i++)
            {
                longitude_Data[i] = Package_GPS(flash_union_buffer[i + 1].int32_type);
            }
        }
    }

    if (flash_check(0, lat_page) == 1 && waypoint_count > 0)
    {
        flash_read_page_to_buffer(0, lat_page);
        for (int i = 0; i < waypoint_count; i++)
        {
            latitude_Data[i] = Package_GPS(flash_union_buffer[i + 1].int32_type);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将float结构体存入Flash
// 参数说明     page        页码
// 参数说明     struct_ptr  结构体地址 (float指针)
// 参数说明     float_count 成员个数
// 返回参数     void
// 使用示例     Save_Struct_To_Flash(page, (float *)&pid, PID_SAVE_LEN);
// 备注信息     仅适用于全float成员的结构体
//-------------------------------------------------------------------------------------------------------------------
static void Save_Struct_To_Flash(uint8 page, const float *struct_ptr, uint16 float_count)
{
    flash_erase_page(0, page);
    flash_buffer_clear();

    flash_union_buffer[0].int32_type = (int32)float_count;

    for (int i = 0; i < float_count; i++)
    {
        flash_union_buffer[i + 1].float_type = struct_ptr[i];
    }
    flash_write_page_from_buffer(0, page);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash读取并还原到结构体
// 参数说明     page        页码
// 参数说明     struct_ptr  结构体地址 (float指针)
// 参数说明     float_count 成员个数
// 返回参数     void
// 使用示例     Load_Struct_From_Flash(page, (float *)&pid, PID_SAVE_LEN);
// 备注信息     仅适用于全float成员的结构体
//-------------------------------------------------------------------------------------------------------------------
static void Load_Struct_From_Flash(uint8 page, float *struct_ptr, uint16 float_count)
{
    if (flash_check(0, page) == 0) return;

    flash_read_page_to_buffer(0, page);

    if (flash_union_buffer[0].int32_type != (int32)float_count) return;

    for (int i = 0; i < float_count; i++)
    {
        struct_ptr[i] = flash_union_buffer[i + 1].float_type;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存PID配置到Flash
// 参数说明     page        页码
// 参数说明     pid         PID结构体指针
// 返回参数     void
// 使用示例     Save_PID_Config(PAGE_PID_YAW, &pid_yaw);
// 备注信息     保存PID参数：Kp, Ki, Kd, T, tau, deadband, out_max, integral_max, feedforward
//-------------------------------------------------------------------------------------------------------------------
void Save_PID_Config(uint8 page, const PID_Positional *pid)
{
    Save_Struct_To_Flash(page, (const float *)pid, PID_SAVE_LEN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash加载PID配置
// 参数说明     page        页码
// 参数说明     pid         PID结构体指针
// 返回参数     void
// 使用示例     Load_PID_Config(PAGE_PID_YAW, &pid_yaw);
// 备注信息     如果Flash无数据则不操作，保持初始化值
//-------------------------------------------------------------------------------------------------------------------
void Load_PID_Config(uint8 page, PID_Positional *pid)
{
    if (flash_check(0, page) == 0)
    {
        return;
    }

    Load_Struct_From_Flash(page, (float *)pid, PID_SAVE_LEN);
}
