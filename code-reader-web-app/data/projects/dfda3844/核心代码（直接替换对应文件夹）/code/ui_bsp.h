/************************************************************************************
 * Copyright (c) 2024,逐飞科技
 * 
 * 文件名:    ui_bsp.h
 * 简介:      UI界面BSP层，提供菜单系统和按键扫描功能
 * 作者:      GLM-5
 * 日期:      2024
 ************************************************************************************/

#ifndef __UI_BSP_H__
#define __UI_BSP_H__

#include "zf_common_headfile.h"
#include "pose.h"
#include "flash_bsp.h"
#include "navigation.h"

#define KEY2            (P11_2)
#define KEY1            (P13_2)
#define KEY3            (P13_1)
#define KEY4            (P11_3)
#define KEY5            (P13_3)

#define SWITCH1         (P13_0)
#define SWITCH2         (P11_11)
#define SWITCH3         (P11_10)
#define SWITCH4         (P11_9)

#define SWITCH1_STATE   !gpio_get_level(SWITCH1)
#define SWITCH2_STATE   !gpio_get_level(SWITCH2)
#define SWITCH3_STATE   !gpio_get_level(SWITCH4)
#define SWITCH4_STATE   !gpio_get_level(SWITCH4)

#define CHARWIDTH       (14)
#define CHARHEIGHT      (16)

extern float SPE_PID_Data[10];

typedef enum {
    IDLE,
    DEBOUNCE,
    PRESSED
} KEYState;

typedef enum {
    NORETURN,
    UP,
    DOWN,
    ENTER,
    OUT
} KEYRetuen;

typedef enum {
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_INT,
    PARAM_TYPE_DOUBLE
} ParamType;

typedef struct MenuItem {
    const char* name;
    struct MenuItem* parent;
    struct MenuItem* child;
    struct MenuItem* next;
    void (*display_func)(void);
    void (*action_func)(void);
} MenuItem;

typedef union {
    float* value_f;
    int* value_i;
    uint8* value_ui;
    double* value_d;
} ParamValue;

typedef struct Parameter {
    const char* name;
    ParamType type;
    ParamValue value;
    uint8 modifiable;
    float base_step;
} Parameter;

/**
 * @brief  初始化UI界面
 * @param  无
 * @return 无
 */
void ui_init(void);

/**
 * @brief  按键扫描函数（改进版）
 * @param  无
 * @return 按键编号(1-4)，无按键按下返回0
 */
int8 key_scan_improvement(void);

/**
 * @brief  菜单主函数
 * @param  无
 * @return 无
 */
void menu(void);

#endif
