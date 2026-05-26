# MC_LIB - 六步换相模块详解

**文档版本:** v1.0  
**生成日期:** 2026-04-27  
**适用对象:** 电机控制工程师、嵌入式开发者  
**前置知识:** 六步换相理论基础、C语言编程

---

## 目录

1. [模块概述](#1-模块概述)
2. [文件结构](#2-文件结构)
3. [数据结构详解](#3-数据结构详解)
4. [核心函数分析](#4-核心函数分析)
5. [状态机设计](#5-状态机设计)
6. [使用示例](#6-使用示例)

---

## 1. 模块概述

### 1.1 模块定位

六步换相模块（MCSQ）是MC_LIB电机控制库的核心模块之一，实现了完整的无刷直流电机（BLDC）六步换相控制。

```
MC_LIB架构中的位置：

┌─────────────────────────────────────────────────────────┐
│  第3层：电机控制层 (3_MC)                                 │
│  ├── 31_FOC/       (FOC矢量控制)                         │
│  └── 30_SQ/        (六步换相控制) ← 本模块                │
│      ├── MCSQ_BLDC.c/h  (核心算法)                       │
│      ├── MCSQ_API.c/h   (应用接口)                       │
│      ├── MCSQ_TASK.c/h  (任务调度)                       │
│      └── MCSQ_PARA.c/h  (参数配置)                       │
└─────────────────────────────────────────────────────────┘
```

### 1.2 核心特性

```
┌─────────────────────────────────────────────────────────────┐
│                  MCSQ模块核心特性                            │
├─────────────────────────────────────────────────────────────┤
│  ✓ 三种换相方法：磁链法、反电动势法、比较器法                │
│  ✓ 智能切换：低速磁链法，高速反电动势法                      │
│  ✓ 多种启动方式：定位启动、飞启动                            │
│  ✓ 完整控制链：速度环、电流环、PWM频率自适应                 │
│  ✓ 保护机制：堵转检测、过流保护、偏置校准                    │
│  ✓ 定点实现：适合低端MCU，资源占用少                         │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 文件结构

### 2.1 文件列表

```
MC_LIB/3_MC/30_SQ/
├── MCSQ_BLDC.h      # 核心算法头文件（数据结构定义）
├── MCSQ_BLDC.c      # 核心算法实现
├── MCSQ_API.h       # 应用接口头文件
├── MCSQ_API.c       # 应用接口实现
├── MCSQ_TASK.h      # 任务调度头文件
├── MCSQ_TASK.c      # 任务调度实现（状态机）
├── MCSQ_PARA.h      # 参数配置头文件
└── MCSQ_PARA.c      # 参数配置实现
```

### 2.2 文件依赖关系

```
┌─────────────┐
│ MCSQ_API.h  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ MCSQ_PARA.h │
└──────┬──────┘
       │
       ▼
┌─────────────┐     ┌──────────────┐
│ MCSQ_BLDC.h │────▶│ MATH_PID_T.h │
└──────┬──────┘     │ MATH_LPF_T.h │
       │            │ MATH_RAMP_T.h│
       │            └──────────────┘
       ▼
┌─────────────┐
│ MCSQ_TASK.h │
└─────────────┘
```

---

## 3. 数据结构详解

### 3.1 核心数据结构

#### 3.1.1 扇区枚举

```c
// 六个扇区定义
typedef enum
{
    sector_1,   // 扇区1: 0°  ~ 60°
    sector_2,   // 扇区2: 60°  ~ 120°
    sector_3,   // 扇区3: 120° ~ 180°
    sector_4,   // 扇区4: 180° ~ 240°
    sector_5,   // 扇区5: 240° ~ 300°
    sector_6,   // 扇区6: 300° ~ 360°
}EM_SECTOR_NUM;
```

#### 3.1.2 换相方法枚举

```c
// 三种换相方法
typedef enum{
    SWITCH_FLUX,    // 磁链法：积分反电动势得到磁链
    SWITCH_BEMF,    // 反电动势法：检测反电动势过零点
    SWITCH_CMP,     // 比较器法：硬件比较器直接检测
}EM_SWITCH_FLOW;
```

#### 3.1.3 换相流程状态

```c
// 换相流程状态
typedef enum{
    SQUARE_DIAG_ING,      // 诊断中：检测反电动势状态
    SQUARE_CROSS_ING,     // 换相中：等待过零点
    SQUARE_CROSS_SUCC,    // 换相成功：检测到过零点
    SQUARE_SWITCH_SUCC,   // 切换成功：完成换相动作
}EM_SQUARE_FLOW;
```

#### 3.1.4 电机状态流程

```c
// 电机运行状态
typedef enum{
    MOTOR_STATE_PRE,            // 准备阶段
    MOTOR_STATE_INIT,           // 硬件初始化阶段
    MOTOR_STATE_IDLE,           // 电机静止阶段
    MOTOR_STATE_BOOT,           // 启动爬坡阶段
    MOTOR_STATE_POSITION,       // 脉冲定位阶段
    MOTOR_STATE_RUN,            // 电机运行阶段
    MOTOR_STATE_BRAKE,          // 刹车阶段
}EM_MOTOR_STATE_FLOW;
```

### 3.2 主要结构体

#### 3.2.1 BLDC控制结构体

```c
typedef struct
{
    EM_DIRECTION            DIR_Set;        // 设定方向
    EM_DIRECTION            DIR_Target;     // 目标方向
    EM_SECTOR_NUM           Sector;         // 当前扇区
    EM_SECTOR_NUM           Sector_Last;    // 上次扇区
    
    ST_FREQ_CAL             Freq_Cal;       // 频率计算
    
    ST_LPF_T                FL_Freq;        // 频率低通滤波
    ST_LPF_T                FL_Ibus;        // 母线电流低通滤波
    ST_LPF_T                FL_Iphase;      // 相电流低通滤波
    
    EM_SQUARE_FLOW          SQ_Flow;        // 换相流程状态
    EM_SWITCH_FLOW          SW_Math;        // 换相方法选择
    
    Q32U_                   _V_Q12U_Bemf_ADC_tmp[3];    // 反电动势ADC值
    Q32U_                   _V_Q12U_Bemf_ON_ADC;        // 开通时反电动势
    Q32U_                   _V_Q12U_Bemf_ZI_ADC;        // 过零时反电动势
    Q32U_                   _V_Q12U_Bemf_OF_ADC;        // 关断时反电动势
    
    Q32U_                   _V_Q12U_Vbus_ADC;           // 母线电压ADC值
    Q32U_                   _V_Q12U_Iphase_ADC;         // 相电流ADC值
    Q32U_                   _V_Q12U_Iphase_ADC_Offset;  // 相电流偏置
    
    Q32U_                   _V_Q14U_Iphase_pu;          // 相电流标幺值
    Q32U_                   _V_Q14U_Vbus_pu;            // 母线电压标幺值
    
    Q32U_                   _P_Q14U_Iphase_Max_pu;      // 相电流最大值
    Q32U_                   _P_Q14U_Ibus_Max_pu;        // 母线电流最大值
    Q32U_                   _P_Q14U_Vbus_Max_pu;        // 母线电压最大值
    Q32U_                   _P_Q28U_Freq_Scale;         // 频率标度
}ST_MCSQ_BLDC;
```

#### 3.2.2 频率计算结构体

```c
typedef struct
{
    Q32U_   _I_Q32U_Time_Count;              // 输入：定时器计数值
    Q32U_   _O_Q32U_Switch_cnt;              // 输出：换相计数
    
    Q32U_   _V_Q32U_60Deg_Time_cnt;          // 60°时间计数
    Q32U_   _V_Q32U_60Deg_Time_cnt_last;     // 上次60°时间
    Q32U_   _V_Q32U_60Deg_Time_cnt_tmp[6];   // 6个扇区时间缓存
    Q32U_   _V_Q32U_60Deg_Time_cnt_Filter;   // 滤波后的60°时间
    
    Q32U_   _P_Q32U_Hall_Time_Freq;          // 霍尔定时器频率
    Q32U_   _P_Q32U_Hall_Time_Max_count;     // 霍尔定时器最大计数
}ST_FREQ_CAL;
```

#### 3.2.3 控制系统结构体

```c
typedef struct{
    ST_MCSQ_BLDC            MCSQ_BLDC;       // BLDC控制
    
    ST_MCSQ_OFFSET          MCSQ_OFFSET;     // 偏置校准
    ST_MCSQ_FLYING          MCSQ_FLYING;     // 飞启动
    ST_MCSQ_BOOT            MCSQ_BOOT;       // 启动
    ST_MCSQ_POSITION        MCSQ_POSITION;   // 定位
    ST_MCSQ_BRAKE           MCSQ_BRAKE;      // 刹车
    
    ST_MS_DIAG              MCSQ_DIAG;       // 诊断
    ST_MS_FLUX              MCSQ_FLUX;       // 磁链法
    ST_MS_BEMF              MCSQ_BEMF;       // 反电动势法
    ST_MS_CMP               MCSQ_CMP;        // 比较器法
    
    ST_RAMP_T               Ramp_Freq;       // 频率斜坡
    ST_PID_INC_T            PID_Freq;        // 频率PID
    ST_PID_INC_T            PID_Ibus;        // 母线电流PID
    ST_PID_INC_T            PID_Iphase;      // 相电流PID
    
    ST_PWM_CONTROL          PWM_CTRL;        // PWM控制
    
    ST_STALL_CONTROL        STALL_CTRL;      // 堵转控制
}ST_MCSQ_CONTROL;
```

---

## 4. 核心函数分析

### 4.1 初始化函数

```c
/**********************************************************************************************
Function: MCSQ_Init
Description: 方波算法初始化
Input: 无
Output: 无
Input_Output: 控制参数指针
Return: 无
Author: CJYS
***********************************************************************************************/
void MCSQ_Init(ST_MCSQ_CONTROL* pMS_CTRL)
{
    // 1. 设置默认换相方法为磁链法
    pMS_CTRL->MCSQ_BLDC.SW_Math = SWITCH_FLUX;
    
    // 2. 设置换相流程状态
    pMS_CTRL->MCSQ_BLDC.SQ_Flow = SQUARE_CROSS_ING;
    
    // 3. 初始化方向
    pMS_CTRL->MCSQ_BLDC.DIR_Set = pMS_CTRL->MCSQ_BLDC.DIR_Target;
    
    // 4. 初始化频率计算
    pMS_CTRL->MCSQ_BLDC.Freq_Cal._V_Q32U_60Deg_Time_cnt_last = 0U;
    pMS_CTRL->MCSQ_BLDC.Freq_Cal._O_Q32U_Switch_cnt = 0U;
    
    // 5. 初始化低通滤波器
    LPF_Init_T(&pMS_CTRL->MCSQ_BLDC.FL_Iphase, 0);
    LPF_Init_T(&pMS_CTRL->MCSQ_BLDC.FL_Freq, 0);
    LPF_Init_T(&pMS_CTRL->MCSQ_BLDC.FL_Ibus, 0);
    
    // 6. 初始化PID控制器
    PID_Inc_Init_T(&pMS_CTRL->PID_Freq, pMS_CTRL->PWM_CTRL._P_Q14U_Duty_Min);
    PID_Inc_Init_T(&pMS_CTRL->PID_Ibus, pMS_CTRL->PWM_CTRL._P_Q14U_Duty_Max);
    PID_Inc_Init_T(&pMS_CTRL->PID_Iphase, pMS_CTRL->PWM_CTRL._P_Q14U_Duty_Max);
    
    // 7. 初始化斜坡函数
    Ramp_Init_T(&pMS_CTRL->Ramp_Freq, pMS_CTRL->Ramp_Freq.Q14I_Init);
    Ramp_Init_T(&pMS_CTRL->PWM_CTRL.Ramp_Duty, pMS_CTRL->PWM_CTRL.Ramp_Duty.Q14I_Init);
}
```

### 4.2 换相函数

#### 4.2.1 磁链法换相

```c
void MCSQ_FLUX(ST_MS_FLUX* pMS_FLUX, ST_MCSQ_BLDC* pBLDC)
{
    // 1. 磁链积分计算
    // ψ += (U - I·R) · Δt
    
    // 2. 检测磁链过零点
    if (磁链上升且超过阈值) {
        // 触发换相
        pBLDC->SQ_Flow = SQUARE_CROSS_SUCC;
    }
    else if (磁链下降且低于阈值) {
        // 触发换相
        pBLDC->SQ_Flow = SQUARE_CROSS_SUCC;
    }
    
    // 3. 磁链法与反电动势法切换判断
    if (速度 > 切换阈值) {
        pMS_FLUX->_V_Q32U_Flux_to_Bemf_cnt++;
        if (pMS_FLUX->_V_Q32U_Flux_to_Bemf_cnt > FLUX_TO_BEMF_NUM) {
            // 切换到反电动势法
            pBLDC->SW_Math = SWITCH_BEMF;
        }
    }
}
```

#### 4.2.2 反电动势法换相

```c
void MCSQ_BEMF(ST_MS_BEMF* pMS_BEMF, ST_MCSQ_BLDC* pBLDC)
{
    // 1. 获取悬空相的反电动势
    Q32U_ bemf_value = Get_Floating_Phase_BEMF(pBLDC->Sector);
    
    // 2. 检测反电动势过零点
    if (反电动势上升且过零) {
        // 计算换相延迟
        Q32U_ delay_time = pBLDC->Freq_Cal._V_Q32U_60Deg_Time_cnt_Filter 
                          * pMS_BEMF->_P_Q14U_Bemf_Delay_Coeff / 8192;
        Set_Commutation_Delay(delay_time);
        pBLDC->SQ_Flow = SQUARE_CROSS_SUCC;
    }
    else if (反电动势下降且过零) {
        // 计算换相延迟
        Q32U_ delay_time = pBLDC->Freq_Cal._V_Q32U_60Deg_Time_cnt_Filter 
                          * pMS_BEMF->_P_Q14U_Bemf_Delay_Coeff / 8192;
        Set_Commutation_Delay(delay_time);
        pBLDC->SQ_Flow = SQUARE_CROSS_SUCC;
    }
    
    // 3. 反电动势法与磁链法切换判断
    if (速度 < 切换阈值) {
        pMS_BEMF->_V_Q32U_Bemf_to_Flux_cnt++;
        if (pMS_BEMF->_V_Q32U_Bemf_to_Flux_cnt > BEMF_TO_FLUX_NUM) {
            // 切换到磁链法
            pBLDC->SW_Math = SWITCH_FLUX;
        }
    }
}
```

### 4.3 频率计算函数

```c
void MCSQ_Freq_Cal(ST_FREQ_CAL* pFREQ_CAL, ST_MCSQ_CONTROL* pMS_CTRL, Q32U_ Q32U_Time_Count)
{
    // 1. 计算60°时间
    pFREQ_CAL->_V_Q32U_60Deg_Time_cnt = Q32U_Time_Count - pFREQ_CAL->_V_Q32U_60Deg_Time_cnt_last;
    pFREQ_CAL->_V_Q32U_60Deg_Time_cnt_last = Q32U_Time_Count;
    
    // 2. 存储到缓存数组
    static uint8_t index = 0;
    pFREQ_CAL->_V_Q32U_60Deg_Time_cnt_tmp[index] = pFREQ_CAL->_V_Q32U_60Deg_Time_cnt;
    index = (index + 1) % 6;
    
    // 3. 滤波计算（取平均值）
    Q32U_ sum = 0;
    for (uint8_t i = 0; i < 6; i++) {
        sum += pFREQ_CAL->_V_Q32U_60Deg_Time_cnt_tmp[i];
    }
    pFREQ_CAL->_V_Q32U_60Deg_Time_cnt_Filter = sum / 6;
    
    // 4. 计算频率
    // f = Hall_Freq / (60°时间 × 6)
    // 因为 60°时间 = 定时器计数，一个电周期 = 6 × 60°时间
}
```

### 4.4 启动函数

#### 4.4.1 定位启动

```c
EM_FLAG_STATE MCSQ_Pluse_Positon(ST_MCSQ_POSITION* MCSQ_POSITION, ST_MCSQ_BLDC* pBLDC, Q32U_ Q32U_Iphase_adc_value)
{
    EM_FLAG_STATE state = ING;
    
    // 1. 依次给六相注入电流
    // 定位顺序：U+ → V+ → W+ → U- → V- → W-
    
    // 2. 检测电流是否达到阈值
    MCSQ_POSITION->_V_Q12U_Position_Iphase_ADC_tmp[MCSQ_POSITION->_V_Q32U_Position_cnt] = Q32U_Iphase_adc_value;
    
    if (Q32U_Iphase_adc_value > MCSQ_POSITION->_P_Q12U_Position_Iphase_TL) {
        // 电流达到阈值，定位成功
        MCSQ_POSITION->_V_Q32U_Position_cnt++;
        
        if (MCSQ_POSITION->_V_Q32U_Position_cnt >= 6) {
            // 六相定位完成
            state = SUCS;
        }
    }
    else {
        // 定位失败
        state = FAIL;
    }
    
    return state;
}
```

#### 4.4.2 飞启动

```c
EM_FLAG_STATE MCSQ_Flying_Check(ST_MCSQ_FLYING* pFLYING, ST_MCSQ_BLDC* pBLDC, Q32U_ Q32U_Time_Count)
{
    EM_FLAG_STATE state = ING;
    
    // 1. 检测反电动势是否足够大
    if (pBLDC->_V_Q12U_Bemf_ADC_tmp[0] > FREE_FLYING_TL_lsb ||
        pBLDC->_V_Q12U_Bemf_ADC_tmp[1] > FREE_FLYING_TL_lsb ||
        pBLDC->_V_Q12U_Bemf_ADC_tmp[2] > FREE_FLYING_TL_lsb) {
        
        // 反电动势足够，电机在旋转
        pFLYING->_V_Q32U_Flying_Low_Bemf_cnt = 0;
        
        // 2. 估计当前扇区
        // 根据三相反电动势的大小关系判断扇区
        
        // 3. 估计当前速度
        // 根据反电动势幅度估计速度
        
        // 4. 同步成功
        state = SUCS;
    }
    else {
        // 反电动势太小，电机未旋转或速度太低
        pFLYING->_V_Q32U_Flying_Low_Bemf_cnt++;
        
        if (pFLYING->_V_Q32U_Flying_Low_Bemf_cnt > FREE_FLYING_COUNT) {
            // 飞启动失败
            state = FAIL;
        }
    }
    
    return state;
}
```

---

## 5. 状态机设计

### 5.1 主状态机

```c
// 状态机函数指针数组
pMOTOR_FUN Motor_Flow_Function[MOTOR_STATE_BRAKE+1] =
{
    MotorTask_Pre_Flow,        // 准备阶段
    MotorTask_Init_Flow,       // 初始化阶段
    MotorTask_Idle_Flow,       // 空闲阶段
    MotorTask_Boot_Flow,       // 启动阶段
    MotorTask_Position_Flow,   // 定位阶段
    MotorTask_Run_Flow,        // 运行阶段
    MotorTask_Brake_Flow       // 刹车阶段
};

// 状态机执行
void MotorTask_Current_Flow(ST_MOTOR_TASK* pMotor)
{
    // 检查错误标志
    if(pMotor->MC_ERR.Motor_Error_Flag.all != 0U)
    {
        pMotor->Motor_State_Flag.bit.motor_run_flag = 0U;
    }
    
    // 执行当前状态对应的函数
    Motor_Flow_Function[pMotor->Motor_Flow](pMotor);
}
```

### 5.2 状态转换图

```
┌──────────────────────────────────────────────────────────────┐
│                        主状态机流程                            │
└──────────────────────────────────────────────────────────────┘

         ┌─────────┐
         │   PRE   │ 准备阶段
         └────┬────┘
              │ 启动命令
              ▼
         ┌─────────┐
         │  INIT   │ 初始化：偏置校准
         └────┬────┘
              │ 校准成功
              ▼
         ┌─────────┐
         │  IDLE   │ 空闲：飞启动检测
         └────┬────┘
              │
    ┌─────────┼─────────┐
    │ 成功    │         │ 失败
    ▼         │         ▼
┌─────────┐   │    ┌─────────┐
│  RUN    │   │    │  BOOT   │ 启动：强制旋转
└─────────┘   │    └────┬────┘
              │         │ 检测到反电动势
              │         ▼
              │    ┌─────────┐
              │    │POSITION │ 定位：确定初始位置
              │    └────┬────┘
              │         │ 定位成功
              │         ▼
              │    ┌─────────┐
              └───▶│  RUN    │ 运行：闭环换相
                   └────┬────┘
                        │ 停止命令
                        ▼
                   ┌─────────┐
                   │  BRAKE  │ 刹车
                   └────┬────┘
                        │ 刹车完成
                        ▼
                   ┌─────────┐
                   │   PRE   │ 返回准备状态
                   └─────────┘

**故障恢复路径**：任何运行状态（RUN）下检测到堵转（STALL）或过流等严重故障时，自动跳转：
- RUN → ERROR_STATE（立即禁止PWM输出）→ BRAKE（强制刹车至停止）→ PRE（等待清除故障标志）
- 堵转条件：反电动势幅值 < 阈值 且 持续超过设定时间窗口
```

### 5.3 换相状态机

```c
// 换相方法函数指针数组
pMOTOR_FUN Motor_Math_Function[SWITCH_CMP+1] = 
{
    MotorTask_Switch_Flux_Flow,   // 磁链法
    MotorTask_Switch_Bemf_Flow,   // 反电动势法
    MotorTask_Switch_Cmp_Flow     // 比较器法
};

// 换相状态机执行
void MotorTask_Run_Flow(ST_MOTOR_TASK* pMotor)
{
    // 1. 读取ADC值
    pMotor->MCSQ_CTRL.MCSQ_BLDC._V_Q12U_Bemf_ADC_tmp[0] = ADC_DATA_READ_U_BEMF;
    pMotor->MCSQ_CTRL.MCSQ_BLDC._V_Q12U_Bemf_ADC_tmp[1] = ADC_DATA_READ_V_BEMF;
    pMotor->MCSQ_CTRL.MCSQ_BLDC._V_Q12U_Bemf_ADC_tmp[2] = ADC_DATA_READ_W_BEMF;
    pMotor->MCSQ_CTRL.MCSQ_BLDC._V_Q12U_Iphase_ADC = ADC_DATA_READ_CURRENT;
    
    // 2. 根据换相流程状态执行
    if(pMotor->MCSQ_CTRL.MCSQ_BLDC.SQ_Flow == SQUARE_DIAG_ING)
    {
        // 诊断中：检测反电动势状态
        MCSQ_DIAG(&pMotor->MCSQ_CTRL.MCSQ_DIAG, &pMotor->MCSQ_CTRL.MCSQ_BLDC);
    }
    else if(pMotor->MCSQ_CTRL.MCSQ_BLDC.SQ_Flow == SQUARE_CROSS_ING)
    {
        // 换相中：执行换相算法
        Motor_Math_Function[pMotor->MCSQ_CTRL.MCSQ_BLDC.SW_Math](pMotor);
    }
    
    // 3. 执行控制环
    // 速度环、电流环、PWM更新
}
```

---

## 6. 使用示例

### 6.1 初始化示例

```c
// 1. 定义电机任务结构体
ST_MOTOR_TASK Motor;

// 2. 系统初始化
void System_Init(void)
{
    // MCU初始化
    SystemClock_Config();
    
    // BSP初始化
    BSP_CLK_Init();
    BSP_GPIO_Init();
    BSP_PWM_Init();
    BSP_ADC_Init();
    BSP_TIM_Init();
    
    // 电机控制初始化
    MCSQ_Init(&Motor.MCSQ_CTRL);
    
    // 设置初始状态
    Motor.Motor_Flow = MOTOR_STATE_PRE;
    Motor.Motor_State_Flag.bit.motor_run_flag = 0U;
}

// 3. 主循环
void main(void)
{
    System_Init();
    
    while(1)
    {
        // 等待ADC采样完成中断
        // 在中断中调用 MotorTask_Current_Flow(&Motor);
    }
}
```

### 6.2 启动电机示例

```c
// 启动电机
void Motor_Start(ST_MOTOR_TASK* pMotor)
{
    // 1. 清除错误标志
    MC_Error_Clear(&pMotor->MC_ERR);
    
    // 2. 设置运行标志
    pMotor->Motor_State_Flag.bit.motor_run_flag = 1U;
    
    // 3. 设置目标速度（如果启用速度闭环）
    pMotor->Motor_State_Flag.bit.motor_speed_flag = 1U;
    Motor_Set_Target_Speed(pMotor, 3000);  // 3000 rpm
    
    // 4. 设置电流限制（如果启用电流闭环）
    pMotor->Motor_State_Flag.bit.motor_busA_flag = 1U;
}

// 停止电机
void Motor_Stop(ST_MOTOR_TASK* pMotor)
{
    // 1. 清除运行标志
    pMotor->Motor_State_Flag.bit.motor_run_flag = 0U;
    
    // 2. 进入刹车状态
    pMotor->Motor_Flow = MOTOR_STATE_BRAKE;
}
```

### 6.3 速度控制示例

```c
// 设置目标速度
void Motor_Set_Target_Speed(ST_MOTOR_TASK* pMotor, Q32I_ Speed)
{
    // 将速度转换为标幺值
    Q32U_ speed_pu = (Q32U_)(Speed * MOTOR_Q14_PU / MOTOR_MAX_SPEED_RPM);
    
    // 设置速度环给定
    pMotor->MCSQ_CTRL.Ramp_Freq.Q14I_Target = speed_pu;
}

// 读取当前速度
Q32U_ Motor_Read_Speed(ST_MOTOR_TASK* pMotor)
{
    // 从滤波后的频率计算速度
    Q32U_ freq_pu = pMotor->MCSQ_CTRL.MCSQ_BLDC.FL_Freq.Q14I_LPF_Out;
    
    // 转换为RPM
    Q32U_ speed_rpm = (freq_pu * MOTOR_MAX_SPEED_RPM) / MOTOR_Q14_PU;
    
    return speed_rpm;
}
```

### 6.4 ADC中断处理示例

```c
// ADC采样完成中断
void ADC_IRQHandler(void)
{
    // 1. 清除中断标志
    ADC_ClearFlag(ADC_FLAG_EOC);
    
    // 2. 执行电机控制任务
    MotorTask_Current_Flow(&Motor);
    
    // 3. 执行速度环任务（较低频率）
    static uint8_t speed_loop_cnt = 0;
    speed_loop_cnt++;
    if (speed_loop_cnt >= 10) {  // 速度环频率 = 电流环频率 / 10
        speed_loop_cnt = 0;
        MotorTask_Speed_Flow(&Motor);
    }
}
```

---

## 总结

### MCSQ模块核心要点

1. **模块化设计**：核心算法、应用接口、任务调度分离
2. **状态机驱动**：清晰的状态转换逻辑
3. **多种换相方法**：磁链法、反电动势法、比较器法
4. **智能切换**：低速磁链法，高速反电动势法
5. **完整控制链**：速度环、电流环、PWM控制
6. **定点实现**：适合低端MCU

### 下一步学习

- [六步换相控制-检验题目](六步换相控制-检验题目.md)：检验学习成果
- [MC-LIB-Porting-Guide](MC-LIB-Porting-Guide.md)：学习如何移植到自己的项目

---

## 🆚 与 hpm_MC 方波控制对比

| 维度 | MC_LIB 六步换相 | hpm_MCL |
|------|---------------|---------|
| **换相方法** | 磁链法/反电动势法/比较器法 三选一 | v1: `hpm_block` Hall换相 + `hpm_over_zero` 过零检测；v2: `mcl_mode_block` |
| **智能切换** | 低速磁链法→高速反电动势法自动切换 | v1 over_zero 有 FSM 状态机（init→location→running） |
| **启动方式** | 定位启动/飞启动 | v1 block: Hall信号直接换相; over_zero: 定位→开环拖动→过零切换 |
| **速度闭环** | PID速度闭环 | v1: `BLDC_CONTRL_SPD_PARA` PID配置 |
| **适用电机** | BLDC（梯形波反电动势） | BLDC（梯形波反电动势），v2 同时支持步进电机 |

---

*文档更新时间: 2026-04-27*
