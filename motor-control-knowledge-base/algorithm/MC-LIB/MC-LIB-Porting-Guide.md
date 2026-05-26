# MC_LIB - 移植使用指南

**文档版本：** v1.0  
**生成日期：** 2026-04-26  
**适用对象：** 电机控制工程师、嵌入式开发者

---

## 目录

1. [移植概述](#1-移植概述)
2. [硬件配置](#2-硬件配置)
3. [BSP层移植](#3-bsp层移植)
4. [参数配置](#4-参数配置)
5. [初始化流程](#5-初始化流程)
6. [控制流程](#6-控制流程)
7. [调试指南](#7-调试指南)
8. [常见问题](#8-常见问题)

---

## 1. 移植概述

### 1.1 移植步骤

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          MC_LIB移植步骤                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  步骤1：硬件配置                                                         │
│  ├── 确定MCU型号                                                         │
│  ├── 确定采样方式（三电阻/单电阻）                                        │
│  └── 确定PWM频率和死区时间                                               │
│                                                                          │
│  步骤2：BSP层移植                                                        │
│  ├── 移植ADC驱动                                                         │
│  ├── 移植PWM驱动                                                         │
│  ├── 移植GPIO驱动                                                        │
│  └── 移植定时器驱动                                                      │
│                                                                          │
│  步骤3：参数配置                                                         │
│  ├── 配置电机参数（Rs, Ld, Lq, ψf）                                      │
│  ├── 配置控制参数（Kp, Ki）                                              │
│  └── 配置保护参数（过流、过压、欠压）                                     │
│                                                                          │
│  步骤4：初始化流程                                                       │
│  ├── 系统时钟初始化                                                      │
│  ├── 外设初始化                                                          │
│  └── 电机控制初始化                                                      │
│                                                                          │
│  步骤5：控制流程                                                         │
│  ├── 电流环中断                                                          │
│  ├── 速度环任务                                                          │
│  └── 状态机管理                                                          │
│                                                                          │
│  步骤6：调试验证                                                         │
│  ├── 开环测试                                                            │
│  ├── 闭环测试                                                            │
│  └── 性能优化                                                            │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.2 移植工作量估计

| 移植内容 | 工作量 | 难度 |
|---------|--------|------|
| BSP层移植 | 1~2天 | 中等 |
| 参数配置 | 0.5天 | 简单 |
| 初始化流程 | 0.5天 | 简单 |
| 控制流程 | 1天 | 中等 |
| 调试验证 | 2~5天 | 较高 |
| **总计** | **5~9天** | - |

---

## 2. 硬件配置

### 2.1 MCU选型要求

| 参数 | 最低要求 | 推荐配置 |
|------|---------|---------|
| CPU核心 | ARM Cortex-M3 | ARM Cortex-M4/M7 |
| 主频 | 72MHz | 168MHz以上 |
| Flash | 64KB | 256KB以上 |
| RAM | 16KB | 64KB以上 |
| FPU | 无 | 单精度浮点 |
| ADC | 2路12位 | 3路12位，1Msps |
| PWM | 6路互补 | 高级定时器 |
| DSP指令 | 无 | 有 |

### 2.2 硬件配置文件

```c
// HAL_CFG.h - 硬件配置
#ifndef HAL_CFG_H
#define HAL_CFG_H

// ================== MCU平台选择 ==================
#define MCU_STM32F4     1
#define MCU_RX32H6      2
#define MCU_Z20K14xM    3

#define CURRENT_MCU     MCU_STM32F4

// ================== PWM配置 ==================
#define PWM_FREQUENCY       10000       // PWM频率 10kHz
#define PWM_DEAD_TIME_NS    500         // 死区时间 500ns
#define PWM_PERIOD          (SystemCoreClock / PWM_FREQUENCY)

// ================== ADC配置 ==================
#define ADC_RESOLUTION      4096        // 12位ADC
#define ADC_VREF            3.3f        // 参考电压
#define ADC_SAMPLE_TIME     10          // 采样周期数

// ================== 电流采样配置 ==================
#define SHUNT_RESISTOR      0.01f       // 采样电阻 10mΩ
#define CURRENT_GAIN        20.0f       // 运放增益
#define CURRENT_OFFSET      1.65f       // 偏置电压

// ================== 电压采样配置 ==================
#define VOLTAGE_DIVIDER     0.01f       // 分压比
#define VOLTAGE_OFFSET      0.0f        // 偏置电压

// ================== 采样方式选择 ==================
#define SAMPLE_THREE_SHUNT  1           // 三电阻采样
#define SAMPLE_ONE_SHUNT    2           // 单电阻采样

#define CURRENT_SAMPLE_MODE SAMPLE_THREE_SHUNT

#endif
```

### 2.3 引脚配置

```c
// BSP_GPIO.h - 引脚配置
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

// ================== PWM引脚 ==================
#define PWM_UH_PIN      GPIO_PIN_8
#define PWM_UH_PORT     GPIOA
#define PWM_UL_PIN      GPIO_PIN_7
#define PWM_UL_PORT     GPIOB

#define PWM_VH_PIN      GPIO_PIN_9
#define PWM_VH_PORT     GPIOA
#define PWM_VL_PIN      GPIO_PIN_0
#define PWM_VL_PORT     GPIOB

#define PWM_WH_PIN      GPIO_PIN_10
#define PWM_WH_PORT     GPIOA
#define PWM_WL_PIN      GPIO_PIN_1
#define PWM_WL_PORT     GPIOB

// ================== ADC引脚 ==================
#define ADC_IU_PIN      GPIO_PIN_0
#define ADC_IU_PORT     GPIOC
#define ADC_IU_CHANNEL  ADC_CHANNEL_10

#define ADC_IV_PIN      GPIO_PIN_1
#define ADC_IV_PORT     GPIOC
#define ADC_IV_CHANNEL  ADC_CHANNEL_11

#define ADC_IW_PIN      GPIO_PIN_2
#define ADC_IW_PORT     GPIOC
#define ADC_IW_CHANNEL  ADC_CHANNEL_12

#define ADC_VBUS_PIN    GPIO_PIN_3
#define ADC_VBUS_PORT   GPIOC
#define ADC_VBUS_CHANNEL ADC_CHANNEL_13

// ================== 使能引脚 ==================
#define EN_PIN          GPIO_PIN_4
#define EN_PORT         GPIOC

// ================== 故障引脚 ==================
#define FAULT_PIN       GPIO_PIN_5
#define FAULT_PORT      GPIOC

#endif
```

---

## 3. BSP层移植

### 3.1 BSP层接口

```c
// BSP.h - BSP层统一接口
#ifndef BSP_H
#define BSP_H

// ================== 初始化函数 ==================
void BSP_CLK_Init(void);         // 时钟初始化
void BSP_GPIO_Init(void);        // GPIO初始化
void BSP_PWM_Init(void);         // PWM初始化
void BSP_ADC_Init(void);         // ADC初始化
void BSP_TIM_Init(void);         // 定时器初始化
void BSP_ISR_Init(void);         // 中断初始化

// ================== PWM控制函数 ==================
void BSP_PWM_Enable(void);       // PWM使能
void BSP_PWM_Disable(void);      // PWM禁止
void BSP_PWM_SetDuty(float duty_a, float duty_b, float duty_c);  // 设置占空比
uint32_t BSP_PWM_GetPeriod(void); // 获取PWM周期

// ================== ADC采样函数 ==================
void BSP_ADC_Start(void);        // 启动ADC采样
uint16_t BSP_ADC_GetIa(void);    // 获取a相电流ADC值
uint16_t BSP_ADC_GetIb(void);    // 获取b相电流ADC值
uint16_t BSP_ADC_GetIc(void);    // 获取c相电流ADC值
uint16_t BSP_ADC_GetVbus(void);  // 获取母线电压ADC值

// ================== GPIO控制函数 ==================
void BSP_EN_SetHigh(void);       // 使能信号高
void BSP_EN_SetLow(void);        // 使能信号低
uint8_t BSP_FAULT_Read(void);    // 读取故障信号

#endif
```

### 3.2 PWM驱动移植示例（STM32F4）

```c
// BSP_PWM.c - PWM驱动实现
#include "BSP.h"
#include "stm32f4xx.h"

void BSP_PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    // GPIO配置为复用功能
    GPIO_InitStructure.GPIO_Pin = PWM_UH_PIN | PWM_VH_PIN | PWM_WH_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = PWM_UL_PIN | PWM_VL_PIN | PWM_WL_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 设置复用功能
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM1);
    
    // 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD / 2;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    
    // 死区配置
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = (uint8_t)(PWM_DEAD_TIME_NS * SystemCoreClock / 1000000000);
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    
    // 使能预装载
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    
    // 使能定时器
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void BSP_PWM_SetDuty(float duty_a, float duty_b, float duty_c)
{
    uint32_t period = TIM1->ARR;
    
    // 限幅
    if(duty_a > 0.95f) duty_a = 0.95f;
    if(duty_a < 0.05f) duty_a = 0.05f;
    if(duty_b > 0.95f) duty_b = 0.95f;
    if(duty_b < 0.05f) duty_b = 0.05f;
    if(duty_c > 0.95f) duty_c = 0.95f;
    if(duty_c < 0.05f) duty_c = 0.05f;
    
    // 设置占空比
    TIM1->CCR1 = (uint32_t)(duty_a * period);
    TIM1->CCR2 = (uint32_t)(duty_b * period);
    TIM1->CCR3 = (uint32_t)(duty_c * period);
}

void BSP_PWM_Enable(void)
{
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void BSP_PWM_Disable(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}

uint32_t BSP_PWM_GetPeriod(void)
{
    return TIM1->ARR;
}
```

### 3.3 ADC驱动移植示例

```c
// BSP_ADC.c - ADC驱动实现
#include "BSP.h"
#include "stm32f4xx.h"

void BSP_ADC_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    // GPIO配置
    GPIO_InitStructure.GPIO_Pin = ADC_IU_PIN | ADC_IV_PIN | ADC_IW_PIN | ADC_VBUS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // ADC通用配置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    
    // ADC配置
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 4;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 配置通道
    ADC_RegularChannelConfig(ADC1, ADC_IU_CHANNEL, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_IV_CHANNEL, 2, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_IW_CHANNEL, 3, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_VBUS_CHANNEL, 4, ADC_SampleTime_15Cycles);
    
    // 使能ADC
    ADC_Cmd(ADC1, ENABLE);
}

static uint16_t adc_dma_buffer[4];

void BSP_ADC_Start(void)
{
    ADC_SoftwareStartConv(ADC1);
}

void BSP_ADC_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_dma_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    
    DMA_Cmd(DMA2_Stream0, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
}

uint16_t BSP_ADC_GetIa(void)
{
    return adc_dma_buffer[0];
}

uint16_t BSP_ADC_GetIb(void)
{
    return adc_dma_buffer[1];
}

uint16_t BSP_ADC_GetIc(void)
{
    return adc_dma_buffer[2];
}

uint16_t BSP_ADC_GetVbus(void)
{
    return adc_dma_buffer[3];
}
```

---

## 4. 参数配置

### 4.1 电机参数配置

```c
// PMSM_PARA.h - 电机参数配置
#ifndef PMSM_PARA_H
#define PMSM_PARA_H

// ================== 电机基本参数 ==================
#define MOTOR_POLE_PAIRS        4           // 极对数
#define MOTOR_RATED_SPEED       3000        // 额定转速 rpm
#define MOTOR_RATED_CURRENT     10.0f       // 额定电流 A
#define MOTOR_RATED_VOLTAGE     24.0f       // 额定电压 V

// ================== 电机电气参数 ==================
#define MOTOR_Rs                0.5f        // 定子电阻 Ω
#define MOTOR_Ld                0.001f      // d轴电感 H
#define MOTOR_Lq                0.001f      // q轴电感 H
#define MOTOR_Flux              0.01f       // 永磁体磁链 Wb

// ================== 控制参数 ==================
#define CURRENT_LOOP_FREQ       10000       // 电流环频率 Hz
#define SPEED_LOOP_FREQ         1000        // 速度环频率 Hz

// ================== 电流环参数 ==================
#define CURRENT_KP              1.0f        // 电流环Kp
#define CURRENT_KI              100.0f      // 电流环Ki
#define CURRENT_MAX             15.0f       // 最大电流 A

// ================== 速度环参数 ==================
#define SPEED_KP                1.0f        // 速度环Kp
#define SPEED_KI                10.0f       // 速度环Ki
#define SPEED_MAX               400.0f      // 最大速度 Hz

// ================== IF启动参数 ==================
#define IF_START_CURRENT        5.0f        // IF启动电流 A
#define IF_START_ACCEL          100.0f      // IF启动加速度 Hz/s
#define IF_SWITCH_SPEED         20.0f       // 切换速度 Hz

// ================== 定位参数 ==================
#define ALIGN_CURRENT           5.0f        // 定位电流 A
#define ALIGN_TIME              500         // 定位时间 ms

// ================== 保护参数 ==================
#define OVER_CURRENT_LIMIT      20.0f       // 过流保护阈值 A
#define OVER_VOLTAGE_LIMIT      30.0f       // 过压保护阈值 V
#define UNDER_VOLTAGE_LIMIT     18.0f       // 欠压保护阈值 V

#endif
```

### 4.2 参数初始化代码

```c
// MCFOC_PARA_F.c - 参数初始化
#include "MCFOC_PARA_F.h"
#include "PMSM_PARA.h"

void MCFOC_PMSM_Para_Init_F(ST_PMSM_PARA_F* pPMSMa)
{
    // 电机参数
    pPMSMa->_P_F_Rs = MOTOR_Rs;
    pPMSMa->_P_F_Ld = MOTOR_Ld;
    pPMSMa->_P_F_Lq = MOTOR_Lq;
    pPMSMa->_P_F_Flux = MOTOR_Flux;
    pPMSMa->_P_F_Ts = 1.0f / CURRENT_LOOP_FREQ;
    
    // 输出参数
    pPMSMa->_O_F_Rs = pPMSMa->_P_F_Rs;
    pPMSMa->_O_F_Ld = pPMSMa->_P_F_Ld;
    pPMSMa->_O_F_Lq = pPMSMa->_P_F_Lq;
    pPMSMa->_O_F_Ls = (pPMSMa->_P_F_Ld + pPMSMa->_P_F_Lq) / 2.0f;
    pPMSMa->_O_F_Flux = pPMSMa->_P_F_Flux;
    pPMSMa->_O_F_Ts = pPMSMa->_P_F_Ts;
}

void MCFOC_CurrentLoop_Init_F(ST_CURRENT_CONTROL_F* pCurrentCtrl)
{
    // d轴电流环
    pCurrentCtrl->PID_Id._P_F_Kp = CURRENT_KP;
    pCurrentCtrl->PID_Id._P_F_Ki = CURRENT_KI;
    pCurrentCtrl->PID_Id._P_F_OutMax = CURRENT_MAX * MOTOR_Rs;
    pCurrentCtrl->PID_Id._P_F_OutMin = -CURRENT_MAX * MOTOR_Rs;
    
    // q轴电流环
    pCurrentCtrl->PID_Iq._P_F_Kp = CURRENT_KP;
    pCurrentCtrl->PID_Iq._P_F_Ki = CURRENT_KI;
    pCurrentCtrl->PID_Iq._P_F_OutMax = CURRENT_MAX * MOTOR_Rs;
    pCurrentCtrl->PID_Iq._P_F_OutMin = -CURRENT_MAX * MOTOR_Rs;
    
    // 电流限幅
    pCurrentCtrl->_P_F_Imax = CURRENT_MAX;
    pCurrentCtrl->_P_F_Idmax = CURRENT_MAX;
    pCurrentCtrl->_P_F_Iqmax = CURRENT_MAX;
}

void MCFOC_SpeedLoop_Init_F(ST_FREQ_CONTROL_F* pFreqCtrl)
{
    // 速度环
    pFreqCtrl->PID_Speed._P_F_Kp = SPEED_KP;
    pFreqCtrl->PID_Speed._P_F_Ki = SPEED_KI;
    pFreqCtrl->PID_Speed._P_F_OutMax = CURRENT_MAX;
    pFreqCtrl->PID_Speed._P_F_OutMin = -CURRENT_MAX;
    
    // 速度限幅
    pFreqCtrl->_P_F_SpeedMax = SPEED_MAX;
    pFreqCtrl->_P_F_SpeedMin = -SPEED_MAX;
    
    // 速度斜坡
    pFreqCtrl->Ramp_Speed.F_Ramp = IF_START_ACCEL;
}
```

---

## 5. 初始化流程

### 5.1 系统初始化代码

```c
// Main.c - 主程序
#include "BSP.h"
#include "MCFOC_API_F.h"
#include "PMSM_PARA.h"

// 全局变量
ST_PMSM_ELEC_F      PMSM_Elec;
ST_PMSM_PARA_F      PMSM_Para;
ST_SVPWM_CONTROL_F  SVPWM_Ctrl;
ST_SMO_CONTROL_F    SMO_Ctrl;
ST_CURRENT_CONTROL_F Current_Ctrl;
ST_FREQ_CONTROL_F   Freq_Ctrl;
ST_IF_CONTROL_F     IF_Ctrl;
ST_ALIGN_CONTROL_F  ALIGN_Ctrl;

int main(void)
{
    // ================== 步骤1：系统时钟初始化 ==================
    SystemInit();
    BSP_CLK_Init();
    
    // ================== 步骤2：外设初始化 ==================
    BSP_GPIO_Init();
    BSP_PWM_Init();
    BSP_ADC_Init();
    BSP_TIM_Init();
    
    // ================== 步骤3：电机控制初始化 ==================
    MCFOC_PMSM_Para_Init_F(&PMSM_Para);
    memset(&PMSM_Elec, 0, sizeof(ST_PMSM_ELEC_F));
    MCFOC_SVPWM_Init_F(&SVPWM_Ctrl);
    MCFOC_EST_SMO_Init_F(&SMO_Ctrl);
    MCFOC_CurrentLoop_Init_F(&Current_Ctrl);
    MCFOC_SpeedLoop_Init_F(&Freq_Ctrl);
    MCFOC_IF_Init_F(&IF_Ctrl);
    MCFOC_ALIGN_Init_F(&ALIGN_Ctrl);
    
    // ================== 步骤4：中断初始化 ==================
    BSP_ISR_Init();
    
    // ================== 步骤5：使能PWM ==================
    BSP_PWM_Enable();
    
    // ================== 步骤6：主循环 ==================
    while(1)
    {
        // 状态机处理
        MCFOC_State_Machine_F();
        
        // 通信处理
        Communication_Process();
        
        // 故障检测
        Fault_Detection();
    }
}
```

### 5.2 中断服务程序

```c
// BSP_ISR.c - 中断服务程序
#include "BSP.h"
#include "MCFOC_API_F.h"

extern ST_PMSM_ELEC_F      PMSM_Elec;
extern ST_PMSM_PARA_F      PMSM_Para;
extern ST_SVPWM_CONTROL_F  SVPWM_Ctrl;
extern ST_SMO_CONTROL_F    SMO_Ctrl;
extern ST_CURRENT_CONTROL_F Current_Ctrl;
extern ST_FREQ_CONTROL_F   Freq_Ctrl;

// 电流环中断（10kHz）
void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        
        // ADC采样
        uint16_t adc_ia = BSP_ADC_GetIa();
        uint16_t adc_ib = BSP_ADC_GetIb();
        uint16_t adc_ic = BSP_ADC_GetIc();
        uint16_t adc_vbus = BSP_ADC_GetVbus();
        
        // ADC值转换为实际值
        float Ia = (adc_ia - ADC_OFFSET) * ADC_TO_CURRENT;
        float Ib = (adc_ib - ADC_OFFSET) * ADC_TO_CURRENT;
        float Ic = (adc_ic - ADC_OFFSET) * ADC_TO_CURRENT;
        float Vbus = adc_vbus * ADC_TO_VOLTAGE;
        
        // 更新电气量
        PMSM_Elec._V_F_Ia = Ia;
        PMSM_Elec._V_F_Ib = Ib;
        PMSM_Elec._V_F_Ic = Ic;
        PMSM_Elec._O_F_Vbus = Vbus;
        PMSM_Elec._O_F_One_Over_Vbus = 1.0f / Vbus;
        
        // 坐标变换
        MCFOC_Clark_F(&PMSM_Elec);
        MCFOC_Park_F(&PMSM_Elec);
        
        // 观测器
        MCFOC_EST_SMO_F(&SMO_Ctrl, &PMSM_Elec, &PMSM_Para);
        
        // 电流环
        MCFOC_CurrentLoop_F(&Current_Ctrl, &PMSM_Elec, &PMSM_Para);
        
        // 逆Park变换
        MCFOC_Ipark_F(&PMSM_Elec);
        
        // SVPWM
        MCFOC_SVPWM_ThreeShunt_F(&SVPWM_Ctrl, &PMSM_Elec);
        
        // 死区补偿
        MCFOC_DeadTime_COMP_F(&SVPWM_Ctrl, &PMSM_Elec);
        
        // PWM输出
        BSP_PWM_SetDuty(SVPWM_Ctrl._O_F_Dutya, 
                        SVPWM_Ctrl._O_F_Dutyb, 
                        SVPWM_Ctrl._O_F_Dutyc);
    }
}

// 速度环中断（1kHz）
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // 速度环
        MCFOC_SpeedLoop_F(&Freq_Ctrl, &PMSM_Elec, &PMSM_Para);
        
        // 参数自适应
        MCFOC_PMSM_Para_Adapt_F(&PMSM_Elec, &PMSM_Para);
        MCFOC_EST_SMO_Adapt_F(&SMO_Ctrl, &PMSM_Elec, &PMSM_Para);
    }
}
```

---

## 6. 控制流程

### 6.1 状态机设计

```c
// MCFOC_API_F.c - 状态机实现
typedef enum
{
    STATE_IDLE = 0,
    STATE_ALIGN,
    STATE_IF_START,
    STATE_CLOSED_LOOP,
    STATE_ERROR
}Motor_State_e;

Motor_State_e Motor_State = STATE_IDLE;

void MCFOC_State_Machine_F(void)
{
    switch(Motor_State)
    {
        case STATE_IDLE:
            // 空闲状态
            MCFOC_Idle_Process();
            break;
            
        case STATE_ALIGN:
            // 定位状态
            MCFOC_ALIGN_CurrentLoop_F(&ALIGN_Ctrl, &Current_Ctrl, &PMSM_Elec);
            if(ALIGN_Ctrl._O_Q32U_ALIGN_Done)
            {
                Motor_State = STATE_IF_START;
            }
            break;
            
        case STATE_IF_START:
            // IF启动状态
            MCFOC_IF_CurrentLoop_F(&IF_Ctrl, &Current_Ctrl, &PMSM_Elec, &PMSM_Para);
            if(IF_Ctrl._O_Q32U_Switch_Flag)
            {
                Motor_State = STATE_CLOSED_LOOP;
            }
            break;
            
        case STATE_CLOSED_LOOP:
            // 闭环状态
            Current_Ctrl._I_F_IqRef = Freq_Ctrl._O_F_IqRef;
            break;
            
        case STATE_ERROR:
            // 故障状态
            MCFOC_Error_Process();
            break;
            
        default:
            Motor_State = STATE_IDLE;
            break;
    }
}

void Motor_Start_F(void)
{
    if(Motor_State == STATE_IDLE)
    {
        Motor_State = STATE_ALIGN;
        ALIGN_Ctrl._O_Q32U_ALIGN_Done = 0;
    }
}

void Motor_Stop_F(void)
{
    Motor_State = STATE_IDLE;
    BSP_PWM_Disable();
}
```

---

## 7. 调试指南

### 7.1 调试步骤

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          调试步骤                                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  步骤1：硬件检查                                                         │
│  ├── 检查PWM输出波形（占空比、死区）                                      │
│  ├── 检查ADC采样值（偏置、量程）                                          │
│  └── 检查使能信号                                                         │
│                                                                          │
│  步骤2：开环测试                                                         │
│  ├── 定位测试：给定Id电流，观察转子是否对齐                               │
│  ├── IF启动测试：观察电机是否平稳加速                                     │
│  └── 检查电流波形                                                         │
│                                                                          │
│  步骤3：观测器调试                                                       │
│  ├── 检查观测器收敛性                                                     │
│  ├── 调整观测器增益                                                       │
│  └── 检查角度补偿                                                         │
│                                                                          │
│  步骤4：闭环调试                                                         │
│  ├── 切换到闭环状态                                                       │
│  ├── 调整电流环参数                                                       │
│  └── 调整速度环参数                                                       │
│                                                                          │
│  步骤5：性能优化                                                         │
│  ├── 优化动态响应                                                         │
│  ├── 优化稳态精度                                                         │
│  └── 优化效率                                                             │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 7.2 常见调试工具

| 工具 | 用途 |
|------|------|
| 示波器 | 观察PWM波形、电流波形 |
| 逻辑分析仪 | 分析通信协议、时序 |
| RTT调试 | 实时查看变量 |
| 上位机 | 参数调整、波形显示 |

---

## 8. 常见问题

### 8.1 电机无法启动

**可能原因：**
1. 电机参数错误
2. 电流采样偏置错误
3. PWM输出异常
4. 使能信号未打开

**解决方法：**
1. 检查电机参数配置
2. 校准电流采样偏置
3. 检查PWM输出波形
4. 检查使能信号

### 8.2 电机抖动

**可能原因：**
1. 观测器参数不当
2. 电流环参数不当
3. 角度补偿错误
4. 死区补偿错误

**解决方法：**
1. 调整观测器增益
2. 调整电流环Kp/Ki
3. 检查角度补偿表
4. 检查死区补偿

### 8.3 切换失败

**可能原因：**
1. 切换速度阈值过低
2. 观测器未收敛
3. 电流给定突变

**解决方法：**
1. 提高切换速度阈值
2. 增加IF启动时间
3. 平滑切换电流给定

---

## 总结

### 移植要点

1. **BSP层移植**：实现PWM、ADC、GPIO驱动
2. **参数配置**：正确配置电机参数和控制参数
3. **初始化流程**：按顺序初始化各模块
4. **控制流程**：正确实现状态机和中断服务程序
5. **调试验证**：逐步验证各功能模块

### 关键配置项

| 配置项 | 说明 |
|--------|------|
| PWM_FREQUENCY | PWM频率 |
| PWM_DEAD_TIME_NS | 死区时间 |
| MOTOR_Rs | 定子电阻 |
| MOTOR_Ld/Lq | d/q轴电感 |
| CURRENT_KP/KI | 电流环参数 |
| SPEED_KP/KI | 速度环参数 |

### 相关文档

- [MC-LIB-Architecture](MC-LIB-Architecture.md)
- [MC-LIB-FOC-Core](MC-LIB-FOC-Core.md)
- [MC-LIB-SVPWM](MC-LIB-SVPWM.md)
- [MC-LIB-Observer](MC-LIB-Observer.md)
- [MC-LIB-Control-Loop](MC-LIB-Control-Loop.md)

---

## 🆚 hpm_MC 平台移植要点

hpm_MCL 是 HPM 平台专用 SDK，与 MC_LIB 的多平台策略不同：

| 移植方向 | 要点 |
|---------|------|
| **MC_LIB → hpm_MCL** | 核心算法（Clarke/Park/PID/SVPWM）数学原理相同，主要差异在 API 风格（函数式→聚合体）和初始化方式（逐步→一次聚合）。移植重点在应用层适配 `mcl_loop_t` 聚合体 |
| **hpm_MCL → 其他MCU** | hpm_MCL 深度依赖 VSC/CLC/QEO 硬件加速器，移植到其他平台需用软件实现替代这三者的功能 |
| **hpm_MCL v1 → v2** | API 重构：`hpm_foc_*` 函数式 → `mcl_control_method_t` 函数指针表。运行模式从二选一(foc/block)变为6种模式。建议新项目直接用 v2 |

**关键差异**:
- 硬件加速: HPM VSC（Clarke/Park）→ STM32 DSP指令手动实现; CLC（PID）→ 软件PID; QEO（Ipark/SVPWM）→ 软件实现
- 编码器: hpm_MCL 内置 ABZ/UVW 编码器驱动抽象，MC_LIB 需 BSP 层手动处理

---

*文档更新时间: 2026-04-26*
