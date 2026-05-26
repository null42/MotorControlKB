/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "delay.h"
#include "Filter.h"
#include "tim.h"
#include "math.h"
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
#define LPF_I_STOP_A		0.00314159265f			//截止频率10Hz
#define LPF_I_STOP_B		0.99685840735f

#define LPF_I_RUN_A			0.37699f			//截止频率1200Hz
#define LPF_I_RUN_B			0.62301f
	
#define LPF_U_A					0.015465039f			//截止频率100Hz
#define LPF_U_B					0.984534961f	

#define I_Gain					16.66666666666667f	//电流放大系数
#define Vbus_Gain				16.0f								//母线电压采样电路放大倍数

#define Ts_VS						0.000025f

#define NTC_R0 				 	10000 											//10K, NTC电阻25℃的阻值
#define NTC_T0 				 	298.15f											//转换为开尔文温度 25+273.15f
#define NTC_B 				 	3455 												//B值
#define NTC_Ka  			 	273.15f 										//K值,开尔文和摄氏度之间的换算值
#define NTC_R1 					3300												//采样电路中分压电阻阻值
#define NTC_R2					974													//0.974K,NTC电阻100℃的阻值

typedef struct {
	float VBUS;		//母线电压
	float	Temperature;		//温度
	
	float PhaseV_Curr;		//V相电流
	float PhaseW_Curr;		//W相电流

	s16		Offset_PhaseV_Curr;		//V相电流偏置值
	s16 	Offset_PhaseW_Curr;		//W相电流偏置值
	} ADC_Sample;

#define ADC_Sample_DEFAULTS		{0,0,0,0,0,0}			//初始化参数

typedef struct {
	float VBUS;		//母线电压
	float Temperature;
	
	float PhaseV_Curr;
	float PhaseW_Curr;

	s16	  Offset_PhaseV_Curr;
	s16 	Offset_PhaseW_Curr;
	} ADC_Sample_F;

#define ADC_Sample_F_DEFAULTS		{0,0,0,0,0,0}				//初始化参数

typedef struct {
	float Yn;
	float Yn_last;
	float Xn;
	float Xn_last;
	float Tau;
	} V_Comp, *P_Vcomp;

#define Voltage_Compensation_DEFAULTS		{0,0,0,0,0}

extern u16	ADC_Value[7];	//ADC接收数组
extern u8 	NTC_ERROR;		//NTC温度报警

extern ADC_Sample			ADC_Sample_Para;
extern ADC_Sample_F		ADC_Sample_F_Para;

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
void ADC_Sample_Offset(void);										//校准零漂
void Get_ADC(void);															//获得滤波后采样数据
void temp_trans(u16 ADC_value, float* temp);		//NTC温度采样
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

