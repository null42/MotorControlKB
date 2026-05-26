/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "FOC_Control.h"
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN Private defines */
#define PWMZD_Count		100							//有感速度环计算频率，1K
#define TIM1_Frq			20000						//定时器1的中断频率
#define TS 						0.00005f 				//SVPWM的方波周期
#define Encoder_Fre		12000						// TIM1_Frq/PWMZD_Count*60

#define Encoder_lines	4096
#define Encoder_Pulse	16384			//Encoder_lines*4
#define TIM3_Pulse		16384			//定时器3重装载值

#define RGB_Luminance		3				//RGB LED亮度调节，一共8档(0~7)

extern s8 			encoder_overflow_count;
extern u16 			Speed_Clock;
extern float 		NTC_Temp;
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void LED_RGB_Init(void);
void LED_RGB_Color(u8 R, u8 G, u8 B);
void Motor_Angle_Get(void);							//计算电机电角度
void Motor_Speed_Get(void);							//计算电机速度
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

