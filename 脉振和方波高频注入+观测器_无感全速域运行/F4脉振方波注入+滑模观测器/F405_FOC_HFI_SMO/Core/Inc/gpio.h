/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "delay.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED1	PCout(13)
#define LED2	PCout(14)
#define LED3	PCout(15)
#define LED4	PAout(15)

#define KEY1		PDin(2)
#define KEY2		PCin(11)
#define KEY3		PBin(10)
#define KEY4		PBin(12)
#define	WK_UP		PAin(0)

#define KEY1_PRES		1
#define KEY2_PRES		2
#define	KEY3_PRES		3
#define KEY4_PRES		4
#define WKUP_PRES		5

#define BEEP	PCout(9)

//-----------------OLED端口定义----------------
#define OLED_CS   	GPIO_PIN_5   //片选信号           PB5->CS
#define OLED_DC   	GPIO_PIN_4   //数据/命令控制信号  PB4->DC
#define OLED_RST  	GPIO_PIN_3   //复位信号           PB3->RES

//-----------------OLED端口操作定义---------------- 
#define OLED_CS_Clr()  	HAL_GPIO_WritePin(GPIOB,OLED_CS,GPIO_PIN_RESET)
#define OLED_CS_Set()  	HAL_GPIO_WritePin(GPIOB,OLED_CS,GPIO_PIN_SET)

#define OLED_DC_Clr()  	HAL_GPIO_WritePin(GPIOB,OLED_DC,GPIO_PIN_RESET)
#define OLED_DC_Set()  	HAL_GPIO_WritePin(GPIOB,OLED_DC,GPIO_PIN_SET)
 					   
#define OLED_RST_Clr()  HAL_GPIO_WritePin(GPIOB,OLED_RST,GPIO_PIN_RESET)
#define OLED_RST_Set()  HAL_GPIO_WritePin(GPIOB,OLED_RST,GPIO_PIN_SET)

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
u8 KEY_Scan(u8 mode);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

