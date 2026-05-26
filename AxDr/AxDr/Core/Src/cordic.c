/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cordic.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;
DMA_HandleTypeDef hdma_cordic_read;
DMA_HandleTypeDef hdma_cordic_write;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CORDIC DMA Init */
    /* CORDIC_READ Init */
    hdma_cordic_read.Instance = DMA1_Channel2;
    hdma_cordic_read.Init.Request = DMA_REQUEST_CORDIC_READ;
    hdma_cordic_read.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cordic_read.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_read.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_read.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_read.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_read.Init.Mode = DMA_NORMAL;
    hdma_cordic_read.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_cordic_read) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaOut,hdma_cordic_read);

    /* CORDIC_WRITE Init */
    hdma_cordic_write.Instance = DMA1_Channel3;
    hdma_cordic_write.Init.Request = DMA_REQUEST_CORDIC_WRITE;
    hdma_cordic_write.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cordic_write.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_write.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_write.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_write.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_write.Init.Mode = DMA_NORMAL;
    hdma_cordic_write.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_cordic_write) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaIn,hdma_cordic_write);

  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC DMA DeInit */
    HAL_DMA_DeInit(cordicHandle->hdmaOut);
    HAL_DMA_DeInit(cordicHandle->hdmaIn);
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

CORDIC_ConfigTypeDef sCordicConfig;

void cordic_config(void)
{
	sCordicConfig.Function         = CORDIC_FUNCTION_SINE;     /* sine function */
	sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES; /* max precision for q1.31 sine */
 	sCordicConfig.Scale            = CORDIC_SCALE_0;           /* no scale */
 	sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
 	sCordicConfig.NbRead           = CORDIC_NBREAD_2;          /* One output data: sine */
 	sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;     /* q1.31 format for input data */
  sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;    /* q1.31 format for output data */
	
	HAL_CORDIC_Configure(&hcordic, &sCordicConfig);
}
int32_t value_to_cordic31(float value, float coeff)
{
	int32_t cordic31;
	cordic31 = (int32_t)((value/coeff)*0x80000000);		//value对coeff归一化，然后扩大2^31倍，取整得到Q31定点数据
	return cordic31;
}
void cordic31_to_value(int cordic31, float *res)
{
	if(cordic31&0x80000000)			//为负数
	{
		cordic31 = cordic31&0x7fffffff;
		*res = ((float)(cordic31)-0x80000000)/0x80000000;
	}
	else							//为正数
	{
		*res = (float)(cordic31)/0x80000000;
	}
}

int32_t pInBuff;			//输入数据
int32_t pOutBuff[2];		//输出数据
void cordic_calculate_start(float arg1)			
{
	//传入的是归一化角度[0, 1.0]
	if(arg1 > 0.5f)								//将角度从[0, 1.0]转换到[-0.5, 0.5]
		arg1 -= 1.0f;
	pInBuff = value_to_cordic31(arg1, 0.5f);	//[-0.5, 0.5]对0.5f归一化，转换到[-1.0, 1.0]，并用Q31定点数据表示
	
	//传入的是弧度值[0, 6.28]
	/*if(arg1 > 3.14f)							//将角度从[0, 6.28]转换到[-3.14, 3.14]
		arg1 -= 6.28f;
	pInBuff = value_to_cordic31(arg1, 3.14f);	//[-3.14, 3.14]对3.14f归一化，转换到[-1.0, 1.0]，并用Q31定点数据表示
	*/
	
	HAL_CORDIC_Calculate_DMA(&hcordic, &pInBuff, pOutBuff, 1, CORDIC_DMA_DIR_IN_OUT);			//开始转换
}
void cordic_get_result(float *res1, float *res2)
{	
	while (HAL_CORDIC_GetState(&hcordic) != HAL_CORDIC_STATE_READY);	//等待cordic模块空闲

	cordic31_to_value(pOutBuff[0], res1);								//正弦值转浮点
	cordic31_to_value(pOutBuff[1], res2);								//余弦值转浮点
}


/* USER CODE END 1 */
