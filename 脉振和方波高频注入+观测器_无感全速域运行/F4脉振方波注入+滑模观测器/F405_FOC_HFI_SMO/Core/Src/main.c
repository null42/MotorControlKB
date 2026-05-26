/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "delay.h"
#include "oled.h"
#include "gui.h"
#include "bmp.h"
#include "vofa.h"
#include "arm_math.h"
#include "FOC_Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
vu8  key_value_last;
vu8  key_value_now;

volatile bool K0_flag = 0;
volatile bool K1_flag = 0;
volatile bool K2_flag = 0;
volatile bool K3_flag = 0;
volatile bool K4_flag = 0;
volatile bool SMO_flag;
volatile bool SMO_Switch_flag;
volatile bool HFI_Init_Angle_flag = 0;
volatile bool Init_Over = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void check_key(void);
void screen_display_static(void);
void screen_display_dynamic(void);
void UART_RX_IRQ(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	delay_init(168);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	LED_RGB_Init();										//RGB_LED灯PWM初始化
	OLED_Init();				 							//初始化OLED屏幕
	Motor_Init();											//电机参数初始化
	GUI_DrawBMP(0,0,128,64,BMP2,1);		//显示图片
	delay_ms(500);
	HAL_ADC_Start_DMA(&hadc1,(u32 *)&ADC_Value,7);		//开启ADC-DMA转换
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);		//开启编码器使能函数
	
	ADC_Sample_Offset();							//零点校准
	OLED_Clear(0);										//屏幕清零
	
	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
 
		/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	screen_display_static();					//屏幕静态显示
	Init_Over = 1;
	SMO_flag = 1;
//	Motor.M_State = 6;
  while (1)
  {
    check_key();										//按键检测
		screen_display_dynamic();				//屏幕动态刷新
		Send_TO_PC();										//数据发送至上位机
		UART_RX_IRQ();									//串口接收完成，进行数据分析并赋值
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//按键检测
void check_key(void)
{
	key_value_last = key_value_now;
	key_value_now = KEY_Scan(0);
	if((key_value_now == 0xff) && (key_value_last != 0xff))
	{
		switch(key_value_last)
		{
			case 1:											//按键K1
			{
				K0_flag =	!K0_flag;
				if(K0_flag)
				{
					M_RUN();								//启动PWM
					Motor.M_State = 1;			//有感，速度&电流闭环
				}
				else
				{
					Motor.M_State = 0;
				}
				LED1 = !LED1;
				break;
			}
			case 2:											//按键K2
			{
				LED2 =	!LED2;
				K1_flag =	!K1_flag;
				
				switch(QT_IF_Mode)
				{
					case 1:
					{
						M_RUN();								//启动PWM
						Motor.M_State = 2;			//强拖至QT_Angle角度
						break;
					}
					case 2:
					{
						if(K1_flag)
						{
							M_RUN();								//启动PWM
							Motor.M_State = 2;			//强拖至QT_Angle角度
						}
						else
						{
							Motor.M_State = 0;			//停机
						}
						break;
					}
					case 3:
					{
						if(K1_flag)
						{
							M_RUN();								//启动PWM
							IF_Theta = 0;	//从270度开始拖动
							Motor.M_State = 2;			//IF启动
						}
						else
						{
							Motor.M_State = 0;			//停机
						}
						break;
					}
					default: break;
				}
				break;
			}
			case 3:											//按键K3
			{
				K2_flag =	!K2_flag;
				if(K2_flag)
				{
					PLL_HFI_Para.Theta = 0;
					M_RUN();									//启动PWM
					Motor.M_State = 5;				//HFI模式
				}
				else
				{
					Motor.M_State = 0;
				}
				LED3 =	!LED3;
				break;
			}
			case 4:											//按键K4
			{
				LED4 =	!LED4;
				K3_flag =	!K3_flag;
				if(K3_flag)
				{
					M_RUN();									//启动PWM
					IF_Theta = 0;
					Motor.M_State = 6;				//低速HFI，中高速SMO模式
				}
				else
				{
					Motor.M_State = 0;
				}
//				SMO_Switch_flag =! SMO_Switch_flag;
//				SMO_flag = !SMO_flag;
				break;
			}
			case 5:											//按键K0，WK_UP
			{
				K4_flag = !K4_flag;
				Motor.M_State = 3;
				Postion_To_Zero_Double();		//二次调零
				Motor.M_State = 0;
		
//				if(k4_flag)
//					Sensorless.Speed_Max = 200;
//				else
//					Sensorless.Speed_Max = 1000;
//				
//				LED2 =! LED2;
//				HFI_Speed = -200;
				break;
			}
			default: break;
		}
	}
}

//屏幕静态显示
void screen_display_static(void)
{
	GUI_ShowString(0, 0,  "BUS_Vol:", 8, 1);
	GUI_ShowString(0, 8,  "M_START:", 8, 1);	
	GUI_ShowString(0, 16,  "Id:", 8, 1);
	GUI_ShowString(0, 24, "Iq:", 8, 1);
	GUI_DrawLine(64, 16, 64, 32, 1);
	GUI_ShowCHinese(112, 16, 16, "℃",1);
//	GUI_ShowString(64, 24, "Temp:", 8, 1);
	GUI_ShowString(0, 32, "RPM_T:", 8, 1);
	GUI_ShowString(0, 40, "RPM_A:", 8, 1);
	GUI_ShowString(0, 48, "E_Theta:", 8, 1);
	GUI_ShowString(0, 56, "Speed_T:", 8, 1);
}

//屏幕动态显示
void screen_display_dynamic(void)
{
	static bool clear_flag;
	
	GUI_ShowNum_F(64, 0, ADC_Sample_F_Para.VBUS, 3, 2, 8, 1);
	GUI_ShowNum_F(20, 16, PARK_PCurr.Ds, 2, 2, 8, 1);
	GUI_ShowNum_F(20, 24, PARK_PCurr.Qs, 2, 2, 8, 1);
	GUI_ShowNum_F(40, 32, pi_spd.Ref/Motor.P, 5, 2, 8, 1);
	GUI_ShowNum_F(40, 40, pi_spd.Fbk/Motor.P, 5, 2, 8, 1);
//	GUI_ShowNum_F(40, 40,Motor.speed_M_rpm, 5, 2, 8, 1);
	GUI_ShowNum_F(64, 48, PARK_PCurr.Theta, 3, 2, 8, 1);
	GUI_ShowNum_Int(64, 56, Sensorless.Speed_Max, 5, 8, 1);
	
	if(Motor.M_State != M_ERR)											//状态异常报警
	{
		GUI_ShowNum(64, 8, Motor.M_State, 1, 8, 1);
		GUI_ShowString(72, 8, "        ", 8, 1);
	}
	else
	{
		GUI_ShowString(64, 8, "ERROR", 8, 1);
		GUI_ShowNum(112, 8, Fault_Code, 1, 8, 1);
	}
	
	if(NTC_ERROR != 0)
	{
		if(clear_flag == 0)
		{
			GUI_ShowString(68, 16, "     ", 16, 1);
			clear_flag = 1;
		}
		switch(NTC_ERROR)
		{
			case 1:
			{			
				GUI_ShowChar(80, 16, '<', 16, 1);
				GUI_ShowNum(96, 16, 25, 2, 16, 1);
				break;
			}
			case 2:
			{
				GUI_ShowChar(80, 16, '>', 16, 1);
				GUI_ShowNum(88, 16, 100, 3, 16, 1);
				break;
			}
			default: break;
		}
	}
	else
	{
		if(clear_flag == 1)
		{
			GUI_ShowString(68, 16, "     ", 16, 1);
			clear_flag = 0;
		}
		GUI_ShowNum_FU(68, 16, NTC_Temp, 3, 1, 16, 1);
	}
}

//串口调参
void UART_RX_IRQ(void)
{
	if(USART_RX_STA)
	{
//		printf("RXLen=%d\r\n",RxLine);
//		for(int i=0;i<RxLine;i++)
//			 printf("UART DataBuff[%d] = %x\r\n",i,USART_RX_BUF[i]);
		if(USART_RX_BUF[5] == 0xfb)											//校验位，检测接收到的数据是否正确
			VOFA_USART_PI_Adjust(USART_RX_BUF, 1);				//数据解析和参数赋值函数
		memset(USART_RX_BUF, 0, 8);  										//清空缓存数组
		RxLine = 0;  																		//清空接收长度
		USART_RX_STA = 0;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
