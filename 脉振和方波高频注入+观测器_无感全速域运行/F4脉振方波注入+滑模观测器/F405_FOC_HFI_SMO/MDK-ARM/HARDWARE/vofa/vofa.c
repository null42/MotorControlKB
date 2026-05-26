#include "vofa.h"

FloatToChar fc;

const uint8_t tail [4] = {0x00, 0x00, 0x80, 0x7f};	//JustFloat规定{0x00, 0x00, 0x80, 0x7f}为一组数据的结束
uint8_t c_data[4];
/*JustFloat*/
//定义一个联合体，将传入的浮点型参数存入到传入的unsigned char数组里
void float_turn_u8(float f,uint8_t * c)
{
	uint8_t x;
	FloatLongType data;
	data.fdata = f;
	
	for(x=0; x<4; x++)
	{
		c[x] = (uint8_t)(data.ldata>>(x*8));
	}
}

/*

*JustFloat:
*fdata:传入数组指针
*fdata_num:数组数据个数
*Usart_choose:发送的串口地址

前两个for将 float类型转换位四位unsigned char类型并发送出去，第三个for循环将尾部发出去
*/

void JustFloat_Send(float *fdata, uint16_t fdata_num, USART_TypeDef *Usart_choose)
{
	uint16_t 	x;
	uint8_t 	y;
	for(x=0; x<fdata_num; x++)
	{
		float_turn_u8(fdata[x], c_data);
		for(y=0; y<4; y++)
		{
			Usart_choose->DR = c_data[y];
			while((Usart_choose->SR & 0X40) == 0);
		}
	}
	
	for(y=0;y<4;y++)
	{
		Usart_choose->DR = tail[y];
		while((Usart_choose->SR & 0X40) == 0);
	}
}


/*
 * 根据串口信息进行PID调参
 */
void VOFA_USART_PI_Adjust(uint8_t *DataBuff ,uint8_t on)
{
	memcpy(&fc.c_data, DataBuff, 4);															//遵循IEEE 754协议，将数据前四位赋予共用体

	if(on)																												//是否开启调试
	{
		if(DataBuff[6] == 0x01) 																		//速度环Kp
			pi_spd.Kp = fc.f_data;
		else if(DataBuff[6] == 0x02) 																//电流环Iq_Kp
			pi_iq.Kp = fc.f_data;
		else if(DataBuff[6] == 0x03) 																//电流环Id_Kp
			pi_id.Kp = fc.f_data;
		else if(DataBuff[6] == 0x04) 																//速度环Ki
			pi_spd.Ki = fc.f_data;
		else if(DataBuff[6] == 0x05) 																//电流环Iq_Ki
			pi_iq.Ki = fc.f_data;
		else if(DataBuff[6] == 0x06) 																//电流环Id_Ki
			pi_id.Ki = fc.f_data;
		else if(DataBuff[6] == 0x07) 																//目标速度
			Sensorless.Speed_Max = fc.f_data;
		else if(DataBuff[6] == 0x08)																//启动
		{
			switch((u8)fc.f_data)
			{
				case 0:
				{
					Motor.M_State = 0;
					LED1 = 1;
					LED2 = 1;
					LED3 = 1;
					LED4 = 1;
					break;
				}
				case 1:
				{
					M_RUN();								//启动PWM
					Motor.M_State = 1;			//有感，速度&电流闭环
					LED1 = !LED1;
					break;
				}
				case 2:
				{
					M_RUN();								//启动PWM
					Motor.M_State = 2;			//IF启动
					LED2 = !LED2;
					break;
				}
				case 5:
				{
					PLL_HFI_Para.Theta = 0;
					M_RUN();									//启动PWM
					Motor.M_State = 5;				//HFI模式
					LED3 = !LED3;
					break;
				}
				case 6:
				{
					M_RUN();									//启动PWM
					IF_Theta = 0;
					Motor.M_State = 6;				//低速HFI，中高速SMO模式
					LED4 = !LED4;
					break;
				}
				case 3:
				{
					Motor.M_State = 3;
					Postion_To_Zero_Double();		//二次调零
					Motor.M_State = 0;
					break;
				}
				default: break;
			}
		}
	}
}

