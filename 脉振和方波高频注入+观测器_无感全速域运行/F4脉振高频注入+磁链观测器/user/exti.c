/**********************************
             慧驱动
        视频教程详见QQ空间或B站
        其他事宜详见readme.txt
          QQ：1323279545
         微信：huiqudong
        官网：www.huiqudong.cn
**********************************/
#include "main.h"
#include "exti.h"


//u8 usb_open_flag=0;

u8 key1_flag;
u8 key2_flag;
u8 key3_flag;
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(KEY_3_EXTI_LINE) != RESET)
  {
    key3_flag=1;
    EXTI_ClearITPendingBit(KEY_3_EXTI_LINE);
  }
}


void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(KEY_2_EXTI_LINE) != RESET)
  {
    key2_flag=1;
    EXTI_ClearITPendingBit(KEY_2_EXTI_LINE);
  }
}




void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(KEY_1_EXTI_LINE) != RESET)
  {
    
    key1_flag=1;
    EXTI_ClearITPendingBit(KEY_1_EXTI_LINE);
  }
}

void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    //key2_flag=1;
    ENCODER_TIM->CNT = 0;
    EXTI_ClearITPendingBit(ENCODER_Z_EXTI_LINE);
  }
}
