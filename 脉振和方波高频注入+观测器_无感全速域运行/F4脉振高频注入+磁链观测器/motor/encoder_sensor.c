/**********************************
             慧驱动
        视频教程详见QQ空间或B站
        其他事宜详见readme.txt
          QQ：1323279545
         微信：huiqudong
        官网：www.huiqudong.cn
**********************************/
#include "main.h"
#include "encoder_sensor.h"

float encoder_angle;


void Encoder_Process(void)
{
    s32 temp;
    temp = ENCODER_TIM->CNT;
    encoder_angle = (float)(temp) * (float)(MOTOR_POLE_PAIRS) * PI / 2000.0f;
    encoder_angle += ENCODER_ANGLE_OFFSET;
    if(encoder_angle < 0.0f)
    {
        encoder_angle += 2.0f * PI;
    }
    else if(encoder_angle > (2.0f * PI))
    {
        encoder_angle -= 2.0f * PI;
    }

}
