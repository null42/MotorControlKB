/**********************************
慧驱动
视频教程详见QQ空间或B站
其他事宜详见readme.txt
QQ：1323279545
微信：huiqudong
官网：www.huiqudong.cn
**********************************/
#include "main.h"
#include "adc.h"



int32_t ia_test, ib_test, ic_test;
//float err;

uint16_t D_direction = 0;
uint8_t test;
float Ia, Ib, Ic;
float Ia_test, Ib_test, Ic_test;
float Vbus;
uint16_t ADC1ConvertedValue[5];
uint16_t i = 0;
uint32_t A_offset, B_offset;
uint8_t get_offset_flag = 0;
uint8_t get_offset_sample_cnt = 0;
u8 oled_display_sample_freq = 0;
u8 speed_close_loop_flag;
float Iq_ref;
float EKF_Hz;

uint8_t hfi_to_flux_ob_flag = 0;
//float theta_add;
//float theta;

extern float Rs;
extern float Ls;
extern float flux;

void get_offset(uint32_t* a_offset, uint32_t* b_offset)
{
    if(get_offset_sample_cnt < 128)
    {
        *a_offset += ADC1->JDR2;
        *b_offset += ADC1->JDR3;
        get_offset_sample_cnt++;
    }
    else
    {
        *a_offset >>= 7;
        *b_offset >>= 7;
        get_offset_sample_cnt = 0;
        TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
        get_offset_flag = 2;
    }
}
void motor_run(void)
{
    float vbus_temp;
    float ia_temp, ib_temp;
    vbus_temp = (float)(ADC1->JDR1);                                //得到母线电压 adc转换值
    ia_temp = (float)((int16_t)A_offset - (int16_t)ADC1->JDR2);   //得到A相电流 adc转换值
    ib_temp = (float)((int16_t)B_offset - (int16_t)ADC1->JDR3);   //得到B相电流 adc转换值
    Vbus = vbus_temp * VBUS_CONVERSION_FACTOR;                      //通过电压转换因子（通过分压电阻得到）把adc转换值 转化为 真实电压
    Ia = ia_temp * SAMPLE_CURR_CON_FACTOR;                          //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
    Ib = ib_temp * SAMPLE_CURR_CON_FACTOR;                          //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
    Ic = -Ia - Ib;                                                  //基于基尔霍夫电流定律，根据AB相电流去计算C相电流

    Encoder_Process();

    if(hfi_to_flux_ob_flag == 0)
    {
        FOC_Input.theta = HFI_Par.hfi_theta;
        if(d_direction_ok == 1)   //极性辨识成功
        {
            FOC_Input.Iq_ref = 0.3f;  //增加饱和
            if((HFI_Par.hfi_we > HFI_TO_FLUX_DOWN_FLAG) && (HFI_Par.hfi_we < HFI_TO_FLUX_UP_FLAG)) //注入后的速度在一个小区间内
            {
                if(hfi_to_flux_cnt > 100)
                {
                    hfi_to_flux_cnt = 0;
                    hfi_to_flux_ob_flag = 1; //持续一段时间便切换
                }
                hfi_to_flux_cnt++;
            }
        }
    }
    else
    {
        if(speed_close_loop_flag == 1) //已经速度闭环
        {
            FOC_Input.Id_ref = 0.0f;
            Speed_Fdk = Flux_Observer.lpf_we;
            FOC_Input.Iq_ref = Speed_Pid_Out;
            int_test1 = 5;
        }
        else     //未速度闭环，处于切换的中间态
        {
            FOC_Input.Id_ref = 0.0f;
            FOC_Input.Iq_ref = 0.5f;
            Speed_Pid.I_Sum = 0.5f;
            if(Flux_Observer.lpf_we > SPEED_CLOSE_FLAG) //观测器低通速度大于门槛，则切换
            {
                speed_close_loop_flag = 1;
            }
        }
        FOC_Input.theta = Flux_Observer.theta;
    }
    float_test3 = Ia;
    FOC_Input.Udc = Vbus;
    FOC_Input.ia = Ia;
    FOC_Input.ib = Ib;
    FOC_Input.ic = Ic;
    foc_algorithm_step();       //整个FOC运行函数（包括无感状态观测器，电流环，SVPWM，坐标变换，电机参数识别）

    if(motor_start_stop == 1)
    {
        PWM_TIM->CCR1 = (u16)(FOC_Output.Tcmp1);     //通过SVPWM得到的占空比赋值给定时器的寄存器
        PWM_TIM->CCR2 = (u16)(FOC_Output.Tcmp2);
        PWM_TIM->CCR3 = (u16)(FOC_Output.Tcmp3);
    }
    else
    {
        PWM_TIM->CCR1 = PWM_TIM_PULSE >> 1;
        PWM_TIM->CCR2 = PWM_TIM_PULSE >> 1;
        PWM_TIM->CCR3 = PWM_TIM_PULSE >> 1;
    }


    drv8301_protection();

    communication_task();

    oled_display_sample_freq++;
    if(oled_display_sample_freq == 10)         //电路板自带OLED显示屏 模拟示波器显示波形功能
    {
        if(display_data_flag == 0)
        {
            display_data_buff[display_data_buff_cnt] = (s8)(FOC_Output.EKF[3] * 15.0f);
            display_data_buff_cnt++;
            if(display_data_buff_cnt == 127)
            {
                display_data_buff_cnt = 0;
                display_data_flag = 1;
            }
        }
        oled_display_sample_freq = 0;
    }
}


void ADC_IRQHandler(void)
{

    if((SAMPLE_ADC->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
    {
        if(get_offset_flag == 2)
        {
            hall_angle += hall_angle_add;
            if(hall_angle < 0.0f)
            {
                hall_angle += 2.0f * PI;
            }
            else if(hall_angle > (2.0f * PI))
            {
                hall_angle -= 2.0f * PI;
            }
            motor_run();

        }
        else
        {
            if(get_offset_flag == 1)
            {
                get_offset(&A_offset, &B_offset);
            }
        }
        ADC_ClearFlag(SAMPLE_ADC, ADC_FLAG_JEOC);
    }
}
