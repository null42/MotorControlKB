#ifndef __HFI_H
#define __HFI_H

#include "delay.h"
#include "stdint.h"
#include "stdbool.h"

#define LPF_HFI_A		0.011f			//低通滤波系数，截止频率300Hz
#define LPF_HFI_B		0.989f

#define LPF_HFI_I_A	0.9424778f	//截止频率3KHz
#define LPF_HFI_I_B	0.0575222f

#define HFI_Id_offset		2.0f		//HFI初始位置检测时，注入的Id电流偏置

typedef struct
{
    float theta;							//HFI观测的角度
    float theta_err;					//HFI观测的角度误差
    float Ialpha_last;				//上一次采样的α轴电流
    float Ibeta_last;					//上一次采样的β轴电流
    float ialpha_h;						//α轴高频电流分量
    float ibeta_h;						//β轴高频电流分量
    float ialpha_h_last;			//上一次的α轴高频电流分量
    float ibeta_h_last;				//上一次的β轴高频电流分量
    float ialpha_f;						//α轴低频电流分量
    float ibeta_f;						//β轴低频电流分量
    float Ialpha_h;						//经过包络检测后的α轴高频电流分量
    float Ibeta_h;						//经过包络检测后的β轴高频电流分量
    float Idf;								//d轴低频电流
    float Iqf;								//q轴低频电流
    float Ids;
    float Iqs;
    float Iq_last;						//q轴上次采样电流
    float Id_last;						//d轴上次采样电流
    float Idh;								//d轴高频响应电流
    s8 		Uin;								//d轴注入的高频电压
    s8 		SIGN;								//符号函数后的结果
    float theta_Init;					//初始角度
} HFI_Cal, * p_HFI;

#define HFI_DEFAULTS		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			//状态参数初始化

typedef struct
{
    float 	Kp;										//锁相环PI控制器Kp
    double 	Ki;										//锁相环PI控制器Ki
    float 	Theta_Err;						//角度误差
    float 	Theta_Err_last;				//上一次角度误差
    float 	Omega;								//电角速度
    float 	Omega_F;							//滤波后的电角速度
    float 	Theta;								//锁相环计算得到的电角度
    float 	Angle;								//经过角度补偿的电角度
} HFI_PLL, * p_HFI_PLL;

#define HFI_PLL_DEFAULTS {0,0,0,0,0,0,0,0};														//初始化参数

extern bool 	HFI_Uin_flag;

extern HFI_Cal			HFI;

void HFI_Angle_Cale(p_HFI pv);				//HFI角度识别
void HFI_Injection(void);							//HFI测试，注入高频方波脉冲
void HFI_Init_Theta(void);						//HFI初始角度识别
void HFI_Speed_Closeloop(void);				//HFI速度闭环
void HFI_Observer(float Id, float Iq);//高频方波注入观测器
void extract_Idq_f(void);							//提取低频电流分量

#endif

