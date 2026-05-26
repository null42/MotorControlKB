#ifndef __OFFLINE_PARAMETER_IDENTIFICATION_H
#define __OFFLINE_PARAMETER_IDENTIFICATION_H

#include "delay.h"

#define LPF_Is_A		0.03045902795f			//低通滤波频率100Hz
#define LPF_Is_B		0.96954097205f

typedef struct
{
    float Is;
    float Rs;
    float Is_sqr;
    float Is_f;
} Rs_Cal, * P_Rs;

#define Rs_DEFAULTS {0,0,0};

typedef struct
{
    float Alpha;
    float Beta;
    float f;
    float Alpha_hpf;
    float Beta_hpf;
    float f_sq;
    float fcos2;
    float fsin2;
} Phi_Cal, * P_Phi;

#define PHI_DEFAULTS {0,0,0,0,0,0,0,0};

extern bool Rs_Idt_Over_flag;					//电阻识别完成标志位
extern bool Is_flag;

extern Phi_Cal Phi;
extern Rs_Cal  Rs_Idt;

void Rs_Identification(P_Rs pv, float Ialpha, float Ibeta);													//识别电机相电阻Rs
void Phi_Identification(P_Phi pv, float Ialpha, float Ibeta, float Ualpha, float Ubeta);			//识别永磁体磁链

#endif

