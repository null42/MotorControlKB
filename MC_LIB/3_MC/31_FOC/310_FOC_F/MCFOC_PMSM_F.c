/**************************************************************************************************
*     File Name :                        MCFOC_PMSM_F.c
*     Library/Module Name :              MCFOC_F
*     Author :                           CJYS
*     Create Date :                      2024/1/1
*     Abstract Description :             FOC核心模块源文件 - 坐标变换与参数自适应
**************************************************************************************************/

#include "MCFOC_PMSM_F.h"


/**********************************************************************************************
Function: MCFOC_PMSM_Para_Init_F
Description: PMSM电气量参数初始化
             初始化角度补偿三角函数结构体，设置初始补偿角度为0
Input: 无
Output: 无
Input_Output: PMSM电气量结构体指针
Return: 无
Author: CJYS

详细说明:
    该函数在系统初始化时调用，用于初始化PMSM电气量结构体中的角度补偿三角函数。
    初始状态下，补偿角度为0，即sin(0)=0, cos(0)=1。
    这意味着初始时不进行角度补偿。
***********************************************************************************************/
void MCFOC_PMSM_Para_Init_F(ST_PMSM_ELEC_F* pPMSMe)
{
    pPMSMe->TG_Triangle_Comp.F_Angle = 0.0f;
    pPMSMe->TG_Triangle_Comp.F_Cos = 1.0f;
    pPMSMe->TG_Triangle_Comp.F_Sin = 0.0f;
    pPMSMe->TG_Triangle_Comp.F_ReAngle = 0.0f;
}

/**********************************************************************************************
Function: MCFOC_PMSM_Para_Adapt_F
Description: PMSM参数自适应
             根据运行状态自适应调整电机参数和控制参数
Input: 无
Output: 无
Input_Output: PMSM滤波器指针，PMSM电气量指针，PMSM参数指针
Return: 无
Author: CJYS

详细说明:
    该函数在速度环周期调用（通常1ms），用于：
    1. 计算电气量平均值（频率、电压、电流）
    2. 计算调制深度
    3. PWM频率自适应（低速时降低PWM频率）
    4. 电感饱和补偿（根据电流调整Lq）
    5. 计算预测角度（用于逆Park变换）
***********************************************************************************************/
void MCFOC_PMSM_Para_Adapt_F(ST_PMSM_FILTER_F* pPMSMf, ST_PMSM_ELEC_F* pPMSMe, ST_PMSM_PARA_F* pPMSMa)
{
    float F_UsMax_tmp = 0.0f;
    
    // ================== 步骤1：计算电气量平均值 ==================
    // 从滤波器获取平均频率
    pPMSMe->_O_F_Freq = pPMSMf->Mean_Freq.F_MEAN_Out;
    // 从滤波器获取平均母线电压
    pPMSMe->_O_F_Vbus = pPMSMf->Mean_Vbus.F_MEAN_Out;
    // 计算定子电流幅值: Is = sqrt(Id^2 + Iq^2)
    pPMSMe->_O_F_Is = MATH_SQADD_F(pPMSMf->Mean_Id.F_MEAN_Out, pPMSMf->Mean_Iq.F_MEAN_Out);
    // 计算定子电压幅值: Us = sqrt(Ud^2 + Uq^2)
    pPMSMe->_O_F_Us = MATH_SQADD_F(pPMSMf->Mean_Ud.F_MEAN_Out, pPMSMf->Mean_Uq.F_MEAN_Out);
    // 获取平均反电动势
    pPMSMe->_O_F_Es = pPMSMf->Mean_Es.F_MEAN_Out;
    
    // ================== 步骤2：计算调制深度 ==================
    // 最大电压 = 母线电压 × 调制模式系数
    // 七段式调制: 系数 = 1.0
    // 五段式调制: 系数 = 0.866
    F_UsMax_tmp = pPMSMe->_O_F_Vbus*pPMSMe->_P_F_Modulation_Mode;
    if(F_UsMax_tmp != 0.0f)
    {
        // 计算 1/Vbus，用于后续SVPWM计算
        pPMSMe->_O_F_One_Over_Vbus = 1.0f/pPMSMe->_O_F_Vbus;
        // 计算调制深度 = Us / UsMax
        pPMSMe->_O_F_Modulation_Rate = pPMSMe->_O_F_Us/F_UsMax_tmp;
    }
    // 计算参考电压，用于弱磁控制
    pPMSMe->_O_F_UsRef = F_UsMax_tmp*pPMSMe->_P_F_UsRef_Scale;
    
    // ================== 步骤3：长时间平均值计算 ==================
    // 10ms母线电流平均，用于功率计算
    pPMSMf->Mean_Ibus_10ms.F_MEAN_In = pPMSMf->Mean_Ibus.F_MEAN_Out;
    // 1s定子电流平均，用于电感饱和补偿
    pPMSMf->Mean_Is_1000ms.F_MEAN_In = pPMSMe->_O_F_Is;
    MEAN_Cal_F(&pPMSMf->Mean_Ibus_10ms);
    MEAN_Cal_F(&pPMSMf->Mean_Is_1000ms);
    
    pPMSMe->_O_F_Ibus_10ms = pPMSMf->Mean_Ibus_10ms.F_MEAN_Out;
    pPMSMe->_O_F_Is_1000ms = pPMSMf->Mean_Is_1000ms.F_MEAN_Out;

    // ================== 步骤4：PWM频率自适应 ==================
    // 低速时降低PWM频率，减少开关损耗
    // 使能检测
    pPMSMa->PWM_FREQ_CHECK.Check_Flag.bit.Enable = 1U;
    // 设置条件：频率低于设置阈值
    pPMSMa->PWM_FREQ_CHECK.Check_Flag.bit.Condition_Set = (pPMSMe->_O_F_Freq <= pPMSMa->PWM_FREQ_CHECK._P_F_Check_TL);
    // 清除条件：频率高于清除阈值
    pPMSMa->PWM_FREQ_CHECK.Check_Flag.bit.Condition_Clear = (pPMSMe->_O_F_Freq >= pPMSMa->PWM_FREQ_CHECK._P_F_Clear_TL);
    // 计算低速PWM标志
    pPMSMa->_O_Q32U_Low_PWM_Flag = Check_Cal(&pPMSMa->PWM_FREQ_CHECK, pPMSMa->_O_Q32U_Low_PWM_Flag);

    // 根据低速标志设置PWM频率目标值
    if(pPMSMa->_O_Q32U_Low_PWM_Flag)
    {
        pPMSMa->Ramp_PWM_FREQ.F_Target = pPMSMa->_P_F_PWM_FREQ_MIN;  // 低速PWM频率
    }
    else
    {
        pPMSMa->Ramp_PWM_FREQ.F_Target = pPMSMa->_P_F_PWM_FREQ_MAX;  // 高速PWM频率
    }
    // 斜坡变化，避免频率突变
    Ramp_Cal_F(&pPMSMa->Ramp_PWM_FREQ);
    // 计算PWM频率系数
    pPMSMa->_O_F_PWM_Freq_Coeff = pPMSMa->Ramp_PWM_FREQ.F_Output/pPMSMa->_P_F_PWM_FREQ_MAX;
    // 计算PWM周期系数
    pPMSMa->_O_F_PWM_Period_Coeff = 1.0f/pPMSMa->_O_F_PWM_Freq_Coeff;
    

    // ================== 步骤5：电机参数更新 ==================
    // 定子电阻
    pPMSMa->_O_F_Rs = pPMSMa->_P_F_Rs;
    // d轴电感
    pPMSMa->_O_F_Ld = pPMSMa->_P_F_Ld;
    // q轴电感（含饱和补偿）
    // 电感会随电流增大而饱和，通过查表获得饱和系数
    pPMSMa->_O_F_Lq = pPMSMa->_P_F_Lq*TABLE_1D_Inter_F(&pPMSMa->TAB_Lq_Coeff, pPMSMe->_O_F_Is_1000ms);
    // 定子电感（平均值）
    pPMSMa->_O_F_Ls = pPMSMa->_P_F_Ls;
    // 永磁体磁链
    pPMSMa->_O_F_Flux = pPMSMa->_P_F_Flux;
    // 采样周期（考虑PWM频率变化）
    pPMSMa->_O_F_Ts = pPMSMa->_P_F_Ts*pPMSMa->_O_F_PWM_Period_Coeff;
    
    // ================== 步骤6：计算预测角度 ==================
    // 预测角度用于逆Park变换，补偿计算延迟
    // 预测角度 = 频率 × 采样周期 × 预测周期数
    pPMSMe->TG_Triangle_Pre.F_Angle = pPMSMe->_P_F_Pre_Period*pPMSMe->_O_F_Freq*pPMSMa->_O_F_Ts;
    // 角度归一化到 [0, 2π]
    MATH_ANGLE_MOD_F(pPMSMe->TG_Triangle_Pre.F_Angle);
    // 计算sin和cos值
    Math_SinCos_F(&pPMSMe->TG_Triangle_Pre);
}


/*********************************坐标变换*************************************/

/**********************************************************************************************
Function: MCFOC_Clark_F
Description: Clarke变换函数
             将三相静止坐标系(ABC)变换为两相静止坐标系(αβ)
Input: 无
Output: 无
Input_Output: PMSM电气量结构体指针
Return: 无
Author: CJYS

数学原理:
    Clarke变换公式:
    Iα = Ia
    Iβ = (Ib - Ic) / √3
    
    利用三相平衡条件 Ia + Ib + Ic = 0，可以简化计算
    
几何意义:
    A相绕组与α轴重合
    B相绕组超前α轴120°
    C相绕组滞后α轴120°
    
调用时机:
    在电流环周期调用（通常100μs），在ADC采样完成后
***********************************************************************************************/
void MCFOC_Clark_F(ST_PMSM_ELEC_F* pPMSMe)
{
    // α轴电流 = a相电流
    // 因为A相绕组与α轴重合
    pPMSMe->_V_F_Ialfa = pPMSMe->_V_F_Ia;
    
    // β轴电流 = (Ib - Ic) / √3
    // 利用三相平衡条件简化计算
    // MATH_ONE_OVER_SQRT_THREE_F = 1/√3 ≈ 0.57735026919
    pPMSMe->_V_F_Ibeta = MATH_ONE_OVER_SQRT_THREE_F*(pPMSMe->_V_F_Ib - pPMSMe->_V_F_Ic);
}

/**********************************************************************************************
Function: MCFOC_Park_F
Description: Park变换函数
             将两相静止坐标系(αβ)变换为两相旋转坐标系(dq)
Input: 无
Output: 无
Input_Output: PMSM电气量结构体指针
Return: 无
Author: CJYS

数学原理:
    Park变换公式:
    Id = Iα·cos(θ) + Iβ·sin(θ)
    Iq = -Iα·sin(θ) + Iβ·cos(θ)
    
    其中θ为转子电角度
    
角度补偿:
    实际角度 = 估计角度 + 补偿角度
    使用和角公式计算sin和cos:
    sin(θ_real) = sin(θ_est)·cos(θ_comp) + cos(θ_est)·sin(θ_comp)
    cos(θ_real) = cos(θ_est)·cos(θ_comp) - sin(θ_est)·sin(θ_comp)
    
调用时机:
    在电流环周期调用（通常100μs），在Clarke变换后
***********************************************************************************************/
void MCFOC_Park_F(ST_PMSM_ELEC_F* pPMSMe)
{
    // ================== 步骤1：角度补偿计算 ==================
    // 使用和角公式计算补偿后的sin和cos
    // sin(θ_real) = sin(θ_est)·cos(θ_comp) + cos(θ_est)·sin(θ_comp)
    pPMSMe->_V_F_Sin_Real = pPMSMe->TG_Triangle_Est.F_Sin*pPMSMe->TG_Triangle_Comp.F_Cos
                          + pPMSMe->TG_Triangle_Est.F_Cos*pPMSMe->TG_Triangle_Comp.F_Sin;
    // cos(θ_real) = cos(θ_est)·cos(θ_comp) - sin(θ_est)·sin(θ_comp)
    pPMSMe->_V_F_Cos_Real = pPMSMe->TG_Triangle_Est.F_Cos*pPMSMe->TG_Triangle_Comp.F_Cos
                          - pPMSMe->TG_Triangle_Est.F_Sin*pPMSMe->TG_Triangle_Comp.F_Sin;
    
    // ================== 步骤2：Park变换 ==================
    // Id = Iα·cos(θ_real) + Iβ·sin(θ_real)
    pPMSMe->_V_F_Id_Real =   pPMSMe->_V_F_Ialfa*pPMSMe->_V_F_Cos_Real
                           + pPMSMe->_V_F_Ibeta*pPMSMe->_V_F_Sin_Real;
    // Iq = -Iα·sin(θ_real) + Iβ·cos(θ_real)
    pPMSMe->_V_F_Iq_Real = - pPMSMe->_V_F_Ialfa*pPMSMe->_V_F_Sin_Real
                           + pPMSMe->_V_F_Ibeta*pPMSMe->_V_F_Cos_Real;
}

/**********************************************************************************************
Function: MCFOC_Ipark_F
Description: 逆Park变换函数
             将两相旋转坐标系(dq)变换为两相静止坐标系(αβ)
Input: 无
Output: 无
Input_Output: PMSM电气量结构体指针
Return: 无
Author: CJYS

数学原理:
    逆Park变换公式:
    Uα = Ud·cos(θ) - Uq·sin(θ)
    Uβ = Ud·sin(θ) + Uq·cos(θ)
    
预测角度:
    由于计算出的电压将在下一个PWM周期输出
    需要预测下一周期的转子位置
    使用和角公式计算预测角度的sin和cos
    
调用时机:
    在电流环周期调用（通常100μs），在电流环PI控制后
***********************************************************************************************/
void MCFOC_Ipark_F(ST_PMSM_ELEC_F* pPMSMe)
{
    // ================== 步骤1：预测角度计算 ==================
    // 使用和角公式计算预测角度的sin和cos
    // sin(θ_pre) = sin(θ_real)·cos(θ_predict) + cos(θ_real)·sin(θ_predict)
    pPMSMe->_V_F_Sin_Pre = pPMSMe->_V_F_Sin_Real*pPMSMe->TG_Triangle_Pre.F_Cos
                         + pPMSMe->_V_F_Cos_Real*pPMSMe->TG_Triangle_Pre.F_Sin;
    // cos(θ_pre) = cos(θ_real)·cos(θ_predict) - sin(θ_real)·sin(θ_predict)
    pPMSMe->_V_F_Cos_Pre = pPMSMe->_V_F_Cos_Real*pPMSMe->TG_Triangle_Pre.F_Cos
                         - pPMSMe->_V_F_Sin_Real*pPMSMe->TG_Triangle_Pre.F_Sin;
    
    // ================== 步骤2：逆Park变换 ==================
    // Uα = Ud·cos(θ_pre) - Uq·sin(θ_pre)
    pPMSMe->_V_F_Ualfa_Pre = pPMSMe->_V_F_Ud_Real*pPMSMe->_V_F_Cos_Pre
                           - pPMSMe->_V_F_Uq_Real*pPMSMe->_V_F_Sin_Pre;
    // Uβ = Ud·sin(θ_pre) + Uq·cos(θ_pre)
    pPMSMe->_V_F_Ubeta_Pre = pPMSMe->_V_F_Ud_Real*pPMSMe->_V_F_Sin_Pre
                           + pPMSMe->_V_F_Uq_Real*pPMSMe->_V_F_Cos_Pre;
}

/**********************************************************************************************
Function: MCFOC_Iclark_F
Description: 逆Clarke变换函数
             将两相静止坐标系(αβ)变换为三相静止坐标系(ABC)
Input: 无
Output: 无
Input_Output: PMSM电气量结构体指针
Return: 无
Author: CJYS

数学原理:
    逆Clarke变换公式:
    Ia = Iα
    Ib = -0.5·Iα + (√3/2)·Iβ
    Ic = -0.5·Iα - (√3/2)·Iβ
    
用途:
    用于预测三相电流，进行死区补偿和电流重构
    
调用时机:
    在需要预测三相电流时调用
***********************************************************************************************/
void MCFOC_Iclark_F(ST_PMSM_ELEC_F* pPMSMe)
{
    float F_Ialfa_Pre = 0.0f, F_Ibeta_Pre = 0.0f;
     
    // ================== 步骤1：逆Park变换（dq → αβ） ==================
    // Iα = Id·cos(θ_pre) - Iq·sin(θ_pre)
    F_Ialfa_Pre = pPMSMe->_V_F_Id_Real*pPMSMe->_V_F_Cos_Pre
                - pPMSMe->_V_F_Iq_Real*pPMSMe->_V_F_Sin_Pre;
    // Iβ = Id·sin(θ_pre) + Iq·cos(θ_pre)
    F_Ibeta_Pre = pPMSMe->_V_F_Id_Real*pPMSMe->_V_F_Sin_Pre
                + pPMSMe->_V_F_Iq_Real*pPMSMe->_V_F_Cos_Pre;
    
    // ================== 步骤2：逆Clarke变换（αβ → ABC） ==================
    // Ia = Iα
    pPMSMe->_V_F_Ia_Pre = F_Ialfa_Pre;
    // Ib = -0.5·Iα + (√3/2)·Iβ
    pPMSMe->_V_F_Ib_Pre = 0.5f*( - F_Ialfa_Pre + MATH_SQRT_THREE_F*F_Ibeta_Pre);
    // Ic = -0.5·Iα - (√3/2)·Iβ
    pPMSMe->_V_F_Ic_Pre = 0.5f*( - F_Ialfa_Pre - MATH_SQRT_THREE_F*F_Ibeta_Pre);
}
