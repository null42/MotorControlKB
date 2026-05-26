/*
 * control.h
 *
 *  Created on: 6 Apr 2018
 *      Author: LMD
 */

#ifndef SOURCE_CONTROL_H_
#define SOURCE_CONTROL_H_


#define Svpwm_trigonometric			//如果调用DQ变换，SVPWM可以和DQ变换共用三角函数

#define PI          3.141592654   //角度为Q24

#define SPD_RPM_TO_RAD          2 * 3.14 / 60

#define PERIORD_PWM        8                //PWM Periord


///////////////////////////速度环参数//////////////////////////////////////////////
#define SPD_KP              _IQ(6.28*0.0001*1024/64*1000)
#define SPD_KI              _IQ(1.0/2000)

///////////////////////////电流环参数//////////////////////////////////////////////
#define U_FFWD_TRQLOOP       0.7               //转矩环电压前馈系数
#define W_Hz_ILOOP           500.0             //电流环带宽 单位：Hz
#define I_MS_ILOOP           0.6              //电流环积分时间常数  单位：ms
#define SECTION_7MODE        0                //7段PWM
#define SECTION_5MODE        1                //5段PWM

#define YOUBENG

#ifdef  YOUBENG
/////////////////////////电机参数////////////////////////////
#define L__H_MOTOR           _IQ(0.024)            //电机电感值 单位:L
#define R_MOTOR              _IQ(1.2)            //电机电阻值 单位:欧姆
#define VPK_PERKRPM_MOTOR    _IQ(188/1.732/(1000 * SPD_RPM_TO_RAD))      //1000r/min时相电压峰值
#define Pole_MOTOR           5                //电机的极对数
#define INERTIA_MOTOR        0.000067           //电机的惯量  单位：kg*m^2
#define KT_MOTOR_RECIP             _IQ(1.0/((1.5 * 245 ) / (1300 * 2 * 3.14 / 60 )))    //((1.5 * 60 / 1.732 ) / (1000 * 2 * 3.14 / 60 ))   // Te = Iq * Kt  0.496
#endif

#ifdef  QIBENG
/////////////////////////电机参数////////////////////////////
#define L__H_MOTOR           _IQ(0.011)            //电机电感值 单位:L
#define R_MOTOR              _IQ(0.437)            //电机电阻值 单位:欧姆
#define VPK_PERKRPM_MOTOR    _IQ(103.3/1.732*1.414/(1000 * SPD_RPM_TO_RAD))      //1000r/min时相电压峰值
#define Pole_MOTOR           3                //电机的极对数
#define INERTIA_MOTOR        0.000067           //电机的惯量  单位：kg*m^2
#define KT_MOTOR_RECIP             _IQ(1.0/((1.5 * 245 ) / (1300 * 2 * 3.14 / 60 )))    //((1.5 * 60 / 1.732 ) / (1000 * 2 * 3.14 / 60 ))   // Te = Iq * Kt  0.496
#endif


typedef struct{
	_iq Vd;
	_iq Vq;
	_iq Vbus;
	_iq Angle;
	_iq pwma;
	_iq pwmb;
	_iq pwmc;
}SvpwmParameterModel;


typedef struct{
	_iq VA;
	_iq VB;
	_iq VC;
	_iq angle;
	_iq Vd;
	_iq Vq;
}FocdqParameterModel;


typedef struct
{
	_iq  Ref;
	_iq  Fdb;
	_iq  Err;
	_iq  Kp;
	_iq  Up;
	_iq  Ui;
	_iq  Out;
	_iq  Ki;
	_iq  integral_stop_flag;
	_iq  Ui64;
	_iq  OutMax;
	_iq  OutMin;
} PidRegMode;

typedef struct
{
	_iq IdsEst;
	_iq Ids;
	_iq IdsOld;
	_iq Vd;
	_iq VdOld;
	_iq ErrIdlpf;
	_iq IqsEst;
	_iq Iqs;
	_iq IqsOld;
	_iq Vq;
	_iq VqOld;
	_iq ErrIqlpf;
	_iq Ts;
	_iq MotorR;
	_iq MotorL;
	int32 pole;
	_iq WsEst;
	_iq ErrILpfCoef;
	_iq EmfEst;
	_iq Kf;
	_iq WsCoef;
	_iq beta;
	_iq WsEstLpfCoef;
	_iq WsEstlpf;
	_iq Theta;
	_iq ThetaOut;
	_iq Test;
} SensorlessVarsMode;




inline float SGN(float Value)
{
	return ( (Value > 0)? 1 : -1 );

}

#define SAT(A, Pos, Neg)        (((A) > (Pos)) ?(Pos) : (((A) < (Neg)) ? (Neg) : (A)))


extern volatile SvpwmParameterModel SvpwmParameter;
extern volatile FocdqParameterModel FocIoutParameter;
extern PidRegMode    anxid_reg;
extern PidRegMode    anxiq_reg;
extern PidRegMode    spd_reg;
extern SensorlessVarsMode SensorlessVars;


void ControlParameter_Init(void);
void Svpwm_Fun(volatile SvpwmParameterModel *p);
void ABC2DQ_Fun(volatile FocdqParameterModel *p);
void VoltageCompensation_Fun(void);
void SensorlessCtrl(SensorlessVarsMode * p);
void pid_reg_calc(PidRegMode *v);



#endif /* SOURCE_CONTROL_H_ */
