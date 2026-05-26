/*
 * control.c
 *
 *  Created on: 6 Apr 2018
 *      Author: LMD
 */

#include "control.h"


#define TS                       _IQ(1.0)

const char Sanqu_List[7] = {0,1,5,0,3,2,4};

volatile SvpwmParameterModel SvpwmParameter;
volatile FocdqParameterModel FocIoutParameter;
PidRegMode    anxid_reg;
PidRegMode    anxiq_reg;
PidRegMode    spd_reg;
SensorlessVarsMode SensorlessVars;

volatile _iq TempCos,TempSin;


void ControlParameter_Init(void)
{
	int i;
	for(i=0;i<sizeof(SvpwmParameter);i++)
	{
		((char*)&SvpwmParameter)[i]		= 0;
	}
	for(i=0;i<sizeof(FocIoutParameter);i++)
	{
		((char*)&FocIoutParameter)[i]	= 0;
	}
	for(i=0;i<sizeof(anxid_reg);i++)
	{
		((char*)&anxid_reg)[i]			= 0;
	}
	for(i=0;i<sizeof(anxiq_reg);i++)
	{
		((char*)&anxiq_reg)[i]			= 0;
	}
	for(i=0;i<sizeof(spd_reg);i++)
	{
		((char*)&spd_reg)[i]			= 0;
	}
	for(i=0;i<sizeof(SensorlessVars);i++)
	{
		((char*)&SensorlessVars)[i]		= 0;
	}

	spd_reg.Kp                          = SPD_KP;  // Kp * 100
	spd_reg.Ki                          = SPD_KI;
	spd_reg.OutMax                      = _IQ(12.0/64*100);                  // Iout  * 100
	spd_reg.OutMin                      = _IQ(-12.0/64*100);                 // Iout  * 100

	anxid_reg.Kp						= (long)(6.28/1024*64*W_Hz_ILOOP*L__H_MOTOR);
	anxid_reg.Ki						= _IQ(1.0/ (I_MS_ILOOP * PERIORD_PWM));  //积分时间常数：0.3ms
	anxid_reg.OutMax					= _IQ(180.0/1024);
	anxid_reg.OutMin					= _IQ(-180.0/1024);

	anxiq_reg.Kp						= (long)(6.28/1024*64*W_Hz_ILOOP*L__H_MOTOR);
	anxiq_reg.Ki						= _IQ(1.0/ (I_MS_ILOOP * PERIORD_PWM));  //积分时间常数：0.3ms
	anxiq_reg.OutMax					= _IQ(180.0/1024);
	anxiq_reg.OutMin					= _IQ(-180.0/1024);
}

//#pragma CODE_SECTION(Svpwm_Fun,"ramfuncs");
void Svpwm_Fun(volatile SvpwmParameterModel *p)
{
	char a=0,b=0,c=0;
	_iq Alfa,Beta;
	_iq t1,t2,t0;
	char Sanqu;
	_iq Xt=0,Yt=0,Zt=0;
//	_iq TempCos,TempSin;
	_iq TempOne,TempTwo;
//	Alfa = cos(p->angle)*p->vd -sin(p->angle)*p->vq;
//	Beta = sin(p->angle)*p->vd +cos(p->angle)*p->vq;
//	TempCos = _IQcos(p->Angle);                          //COS和SIN详见ABC2DQ
//	TempSin = _IQsin(p->Angle);
#ifndef Svpwm_trigonometric
	TempCos	= _IQcos(p->angle);
	TempSin	= _IQsin(p->angle);
#endif

	Alfa = _IQmpy(TempCos,p->Vd) - _IQmpy(TempSin,p->Vq);
	Beta = _IQmpy(TempSin,p->Vd) + _IQmpy(TempCos,p->Vq);
	TempOne = _IQmpy(Alfa,_IQ(1.732051));
	if(Beta > 0)
	{
		a	= 1;
	}
	if(TempOne > Beta)
	{
		b	= 1;
	}
	if(-TempOne > Beta)
	{
		c	= 1;
	}
	a = 4*c + 2*b + a;
	Sanqu = Sanqu_List[a];
	/*Xt = sqrt(3)*Ts*be/Ud;*/
	Xt	= _IQdiv(_IQmpy(_IQ(1.732051),Beta),p->Vbus);
	/*Yt = 1.5*Ts*al/Ud+Xt/2;*/
	/*Zt = Xt/2 -1.5*Ts*al/Ud;*/
	TempTwo = _IQdiv(_IQmpy(_IQ(1.5),Alfa),p->Vbus);
	Yt	= TempTwo + Xt/2;
	Zt	= Xt/2 - TempTwo;
	switch(Sanqu)
	{
	   case 0:
		{
			t1 = -Zt;
			t2 = Xt;
			t0 = t1 + t2;
			if(t0 > TS)
			{
//				t1 = t1/t0*Ts;
//				t2 = t2/t0*Ts;
				t1	= _IQdiv(t1,t0);
				t2	= _IQdiv(t2,t0);
			}
			t0 = TS - t1 - t2;
			p->pwma = t0/4;
			p->pwmb = p->pwma + t1/2;
			p->pwmc = p->pwmb + t2/2;
		}
		break;
		case 1:
		{
			t1 = Zt;
			t2 = Yt;
			t0 = t1+t2;
			if(t0 > TS)
			{
				t1	= _IQdiv(t1,t0);
				t2	= _IQdiv(t2,t0);
			}
			t0 = TS - t1 - t2;
			p->pwmb = t0/4;
			p->pwma = p->pwmb + t1/2;
			p->pwmc = p->pwma + t2/2;
		}
		break;
	    case 2:
		{
			t1 = Xt;
			t2 = -Yt;
			t0 = t1+t2;
			if(t0 > TS)
			{
				t1	= _IQdiv(t1,t0);
				t2	= _IQdiv(t2,t0);
			}
			t0 = TS - t1 - t2;
			p->pwmb = t0/4;
			p->pwmc = p->pwmb + t1/2;
			p->pwma = p->pwmc + t2/2;
		}
		break;
		case 3:
		{
			t1 = -Xt;
			t2 = Zt;
			t0 = t1 + t2;
			if(t0 > TS)
			{
				t1	= _IQdiv(t1,t0);
				t2	= _IQdiv(t2,t0);
			}
			t0 = TS - t1 - t2;
			p->pwmc = t0/4;
			p->pwmb = p->pwmc + t1/2;
			p->pwma = p->pwmb + t2/2;
		}
		break;
	   case 4:
		{
			t1 = -Yt;
			t2 = -Zt;
			t0 = t1+t2;
			if(t0 > TS)
			{
				t1	= _IQdiv(t1,t0);
				t2	= _IQdiv(t2,t0);
			}
			t0 = TS - t1 - t2;
			p->pwmc = t0/4;
			p->pwma = p->pwmc + t1/2;
			p->pwmb = p->pwma + t2/2;
		}
		break;
		case 5:
		{
			t1 = Yt;
			t2 = -Xt;
			t0 = t1+t2;
			if(t0 > TS)
			{
				t1	= _IQdiv(t1,t0);
				t2	= _IQdiv(t2,t0);
			}
			t0 = TS - t1 - t2;
			p->pwma = t0/4;
			p->pwmc = p->pwma + t1/2;
			p->pwmb = p->pwmc + t2/2;
		}
		break;
		default:break;
	}
}


//#pragma CODE_SECTION(ABC2DQ_Fun,"ramfuncs");
void ABC2DQ_Fun(volatile FocdqParameterModel *p)
{
	_iq Alfa,Beta;

	Alfa	= _IQmpy((2*p->VA - p->VB - p->VC),_IQ(1.0/3));
	Beta	= _IQmpy((p->VB - p->VC),_IQ(0.577350269));
	TempCos	= _IQcos(p->angle);
	TempSin	= _IQsin(p->angle);
	p->Vd	= _IQmpy(TempCos,Alfa) + _IQmpy(TempSin,Beta);
	p->Vq	= _IQmpy(TempCos,Beta) - _IQmpy(TempSin,Alfa);
}




//#pragma CODE_SECTION(pid_reg_calc,"ramfuncs");
void pid_reg_calc(PidRegMode *v)  //电流环   速度环
{
    _iq  output_tmp;
    _iq  ui_tmp64;

    // error computation
    v->Err = v->Ref - v->Fdb;  //给定-反馈
    v->Up = _IQmpy(v->Kp,v->Err);   //                                   // Integral term computation
    ui_tmp64 = _IQmpy(v->Ki,v->Up); //   //sample_time/T/10000

    //output_tmp =v->Up+(v->Ui64>>32);
    output_tmp = v->Up + v->Ui64;

    if (output_tmp >= v->OutMax)
    {
        v->Out = v->OutMax;
    }
    else if (output_tmp <= v->OutMin)
    {
        v->Out = v->OutMin;
    }
    else
    {
        ui_tmp64 = v->Ui64 + ui_tmp64;

        v->Ui64  = _IQsat(ui_tmp64,v->OutMax,v->OutMin);

        output_tmp = v->Up + v->Ui64;

        v->Out = _IQsat(output_tmp,v->OutMax,v->OutMin);

    }
}



