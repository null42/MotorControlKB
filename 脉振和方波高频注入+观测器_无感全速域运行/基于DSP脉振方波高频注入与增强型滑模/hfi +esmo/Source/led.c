/*
 * led.c
 *
 *  Created on: 2016Äê11ÔÂ21ÈÕ
 *      Author: mingdong.li
 */
#include "DSP28x_Project.h"
#include "application.h"
#include "led.h"

LedParameterMode LedParameter;
void LedInit(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO15		= 0;
	GpioCtrlRegs.GPADIR.bit.GPIO15		= 1;
	EDIS;
	Led_On();
	LedParameter.LedTime				= 0;
	LedParameter.LedRrrorTime			= 0;
}

void LedCtrl(enum STATEMACHINE state,LedParameterMode *p,Uint16 Error)
{
		switch(state)
		{
			case system_run:
			{
				if(p->LedTime>50)
				{
					Led_On();
				}
				else
				{
					Led_Off();
				}
				if(++p->LedTime>100)
				{
					p->LedTime	= 0;
				}
			}break;
			case system_alarm:
			{
				if(p->LedTime>200)
				{
					if(p->LedRrrorTime!=0)
					Led_On();
				}
				else
				{
					Led_Off();
				}
				if(++p->LedTime>300)
				{
					p->LedTime	= 0;
					if(++p->LedRrrorTime > Error)
					{
						p->LedRrrorTime	= 0;
					}
				}
			}break;
			default:
			{
				if(p->LedTime>200)
				{
					Led_On();
				}
				else
				{
					Led_Off();
				}
				if(++p->LedTime>400)
				{
					p->LedTime	= 0;
				}
			}break;
		}

}




