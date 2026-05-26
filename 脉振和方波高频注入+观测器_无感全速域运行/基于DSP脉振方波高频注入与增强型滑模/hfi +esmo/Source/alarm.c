/*
 * alarm.c
 *
 *  Created on: 2 Apr 2018
 *      Author: LMD
 */
#include "DSP28x_Project.h"
#include "alarm.h"


volatile union FAULTMODE Fault;
volatile ALARMTIMERMODE AlarmTimer;

void Fault_init(void)
{
	int i;
	for(i=0;i<sizeof(AlarmTimer);i++)
	{
		((char*)&AlarmTimer)[i]			= 0;
	}

	Fault.all							= 0;

}

