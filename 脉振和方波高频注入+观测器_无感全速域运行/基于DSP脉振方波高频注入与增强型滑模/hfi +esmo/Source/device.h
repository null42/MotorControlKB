/*
 * device.h
 *
 *  Created on: 2016Äê4ÔÂ21ÈÕ
 *      Author: mingdong.li
 */

#ifndef DEVICE_H_
#define DEVICE_H_

#include "communication.h"

#define SVPWMTBPRD			3750


extern Uint32 EPwm1TimerIntCount;


void GpioRegs_Init(void);
void AdcRegs_Init(void);
void EpwmReg_Init(void);

#endif /* DEVICE_H_ */
