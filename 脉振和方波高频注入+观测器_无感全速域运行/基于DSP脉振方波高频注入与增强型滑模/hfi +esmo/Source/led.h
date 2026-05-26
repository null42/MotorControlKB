/*
 * led.h
 *
 *  Created on: 2016Äê11ÔÂ20ÈÕ
 *      Author: mingdong.li
 */

#ifndef LED_H_
#define LED_H_

static inline void Led_On(void)
{
	GpioDataRegs.GPACLEAR.bit.GPIO15	= 1;
}
static inline void Led_Off(void)
{
	GpioDataRegs.GPASET.bit.GPIO15		= 1;
}
typedef struct {
	 Uint16 LedTime;
	 Uint16 LedRrrorTime;
}LedParameterMode;

extern LedParameterMode LedParameter;

void LedInit(void);
void LedCtrl(enum STATEMACHINE state,LedParameterMode *p,Uint16 Error);


#endif /* LED_H_ */
