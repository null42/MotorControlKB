#ifndef __FOC_CONTROL_H__
#define __FOC_CONTROL_H__

#include "foc_drv.h"
#include "foc_calc.h"

extern float theta;

void foc_speed_ctrl(uint8_t N);
void foc_ctrl_mode(void);
void motor_ctrl(void);
void foc_update_current_ctrl_gain(uint16_t bandwidth);



#endif /* __FOC_CONTROL_H__ */



