#include "foc_ctrl.h"
#include "foc_drv.h"
#include "vofa.h"
#include "encoder.h"
#include "tim.h"
#include "pid.h"

int32_t start_time;
int32_t stop_time;
int32_t run_time, runt_max;

float theta;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	TIM3->CNT = 0;
	start_time = TIM3->CNT;
	
	mc_adc.ia 	= ADC1->JDR3;
	mc_adc.ib 	= ADC1->JDR2;
	mc_adc.ic 	= ADC1->JDR1;
	mc_adc.vbus = ADC2->JDR1;
	
	foc.vbus = ((float)mc_adc.vbus * vratio);

	foc.i_a = ((float)mc_adc.ia - mc_adc.ia_off)*iratio;
	foc.i_b = ((float)mc_adc.ib - mc_adc.ib_off)*iratio;
	foc.i_c = ((float)mc_adc.ic - mc_adc.ic_off)*iratio;
	
	motor_ctrl();
	
	stop_time = TIM3->CNT;
	
	run_time = stop_time - start_time;
	
	if(run_time>runt_max)
		runt_max=run_time;
}
void motor_ctrl(void)
{
	switch (mt.state)
  {
  	case mt_start:
			foc_pwm_start();
			mt.period.start_cnt = 0;
			mt.state = mt_precharge;
  		break;
		
  	case mt_precharge:
			if(++mt.period.start_cnt > 10)
			{
				mt.period.start_cnt = 0;
				mt.ctrl_mode = dragif_mode;
				mt.state = mt_align;
			}
  		break;
		
		case mt_align:
			foc_pwm_run(&foc);
			foc_ctrl_mode();
			foc_calc(&foc);
			foc_ctrl_mode();
			if(++mt.period.start_cnt > 200)
			{
				mt.period.start_cnt = 0;
				pid_clear(&spd_pi);
				pid_clear(&id_pi);
				pid_clear(&iq_pi);
				mt.ctrl_mode = spd_curr;
				mt.state = mt_run;
			}
  		break;
		
		case mt_stop:
			if(++mt.period.stop_cnt > 5)
			{
				foc_pwm_stop();
				mt.period.stop_cnt = 0;
				mt.state = mt_wait;
			}
  		break;
		
		case mt_wait:
  		break;
			
		case mt_run:
			foc_pwm_run(&foc);
			foc_ctrl_mode();
			foc_calc(&foc);
			foc_ctrl_mode();
  		break;
		
  	default:
  		break;
  }
}	

void foc_ctrl_mode(void)
{
	switch (mt.ctrl_mode)
	{
		
	/* Voltage open-loop strong drag, self increasing electrical angle */
	case dragvf_mode:
	{
		foc.theta += mc.theta_acc;
		if (foc.theta > M_2PI)
			foc.theta = 0.0f;
		else if (foc.theta < 0)
			foc.theta = M_2PI;
		foc.v_d = mc.vd_set;
		foc.v_q = mc.vq_set;
		break;
	}
	/* Current closed-loop strong drag, self increasing electrical angle */
	case dragif_mode:
	{
		foc.theta += mc.theta_acc;
		if (foc.theta > M_2PI)
			foc.theta = 0.0f;
		else if (foc.theta < 0)
			foc.theta = M_2PI;
		
		serial_pid_ctrl(&id_pi, mc.id_set, foc.i_d);
		foc.v_d = id_pi.out_value;

		serial_pid_ctrl(&iq_pi, mc.iq_set, foc.i_q);
		foc.v_q = iq_pi.out_value;
		break;
	}	
	/* Voltage open-loop, using encoder or observer angle */
	case volt_mode:
	{
//		foc.theta = mt6825.elec_angle;
		foc.v_d = mc.vd_set;
		foc.v_q = mc.vq_set;
		break;
	}
	/* Current closed-loop, using encoder or observer angle */
	case curr_mode:
	{
		MT6816_Calc_Elec_Angle(mt.para.pairs);
		foc.theta = mt6816.elec_angle;
		serial_pid_ctrl(&id_pi, mc.id_set, foc.i_d);
		foc.v_d = id_pi.out_value;
		serial_pid_ctrl(&iq_pi, mc.iq_set, foc.i_q);
		foc.v_q = iq_pi.out_value;
		break;
	}
	/* Speed current closed-loop, using encoder or observer angle */
	case spd_curr:
	{
		MT6816_Calc_Elec_Angle(mt.para.pairs);
		foc.theta = mt6816.elec_angle;
		
		foc_speed_ctrl(2);

		if (++mt.period.spd_pid_cnt >= 4)
		{ 
			mt.period.spd_pid_cnt = 0;
			serial_pid_ctrl(&spd_pi, mc.spd_set, mt6816.speed);
			mc.iq_set = spd_pi.out_value;
		}
		if (++mt.period.cur_pid_cnt >= 1)
		{
			mt.period.cur_pid_cnt = 0;
			serial_pid_ctrl(&id_pi, mc.id_set, foc.i_d);
			foc.v_d = id_pi.out_value;
			serial_pid_ctrl(&iq_pi, mc.iq_set, foc.i_q);
			foc.v_q = iq_pi.out_value;
		}
	
		break;
	}
	default:
		break;
	}
}

void foc_speed_ctrl(uint8_t N)
{
	if (++mt.period.spd_set_cnt >= N && N != 0)
	{
		if (mc.new_spd > mc.spd_set)
		{
			mc.spd_set += mc.spd_acc;
			if (mc.spd_set > mc.new_spd)
				mc.spd_set = mc.new_spd;
		}
		else if (mc.new_spd < mc.spd_set)
		{
			mc.spd_set -= mc.spd_acc;
			if (mc.spd_set < mc.new_spd)
				mc.spd_set = mc.new_spd;
		}
		else
			mt.period.spd_set_cnt = 0;
	}
	else if (N == 0)
	{
		mc.spd_set = mc.new_spd;
	}
}

