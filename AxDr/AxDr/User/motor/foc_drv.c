#include "foc_drv.h"
#include "foc_ctrl.h"
#include "foc_calc.h"
#include "pid.h"
#include "encoder.h"
#include "tim.h"

motor_ctrl_t mt;

pid_para_t vq_pi;
pid_para_t id_pi;
pid_para_t iq_pi;
pid_para_t spd_pi;
pid_para_t pos_pi;

foc_para_t foc;
mc_para_t  mc;
mc_adc_val_t mc_adc;
float K;
void foc_update_current_ctrl_gain(uint16_t bandwidth)
{
	id_pi.kp = mt.para.Ls * bandwidth*i_base;
	id_pi.ki = mt.para.Rs / mt.para.Ls * 0.00005f;
//	id_pi.ki = mt.para.Rs * bandwidth / 20000;

	iq_pi.kp = mt.para.Ls * bandwidth*i_base;
	iq_pi.ki = mt.para.Rs / mt.para.Ls * 0.00005f;
//	iq_pi.ki = mt.para.Rs * bandwidth / 20000;
	
	K = (3.0f*mt.para.pairs*mt.para.flux) / (4.0f * mt.para.Jx);
	
	spd_pi.kp = iq_pi.kp/(mt.para.Ls*mt.para.delta*K);
	spd_pi.ki = iq_pi.kp/(mt.para.delta*mt.para.delta*mt.para.Ls)/10000;
//	spd_pi.kp = (spd_pi.ki*10000*mt.para.delta)/K;
}

void mt_foc_para_init(void)
{
	mt.state = mt_start;
	mt.ctrl_mode = dragif_mode;

	mc.theta_acc = 0.001f;
	mc.vd_set = 0.0f;
	mc.vq_set = 0.5f;

	mt.para.pairs = 7;
	mt.para.Rs = 0.17780f;
	mt.para.Ls = 0.000095f;
	mt.para.flux = 0.00192f;
	mt.para.Jx = 0.0000009935f;
	mt.para.delta = 4.0f;

	foc.inv_vbus = 1.0f / 24.0f;
	
	mc.id_set = 0.0f;
	mc.iq_set = 1.0f;
	
	mc.spd_set = 10.0f;
	mc.spd_acc = 1.0f;
	mc.new_spd = 10.0f;

	foc_update_current_ctrl_gain(500);
	
	pos_pi.kp = 0.5f;
	pos_pi.ki = 0.0f;
	
	pid_limit_init(&id_pi, 10.8f, -10.8f, 10.8f, -10.8f);
	pid_limit_init(&iq_pi, 10.8f, -10.8f, 10.8f, -10.8f);
	pid_limit_init(&spd_pi, 5.0f, -5.0f, 5.0f, -5.0f);
	pid_limit_init(&pos_pi, 2.0f, -2.0f, 2.0f, -2.0f);

	get_curr_off();
}

void get_curr_off(void)
{
	float sum_a, sum_b, sum_c;
	for(int i=0;i<1000;i++)
  {
		HAL_Delay(1);
		sum_a += (float)(ADC1->JDR3);
		sum_b += (float)(ADC1->JDR2);
		sum_c += (float)(ADC1->JDR1);
	}
	
	mc_adc.ia_off = sum_a/1000;
	mc_adc.ib_off = sum_b/1000;
	mc_adc.ic_off = sum_c/1000;
}

void foc_pwm_start(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void foc_pwm_stop(void)
{
	set_dtc_a(PWM_ARR());
	set_dtc_b(PWM_ARR());
	set_dtc_c(PWM_ARR());
}

void foc_pwm_run(foc_para_t *foc)
{
	set_dtc_a((uint16_t)(foc->dtc_a * PWM_ARR()));
	set_dtc_b((uint16_t)(foc->dtc_b * PWM_ARR()));
	set_dtc_c((uint16_t)(foc->dtc_c * PWM_ARR()));
}
