/**********************************
             慧驱动
        视频教程详见QQ空间或B站
        其他事宜详见readme.txt
          QQ：1323279545
         微信：huiqudong
        官网：www.huiqudong.cn
**********************************/
#include "main.h"
#include "flux_observer.h"



//#include <math.h>


#define Rs 0.6f
#define Ld 0.003f

FLUX_OBSERVER_DEF Flux_Observer;


//float Vbeta;

//float Ibeta;
float pre_theta;
float theta_error;
float pre_theta_error;


float alpha_temp;
float beta_temp;
float alpha_int;
float beta_int;
float Beta_psi;
float Beta_psi_pre;
float Alpha_psi;
float Alpha_psi_pre;

void Flux_Observer_Init(FLUX_OBSERVER_DEF* flux_observer)
{

    alpha_temp = 0.0f;
    beta_temp = 0.0f;
    alpha_int = 0.0f;
    beta_int = 0.0f;
    Beta_psi_pre = 0.0f;
    Alpha_psi_pre = 0.0f;
    flux_observer->theta = 0.0f;
    pre_theta = 0.0f;
    theta_error = 0.0f;
    flux_observer->we = 0.0f;
    flux_observer->lpf_we = 0.0f;
    pre_theta_error = 0.0f;

}
//简单的电压式磁链观测器，以积分得到磁链，后续可以改成低通滤波
void flux_observer_Outputs_wrapper(CURRENT_ALPHA_BETA_DEF i_alpha_beta
                                   , VOLTAGE_ALPHA_BETA_DEF v_alpha_beta
                                   , FLUX_OBSERVER_DEF* flux_observer)
{
    // 下面的0.0001f是 Ts
    alpha_int += (v_alpha_beta.Valpha - i_alpha_beta.Ialpha * Rs) * 0.0001f;
    alpha_temp = alpha_int - i_alpha_beta.Ialpha * Ld;
    Alpha_psi = alpha_temp * 0.001f + (1.0f - 0.001f) * Alpha_psi_pre;
    Alpha_psi_pre = Alpha_psi;

    beta_int += (v_alpha_beta.Vbeta - i_alpha_beta.Ibeta * Rs) * 0.0001f;
    beta_temp = beta_int - i_alpha_beta.Ibeta * Ld;
    Beta_psi = beta_temp * 0.001f + (1.0f - 0.001f) * Beta_psi_pre;
    Beta_psi_pre = Beta_psi;

    flux_observer->theta = atan2f(beta_temp - Beta_psi, alpha_temp - Alpha_psi);
    if(flux_observer->theta < 0.0f)
    {
        flux_observer->theta += 6.2831853f;
    }
    theta_error = flux_observer->theta - pre_theta;
    pre_theta = flux_observer->theta;
    if((theta_error < -1.0f) || (theta_error > 1.0f))
    {
        theta_error = pre_theta_error;
    }
    flux_observer->we = theta_error / 0.0001f;
    pre_theta_error = theta_error;

}
