#ifndef __FOC_CALC_H__
#define __FOC_CALC_H__


#define Dead_Time 80
#define PWM_ARR() 5000  //__HAL_TIM_GET_AUTORELOAD(&htim1)

#define set_dtc_a(value)     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)
#define set_dtc_b(value)     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)
#define set_dtc_c(value)     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)

#define rse    0.005f
#define iop 		 20u
#define iratio	((3.3f / 4095.0f) / rse / iop)

#define vratio  (16.0f * 3.3f / 4095.0f)    



// 定义FOC算法所需的常数
#define M_PI (3.14159265358f)         // 圆周率
#define M_2PI (6.28318530716f)        // 2倍圆周率
#define SQRT3 (1.73205080757f)        // 3的平方根
#define SQRT3_BY_2 (0.86602540378f)   // 3的平方根的一半
#define ONE_BY_SQRT3 (0.57735026919f) // 1除以3的平方根
#define TWO_BY_SQRT3 (1.15470053838f) // 2除以3的平方根

#define SQ(x) ((x) * (x))             // 求平方
#define ABS(x) ((x) > 0 ? (x) : -(x)) // 求绝对值

#define min(x, y) (((x) < (y)) ? (x) : (y)) // 取较小值
#define max(x, y) (((x) > (y)) ? (x) : (y)) // 取较大值

#define i_base  1.0f//0.0243902439f

typedef struct
{
	float vbus;
  float inv_vbus; // 母线电压倒数

  float theta;   // 角度
  float sin_val; // 此角度对应的正弦值
  float cos_val; // 此角度对应的余弦值

  float i_a; // A 相电流
  float i_b; // B 相电流
  float i_c; // C 相电流

  float v_a; // A 相电压
  float v_b; // B 相电压
  float v_c; // C 相电压

  float i_d; // D 坐标系电流
  float i_q; // Q 坐标系电流

  float v_d; // D 坐标系电压
  float v_q; // Q 坐标系电压

  float i_alpha; // Alpha 坐标系电流
  float i_beta;  // Beta 坐标系电流

  float v_alpha; // Alpha 坐标系电压
  float v_beta;  // Beta 坐标系电压

  float dtc_a; // A 相 PWM 占空比
  float dtc_b; // B 相 PWM 占空比
  float dtc_c; // C 相 PWM 占空比

} foc_para_t;

void foc_calc(foc_para_t *foc);
void sin_cos_val(foc_para_t *foc);
void clarke_transform(foc_para_t *foc);
void inverse_clarke(foc_para_t *foc);
void park_transform(foc_para_t *foc);
void inverse_park(foc_para_t *foc);
void svpwm_midpoint(foc_para_t *foc);
void svpwm_sector(foc_para_t *foc);
int svm(float alpha, float beta, float* tA, float* tB, float* tC);
#endif

