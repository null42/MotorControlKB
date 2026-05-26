# LXFOC 观测器与控制周期优化指南 - VESC与ODrive融合版

## 一、概述

本文档基于VESC、ODrive等开源项目的工程实践，结合理论分析，总结FOC电机控制中观测器和控制周期的优化方法。

**核心目标：集VESC和ODrive两家之长**

| 特性 | VESC优势 | ODrive优势 | LXFOC融合方案 |
|------|---------|-----------|--------------|
| 电阻辨识 | 死区补偿、温度补偿 | 积分器控制律、相位平衡检测 | ODrive积分器 + VESC补偿 |
| 电感辨识 | 高频注入法 | 电压翻转法（更可靠） | ODrive电压翻转法 |
| 磁链辨识 | 高通滤波消除漂移 | 精确模型补偿 | VESC高通滤波 + ODrive模型 |
| 电流环PI | 简洁带宽设计 | 延迟补偿(1.5Ts) | ODrive延迟补偿 + VESC带宽设计 |
| 速度环PI | 实用惯性估计 | 精确物理模型 | ODrive模型 + VESC惯性辨识 |
| 抗积分饱和 | Clamping方法 | Back-calculation方法 | 多种方法可选 |

---

## 二、参数辨识融合方案

### 2.1 电阻辨识

#### VESC方法
```c
Rs = V_test / I_measured;
// 优点：简单直接
// 缺点：未考虑死区、温度影响
```

#### ODrive方法
```c
// 积分器控制律（类似PI控制器）
V_test += kI * Ts * (I_target - I_measured);
Rs = V_test / I_target;
// 优点：自动调节电压，稳定电流
// 缺点：未考虑死区补偿
```

#### LXFOC融合方案
```c
// 1. ODrive积分器控制律
V_test += kI * Ts * (I_target - I_measured);

// 2. VESC死区补偿
V_deadtime = sign(I) * deadtime_ns * 1e-9 / Ts;
V_test += V_deadtime * 0.5f;

// 3. VESC温度补偿
Rs_compensated = Rs * (1 + temp_coeff * (T - T_ref));

// 4. ODrive相位平衡检测
I_beta_avg = I_beta_sum / count;
if (|I_beta_avg| / I_target > 0.2f) {
    error = PHASE_UNBALANCED;
}
```

**关键参数**：
| 参数 | 来源 | 典型值 |
|------|------|--------|
| kI | ODrive | 1.0 |
| kIBetaFilt | ODrive | 80.0 |
| temp_coeff | VESC | 0.00386 (铜) |
| meas_time | LXFOC | 3000ms |

### 2.2 电感辨识

#### VESC方法（高频注入）
```c
// 注入高频电压，测量电流响应
V_inject = V_test * sin(ω_inject * t);
L = V_test / (ω_inject * I_ac);
// 优点：可在旋转状态测量
// 缺点：信号处理复杂，噪声敏感
```

#### ODrive方法（电压翻转）
```c
// 交替施加正负电压，测量电流变化率
V_out = +V_test;  // 正半周
V_out = -V_test;  // 负半周
// 计算 dI/dt
delta_I += -sign * (I_alpha - I_alpha_last);
L = V_test / (delta_I / total_time);
// 优点：简单可靠，不受反电动势影响
// 缺点：需要电机静止
```

#### LXFOC融合方案
```c
// 采用ODrive电压翻转法
static void lxfoc_inductance_measurement(...) {
    // 交替翻转电压极性
    sign *= -1.0f;
    toggle_count++;
    
    // 累积电流变化
    delta_I += -sign * (I_alpha - I_alpha_last);
    
    // 达到足够次数后计算电感
    if (toggle_count >= 1250) {
        L = |V_test| / (delta_I / total_time);
    }
}
```

**关键参数**：
| 参数 | 来源 | 典型值 |
|------|------|--------|
| toggle_count | ODrive | 1250 |
| V_test | LXFOC | 2.0V |
| Lq/Ld ratio | 经验 | 1.2 |

### 2.3 磁链辨识

#### VESC方法（电压积分+高通滤波）
```c
// 电压积分
flux_alpha_int += (V_alpha - Rs * I_alpha) * Ts;
flux_beta_int += (V_beta - Rs * I_beta) * Ts;

// 高通滤波消除漂移
flux_alpha_hpf = HPF(flux_alpha_int);
flux_beta_hpf = HPF(flux_beta_int);

// 减去电感磁链
flux_pm_alpha = flux_alpha_hpf - Ls * I_alpha;
flux_pm_beta = flux_beta_hpf - Ls * I_beta;

Flux = sqrt(flux_pm_alpha^2 + flux_pm_beta^2);
```

#### ODrive方法（精确模型补偿）
```c
// 考虑更多因素
V_comp = V_applied - Rs*I - dI/dt*L - deadtime_comp;
flux_int += V_comp * Ts;
Flux = average(flux_mag);
```

#### LXFOC融合方案
```c
// 1. 电阻压降补偿
V_alpha_comp = V_alpha - Rs * I_alpha;
V_beta_comp = V_beta - Rs * I_beta;

// 2. 电压积分
flux_alpha_int += V_alpha_comp * Ts;
flux_beta_int += V_beta_comp * Ts;

// 3. VESC高通滤波（消除漂移）
lxfoc_hpf_update(&flux_alpha_hpf, &flux_alpha_hpf, flux_alpha_int, hpf_alpha);
lxfoc_hpf_update(&flux_beta_hpf, &flux_beta_hpf, flux_beta_int, hpf_alpha);

// 4. 减去电感磁链
flux_alpha = flux_alpha_hpf - Ls * I_alpha;
flux_beta = flux_beta_hpf - Ls * I_beta;

// 5. 计算磁链幅值
Flux = sqrt(flux_alpha^2 + flux_beta^2);
```

**关键参数**：
| 参数 | 来源 | 典型值 |
|------|------|--------|
| hpf_alpha | VESC | 0.995 |
| meas_time | LXFOC | 2000ms |

---

## 三、PI自整定融合方案

### 3.1 电流环PI计算

#### VESC方法（带宽法）
```c
Kp = Ls * bandwidth_rad;
Ki = Rs * bandwidth_rad;
// 优点：简单直观
// 缺点：未考虑数字控制延迟
```

#### ODrive方法（延迟补偿）
```c
// 考虑1.5个采样周期的延迟
delay_sec = 1.5 * Ts;
Kp = Ls * bandwidth_rad / (1 + bandwidth_rad * delay_sec);
Ki = Rs * bandwidth_rad;
Ki_max = 1 / (Ls * Ts * 0.5);  // 限制Ki防止不稳定
// 优点：更精确，稳定性好
// 缺点：计算稍复杂
```

#### LXFOC融合方案
```c
// 采用ODrive延迟补偿公式
float delay_sec = delay_cycles * Ts;  // delay_cycles = 1.5
float Kp = Ls * bw_rad / (1.0f + bw_rad * delay_sec);

// Ki计算（带限制）
float Ki = Rs * bw_rad;
float Ki_max = 1.0f / (Ls * Ts * 0.5f);
if (Ki > Ki_max) Ki = Ki_max;

// VESC阻尼比调整
Kp *= damping;
Ki *= damping * damping;
```

**带宽选择建议**：
| 等级 | 电流环带宽 | 阻尼比 | 适用场景 |
|------|-----------|--------|---------|
| 低 | 200Hz | 1.0 | 稳定性优先，大惯量负载 |
| 中 | 500Hz | 0.707 | 平衡，通用场景 |
| 高 | 1000Hz | 0.707 | 响应优先，伺服应用 |

### 3.2 速度环PI计算

#### VESC方法（惯性估计）
```c
// 简化惯性估计
J_est = Kt^2 * Lq * 0.1;

Kp = J * bandwidth_rad / Kt;
Ki = J * bandwidth_rad^2 * 0.1 / Kt;
```

#### ODrive方法（精确模型）
```c
// 需要已知惯性
Kp = J * bandwidth_rad / Kt;
Ki = J * bandwidth_rad^2 * 0.25 / Kt;
// 更精确的Ki系数
```

#### LXFOC融合方案
```c
// 1. 转矩常数计算
Kt = 1.5 * pole_pairs * Flux;

// 2. 惯性估计（VESC方法作为默认值）
if (J_config < MIN_INERTIA) {
    J_est = LXFOC_EstimateInertia_VESC(Flux, Lq, pole_pairs);
}

// 3. PI计算（ODrive精确模型）
Kp = J * bw_rad / Kt;
Ki = J * bw_rad^2 * 0.25 / Kt;

// 4. 限幅保护
Kp = MIN(Kp, max_current * 50);
Ki = MIN(Ki, max_current * 500);
```

### 3.3 惯性辨识

#### 方法一：阶跃响应法（VESC风格）
```c
// 施加恒定转矩，测量加速度
accel = (speed_now - speed_prev) / dt;
J = torque_cmd / accel;
```

#### 方法二：加速度积分法（ODrive风格）
```c
// 更鲁棒的方法
accel_sum += accel^2 * dt;
torque_integral += torque * accel * dt;
J = torque_integral / accel_sum;
```

#### LXFOC融合方案
```c
// 提供两种方法供选择
typedef enum {
    LXFOC_INERTIA_METHOD_STEP,      // 阶跃响应法
    LXFOC_INERTIA_METHOD_ACCEL_INT  // 加速度积分法
} LXFOC_InertiaMethod_e;

// 加速度积分法实现
float LXFOC_MeasureInertia_Update(LXFOC_InertiaMeas_t* meas, 
                                   float torque_cmd, 
                                   float speed_meas, 
                                   float dt)
{
    float accel = (speed_meas - meas->speed_prev) / dt;
    
    if (|accel| > accel_threshold) {
        meas->accel_sum += accel * accel * dt;
        meas->torque_integral += torque_cmd * accel * dt;
    }
    
    if (meas->accel_sum > 1e-6f) {
        meas->inertia_estimated = meas->torque_integral / meas->accel_sum;
    }
    
    return meas->inertia_estimated;
}
```

---

## 四、抗积分饱和策略

### 4.1 三种方法对比

| 方法 | 原理 | 优点 | 缺点 |
|------|------|------|------|
| Clamping | 积分项直接限幅 | 简单 | 可能产生突变 |
| Back-calculation | 输出饱和时反向修正积分 | 平滑 | 需要调节K_back |
| Conditional | 只在需要时积分 | 精确 | 逻辑复杂 |

### 4.2 LXFOC实现

```c
float LXFOC_PI_Update(LXFOC_PI_Controller_t* pi, float ref, float meas, float dt)
{
    float error = ref - meas;
    pi->integral += error * dt;
    
    switch (pi->anti_windup) {
        case LXFOC_ANTI_WINDUP_CLAMPING:
            // VESC风格：直接限幅
            pi->integral = CLAMP(pi->integral, out_min/Ki, out_max/Ki);
            break;
            
        case LXFOC_ANTI_WINDUP_BACK_CALC:
            // ODrive风格：反向计算
            output_raw = Kp * error + Ki * pi->integral;
            output_sat = CLAMP(output_raw, out_min, out_max);
            pi->integral += (output_sat - output_raw) * K_back_calc * dt;
            break;
            
        case LXFOC_ANTI_WINDUP_CONDITIONAL:
            // 条件积分
            output_test = Kp * error + Ki * pi->integral;
            if (需要积分) {
                pi->integral += error * dt;
            }
            break;
    }
    
    return CLAMP(Kp * error + Ki * pi->integral, out_min, out_max);
}
```

**推荐配置**：
- 电流环：Back-calculation，K_back = 0.5
- 速度环：Clamping

---

## 五、观测器优化

### 5.1 Ortega观测器优化

```c
// 核心改进：非线性校正项
dx1/dt = vα - Rs·iα + (γ/2)·(x1 - Ls·iα)·e
dx2/dt = vβ - Rs·iβ + (γ/2)·(x2 - Ls·iβ)·e

// 误差函数
e = λ? - [(x1 - Ls·iα)? + (x2 - Ls·iβ)?]

// 强制误差方向（加快收敛）
if (err > 0.0f) err = 0.0f;

// 增益自适应
gamma = map(duty, 0, 0.9, gamma_slow, gamma_fast);
```

### 5.2 MXLEMMING观测器

```c
// VESC默认方法：差分代替微分
x1 += (vα - Rs·iα)·dt - Ls·(iα - iα_last);
x2 += (vβ - Rs·iβ)·dt - Ls·(iβ - iβ_last);
```

### 5.3 参数整定

| 参数 | 计算公式 | 说明 |
|------|---------|------|
| γ | 600/L (L单位H) | 观测器增益 |
| 带宽要求 | γ/2 > ωe_max | 需大于最大电角速度 |
| 低速增益 | 0.05 × γ | 占空比低时使用 |

---

## 六、控制周期与PWM频率优化

### 6.1 PWM频率选择

| 因素 | 低频(10-15kHz) | 高频(20-30kHz) |
|------|---------------|----------------|
| 开关损耗 | 小 | 大 |
| 电流纹波 | 大 | 小 |
| 控制带宽 | 低 | 高 |
| CPU负载 | 低 | 高 |
| 噪声 | 可听噪声 | 超声波 |

**推荐**：
- 小功率电机：20-30kHz
- 大功率电机：10-15kHz
- 静音应用：>20kHz

### 6.2 延迟补偿

```c
// ODrive延迟模型
// 总延迟 = 采样延迟(0.5Ts) + 计算延迟(0.5Ts) + PWM更新延迟(0.5Ts)
delay_cycles = 1.5f;

// 相位补偿
float phase_comp = omega_e * delay_cycles * Ts;
theta_compensated = theta_estimated + phase_comp;
```

---

## 七、代码集成示例

### 7.1 完整初始化流程

```c
// 1. 参数辨识初始化
LXFOC_ParamIdentify_t ident;
LXFOC_ParamIdentify_Init(&ident, 1.0f/25000.0f, 24.0f);
LXFOC_ParamIdentify_SetCalibCurrent(&ident, 5.0f);
LXFOC_ParamIdentify_SetMaxVoltage(&ident, 2.0f);
LXFOC_ParamIdentify_SetDeadtime(&ident, 500.0f);  // 500ns

// 2. 启动参数辨识
LXFOC_ParamIdentify_Start(&ident);

// 3. 等待辨识完成
while (LXFOC_ParamIdentify_GetState(&ident) != LXFOC_IDENTIFY_COMPLETE) {
    // 在FOC中断中调用
    LXFOC_ParamIdentify_Update(&ident, &elec, &mod_alpha, &mod_beta);
}

// 4. 获取辨识参数
float Rs, Ld, Lq, Flux;
LXFOC_GetIdentifiedParams(&ident, &Rs, &Ld, &Lq, &Flux);

// 5. 自整定初始化
LXFOC_AutoTune_t tune;
LXFOC_AutoTune_FromIdentifiedParams(&tune, &ident, 4.0f, 4000.0f, 10.0f);

// 6. 设置带宽等级
LXFOC_AutoTune_SetBandwidth(&tune, LXFOC_BANDWIDTH_MEDIUM);

// 7. 计算PI参数
LXFOC_AutoTune_CalculateCurrentPI(&tune, &config);
LXFOC_AutoTune_CalculateSpeedPI(&tune, &config);

// 8. 获取计算结果
float Kp_cur, Ki_cur, Kp_spd, Ki_spd;
LXFOC_AutoTune_GetCurrentPI(&tune, &Kp_cur, &Ki_cur);
LXFOC_AutoTune_GetSpeedPI(&tune, &Kp_spd, &Ki_spd);
```

### 7.2 控制循环

```c
void FOC_CurrentLoop(void)
{
    // 采样
    float Ia = ADC_GetCurrentA();
    float Ib = ADC_GetCurrentB();
    
    // Clarke变换
    float Ialpha = Ia;
    float Ibeta = ONE_BY_SQRT3 * Ia + TWO_BY_SQRT3 * Ib;
    
    // 观测器更新（获取转子角度）
    float theta = Observer_Update(Ialpha, Ibeta, Valpha, Vbeta);
    
    // Park变换
    float Id = cosf(theta) * Ialpha + sinf(theta) * Ibeta;
    float Iq = -sinf(theta) * Ialpha + cosf(theta) * Ibeta;
    
    // PI控制
    float Vd, Vq;
    LXFOC_AutoTune_Update(&tune, 
                          Id_ref, Iq_ref,
                          Id, Iq,
                          speed_ref, speed_meas,
                          &Vd, &Vq);
    
    // 逆Park变换
    float Valpha = cosf(theta) * Vd - sinf(theta) * Vq;
    float Vbeta = sinf(theta) * Vd + cosf(theta) * Vq;
    
    // SVPWM输出
    SVPWM_Update(Valpha, Vbeta);
}
```

---

## 八、参考文献

1. VESC源码：https://github.com/vedderb/bldc
2. ODrive源码：https://github.com/odriverobotics/ODrive
3. 知乎文章：
   - 吃透最强开源FOC--VESC系列
   - 电机参数辨识算法
   - 无感FOC观测器原理
4. 论文：
   - Wang Qiwei. 多工况下永磁同步电机电气参数辨识方法研究[D]. 哈尔滨工业大学, 2022.
   - Ortega R. A Globally Convergent Observer for PMSM

---

## 九、版本历史

| 版本 | 日期 | 作者 | 说明 |
|------|------|------|------|
| 1.0 | 2025/04/05 | LXFOC | 初始版本 |
| 2.0 | 2025/04/05 | LXFOC | VESC与ODrive融合版本 |

---

**注意事项**：
1. 参数辨识需在电机静止或低速状态进行
2. 自整定后建议进行阶跃响应验证
3. 不同电机可能需要微调带宽参数
4. 高速应用建议使用双采样模式
5. 惯性辨识需要电机能够自由旋转
