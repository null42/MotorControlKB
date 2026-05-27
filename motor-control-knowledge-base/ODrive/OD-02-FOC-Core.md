# ⭐ OD-02: FOC 核心算法

---

| 属性 | 值 |
|------|-----|
| 文档编号 | OD-02 |
| 标题 | FOC 核心算法 |
| 版本 | 1.0 |
| 作者 | 嵌入式系统架构师 |
| 创建日期 | 2026-05-26 |
| 适用平台 | STM32F405RG + DRV8301 栅极驱动 |
| 固件版本 | FW v0.5.x 系列 |
| 前置知识 | 矢量控制基础, Clarke/Park 变换, PI 控制, SVPWM |

---

## 目录 (TOC)

1. [概述](#1-概述)
2. [原理推导](#2-原理推导)
3. [数学建模](#3-数学建模)
4. [代码实现](#4-代码实现)
5. [参数整定](#5-参数整定)
6. [硬件约束](#6-硬件约束)
7. [高级主题](#7-高级主题)
8. [文件索引](#8-文件索引)

---

## 1. 概述

FOC (Field-Oriented Control, 磁场定向控制) 是 ODrive 的核心控制算法，实现于 [foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp) 中的 `FieldOrientedController` 类。该控制器继承自 `AlphaBetaFrameController`，同时支持电流控制模式和电压控制模式。

### 1.1 在系统中的位置

```
Controller.torque_setpoint  →  Motor.update(torque→Idq)
  →  Motor.Idq_setpoint_ / Vdq_setpoint_
    →  FieldOrientedController.update()  (读取 setpoint 原子更新)
      →  PhaseControlLaw.on_measurement() (高优先级中断: 存储测量值)
        →  PhaseControlLaw.get_output() (高优先级中断: 计算 PWM)
          →  TIM1/TIM8 CCR 寄存器 → Gate Driver → BLDC Motor
```

### 1.2 控制模式

| 模式 | `enable_current_control_` | 说明 |
|------|:---:|------|
| 电流控制 | `true` | Idq_setpoint 为电流给定, Vdq_setpoint 为前馈电压, PI 控制器闭环调节 |
| 电压控制 | `false` | Vdq_setpoint 直接作为输出电压, 用于云台电机 (MOTOR_TYPE_GIMBAL) |

---

## 2. 原理推导

### 2.1 坐标变换链

FOC 的核心思想是将三相静止坐标系 (abc) 下的交流量变换为两相旋转坐标系 (dq) 下的直流量进行控制。

```mermaid
flowchart LR
    ABC["Ia, Ib, Ic 三相静止 abc轴系"] --> Clarke["Clarke变换"]
    Clarke --> AlphaBeta["iα, iβ 两相静止 αβ轴系"]
    AlphaBeta --> Park["Park变换"]
    Park --> DQ["Id, Iq 两相旋转 dq轴系"]
    DQ --> PI["PI控制器"]
    PI --> VdVq["Vd, Vq"]
    VdVq --> InvPark["逆Park变换"]
    InvPark --> ValphaBeta["Vα, Vβ"]
    ValphaBeta --> SVM["SVM调制"]
    SVM --> TIMCCR["tA, tB, tC → TIM CCR"]
    TIMCCR --> Inverter["3相逆变器 ← PWM信号"]
    Inverter --> Motor["PMSM/BLDC"]
```

### 2.2 Clarke 变换 (abc → αβ)

以 A 相为 α 轴对齐方向:

$$I_\alpha = I_a$$

$$I_\beta = \frac{1}{\sqrt{3}}(I_b - I_c)$$

这利用了三相电流之和为零的约束 ($I_a + I_b + I_c = 0$)，因此只需测量两相电流。

### 2.3 Park 变换 (αβ → dq)

$$\begin{bmatrix} I_d \\ I_q \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} I_\alpha \\ I_\beta \end{bmatrix}$$

其中 $\theta$ 是转子电角度，由编码器或观测器提供。

### 2.4 反 Park 变换 (dq → αβ)

$$\begin{bmatrix} V_\alpha \\ V_\beta \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} V_d \\ V_q \end{bmatrix}$$

### 2.5 空间矢量调制 (SVM)

将 αβ 电压矢量转换为三相 PWM 占空比。ODrive 使用 `SVM()` 函数实现七段式 SVPWM，输出 `tA, tB, tC` 在 [0, 1] 范围。

**调制深度限制**: 最大调制量为 `0.80 × √3/2 ≈ 0.6928`，保留裕量避免过调制导致波形失真。

---

## 3. 数学建模

### 3.1 电机电气模型

PMSM/BLDC 在 dq 坐标系下的电压方程：

$$V_d = R_s I_d + L_d \frac{dI_d}{dt} - \omega_e L_q I_q$$

$$V_q = R_s I_q + L_q \frac{dI_q}{dt} + \omega_e (L_d I_d + \psi_{PM})$$

其中：
- $R_s$: 定子相电阻 [Ω]
- $L_d, L_q$: d/q 轴电感 [H] (SPMSM 时 $L_d = L_q$)
- $\omega_e$: 电角速度 [rad/s]
- $\psi_{PM}$: 永磁体磁链 [Wb = V·s]

### 3.2 电流 PI 控制器

离散时间 PI 控制器（累积式）：

$$V_d[k] = V_d^{ff}[k] + V_d^{int}[k-1] + K_p \cdot (I_d^{ref}[k] - I_d[k])$$

$$V_q[k] = V_q^{ff}[k] + V_q^{int}[k-1] + K_p \cdot (I_q^{ref}[k] - I_q[k])$$

积分项更新（仅在未饱和时）：

$$V_d^{int}[k] = V_d^{int}[k-1] + K_i \cdot T_s \cdot (I_d^{ref}[k] - I_d[k])$$

$$V_q^{int}[k] = V_q^{int}[k-1] + K_i \cdot T_s \cdot (I_q^{ref}[k] - I_q[k])$$

其中 $T_s = \text{current_meas_period}$ 是电流采样周期。

### 3.3 PI 增益计算

根据零极点对消原理（pole-zero cancellation），电流环带宽 $\omega_c$ 与电机参数的关系：

$$K_p = \omega_c \cdot L$$

$$K_i = \omega_c \cdot R = K_p \cdot \frac{R}{L}$$

ODrive 中 `update_current_controller_gains()` 实现 [motor.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L297-L302)：

```cpp
float p_gain = config_.current_control_bandwidth * config_.phase_inductance;
float plant_pole = config_.phase_resistance / config_.phase_inductance;
current_control_.pi_gains_ = {p_gain, plant_pole * p_gain};
```

### 3.4 矢量幅值限幅

为避免电流控制输出超过逆变器能力，对调制矢量进行幅值限制：

$$M_{dq} = \sqrt{V_d^2 + V_q^2}$$

若 $M_{dq} > M_{max}$（$M_{max} = 0.80 \times \sqrt{3}/2$），则按比例缩放：

$$V_d' = V_d \cdot \frac{M_{max}}{M_{dq}}, \quad V_q' = V_q \cdot \frac{M_{max}}{M_{dq}}$$

同时衰减积分项防止积分饱和（windup）：

$$V_d^{int} \gets V_d^{int} \cdot 0.99, \quad V_q^{int} \gets V_q^{int} \cdot 0.99$$

### 3.5 母线电流估算

$$I_{bus} = m_d \cdot I_d + m_q \cdot I_q$$

其中 $m_d = V_d / V_{dc}, m_q = V_q / V_{dc}$ 为调制系数。

---

## 4. 代码实现

### 4.1 文件结构

```
foc.hpp        — FieldOrientedController 类声明
foc.cpp        — FOC 核心实现
  ├── AlphaBetaFrameController::on_measurement()   — Clarke 变换
  ├── AlphaBetaFrameController::get_output()       — SVM + 输出
  ├── FieldOrientedController::on_measurement()    — 存储测量值
  ├── FieldOrientedController::get_alpha_beta_output() — Park/PI/InvPark
  └── FieldOrientedController::update()            — 原子读取 setpoint
phase_control_law.hpp — 控制律抽象基类
utils.cpp     — SVM() 函数实现
```

### 4.2 Clarke 变换实现

[foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L5-L21)：

```cpp
Motor::Error AlphaBetaFrameController::on_measurement(
        std::optional<float> vbus_voltage,
        std::optional<std::array<float, 3>> currents,
        uint32_t input_timestamp) {
    std::optional<float2D> Ialpha_beta;
    if (currents.has_value()) {
        // Clarke transform
        Ialpha_beta = {
            (*currents)[0],                                       // Iα = Ia
            one_by_sqrt3 * ((*currents)[1] - (*currents)[2])     // Iβ = (Ib-Ic)/√3
        };
    }
    return on_measurement(vbus_voltage, Ialpha_beta, input_timestamp);
}
```

### 4.3 Park 变换实现

[foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L97-L112)：

```cpp
// Park transform with phase compensation for timestamp delay
auto [Ialpha, Ibeta] = *Ialpha_beta_measured_;
float I_phase = phase + phase_vel * 
    ((float)(int32_t)(i_timestamp_ - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
float c_I = our_arm_cos_f32(I_phase);
float s_I = our_arm_sin_f32(I_phase);
Idq = {
    c_I * Ialpha + s_I * Ibeta,     // Id = cos·Iα + sin·Iβ
    c_I * Ibeta - s_I * Ialpha      // Iq = cos·Iβ - sin·Iα
};
```

**关键细节**: Park 变换使用的时间戳补偿 (`I_phase = phase + phase_vel * Δt`) 补偿了从电流采样时刻到控制回路计算时刻之间的角度变化。

### 4.4 电流 PI 控制实现

[foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L120-L160)：

```cpp
if (enable_current_control_) {
    auto [p_gain, i_gain] = *pi_gains_;
    auto [Id, Iq] = *Idq;
    auto [Id_setpoint, Iq_setpoint] = *Idq_setpoint_;

    float Ierr_d = Id_setpoint - Id;
    float Ierr_q = Iq_setpoint - Iq;

    // PI control喺
    mod_d = V_to_mod * (Vd + v_current_control_integral_d_ + Ierr_d * p_gain);
    mod_q = V_to_mod * (Vq + v_current_control_integral_q_ + Ierr_q * p_gain);

    // Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / 
        std::sqrt(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // Anti-windup: decay integrator when saturated
        v_current_control_integral_d_ *= 0.99f;
        v_current_control_integral_q_ *= 0.99f;
    } else {
        v_current_control_integral_d_ += Ierr_d * (i_gain * current_meas_period);
        v_current_control_integral_q_ += Ierr_q * (i_gain * current_meas_period);
    }
}
```

### 4.5 反 Park 变换实现

[foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L162-L167)：

```cpp
// Inverse Park transform
float pwm_phase = phase + phase_vel * 
    ((float)(int32_t)(output_timestamp - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
float c_p = our_arm_cos_f32(pwm_phase);
float s_p = our_arm_sin_f32(pwm_phase);
float mod_alpha = c_p * mod_d - s_p * mod_q;  // Vα = cos·Vd - sin·Vq
float mod_beta = c_p * mod_q + s_p * mod_d;   // Vβ = cos·Vq + sin·Vd

// Report final applied voltage in stationary frame (for sensorless estimator)
final_v_alpha_ = mod_to_V * mod_alpha;
final_v_beta_ = mod_to_V * mod_beta;
```

**关键细节**: 与 Park 变换一样，反 Park 也进行了时间戳补偿，使用 `output_timestamp` 补偿到 PWM 输出实际生效时刻。

### 4.6 SVM 与 PWM 输出

[AlphaBetaFrameController::get_output](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L23-L45)：

```cpp
Motor::Error AlphaBetaFrameController::get_output(
        uint32_t output_timestamp, float (&pwm_timings)[3],
        std::optional<float>* ibus) {
    std::optional<float2D> mod_alpha_beta;
    Motor::Error status = get_alpha_beta_output(output_timestamp, 
        &mod_alpha_beta, ibus);

    if (status != Motor::ERROR_NONE) {
        return status;
    } else if (!mod_alpha_beta.has_value() || 
               is_nan(mod_alpha_beta->first) || is_nan(mod_alpha_beta->second)) {
        return Motor::ERROR_MODULATION_IS_NAN;
    }

    auto [tA, tB, tC, success] = SVM(mod_alpha_beta->first, mod_alpha_beta->second);
    if (!success) {
        return Motor::ERROR_MODULATION_MAGNITUDE;
    }

    pwm_timings[0] = tA;
    pwm_timings[1] = tB;
    pwm_timings[2] = tC;
    return Motor::ERROR_NONE;
}
```

### 4.7 原子 Setpoint 更新

[FieldOrientedController::update()](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L184-L193) 在低优先级中断上下文中运行，通过 `CRITICAL_SECTION()` 原子地读取所有输入端口：

```cpp
void FieldOrientedController::update(uint32_t timestamp) {
    CRITICAL_SECTION() {
        ctrl_timestamp_ = timestamp;
        enable_current_control_ = enable_current_control_src_;
        Idq_setpoint_ = Idq_setpoint_src_.present();
        Vdq_setpoint_ = Vdq_setpoint_src_.present();
        phase_ = phase_src_.present();
        phase_vel_ = phase_vel_src_.present();
    }
}
```

### 4.8 电压控制模式

电压控制模式跳过 PI 控制器，直接输出 Vdq_setpoint [foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp#L156-L160)：

```cpp
} else {
    // Voltage control mode
    mod_d = V_to_mod * Vd;
    mod_q = V_to_mod * Vq;
}
```

---

## 5. 参数整定

### 5.1 电流环带宽整定

电流环 PI 增益由电机参数自动计算：

| 参数 | 含义 | 典型值 | 如何获取 |
|------|------|--------|---------|
| `current_control_bandwidth` | 电流环带宽 (rad/s) | 1000 | 用户配置 |
| `phase_resistance` | 相电阻 (Ω) | 0.1~1.0 | 自动测量 (motor calibration) |
| `phase_inductance` | 相电感 (H) | 50e-6 ~ 500e-6 | 自动测量 (motor calibration) |

**经验法则：**
- 电流环带宽应 ≤ 开关频率的 1/10
- 典型设置: 1000 rad/s (≈ 159 Hz)
- 增大带宽 → 电流响应更快 → 但噪声更大
- 减小带宽 → 电流跟踪更平滑 → 但响应变慢

### 5.2 调制深度限制

当前硬编码为 0.80 × √3/2 ≈ 0.6928。此值为经验值，说明：
- 最大相电压 = 0.6928 × vbus_voltage (线电压有效值)
- 预留裕量用于 SVM 过调制保护
- 如需更多电压裕量可调高此值 (不超过 1.0)

### 5.3 抗积分饱和 (Anti-Windup)

饱和时积分衰减系数默认为 0.99：
- 设为 1.0: 不衰减 (严重 windup)
- 设为 0.9: 过快衰减 (积分几乎失效)
- 默认 0.99: 温和衰减

---

## 6. 硬件约束

### 6.1 电流采样

ODrive 使用 3 电阻法 (3-shunt) 采样相电流：

```
          Vbus
           │
     ┌─────┴─────┐
     │ 3-Phase   │
     │ Inverter  │
     └──┬──┬──┬──┘
        │  │  │
    ┌───┘  │  └───┐
    │      │      │
   Rsh    Rsh    Rsh
    │      │      │
   ADC1   ADC2   ADC3
```

**电流采样时序：**

```
  PWM 周期
  ├─────────────────────┤
  ↑                     ↑
  Center                Center
  (ADC trigger at PWM CNT=0)
```

双电阻/三电阻采样在 PWM 中心点触发 ADC，此时无开关噪声。

**DC 偏置校准 (dc_calib):** [motor.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L667-L683) 使用一阶低通滤波器估计 DC 偏置：

```cpp
DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
```

其中 `calib_filter_k = min(dc_calib_period / dc_calib_tau, 1.0f)`，默认 τ = 0.2s。

**校准有效性检查:** 至少需要 7.5τ 时间 (约 1.5s)，且偏置值需在 `±max_dc_calib_` (最大电流的 ±10%) 范围内。

### 6.2 PWM 更新机制

PWM 采用中心对齐模式 (Center-Aligned PWM)，更新流程：

```
1. TIM 溢出中断 → current_meas_cb() (采样电流 + 控制律 on_measurement)
2. TIM 下溢中断 → pwm_update_cb() (控制律 get_output → 更新 CCR)
3. TIM CCR 寄存器通过影子寄存器在下一次更新事件时生效
4. TIM_BDTR_AOE 位自动使能输出
```

**关键保护**: PWM 输出使能由硬件自动管理 (`TIM_BDTR_AOE` 自动输出使能)，一旦撤防通过 `__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY` 立即关闭所有 PWM 输出。

### 6.3 过流保护

电流范数检测 [motor.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L631-L644)：

```cpp
float Itrip = effective_current_lim_ + config_.current_lim_margin;
float Inorm_sq = 2.0f / 3.0f * (SQ(current_meas_->phA)
                              + SQ(current_meas_->phB)
                              + SQ(current_meas_->phC));
if (Inorm_sq > SQ(Itrip)) {
    disarm_with_error(ERROR_CURRENT_LIMIT_VIOLATION);
}
```

电流范数关系（假定 Ia+Ib+Ic=0）：$I_{norm}^2 = I_d^2 + I_q^2 = I_\alpha^2 + I_\beta^2 = \frac{2}{3}(I_a^2 + I_b^2 + I_c^2)$

---

## 7. 高级主题

### 7.1 时间戳补偿详解

ODrive 的 FOC 实现中，Park 和反 Park 变换都包含时间戳补偿：

```cpp
// Park: 补偿电流采样时刻到控制回路时刻
float I_phase = phase + phase_vel * Δt_i;

// InvPark: 补偿控制回路时刻到 PWM 输出时刻
float pwm_phase = phase + phase_vel * Δt_o;
```

其中 `Δt = (timestamp_diff) / TIM_1_8_CLOCK_HZ`，`TIM_1_8_CLOCK_HZ = 168MHz`。

**时序图：**

```
时间轴 →
│              │                     │                │
│ t_meas       │ t_ctrl              │ t_output       │
│ (电流采样)   │ (控制回路)          │ (PWM生效)      │
│─────────────────────────────────────────────────────│
│  ←Δt_i→      │←──── Δt_o ────────→│                │
│              │                     │                │
│ I_phase:     利用 phase_vel 前推   │                │
│ pwm_phase:   利用 phase_vel 后推   │                │
```

这种补偿显著减少了高速运行时的角度误差，提升 FOC 效率。

### 7.2 ibus 计算与功率估算

```cpp
// focs.cpp
*ibus = mod_d * Id + mod_q * Iq;
power_ = vbus_voltage * (*ibus).value();
```

物理意义：$P_{elec} = V_{dc} \cdot I_{bus} = V_d \cdot I_d + V_q \cdot I_q$

此功率估算用于 spinout 检测和无感估算器校准。

### 7.3 前馈控制

前馈由 Motor 层在转矩→电流转换后计算 [motor.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L567-L587)：

**R_wL 前馈** (电阻 + 交叉耦合补偿):

```cpp
if (config_.R_wL_FF_enable) {
    vd -= *phase_vel * config_.phase_inductance * iq;  // -ωL·Iq (交叉耦合)
    vq += *phase_vel * config_.phase_inductance * id;  // +ωL·Id (交叉耦合)
    vd += config_.phase_resistance * id;                 // +R·Id
    vq += config_.phase_resistance * iq;                 // +R·Iq
}
```

**bEMF 前馈** (反电动势补偿):

```cpp
if (config_.bEMF_FF_enable) {
    vq += *phase_vel * (2.0f/3.0f) * (config_.torque_constant / config_.pole_pairs);
}
```

其中反电动势常数 $K_e = \frac{2}{3} \cdot \frac{K_t}{P}$（$K_t$ 为转矩常数，$P$ 为极对数）。

### 7.4 2-Norm 电流限幅

在 Motor 层 [motor.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L544-L548)：

```cpp
// 2-norm clamping where Id takes priority
float iq_lim_sqr = SQ(ilim) - SQ(id);
float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
iq = std::clamp(iq, -Iq_lim, Iq_lim);
```

这确保了：$I_d^2 + I_q^2 \leq I_{lim}^2$，且 $I_d$ 优先分配电流预算（对 ACIM 的 autoflux 模式尤为重要）。

### 7.5 电阻测量控制律

[ResistanceMeasurementControlLaw](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L18-L79):

**原理**: 固定注入电流 $I_{target}$，积分控制器调节电压直到电流跟踪目标，然后 $R = V/I$。

```cpp
test_voltage_ += (kI * current_meas_period) * (target_current_ - actual_current_);
float get_resistance() {
    return test_voltage_ / target_current_;
}
```

### 7.6 电感测量控制律

[InductanceMeasurementControlLaw](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp#L88-L148):

**原理**: 施加交变电压 $V_{test}$，测量电流纹波 $\Delta I$，$L = V \cdot \Delta t / \Delta I$。

```cpp
test_voltage_ *= -1.0f;  // Toggle voltage direction each cycle
float get_inductance() {
    float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ;
    return std::abs(test_voltage_) / (deltaI_ / dt);
}
```

---

### 7.7 开关频率与控制回路频率的关系

ODrive 固件支持灵活的 PWM 频率配置。

| 参数 | 含义 | 默认值 | 范围 |
|------|------|--------|------|
| `pwm_frequency` | PWM 载波频率 | 24000 Hz | 15k-40kHz |
| `current_control_hz` | 电流回路频率 | 8000 Hz | 1k-24kHz |

**频率关系图：**

```
  电流回路频率 ≤ PWM 频率

  PWM 周期 (41.7μs @ 24kHz)
  ├────────────────────────────────────┤
  │                                    │
  │   电流采样 ───▶ 控制计算 ───▶ PWM更新
  │   ↑ 上溢中断     ↑ 下溢中断         │
  │                                    │
  ├────────────────────────────────────┤
  │                                    │
  │   电流采样 ───▶ 控制计算 ───▶ PWM更新
  │                                    │
```

**跟踪速率 (slew rate) 限制：**

PWM 更新频率对应的电流跟踪速率：

$$\frac{dI}{dt}\bigg|_{max} = \frac{V_{dc}}{\sqrt{3} \cdot L_s}$$

其中 $V_{dc}$ 是总线电压，$L_s$ 是相电感。

当 $current\_control\_hz < pwm\_frequency$ 时，PI增益中的 $T_s$ 应与实际控制周期一致。

**频率选择建议：**

| 应用 | PWM频率 | 控制频率 | 说明 |
|------|---------|---------|------|
| 低电感高速电机 | 40 kHz | 10 kHz | 需要高带宽电流环 |
| 通用伺服 | 24 kHz | 8 kHz | 默认配置 |
| 大电感低功率 | 15 kHz | 3 kHz | 足够的电感滤波 |
| 云台电机 (电压模式) | 30 kHz | 8 kHz | 高频PWM减少噪声 |

### 7.8 电流传感器详细分析

ODrive V3.x 采用 3 个独立的分流电阻：

```
         ┌─────┬─────┬─────┐
         │ A+  │ B+  │ C+  │  上桥臂
         ├─────┼─────┼─────┤
  Vbus ──┤     │     │     ├── 电机相
         ├─────┼─────┼─────┤
         │ A-  │ B-  │ C-  │  下桥臂
         └──┬──┴──┬──┴──┬──┘
            │     │     │
           Rsh   Rsh   Rsh
            │     │     │
           PGND  PGND  PGND
```

#### ADC采样配置

| ADC | 通道 | 信号 | 分辨率 |
|-----|------|------|--------|
| ADC1 | INxx | phA 电流 | 12bit |
| ADC2 | INxx | phB 电流 | 12bit |
| ADC3 | INxx | phC 电流 | 12bit |

#### DC偏置校准数学分析

考虑真实的ADC测量值：

$$I_{raw} = I_{real} + I_{bias} + I_{noise}$$

一阶低通滤波器对偏置的估计：

$$\hat{I}_{bias}[k] = \hat{I}_{bias}[k-1] + \alpha \cdot (I_{raw}[k] - \hat{I}_{bias}[k-1])$$

其中 $\alpha = \min(T_s / \tau, 1.0)$，$\tau = 0.2s$。

收敛时间常数：约 $5\tau = 1.0s$ 达 99.3% 精度。校准有效性要求至少 $7.5\tau = 1.5s$。

**温度漂移影响：** DC偏置会随温度漂移（通常 ~100μV/°C）。ODrive 在每次上电和重新arm后都会重新校准。

---

## 8. 文件索引

| 文件 | 关键函数 | 说明 |
|------|---------|------|
| [foc.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.cpp) | `on_measurement`, `get_alpha_beta_output`, `get_output`, `update` | FOC 核心实现 |
| [foc.hpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/foc.hpp) | FieldOrientedController 类 | FOC 类接口 |
| [phase_control_law.hpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/phase_control_law.hpp) | PhaseControlLaw, AlphaBetaFrameController | 控制律抽象 |
| [motor.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.cpp) | `update_current_controller_gains`, `measure_phase_resistance`, `measure_phase_inductance` | 电机校准 + 前馈 |
| [motor.hpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/motor.hpp) | Motor::Config_t | 电机参数配置 |
| [utils.cpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/utils.cpp) | `SVM()` | 空间矢量调制 |
| [low_level.h](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/low_level.h) | `safety_critical_apply_brake_resistor_timings` | 制动电阻 PWM 管理 |
| [current_limiter.hpp](file:///e:/gitee_CodeStorage/学习/MotorControl-main/odriver_core/MotorControl/current_limiter.hpp) | CurrentLimiter 基类 | 电流限制抽象 |

---

> **相关文档**:
> - [OD-01: 系统架构](OD-01-Architecture.md) — 整体架构与组件关系
> - [OD-03: 控制策略](OD-03-Control-Strategy.md) — 位置/速度/转矩控制级联
> - [OD-04: 编码器与无传感器](OD-04-Encoder-Sensorless.md) — 位置反馈与无感估算