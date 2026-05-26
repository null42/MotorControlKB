# HPM-MC v2 力位混合控制模块深度解析 ★★★★☆ 🔰📚🔧

**文档版本：** v1.0
**生成日期：** 2026-05-23
**源码位置：** `hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.{h,c}`
**中间件版本：** v1.11.0+ (SDK v1.11.0)

---

## 目录

1. [概述](#1-概述)
2. [控制模型分析](#2-控制模型分析)
3. [配置结构体分析](#3-配置结构体分析)
4. [编码器速度计算与θ预测](#4-编码器速度计算与θ预测)
5. [与纯FOC控制的对比](#5-与纯foc控制的对比)
6. [示例配置](#6-示例配置)
7. [参数调节指南](#7-参数调节指南)
8. [力位混合控制代码实现分析](#8-力位混合控制代码实现分析)

---

## 1. 概述

### 1.1 混合控制的定位

力位混合控制 (Hybrid Force-Position Control) 是一种**阻抗控制**(Impedance Control)方法，使电机关节表现出期望的机械阻抗特性——弹簧-阻尼系统。它不是单纯的位置控制或力矩控制，而是让关节对外表现为"可编程弹簧"。

```
┌───────────────────────────────────────────────────────────────────────┐
│                    力位混合控制的物理模型                               │
├───────────────────────────────────────────────────────────────────────┤
│                                                                        │
│                     虚拟弹簧-阻尼系统                                   │
│                                                                        │
│          kp (刚度)                                                     │
│         ╱╲╱╲╱╲╱╲                                                     │
│   q_des ───┬───[ kp ]────┬─── τ_output ───▶ 电机                      │
│            │              │                                            │
│         q_actual      [ kd ] (阻尼)                                    │
│                          │                                             │
│                       dq_actual                                        │
│                                                                        │
│   + τ_ff: 前馈力矩（重力补偿、摩擦力补偿等）                            │
│                                                                        │
└───────────────────────────────────────────────────────────────────────┘
```

### 1.2 应用场景

| 场景 | 描述 | 关键参数特征 |
|------|------|-------------|
| **机器人关节控制** | 协作机器人、机械臂关节 | 中等刚度(kp=10~100)，需力矩限幅 |
| **柔顺装配** | 精密零件插入、自适应贴合 | 低刚度(kp=1~10)，允许环境力推动 |
| **人机交互** | 拖动示教、力反馈设备 | 极低刚度(kp=0.1~5)，人手可推动 |
| **精密伺服定位** | 工业伺服、光学平台 | 高刚度(kp=100~500+)，快速响应 |
| **力矩/力控模式** | 恒力打磨、力矩限制 | kp=0，仅前馈力矩+阻尼 |

### 1.3 v2 独有特性

> **重要：** 力位混合控制是 MCL V2 独有的功能，V1 (`hpm_mcl_v1`) 中不存在此模块。

V2 新增特性：

```
┌─────────────────────────────────────────────────────────────────┐
│                   MCL V2 混合控制 v2 独有特性                     │
├─────────────────────────────────────────────────────────────────┤
│  ✓ 新增 mcl_mode_hybrid_foc (6) 运行模式                         │
│  ✓ 独立模块 hpm_mcl_hybrid_ctrl.{h,c} 实现 PD+前馈控制律         │
│  ✓ 速度低通滤波 + 死区处理消除抖动                               │
│  ✓ 力矩限幅保护电机和驱动器                                     │
│  ✓ 支持硬件 CLC 加速电流内环 (可选 MCL_HARDWARE_HYBRID_LOOP)     │
│  ✓ hpm_mcl_loop_init() 支持 hw_loop 参数传递                     │
│  ✓ 与编码器/路径规划/传感器观测器无缝集成                        │
└─────────────────────────────────────────────────────────────────┘
```

HPM MCL V2 六种运行模式中，`mcl_mode_hybrid_foc` (值为6) 与 `mcl_mode_foc` 共享同一个底层调度函数 [hpm_mcl_current_foc_loop()](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L624-L640)，但在硬件混合模式下会通过 [mcl_hw_loop_t](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L120) 使用 VSC/CLC/QEO 硬件模块加速电流环。

---

## 2. 控制模型分析

### 2.1 核心控制律

混合控制的核心是一个**PD + 前馈**的力矩控制器，控制律如下：

```
tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)
```

其中：

| 符号 | 含义 | 单位 |
|------|------|------|
| τ (tau) | 输出力矩指令 | N·m |
| τ_ff (tau_ff) | 前馈力矩（重力补偿等） | N·m |
| kp | 位置刚度增益 | N·m/rad |
| kd | 阻尼增益 | N·m·s/rad |
| q_des | 期望位置 | rad |
| q (q_actual) | 实际位置 | rad |
| dq_des | 期望速度（位置保持时为0） | rad/s |
| dq (dq_actual) | 实际速度（经滤波处理） | rad/s |

**源码出处：** [hpm_mcl_hybrid_ctrl.h:L16](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h#L16) 和 [hpm_mcl_hybrid_ctrl.c:L70-L87](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L70-L87)

### 2.2 控制框图

```
                      ┌──────────────┐
         tau_ff ──────┤              │
                      │      +       │──── tau_output ──▶ [力矩限幅] ──▶ 输出 τ
   q_des ────(+)──────┤              │
              (-)      │   kp * e_p  │
          ┌────┴────┐  │      +      │
          │  kp     │  │              │
          └─────────┘  │   kd * e_v  │
                       │      +      │
   dq_des ───(+)──────┤              │
              (-)      │              │
          ┌────┴────┐  │              │
          │  kd     │  └──────────────┘
          └─────────┘
              ↑
    ┌─────────────────┐
    │ q_actual ───────┤
    │ dq_actual ──────┤── [LPF] ──▶ [Deadzone] ──▶ 速度反馈
    │   (编码器)      │
    └─────────────────┘
```

### 2.3 各组成部分详解

#### 2.3.1 前馈力矩 τ_ff

前馈力矩直接叠加到输出，用于补偿已知的外部力矩，不经过反馈回路。

**典型用途：**
- **重力补偿：** 对机器人关节预先施加克服重力的力矩
- **摩擦力补偿：** 补偿库仑摩擦或粘滞摩擦
- **动力学前馈：** 按逆动力学模型预计算需要力矩

**设置接口：** [mcl_hybrid_ctrl_set_tau_ff()](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h#L110-L112)

#### 2.3.2 位置比例补偿 (kp 项)

```
tau_p = kp * (q_des - q_actual)
```

- **物理意义：** 模拟弹簧，偏离目标位置越远，恢复力矩越大
- **kp 越大：** 刚度越高，响应越快，但容易振荡
- **kp 越小：** 越柔顺，允许外力推动关节偏离目标位置

#### 2.3.3 速度阻尼补偿 (kd 项)

```
tau_d = kd * (dq_des - dq_actual_filtered)
```

- **物理意义：** 模拟阻尼器，抑制速度突变，消除振荡
- **kd 越大：** 阻尼越强，超调越小，但响应变慢
- **kd 越小：** 阻尼不足，容易振荡超调
- **临界阻尼设计：** kd ≈ 2 * sqrt(kp * J)，其中J为关节等效转动惯量

#### 2.3.4 速度低通滤波器 (LPF)

速度信号通常包含采样噪声，直接用于微分项会放大噪声。因此在实际速度 `dq_actual` 进入控制律之前，先经过一阶低通滤波器：

```
dq_filtered[n] = alpha * dq_actual[n] + (1 - alpha) * dq_filtered[n-1]
```

| alpha 值 | 滤波强度 | 效果 |
|----------|---------|------|
| 0.003 | 极强滤波 | 适合高噪声环境，响应较慢 |
| 0.05 | 中等滤波 | 通用推荐值 |
| 0.1 | 轻度滤波 | 低噪声编码器 |
| 0 (禁用) | 无滤波 | 原始速度信号 |
| 1.0 | 无滤波 | 等于原始值 |

**源码出处：** [hpm_mcl_hybrid_ctrl.c:L54-L61](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L54-L61)

**设置接口：** [mcl_hybrid_ctrl_set_speed_filter()](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h#L167-L171)

#### 2.3.5 速度死区 (Deadzone)

静止时编码器微小的速度波动会被 kd 项放大为力矩抖动，死区机制将低于阈值的速度强制归零：

```
if |speed_filtered| < speed_deadzone:
    speed_filtered = 0
```

- **典型值：** 0.01 ~ 0.1 rad/s
- **作用：** 消除静止时的力矩抖动，防止电机"滋滋"响

**源码出处：** [hpm_mcl_hybrid_ctrl.c:L63-L68](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L63-L68)

#### 2.3.6 力矩限幅

最终输出力矩经过饱和处理，防止过流损坏电机或驱动器：

```
tau_output = clamp(tau_raw, tau_min, tau_max)
```

**限幅值计算公式：**

```
tau_max = kt * I_max
```

其中 kt 为电机转矩常数 (N·m/A)，I_max 为允许最大电流 (A)。

**注意：** 当 tau_max 和 tau_min 都设为 0 时，限幅功能禁用。

**源码出处：** [hpm_mcl_hybrid_ctrl.c:L89-L95](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L89-L95)

**设置接口：** [mcl_hybrid_ctrl_set_limits()](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h#L153-L157)

### 2.4 mcl_hybrid_ctrl_step() 完整执行流程

```
mcl_hybrid_ctrl_step(cfg, state)
    │
    ├─ 1. 速度滤波 (若 speed_lpf_alpha > 0)
    │     state->speed_lpf = alpha * dq_actual + (1-alpha) * speed_lpf
    │
    ├─ 2. 速度死区 (若 speed_deadzone > 0)
    │     if |speed_filtered| < deadzone → speed_filtered = 0
    │
    ├─ 3. 计算位置误差
    │     pos_error = q_des - q_actual
    │
    ├─ 4. 计算速度误差
    │     vel_error = dq_des - speed_filtered
    │
    ├─ 5. 计算各控制项
    │     torque_p = kp * pos_error
    │     torque_d = kd * vel_error
    │
    ├─ 6. 合成总力矩
    │     tau_output = tau_ff + torque_p + torque_d
    │
    └─ 7. 力矩限幅
          clamp(tau_output, tau_min, tau_max)
```

**源码出处：** [hpm_mcl_hybrid_ctrl.c:L44-L96](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L44-L96)

---

## 3. 配置结构体分析

### 3.1 mcl_hybrid_ctrl_cfg_t — 配置参数

**定义位置：** [hpm_mcl_hybrid_ctrl.h:L32-L42](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h#L32-L42)

```c
typedef struct {
    float kp;               /* 位置刚度增益 (N·m/rad)           — 越大越硬 */
    float kd;               /* 阻尼增益 (N·m·s/rad)            — 越大越稳定 */
    float tau_ff;           /* 前馈力矩 (N·m)                  — 重力/摩擦补偿 */
    float q_des;            /* 期望位置 (rad)                   — 目标角度 */
    float dq_des;           /* 期望速度 (rad/s)                 — 位置保持=0 */
    float tau_max;          /* 最大输出力矩 (N·m), 0=不限幅    — 保护电机 */
    float tau_min;          /* 最小输出力矩 (N·m), 0=不限幅    — 保护电机 */
    float speed_lpf_alpha;  /* 速度低通滤波系数 (0~1), 0=禁用  — 典型0.003~0.1 */
    float speed_deadzone;   /* 速度死区 (rad/s)                — 典型0.01~0.1 */
} mcl_hybrid_ctrl_cfg_t;
```

**初始化默认值（全部为0）：** [mcl_hybrid_ctrl_init()](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L13-L29)

### 3.2 mcl_hybrid_ctrl_state_t — 运行状态

**定义位置：** [hpm_mcl_hybrid_ctrl.h:L47-L54](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h#L47-L54)

```c
typedef struct {
    float q_actual;     /* 当前实际位置 (rad)        — [输入] 从编码器获取 */
    float dq_actual;    /* 当前实际速度 (rad/s)      — [输入] 从编码器获取 */
    float tau_output;   /* 输出力矩指令 (N·m)       — [输出] 转换为 Iq 电流 */
    float pos_error;    /* 位置跟踪误差 (rad)        — [输出] 用于监控 */
    float vel_error;    /* 速度跟踪误差 (rad/s)      — [输出] 用于监控 */
    float speed_lpf;    /* 内部滤波后速度 (rad/s)    — [内部] 滤波器状态 */
} mcl_hybrid_ctrl_state_t;
```

**数据流示意：**

```
编码器 ──▶ q_actual ──┐
                       ├── mcl_hybrid_ctrl_step() ──▶ tau_output ──▶ Iq_ref ──▶ FOC电流环
编码器 ──▶ dq_actual ──┘
```

### 3.3 运行时 API 接口

| 函数 | 说明 |
|------|------|
| `mcl_hybrid_ctrl_init(cfg)` | 初始化配置，全部参数置零 |
| `mcl_hybrid_ctrl_step(cfg, state)` | 执行一次控制计算 |
| `mcl_hybrid_ctrl_set_kp(cfg, kp)` | 运行时调整位置刚度 |
| `mcl_hybrid_ctrl_set_kd(cfg, kd)` | 运行时调整阻尼系数 |
| `mcl_hybrid_ctrl_set_tau_ff(cfg, tau_ff)` | 设置前馈力矩 |
| `mcl_hybrid_ctrl_set_position(cfg, q_des)` | 设置目标位置 |
| `mcl_hybrid_ctrl_set_velocity(cfg, dq_des)` | 设置目标速度 |
| `mcl_hybrid_ctrl_set_trajectory(cfg, q_des, dq_des)` | 同时设置位置+速度 |
| `mcl_hybrid_ctrl_set_limits(cfg, tau_min, tau_max)` | 设置力矩限幅 |
| `mcl_hybrid_ctrl_set_speed_filter(cfg, alpha, deadzone)` | 设置速度滤波参数 |
| `mcl_hybrid_ctrl_set_params(cfg, kp, kd, tau_ff)` | 同时设置 kp/kd/tau_ff |
| `mcl_hybrid_ctrl_get_torque(state)` | 获取当前输出力矩 |
| `mcl_hybrid_ctrl_get_pos_error(state)` | 获取当前位置误差 |
| `mcl_hybrid_ctrl_get_vel_error(state)` | 获取当前速度误差 |

所有 setter 函数均为 `static inline`，零开销调用。

---

## 4. 编码器速度计算与 θ 预测

### 4.1 四种速度计算方法

编码器模块 [hpm_mcl_encoder.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h) 提供四种速度计算方法，通过 [mcl_encoder_cal_speed_function_t](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h#L32-L38) 枚举选择：

```
┌───────────────────────────────────────────────────────────────────┐
│                    编码器速度计算方法                               │
├───────────────┬───────────────────────────────────────────────────┤
│ T 法          │ speed = delta_theta / delta_time                  │
│               │ 原理：固定角度增量，测量时间间隔                    │
│               │ 优点：低速精度高                                   │
│               │ 缺点：高速时测不准                                 │
├───────────────┼───────────────────────────────────────────────────┤
│ M 法          │ speed = delta_pulse / delta_time                  │
│               │ 原理：固定时间窗口，计脉冲数                        │
│               │ 优点：高速精度高                                   │
│               │ 缺点：低速时脉冲数少，量化误差大                    │
├───────────────┼───────────────────────────────────────────────────┤
│ M/T 法        │ 低速用T法，高速用M法，自动切换                      │
│               │ 切换阈值：speed_abs_switch_m_t (rad/s)             │
│               │ 优点：全速度段精度较高                             │
├───────────────┼───────────────────────────────────────────────────┤
│ PLL 法        │ 通过锁相环跟踪角度，PI估计速度                      │
│               │ 参数：kp(比例), ki(积分)                           │
│               │ 优点：速度平滑，噪声低，适合无感观测器              │
├───────────────┼───────────────────────────────────────────────────┤
│ User 自定义   │ 用户通过 process_by_user 回调自行计算              │
│               │ 输入：theta, 输出：speed, theta_forecast           │
└───────────────┴───────────────────────────────────────────────────┘
```

### 4.2 θ 预测 (theta_forecast)

θ 预测机制 [hpm_mcl_encoder_get_forecast_theta()](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h#L230-L236) 用于补偿 FOC 控制中的**计算延迟**。在电流采样时刻到 PWM 更新时刻之间存在延时，用预测角度代替采样角度做逆 Park 变换，可提高控制精度。

```
theta_forecast = theta_current + speed * ts_delay
```

在 `mcl_mode_foc` / `mcl_mode_hybrid_foc` 模式下，当启用 `MCL_CFG_EN_THETA_FORECAST` 时：

- **Park 变换（采样→d/q）：** 使用当前角度 `theta`
- **逆 Park 变换（d/q→α/β）：** 使用预测角度 `theta_forecast`

**源码出处：** [hpm_mcl_loop.c:L372-L376](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L372-L376)

### 4.3 有感/无感切换架构

MCL V2 的编码器模块中通过 [mcl_encoder_t.sensorless](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h#L154-L157) 字段实现有感传感器和无感观测器之间的衔接：

```c
struct {
    bool *enable;         /* 指向 loop->cfg->enable_smc，控制无感使能 */
    float *theta;         /* 指向 SMC 观测器输出的角度 */
} sensorless;
```

**切换策略：**

```
                  低速段                        高速段
┌────────────────────────────┐    ┌────────────────────────────┐
│  有感编码器 (encoder)      │    │  无感 SMO (smc)            │
│  • 绝对位置精确            │    │  • 无需传感器，降低成本     │
│  • 零速/低速可靠           │    │  • 高速反电动势信号强       │
│  • 编码器分辨率高          │    │  • 追踪性好                │
└────────────────────────────┘    └────────────────────────────┘
                   │                           │
                   │    过渡区域（平滑切换）      │
                   └────────────┬──────────────┘
                                │
                    enable_smc: false → true
                                或
                    编码器超时 → 自动降级到 SMO
```

在混合控制场景下，`hpm_mcl_encoder_get_speed()` 返回的速度信号是混合控制的阻尼项输入，速度信号的平滑程度直接影响力矩输出的稳定性。因此推荐：

- **低速/静止态：** 使用编码器反馈，配合合理的 speed_lpf_alpha 和 speed_deadzone
- **高速运行：** 可切换到 SMO 估计速度，PLL 方法提供天然平滑的速度信号
- **过渡过程：** 使用 IIR 滤波器 [mcl_filter_iir_df1_t](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h#L140) 对速度进行额外平滑

---

## 5. 与纯 FOC 控制的对比

### 5.1 控制链路对比

```
┌───────────────────────────────────────────────────────────────────────┐
│                     纯 FOC 控制链（速度模式）                          │
├───────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  ω_ref ──▶ [速度PI] ──▶ Iq_ref ──▶ [电流PI] ──▶ Vq ──▶ [iPark]      │
│              ↑                                        │               │
│          ω_fbk (编码器)                               ▼               │
│                                                    [SVPWM]            │
│                                                       │               │
│                                                       ▼               │
│                                                     电机              │
│                                                                        │
│  特点：仅保证速度跟踪，位置是开环的                                    │
│                                                                        │
└───────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────────┐
│                   力位混合控制链（位置+力矩模式）                       │
├───────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  q_des ──▶ [混合控制] ──▶ τ_output ──▶ ÷kt ──▶ Iq_ref ──▶ [电流PI]  │
│              ↑    ↑                                                   │
│          q_fbk  dq_fbk (编码器)                                       │
│                                                                        │
│  tau_ff ────┘  (前馈力矩直接叠加)                                     │
│                                                                        │
│  特点：位置闭环 + 力矩前馈 = 高刚度 + 低跟踪误差                       │
│                                                                        │
└───────────────────────────────────────────────────────────────────────┘
```

### 5.2 关键区别总结

| 对比维度 | 纯 FOC (速度模式) | 力位混合控制 |
|---------|------------------|-------------|
| **控制目标** | 速度跟踪 | 位置跟踪 + 力矩输出 |
| **反馈量** | 速度 (ω) | 位置 (q) + 速度 (dq) |
| **外环类型** | PI 速度环 | PD + 前馈 力矩环 |
| **闭环特性** | 速度闭环，位置开环 | 全闭环，具有阻抗特性 |
| **刚度** | 取决于速度环增益，速度波动=位置漂移 | 通过 kp 直接设定位置刚度 |
| **外力响应** | 速度环抵抗外力（位置不可预测） | 可配置柔顺响应（力位折衷） |
| **力矩↔电流转换** | PI 输出直接是 Iq_ref | tau_output / kt = Iq_ref |
| **前馈通道** | 无 | tau_ff 直接叠加 |
| **适用场景** | 风机、泵、主轴 | 机器人关节、伺服、柔顺控制 |

### 5.3 混合控制的核心优势

**1. 更高刚度**
- 速度环的等效位置刚度取决于 PI 参数和编码器分辨率，通常有限
- 混合控制直接用 kp 设定位置刚度，理论上可以无穷大

**2. 更低跟踪误差**
- 纯速度模式：位置误差 = 速度积分误差，随时间累积
- 混合控制：直接闭环位置，误差 = q_des - q，不累积

**3. 可编程阻抗**
- 用户可在线修改 kp/kd，实现变刚度/变阻尼的柔顺控制
- 例如：自由空间时 kp 大（精密定位），接触时 kp 小（柔顺）

**4. 力矩前馈解耦**
- 补偿重力力矩后，位置环只需处理动态偏差
- 减少稳态误差，提高定位精度

---

## 6. 示例配置

### 6.1 典型交互流程

```c
/* ===== 1. 初始化 ===== */
mcl_hybrid_ctrl_init(&cfg);
memset(&state, 0, sizeof(state));

/* ===== 2. 设置控制参数 ===== */
mcl_hybrid_ctrl_set_kp(&cfg, 0.06f);        // 位置刚度
mcl_hybrid_ctrl_set_kd(&cfg, 0.001429f);    // 阻尼
mcl_hybrid_ctrl_set_position(&cfg, 0.0f);    // 目标位置 0°
mcl_hybrid_ctrl_set_velocity(&cfg, 0.0f);    // 位置保持
mcl_hybrid_ctrl_set_limits(&cfg, -0.5f, 0.5f); // ±0.5 N·m 限幅
mcl_hybrid_ctrl_set_speed_filter(&cfg, 0.003f, 0.1f);  // 强滤波 + 死区

/* ===== 3. 控制循环 (ADC中断, 20kHz) ===== */
void isr_adc(void)
{
    /* 获取编码器反馈 */
    hpm_mcl_encoder_get_absolute_theta(&encoder, &state.q_actual);
    state.dq_actual = hpm_mcl_encoder_get_speed(&encoder);

    /* 执行力位混合控制 */
    mcl_hybrid_ctrl_step(&cfg, &state);

    /* 力矩转电流 → 发给FOC电流环 */
    user_current.enable = true;
    user_current.value = state.tau_output / kt;  /* kt: 转矩常数 */
    hpm_mcl_loop_set_current_q(&loop, user_current);

    /* 执行 FOC 电流环 */
    hpm_mcl_loop(&loop);
}

/* ===== 4. 用户交互：设定目标位置 ===== */
mcl_hybrid_ctrl_set_position(&cfg, target_angle_rad);
```

### 6.2 bldc_foc 示例中的混合控制配置

以下来自 [bldc_foc.c](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c) 示例的硬件混合环配置（`MCL_HARDWARE_HYBRID_LOOP_ENABLE` 宏使能时）：

**模式设置：**

```c
motor0.cfg.loop.mode = mcl_mode_hybrid_foc;
```

**硬件加速环初始化：**

```c
motor0.cfg.hw_loop.clc_cfg.base = BOARD_CLC;
motor0.cfg.hw_loop.callback.clc_convert_input = clc_convert_input;
motor0.cfg.hw_loop.callback.clc_convert_output = clc_convert_output;
hpm_mcl_hw_loop_init(&motor0.hw_loop, &motor0.cfg.hw_loop);
hpm_mcl_enable_clc_hardware_loop(&motor0.hw_loop);
```

**Loop 初始化（带 hw_loop 参数）：**

```c
hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                  &motor0.encoder, &motor0.analog, &motor0.control,
                  &motor0.drivers, NULL, &motor0.hw_loop);
```

### 6.3 不同应用场景参数速查

| 场景 | kp | kd | tau_limit | filter_alpha | deadzone |
|------|----|----|-----------|-------------|----------|
| **空载直驱测试** | 0.06 | 0.0014 | ±0.5 | 0.003 | 0.1 |
| **小型协作关节** | 5.0 | 4.5 | ±5.0 | 0.05 | 0.01 |
| **工业伺服定位** | 200.0 | 28.0 | ±50.0 | 0.1 | 0.005 |
| **柔顺拖动示教** | 0.5 | 1.4 | ±2.0 | 0.02 | 0.02 |
| **带减速器关节(N=100)** | 50.0 | 14.0 | ±30.0 | 0.05 | 0.01 |

---

## 7. 参数调节指南

### 7.1 调参原则

```
┌──────────────────────────────────────────────────────────────┐
│                     混合控制参数整定流程                       │
├──────────────────────────────────────────────────────────────┤
│                                                               │
│  Step 1: 确定力矩限幅                                         │
│    tau_max = kt * I_max                                       │
│    → 根据电机额定电流和转矩常数计算                             │
│                                                               │
│  Step 2: 从低 kp 起步                                         │
│    kp = 0.1 ~ 1.0（根据惯量判断）                              │
│                                                               │
│  Step 3: 设置 kd 为临界阻尼                                    │
│    kd ≈ 2 * sqrt(kp)  (未知惯量时的简化公式)                   │
│                                                               │
│  Step 4: 设置速度滤波                                          │
│    alpha = 0.01 ~ 0.1                                         │
│    deadzone = 0.01 ~ 0.1 rad/s                                │
│                                                               │
│  Step 5: 测试响应                                              │
│    振荡/超调 → 增大 kd 或减小 kp                                │
│    响应太慢 → 增大 kp (同时调 kd = 2*sqrt(kp))                  │
│    静止抖动 → 增大 speed_lpf_alpha 或 speed_deadzone            │
│                                                               │
│  Step 6: 逐步增大 kp 至期望刚度                                 │
│    每次增大 kp 后, 按 kd = 2*sqrt(kp) 同步调整 kd               │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

### 7.2 常见问题诊断

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 位置振荡/超调 | kd 过小 | 增大 kd |
| 响应迟缓 | kp 过小 或 kd 过大 | 增大 kp，kd = 2*sqrt(kp) |
| 静止时抖动/异响 | 速度噪声过大 | 减小 speed_lpf_alpha（更强滤波），增大 deadzone |
| 扰动后不能回位 | kp 过小 | 增大 kp |
| 力矩频繁达到限幅 | tau_limit 过小 或 kp 过大 | 检查限幅合理性，适当增大 |
| 定位有稳态误差 | tau_ff 补偿不够 | 增加前馈力矩补偿重力/摩擦 |
| 速度滤波器导致响应振荡 | alpha 太小引入相位滞后 | 适当增大 alpha，牺牲滤波换取相位裕度 |

### 7.3 临界阻尼参考公式

当已知关节等效转动惯量 J 时：

```
kd = 2 * sqrt(kp * J)
```

当 J 未知时（简化）：

```
kd ≈ 2 * sqrt(kp)
```

注意：简化公式假设 J ≈ 1，若实际惯量远大于1，kd 会不足导致振荡。建议用简单测试（步进响应的衰减比）标定实际惯量。

---

## 8. 力位混合控制代码实现分析

本章节从源码层面逐行解析 `mcl_hybrid_ctrl_step()` 的实现细节，将第2章的控制理论与实际代码一一对应。

**源码位置：** [hpm_mcl_hybrid_ctrl.c:L44-L96](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.c#L44-L96)

---

### 8.1 混合控制步进函数 (`mcl_hybrid_ctrl_step`)

#### 完整代码

```c
void mcl_hybrid_ctrl_step(const mcl_hybrid_ctrl_cfg_t *cfg, mcl_hybrid_ctrl_state_t *state)
{
    float pos_error, vel_error;
    float torque_p, torque_d;
    float speed_filtered;

    if (cfg == NULL || state == NULL) {
        return;
    }

    /* Speed filtering with low-pass filter */
    if (cfg->speed_lpf_alpha > 0.0f) {
        state->speed_lpf = cfg->speed_lpf_alpha * state->dq_actual +
                          (1.0f - cfg->speed_lpf_alpha) * state->speed_lpf;
        speed_filtered = state->speed_lpf;
    } else {
        speed_filtered = state->dq_actual;
    }

    /* Apply speed deadzone */
    if (cfg->speed_deadzone > 0.0f) {
        if (speed_filtered > -cfg->speed_deadzone && speed_filtered < cfg->speed_deadzone) {
            speed_filtered = 0.0f;
        }
    }

    /* Calculate position error: e_p = q_des - q_actual */
    pos_error = cfg->q_des - state->q_actual;

    /* Calculate velocity error: e_v = dq_des - dq_filtered */
    vel_error = cfg->dq_des - speed_filtered;

    /* Store errors for monitoring */
    state->pos_error = pos_error;
    state->vel_error = vel_error;

    /* Proportional term: tau_p = kp * e_p */
    torque_p = cfg->kp * pos_error;

    /* Derivative term: tau_d = kd * e_v */
    torque_d = cfg->kd * vel_error;

    /* Hybrid control law: tau = tau_ff + tau_p + torque_d */
    state->tau_output = cfg->tau_ff + torque_p + torque_d;

    /* Apply torque limits if configured */
    if (state->tau_output > cfg->tau_max) {
        state->tau_output = cfg->tau_max;
    }
    if (state->tau_output < cfg->tau_min) {
        state->tau_output = cfg->tau_min;
    }
}
```

#### 逐段解析

**1) 空指针防护 (L50-L52)**

```c
if (cfg == NULL || state == NULL) {
    return;
}
```

防御性编程，防止空指针解引用导致 HardFault。在中断上下文中调用时，空指针崩溃会导致不可恢复的故障。`cfg` 声明为 `const` 指针，表明该函数不会修改配置参数，仅读取。

**2) 速度低通滤波 (L54-L61)** — 详见 [8.3](#83-速度低通滤波器)

**3) 速度死区处理 (L63-L68)** — 详见 [8.4](#84-速度死区处理)

**4) 误差计算与存储 (L70-L78)**

```c
pos_error = cfg->q_des - state->q_actual;
vel_error = cfg->dq_des - speed_filtered;
state->pos_error = pos_error;
state->vel_error = vel_error;
```

- 位置误差：$e_p = q_{des} - q_{actual}$，正值表示需要正向力矩驱动
- 速度误差：$e_v = \dot{q}_{des} - \dot{q}_{filtered}$，注意这里用的是滤波后的速度而非原始速度
- 误差值同时存入 `state` 结构体，供上层监控和调试使用

**5) 目标力矩计算 (L80-L87)** — 详见 [8.2](#82-弹簧-阻尼模型实现)

**6) 力矩限幅 (L89-L95)** — 详见 [8.5](#85-力矩限幅保护)

---

### 8.2 弹簧-阻尼模型实现

#### 核心代码

```c
torque_p = cfg->kp * pos_error;
torque_d = cfg->kd * vel_error;
state->tau_output = cfg->tau_ff + torque_p + torque_d;
```

#### 公式解析

混合控制的核心是弹簧-阻尼模型，输出力矩由三部分叠加：

$$\tau = \tau_{ff} + K_s \cdot (q_{des} - q_{actual}) + K_d \cdot (\dot{q}_{des} - \dot{q}_{actual})$$

| 项 | 代码对应 | 物理意义 |
|----|---------|---------|
| $\tau_{ff}$ | `cfg->tau_ff` | 前馈力矩，补偿已知外力（重力、摩擦） |
| $K_s \cdot e_p$ | `cfg->kp * pos_error` | 弹簧项，位置偏差产生的恢复力矩 |
| $K_d \cdot e_v$ | `cfg->kd * vel_error` | 阻尼项，速度偏差产生的阻尼力矩 |

#### Ks 参数物理意义

$K_s$（代码中 `kp`）是**位置刚度增益**，单位 N·m/rad：

- 物理上等效于弹簧的弹性系数，决定关节的"硬度"
- $K_s$ 越大，同样的位置偏差产生更大的恢复力矩，关节越"硬"
- $K_s = 0$ 时，位置项不起作用，系统退化为纯力矩+阻尼控制
- 工程上 $K_s$ 的取值范围从 0.06（空载直驱柔顺控制）到 200+（工业伺服高刚度定位）

#### Kd 参数物理意义

$K_d$（代码中 `kd`）是**阻尼增益**，单位 N·m·s/rad：

- 物理上等效于阻尼器的阻尼系数，抑制速度突变
- $K_d$ 越大，运动越平稳，超调越小，但响应变慢
- $K_d = 0$ 时，系统无阻尼，在 $K_s > 0$ 时必然振荡
- 临界阻尼设计：$K_d \approx 2\sqrt{K_s \cdot J}$，其中 $J$ 为关节等效转动惯量

#### 工程要点

1. **计算顺序不可调换**：先算 `torque_p` 和 `torque_d`，再合成 `tau_output`，保证每个中间变量可观测
2. **无积分项**：PD 控制器没有积分项，稳态误差由 `tau_ff` 前馈补偿，避免了积分饱和问题
3. **线性叠加**：三项力矩线性叠加，便于独立调试——先调 `kp`，再调 `kd`，最后加 `tau_ff`

---

### 8.3 速度低通滤波器

#### 滤波代码

```c
if (cfg->speed_lpf_alpha > 0.0f) {
    state->speed_lpf = cfg->speed_lpf_alpha * state->dq_actual +
                      (1.0f - cfg->speed_lpf_alpha) * state->speed_lpf;
    speed_filtered = state->speed_lpf;
} else {
    speed_filtered = state->dq_actual;
}
```

#### 原理分析

一阶低通滤波器（IIR 形式）的离散化递推公式：

$$y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1]$$

其中：
- $x[n]$ = `state->dq_actual`，当前采样速度
- $y[n-1]$ = `state->speed_lpf`，上一拍滤波输出
- $y[n]$ = 新的 `state->speed_lpf`，本拍滤波输出
- $\alpha$ = `cfg->speed_lpf_alpha`，滤波系数

**截止频率与 $\alpha$ 的关系**（采样周期 $T_s$）：

$$f_c = \frac{\alpha}{2\pi T_s (1 - \alpha)}$$

以 20kHz 采样率为例：

| $\alpha$ | 截止频率 $f_c$ | 滤波强度 | 适用场景 |
|----------|---------------|---------|---------|
| 0.003 | ~0.01 Hz | 极强 | 高噪声编码器、静止保持 |
| 0.05 | ~1.7 Hz | 中等 | 通用场景 |
| 0.1 | ~3.5 Hz | 轻度 | 低噪声编码器 |
| 0 (禁用) | — | 无 | 不滤波，直接使用原始速度 |

#### 逐行解析

- **L55**: 判断 `speed_lpf_alpha > 0`，等于 0 时跳过滤波，直接使用原始速度。这是一种"零开销禁用"设计——不需要额外的使能标志位
- **L56-L57**: 执行一阶 IIR 低通滤波。`state->speed_lpf` 既作为输入（上一拍状态），又作为输出（本拍结果），节省存储空间
- **L58**: 滤波结果赋给 `speed_filtered`，后续死区和控制律都使用滤波后的速度
- **L60**: alpha=0 时的旁路路径，`speed_filtered = state->dq_actual`，零延迟

#### 工程要点

1. **滤波器状态初始化**：`mcl_hybrid_ctrl_init()` 将 `speed_lpf` 置零，首次调用时 `speed_lpf = alpha * dq_actual + 0`，不会产生异常跳变
2. **相位滞后代价**：低通滤波器在抑制噪声的同时引入相位滞后，$\alpha$ 越小滞后越大。在高速跟踪场景中，过强的滤波会导致速度误差项响应迟缓，降低阻尼效果
3. **与死区的配合**：滤波器先于死区执行，确保进入死区判断的速度信号已经过平滑处理，避免噪声触发死区的频繁切换

---

### 8.4 速度死区处理

#### 死区代码

```c
if (cfg->speed_deadzone > 0.0f) {
    if (speed_filtered > -cfg->speed_deadzone && speed_filtered < cfg->speed_deadzone) {
        speed_filtered = 0.0f;
    }
}
```

#### 逻辑解析

死区判断的数学表达：

$$\dot{q}_{filtered}' = \begin{cases} 0, & |\dot{q}_{filtered}| < d_z \\ \dot{q}_{filtered}, & |\dot{q}_{filtered}| \geq d_z \end{cases}$$

其中 $d_z$ = `cfg->speed_deadzone`。

代码使用 `> -deadzone && < deadzone` 而非 `fabs() < deadzone`，两者逻辑等价，但前者避免了 `fabs()` 函数调用开销，在嵌入式实时环境中更高效。

#### 防止低速抖动的工程意义

1. **噪声放大问题**：编码器在低速/静止时，速度信号受量化噪声影响，典型波动幅度为 ±1 个脉冲对应的速度值。这个微小波动经 `kd` 放大后，会变成可感知的力矩抖动
2. **抖动表现**：电机静止时发出"滋滋"声，电流波形出现高频毛刺，长期运行可能导致电机发热和机械磨损
3. **死区作用**：将低于阈值的速度强制归零，使阻尼项 `kd * vel_error` 在低速时为零，消除抖动力矩
4. **副作用**：死区会引入微小的速度控制盲区，在 $|\dot{q}| < d_z$ 范围内阻尼项失效。因此 $d_z$ 不宜过大，典型值 0.01~0.1 rad/s

#### 工程要点

1. **死区与滤波的执行顺序**：先滤波再死区，这是正确的顺序。若先死区再滤波，死区归零后的阶跃信号会通过滤波器产生缓慢的过渡过程，反而引入额外的力矩波动
2. **死区宽度选择**：应略大于编码器在静止状态下的速度噪声峰值。可通过在静止时打印 `dq_actual` 的波动范围来确定
3. **不对称死区**：当前实现是对称死区（正负阈值相同）。若需要不对称死区（例如重力方向需要更大的死区），需修改代码

---

### 8.5 力矩限幅保护

#### 限幅代码

```c
if (state->tau_output > cfg->tau_max) {
    state->tau_output = cfg->tau_max;
}
if (state->tau_output < cfg->tau_min) {
    state->tau_output = cfg->tau_min;
}
```

#### 逻辑解析

力矩限幅的数学表达：

$$\tau_{output} = \text{clamp}(\tau_{raw},\ \tau_{min},\ \tau_{max})$$

代码使用两个独立的 `if` 判断而非 `else if`，这是因为：
- 当 `tau_max < tau_min`（配置错误）时，第二个 `if` 会覆盖第一个的结果，最终输出 `tau_min`
- 正常配置下 `tau_max > tau_min`，两个 `if` 互斥，等价于 `if-else if`

**限幅禁用条件**：当 `tau_max = 0` 且 `tau_min = 0` 时，任何正力矩都会被限幅为 0，任何负力矩也会被限幅为 0——这意味着输出恒为 0。因此，**两个限幅值都为 0 时实际上禁用了力矩输出**，而非"不限幅"。这与头文件注释中 "0 = no limit" 的描述存在语义差异，使用时需特别注意。

> **注意**：根据头文件注释 `tau_max: Maximum output torque limit (N*m), 0 = no limit`，设计意图是 0 表示不限幅。但当前实现中，`tau_max = 0` 会导致正力矩被截断为 0。若需实现 "0 = 不限幅" 的语义，应在限幅前增加判断条件 `if (cfg->tau_max > 0.0f)`。

#### 与电流环保护的关系

混合控制的力矩输出最终通过 $\tau / k_t$ 转换为电流参考值 $I_q^{ref}$ 送入 FOC 电流环：

$$I_q^{ref} = \frac{\tau_{output}}{k_t}$$

力矩限幅与电流环保护形成**两级保护**：

```
混合控制力矩限幅 (软件级)          FOC 电流环限幅 (硬件级)
        │                                │
  τ_output ∈ [τ_min, τ_max]      I_q ∈ [I_q_min, I_q_max]
        │                                │
        ▼                                ▼
  防止力矩指令过大              防止实际电流过大
  保护机械结构                  保护功率器件和电机
```

- **第一级（混合控制限幅）**：限制力矩指令，间接限制电流参考值。属于"预防性"保护
- **第二级（电流环限幅）**：直接限制 $I_q$ 电流，属于"兜底性"保护
- **设计原则**：混合控制的力矩限幅应小于电流环的等效力矩限幅，即 $\tau_{max} < k_t \cdot I_{q,max}$，确保软件限幅先于硬件限幅生效

#### 工程要点

1. **限幅值计算**：$\tau_{max} = k_t \cdot I_{rated}$，其中 $I_{rated}$ 为电机额定电流（非峰值电流）
2. **对称限幅**：通常设置 $\tau_{min} = -\tau_{max}$，实现正反向对称保护
3. **限幅后的刚度损失**：当输出被限幅时，位置误差持续存在但力矩无法继续增大，等效于"积分饱和"。由于本控制器无积分项，不存在传统意义上的积分饱和，但限幅会导致实际刚度低于设定值——在调试时应注意区分"参数问题"和"限幅问题"
4. **动态限幅**：某些应用需要根据运行状态动态调整限幅值（如低速时允许更大力矩），可通过在控制循环中调用 `mcl_hybrid_ctrl_set_limits()` 实现

---

*文档更新时间: 2026-05-26*