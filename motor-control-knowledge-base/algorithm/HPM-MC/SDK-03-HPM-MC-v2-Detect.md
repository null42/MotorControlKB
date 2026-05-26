# HPM MCL v2 — 离线参数检测模块详解 ★★★★☆ 🔰📚🔧

**文档版本：** v1.1
**生成日期：** 2026-05-26
**适用对象：** 电机控制工程师、嵌入式开发者
**前置知识：** C语言编程、FOC控制基础、PMSM电机模型

---

## 目录

1. [概述](#1-概述)
2. [检测算法原理](#2-检测算法原理)
3. [检测配置](#3-检测配置)
4. [检测流程详解](#4-检测流程详解)
5. [示例应用分析](#5-示例应用分析)
6. [注意事项与局限性](#6-注意事项与局限性)
7. [离线参数检测代码实现分析](#7-离线参数检测代码实现分析)

---

## 1. 概述

### 1.1 模块定位

离线参数检测是 hpm_mcl_v2 相对于 v1 的重大新增功能。v1（[hpm_bldc_define.h](../../../hpm_MC/middleware/hpm_mcl/inc/hpm_bldc_define.h)）仅提供 `hpm_motor_para_t` 手动填写的电机参数结构体，没有任何自动辨识能力。v2 新增完整的离线辨识流程，可在电机启动前自动测量以下关键参数：

| 参数 | 符号 | 单位 | 用途 |
|------|------|------|------|
| 定子电阻 | Rs | Ω | 电流环PI整定、观测器模型 |
| d轴电感 | Ld | H | 电流环PI整定、SMO观测器 |
| q轴电感 | Lq | H | 电流环PI整定、MTPA控制 |
| 相电感 | Ls | H | 方波控制、简化模型 |
| 转子磁链 | Flux | Wb | 速度环整定、无感观测器 |

### 1.2 工程价值

```
┌──────────────────────────────────────────────────────────────────┐
│                    离线参数检测工程价值                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐ │
│  │  免手动测量      │   │  免查电机手册    │   │  任意电机即插   │ │
│  │  无需LCR表       │   │  不用依赖厂家    │   │  即用            │ │
│  └─────────────────┘   └─────────────────┘   └─────────────────┘ │
│                                                                   │
│  ✓ 量产一致性：每台电机个体差异自动补偿                           │
│  ✓ 现场维护：更换电机后重新自学习即可                             │
│  ✓ 温度补偿：冷态/热态参数可分别辨识                               │
│  ✓ 安全保护：内置过流、超时、结果合理性校验                        │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

### 1.3 重要澄清：detect 模块 vs 参数检测

`hpm_mcl_detect.h` 中的 `mcl_detect_t` 是**故障检测**模块，用于运行时监控过流、编码器故障、模拟量异常等，与离线参数检测是两个独立功能。离线参数检测的核心代码位于：

- **配置数据结构：** [hpm_mcl_control.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h)
- **算法实现：** [hpm_mcl_control.c](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c)
- **状态机调度：** [hpm_mcl_loop.c](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c)

---

## 2. 检测算法原理

### 2.1 状态机流程设计

整个检测过程由 `mcl_loop_t` 中的调度状态机驱动，逐阶段串行执行：

```
                              ┌─────────┐
                              │  INIT   │
                              │ (初始化) │
                              └────┬────┘
                                   │
                                   ▼
                              ┌─────────┐
                         ┌───▶│  WAIT   │
                         │    │ (延时)  │
                         │    └────┬────┘
                         │         │ 延时计数达到 delay_times
                         │         ▼
                         │    ┌─────────┐    Ud增至Vbus仍未达到
                         │    │   RS    │◀─── 半额定电流 → ERROR
                         │    │ 电阻检测│
                         │    └────┬────┘
                         │         │ success
                         │         ▼
                         │    ┌─────────┐
                         │    │  WAIT   │
                         │    └────┬────┘
                         │         │
                         │         ▼
                         │    ┌─────────┐
                         │    │   LD    │
                         │    │ d轴电感 │
                         │    └────┬────┘
                         │         │ success
                         │         ▼
                         │    ┌─────────┐
                         │    │  WAIT   │
                         │    └────┬────┘
                         │         │
                         │         ▼
                         │    ┌─────────┐
                         │    │   LQ    │
                         │    │ q轴电感 │
                         │    └────┬────┘
                         │         │ success
                         │         ▼
                         │    ┌─────────┐
                         │    │  WAIT   │
                         │    └────┬────┘
                         │         │
                         │         ▼
                         │    ┌─────────┐
                         │    │   LS    │  Ls = Ld + Lq
                         │    │ 相电感  │  (纯计算，无注入)
                         │    └────┬────┘
                         │         │ success
                         │         ▼
                         │    ┌─────────┐
                         │    │  WAIT   │
                         │    └────┬────┘
                         │         │
                         │         ▼
                         │    ┌─────────┐
                         │    │  FLUX   │
                         │    │ 磁链检测│
                         │    └────┬────┘
                         │         │ success
                         │         ▼
                         │    ┌─────────┐
                         │    │  WAIT   │
                         │    └────┬────┘
                         │         │
                         │    ┌────▼────┐    任一结果为0或无穷大
                         │    │ 结果校验 │────────────────▶ ERROR
                         │    └────┬────┘
                         │         │ 校验通过
                         │         ▼
                         │    ┌─────────┐
                         │    │   END   │
                         │    │  完成   │
                         │    └─────────┘
                         │
                         └── WAIT 阶段：各阶段之间插入延迟
                             等待电流泄放干净，防止干扰下一项检测
```

**状态机定义：** [hpm_mcl_loop.h:L83-L92](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L83-L92)

```c
typedef enum {
    offline_param_detection_mode_init  = 0,  // 初始化
    offline_param_detection_mode_rs    = 1,  // 电阻检测
    offline_param_detection_mode_ld    = 2,  // d轴电感检测
    offline_param_detection_mode_lq    = 3,  // q轴电感检测
    offline_param_detection_mode_ls    = 4,  // 相电感计算
    offline_param_detection_mode_flux  = 5,  // 磁链检测
    offline_param_detection_mode_wait  = 6,  // 阶段间等待
    offline_param_detection_mode_end   = 7,  // 检测完成
    offline_param_detection_mode_error = 8,  // 检测失败
} mcl_offline_param_detection_mode_t;
```

### 2.2 电阻检测（Rs Detection）

**原理：** 注入直流电压（仅d轴），逐步递增 Ud，直到 αβ 轴合成电流幅值达到半额定电流，由欧姆定律 V=I×R 计算电阻。

**算法实现：** [hpm_mcl_control.c:L359-L377](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L359-L377)

```
┌──────────────────────────────────────────────────────────┐
│ Rs检测算法流程                                            │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  每个控制周期执行：                                         │
│                                                           │
│  Ud += ud_delta              // 逐步增大d轴电压            │
│  Uq = 0                     // q轴电压为0                 │
│  Is = |ialpha + j*ibeta|    // 计算电流幅值               │
│                                                           │
│  if (Is >= current_half_rated):                           │
│      检测完成                                               │
│      Rs = Ud_mem / Is        // V = I × R                 │
│      return success                                       │
│                                                           │
│  if (Ud > Vbus):                                          │
│      电压已达母线电压仍未达到目标电流 → 检测失败              │
│      return fail                                          │
│                                                           │
│  角度固定 θ = 0（转子不旋转）                               │
│  ── 等效于在α轴上施加直流电压 ──                            │
│                                                           │
└──────────────────────────────────────────────────────────┘
```

**数学公式：**

```
Rs = Ud_accumulated / Is_measured
```

**注入策略：** 电压步进增量由 `ud_delta` 配置。该值越小精度越高但检测越慢；越大速度越快但步进太粗可能导致电流超调。建议取值范围：0.0001 ~ 0.001（标幺值）。

### 2.3 电感检测（Ld / Lq Detection）

**原理：** 分别在 d 轴和 q 轴施加满幅母线电压（相当于阶跃响应），测量电流在 `inductor_detection_times` 个控制周期内的变化率 di/dt，由 V = L × di/dt 计算电感。

**Ld 检测算法：** [hpm_mcl_control.c:L379-L402](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L379-L402)

```
┌──────────────────────────────────────────────────────────┐
│ Ld检测算法流程                                            │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  Ud = Vbus        // d轴施加满幅母线电压                    │
│  Uq = 0           // q轴电压为0（确保电流沿d轴上升）        │
│  θ = 0            // 角度固定（不旋转转子）                 │
│                                                           │
│  tick_count = 0 时记录:                                   │
│    is_last = |ialpha + j*ibeta|                           │
│                                                           │
│  tick_count 递增...                                        │
│                                                           │
│  当 tick_count > inductor_detection_times:                │
│    is_now = |ialpha + j*ibeta|                            │
│    dt = (inductor_detection_times - 1) * detection_loop_ts│
│    di/dt = (is_now - is_last) / dt                        │
│    Ld = Vbus/2 / (di/dt)                                  │
│                                                           │
│  若 di/dt ≈ 0 → 检测失败（电流不上升）                     │
│                                                           │
└──────────────────────────────────────────────────────────┘
```

**Lq 检测：** 与 Ld 完全对称——Ud=0, Uq=Vbus（q轴注入）。代码位于 [hpm_mcl_control.c:L404-L424](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L404-L424)。

**数学公式：**

```
Ld = Vbus / 2 / (ΔIs / Δt)
Lq = Vbus / 2 / (ΔIs / Δt)
```

> **注：** 公式中 `Vbus/2` 是因为 SVPWM 调制下单相最大输出电压为母线电压的一半（相电压峰值 = Vbus/√3，等效到合成矢量约为 Vbus/2）。

**Ls 计算：** Ls 不是独立检测的，而是简单的代数计算：

```c
detection->result.ls = detection->result.ld + detection->result.lq;
```

见 [hpm_mcl_control.c:L426-L431](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L426-L431)。

### 2.4 磁链检测（Flux Detection）

**原理：** 反电动势法。在 q 轴注入满幅电压使电机开环旋转，对反电动势积分，通过低通滤波提取稳态值，取峰值作为磁链幅值。

**算法实现：** [hpm_mcl_control.c:L433-L457](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L433-L457)

```
┌──────────────────────────────────────────────────────────┐
│ Flux检测算法流程                                          │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  Ud_ref = 0, Uq_ref = Vbus   // q轴满压，转子开环旋转      │
│  PID电流环闭合（ud, uq 由PID输出）                         │
│  θ 由编码器实时读取（需要编码器/霍尔传感器）               │
│                                                           │
│  每个周期累加积分（αβ静止坐标系）：                         │
│    val0 += (ualpha - ialpha × Rs) × loop_ts               │
│    val1 += (ubeta  - ibeta  × Rs) × loop_ts               │
│                                                           │
│  减去电感压降得到磁链分量：                                  │
│    alpha = val0 - ialpha × Ls                             │
│    beta  = val1 - ibeta  × Ls                             │
│    flux_instant = sqrt(alpha² + beta²)                    │
│                                                           │
│  一阶低通滤波：                                             │
│    val_filter = val_filter × (1-k) + flux_instant × k     │
│                                                           │
│  取峰值保持：                                               │
│    val_filter_mem = max(val_filter_mem, val_filter)       │
│                                                           │
│  当 tick_count > flux_detection_times:                    │
│    Flux = val_filter_mem                                  │
│    return success                                        │
│                                                           │
└──────────────────────────────────────────────────────────┘
```

**数学原理（反电动势积分法）：**

```
ψα = ∫(Uα - Iα × Rs) dt - Iα × Ls
ψβ = ∫(Uβ - Iβ × Rs) dt - Iβ × Ls
ψf = sqrt(ψα² + ψβ²)
```

**注入策略：** `flux_detection_times` 决定了检测持续时间，时间越长磁链峰值越容易被捕捉到，但电机转动也越多。典型配置为 10000 个控制周期。

### 2.5 注入信号策略总结

| 检测项 | 注入方式 | 角度处理 | 转子状态 | 关键配置参数 |
|--------|---------|---------|---------|-------------|
| Rs | d轴步进直流 | θ=0 固定 | **静止** | `ud_delta`, `current_half_rated` |
| Ld | d轴满压阶跃 | θ=0 固定 | **静止** | `inductor_detection_times` |
| Lq | q轴满压阶跃 | θ=0 固定 | **静止** | `inductor_detection_times` |
| Ls | 无注入（计算） | — | — | — |
| Flux | q轴满压旋转 | 编码器实时 | **旋转** | `flux_detection_times`, `lowpass_k` |

> **关键差异：** Rs/Ld/Lq 三个阶段要求电机转子**完全静止**（θ固定为0）；Flux 阶段则需要转子**自由旋转**（θ由编码器实时读取）。

---

## 3. 检测配置

### 3.1 完整配置结构体

**定义位置：** [hpm_mcl_control.h:L159-L168](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L159-L168)

```c
typedef struct {
    float current_half_rated;       // 半额定电流（Rs检测目标电流）
    float ud_delta;                 // 电阻检测电压步进增量
    float vbus;                     // 母线电压
    float inductor_detection_times; // 电感检测周期数（通常1ms对应的周期数）
    float flux_detection_times;     // 磁链检测周期数（越长越准）
    float delay_times;              // 阶段间延迟周期数（等电流泄放）
    float detection_loop_ts;        // 检测循环调用间隔（=电流环周期）
    float lowpass_k;                // 磁链低通滤波器系数（0~1）
} mcl_control_offline_param_detection_cfg_t;
```

### 3.2 各参数详解

| 参数 | 含义 | 取值范围 | 调参建议 |
|------|------|---------|---------|
| `current_half_rated` | Rs检测目标电流 | 0 < value < i_rated | 设为 `i_rated / 2`，保证安全同时有足够信噪比 |
| `ud_delta` | 电压步进增量 | 0.00005 ~ 0.005 | 小值=高精度慢速，大值=快速但可能超调。典型 0.0001 |
| `vbus` | 母线电压（V） | 等于实际供电电压 | 必须准确，直接影响所有公式计算 |
| `inductor_detection_times` | 电感检测持续周期数 | 10 ~ 100 | 太短电流变化量小噪声大，太长可能过流。典型 20 |
| `flux_detection_times` | 磁链检测持续周期数 | 1000 ~ 50000 | 越长越易捕捉峰值，但电机转动越多。典型 10000 |
| `delay_times` | 阶段间等待周期数 | 50 ~ 500 | 确保上阶段电流完全泄放。典型 200 |
| `detection_loop_ts` | 检测循环周期（s） | = 电流环周期 | 必须与 `current_loop_ts` 一致 |
| `lowpass_k` | 磁链低通滤波系数 | 0.001 ~ 0.1 | 越小滤波越强但响应越慢。典型 0.01 |

### 3.3 运行时数据结构

**定义位置：** [hpm_mcl_control.h:L174-L190](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L174-L190)

```c
typedef struct {
    mcl_control_offline_param_detection_cfg_t cfg;
    mcl_offline_param_detection_result_t result;
    float tick_count;                   // 通用计数器
    struct {
        float ud_mem;                   // Rs检测：累积Ud值
    } rs;
    struct {
        float is_last;                  // Ld/Lq检测：初始电流值
    } ls;
    struct {
        float val0_mem;                 // 磁链积分 α分量累加
        float val1_mem;                 // 磁链积分 β分量累加
        float val_filter;               // 低通滤波后瞬时值
        float val_filter_mem;           // 峰值保持
    } flux;
} mcl_control_offline_param_detection_t;
```

### 3.4 检测结果结构体

**定义位置：** [hpm_mcl_control.h:L40-L46](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L40-L46)

```c
typedef struct {
    float rs;       // 定子电阻 (Ω)
    float ld;       // d轴电感 (H)
    float lq;       // q轴电感 (H)
    float ls;       // 相电感 = Ld + Lq (H)
    float flux;     // 转子磁链 (Wb)
} mcl_offline_param_detection_result_t;
```

### 3.5 典型配置示例

来自 [bldc_offline_param_detection.c](../../../hpm_MC/samples/motor_ctrl/bldc_offline_param_detection/src/bldc_offline_param_detection.c)：

```c
motor0.cfg.control.offline_param_detection_cfg.cfg.lowpass_k = 0.01;
motor0.cfg.control.offline_param_detection_cfg.cfg.ud_delta = 0.0001;
motor0.cfg.control.offline_param_detection_cfg.cfg.vbus = 24;
motor0.cfg.control.offline_param_detection_cfg.cfg.flux_detection_times = 10000;
motor0.cfg.control.offline_param_detection_cfg.cfg.current_half_rated = 1.5;   // 3/2
motor0.cfg.control.offline_param_detection_cfg.cfg.inductor_detection_times = 20;
motor0.cfg.control.offline_param_detection_cfg.cfg.detection_loop_ts = 1.0/20000; // = current_loop_ts
motor0.cfg.control.offline_param_detection_cfg.cfg.delay_times = 200;
```

对应电机参数（雷赛 BLM57050-1000）：

| 参数 | 值 |
|------|-----|
| 额定电流 i_rated | 3 A |
| 额定功率 | 50 W |
| 极对数 pole_num | 2 |
| 母线电压 vbus | 24 V |
| 最大电流 i_max | 9 A |
| 最高转速 rpm_max | 3500 RPM |

---

## 4. 检测流程详解

### 4.1 完整执行流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                    离线参数检测完整执行流程                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  1. 硬件初始化                                                       │
│     ├── board_init()               // 板级初始化                     │
│     ├── motor_init()               // 配置所有模块                   │
│     └── adc_isr_enable()           // 使能ADC中断                    │
│                                                                      │
│  2. ADC中点校准                                                      │
│     └── motor_adc_midpoint()       // 采集200ms取平均                │
│                                                                      │
│  3. 电机对中（角度校准）                                              │
│     ├── hpm_mcl_loop_enable()      // 使能控制环                     │
│     └── bldc_foc_angle_align()     // Id=1A注入1s，零位标定          │
│                                                                      │
│  4. 切换检测模式                                                     │
│     ├── hpm_mcl_loop_mode_set(mcl_mode_offline_param_detection)     │
│     └── hpm_mcl_loop_enable_offline_param_detecion()                 │
│                                                                      │
│  5. 轮询等待完成                                                     │
│     ├── hpm_mcl_loop_offline_param_detection_is_done()              │
│     ├── hpm_mcl_loop_offline_param_detection_is_error()             │
│     └── hpm_mcl_loop_offline_param_detection_get_result()           │
│                                                                      │
│  6. 结果写回（应用层负责）                                           │
│     └── physical_motor_t.res  ← result.rs                          │
│         physical_motor_t.ld   ← result.ld                           │
│         physical_motor_t.lq   ← result.lq                           │
│         physical_motor_t.flux ← result.flux                         │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 初始化阶段

`hpm_mcl_control_detection_init()`（[hpm_mcl_control.c:L459-L476](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L459-L476)）清零所有内部状态：

```c
detection->result.flux = 0;
detection->result.ld = 0;
detection->result.lq = 0;
detection->result.rs = 0;
detection->result.ls = 0;
detection->flux.val0_mem = 0;
detection->flux.val1_mem = 0;
detection->flux.val_filter = 0;
detection->flux.val_filter_mem = 0;
detection->tick_count = 0;
detection->rs.ud_mem = 0;
```

### 4.3 安全保护机制

| 保护类型 | 实现位置 | 机制 |
|---------|---------|------|
| **过流保护** | Rs检测 | `Is >= current_half_rated` 立即停止电压递增 |
| **过压保护** | Rs检测 | `Ud > Vbus` 判定为检测失败 → ERROR |
| **电流零增量保护** | Ld/Lq检测 | `di/dt ≈ 0`（电流不上升）→ 判定失败 |
| **超时保护** | 各阶段 | tick_count 超过配置值自动进入下一状态 |
| **结果合理性校验** | 完成后 | [hpm_mcl_loop.c:L193-L204](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L193-L204)：任一结果为0、无穷大 → ERROR |
| **故障检测** | 独立mcl_detect_t | 运行时监控过流/编码器故障，`callback.disable_output` 紧急关断 |
| **死区补偿** | 强制启用 | 注释明确指出"Parameter detection function requires dead area compensation" |

### 4.4 阶段间延迟

每个阶段完成后进入 `WAIT` 状态，等待 `delay_times` 个周期，确保：

1. 上一个阶段注入的电压/电流完全泄放
2. 电机回到静止状态（如果上一阶段导致了运动）
3. 避免残余电流干扰下一阶段测量

`delay_times` 在初始化时从配置读取：`loop->const_time.offline_detection_wait_ts = control->cfg->offline_param_detection_cfg.cfg.delay_times;`

### 4.5 结果写回

检测结果存储在 `mcl_offline_param_detection_result_t` 中，最终需要写入 `physical_motor_t` 结构体（[hpm_mcl_physical.h:L28-L43](../../../hpm_MC/middleware/hpm_mcl_v2/hpm_mcl_physical.h#L28-L43)），典型做法由应用层实现：

```c
hpm_mcl_loop_offline_param_detection_get_result(&motor0.loop, &result);

motor0.cfg.mcl.physical.motor.res   = result.rs;
motor0.cfg.mcl.physical.motor.ld    = result.ld;
motor0.cfg.mcl.physical.motor.lq    = result.lq;
motor0.cfg.mcl.physical.motor.ls    = result.ls;
motor0.cfg.mcl.physical.motor.flux  = result.flux;
```

> **注意：** 示例代码中只打印结果，并未写回 `physical_motor_t`。实际量产代码需要完成此步骤，后续 FOC 控制才能使用正确的参数。

---

## 5. 示例应用分析

### 5.1 示例概述

| 属性 | 值 |
|------|-----|
| 路径 | [hpm_MC/samples/motor_ctrl/bldc_offline_param_detection/](../../../hpm_MC/samples/motor_ctrl/bldc_offline_param_detection/) |
| 电机型号 | 雷赛智能 BLM57050-1000 |
| 目标MCU | HPM系列（支持PWM/PWMV2双版本驱动） |
| 编码器 | ABZ增量编码器（QEI/QEIV2） |

### 5.2 关键配置参数

```c
#define PWM_FREQUENCY               (20000)   // 20kHz PWM
#define PWM_DEAD_AREA_TICK   (100)            // 死区时间
#define CURRENT_SET_TIME_MS    (200)          // ADC中点校准时长
#define MOTOR0_CURRENT_LOOP_BANDWIDTH (500)   // 电流环带宽500Hz

// 电机初始参数（作为安全边界，检测后覆盖）
motor.i_max = 9;        // 最大电流9A
motor.i_rated = 3;      // 额定电流3A
motor.vbus = 24;        // 24V供电
motor.ls = 0.00263;     // 预设电感值（仅用作电流环PI初值计算）
motor.res = 0.0011;     // 预设电阻值（仅用作电流环PI初值计算）
motor.flux = 0.0015;    // 预设磁链值
```

### 5.3 主循环逻辑

```c
while (1) {
    hpm_mcl_loop_enable_offline_param_detecion(&motor0.loop);

    while (!(hpm_mcl_loop_offline_param_detection_is_done(&motor0.loop))) {
        if (hpm_mcl_loop_offline_param_detection_is_error(&motor0.loop)) {
            printf("offline param detection error.\r\n");
        }
    }

    hpm_mcl_loop_offline_param_detection_get_result(&motor0.loop, &result);
    printf("flux: %f, ld: %f, lq: %f, ls: %f, rs: %f\r\n",
           result.flux, result.ld, result.lq, result.ls, result.rs);

    board_delay_ms(500);  // 周期性重复检测
}
```

### 5.4 上位机交互协议

v2 版本**未实现**上位机交互协议。当前检测结果直接通过 `printf` 输出到调试串口，没有命令帧格式。实际应用中如需上位机交互，需自行设计协议（如 Modbus、自定义帧）。

### 5.5 典型输出示例

```
motor parameter detection demo.
flux: 0.044300, ld: 0.000968, lq: 0.001808, ls: 0.002777, rs: 1.037876
flux: 0.044056, ld: 0.000992, lq: 0.001803, ls: 0.002795, rs: 1.022872
flux: 0.044366, ld: 0.000987, lq: 0.001785, ls: 0.002773, rs: 1.040307
```

重复多次检测结果高度一致（Rs ±5%，Ld/Lq ±2%），说明算法收敛性好。

---

## 6. 注意事项与局限性

### 6.1 必要条件

```
┌─────────────────────────────────────────────────────────────────────┐
│                     离线参数检测前置条件                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ✓ 电机可自由旋转：Flux检测阶段需要转子转动                           │
│  ✓ 编码器必须可用：Flux检测需要实时角度反馈                          │
│  ✓ 死区补偿必须使能：注释强制要求                                    │
│  ✓ ADC中点必须校准完成：否则电流偏置导致结果错误                      │
│  ✓ 角度对中必须先完成：θ=0标定是静止检测的前提                        │
│  ✓ 电机负载必须脱开：Flux检测阶段电机开环旋转                        │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.2 局限性分析

| 局限 | 说明 | 影响 |
|------|------|------|
| **静止假设** | Rs/Ld/Lq检测假设θ=0转子不动，大摩擦力可能导致角度漂移 | 电感/电阻检测偏差 |
| **线性假设** | 电感检测假设V=L×di/dt线性，未考虑磁路饱和 | 大电流时电感偏小 |
| **IPMSM凸极效应** | 仅区分Ld/Lq，未注入高频信号辨识 | SPMSM（Ld≈Lq）足够，IPMSM可接受 |
| **Flux依赖Rs/Ls** | 磁链积分公式使用已检测的Rs和Ls，误差会累积传递 | 前级误差影响Flux精度 |
| **Vbus/2近似** | 电感公式中的Vbus/2是SVPWM下的简化近似 | 极端调制比时有偏差 |
| **无温度补偿** | 检测时绕组温度取决于环境温度和预通电时间 | 冷态/热态结果不一致 |

### 6.3 误差来源

```
检测误差 = 硬件误差 + 模型误差 + 算法误差

  ┌─────────────────────────────────────────────────┐
  │ 硬件误差    │ ADC量化噪声、采样电阻精度、       │
  │             │ 运放增益偏差、ADC中点漂移          │
  ├─────────────┼───────────────────────────────────┤
  │ 模型误差    │ Vbus/2近似、忽略死区压降、        │
  │             │ 忽略逆变器非线性、磁路饱和         │
  ├─────────────┼───────────────────────────────────┤
  │ 算法误差    │ 低通滤波相位延迟(Flux)、          │
  │             │ 离散导数近似(Ld/Lq)、             │
  │             │ 步进电压量化(Rs)                  │
  └─────────────────────────────────────────────────┘
```

### 6.4 大功率电机适应性

对于大电流（>50A）电机，需要注意：

1. **Rs检测**：`ud_delta` 需相应增大，否则检测时间过长；但增大后精度降低。建议通过调整 `current_half_rated` 来间接控制检测速度。
2. **Ld/Lq检测**：大电流斜率变化快，`inductor_detection_times` 可适当减小（5~10）。
3. **Flux检测**：大惯量电机加速慢，`flux_detection_times` 需大幅增加才能达到稳定速度。
4. **电流采样**：确保运放增益不会在检测电流范围内饱和。

### 6.5 最佳实践建议

```
  ┌─────────────────────────────────────────────────────────────┐
  │                     最佳实践建议                              │
  ├─────────────────────────────────────────────────────────────┤
  │                                                              │
  │  1. 冷态检测：上电后立即执行，电机未运行过                    │
  │                                                              │
  │  2. 多次取平均：示例代码循环检测，建议取5~10次平均值写入      │
  │     physical_motor_t                                         │
  │                                                              │
  │  3. 异常值剔除：对flux/lq/ld/rs分别做±3σ剔除                 │
  │                                                              │
  │  4. 结果合理性范围校验（额外增加）：                           │
  │     Rs: 0.01 ~ 500 Ω                                         │
  │     Ld/Lq: 1e-6 ~ 0.5 H                                      │
  │     Flux: 0.0001 ~ 1.0 Wb                                    │
  │                                                              │
  │  5. 生产流程：先烧录检测固件 → 完成自学习 → 将参数烧录        │
  │     到Flash → 再烧录正式固件读取Flash中的参数                 │
  │                                                              │
  │  6. 安全：检测期间务必有人值守或远程可急停                    │
  │                                                              │
  └─────────────────────────────────────────────────────────────┘
```

---

## 7. 离线参数检测代码实现分析

> 本章从源码层面逐函数解析离线参数检测的完整实现，涵盖状态机调度、各参数检测算法及死区补偿机制。代码取自 [hpm_mcl_loop.c](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c) 和 [hpm_mcl_control.c](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c)。

### 7.1 离线检测状态机调度 (`hpm_mcl_detect_offline_para`)

**源码位置：** [hpm_mcl_loop.c:L120-L278](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L120-L278)

该函数是离线参数检测的**总调度器**，在 `mcl_mode_offline_param_detection` 模式下由 `hpm_mcl_loop()` 每个控制周期调用一次。它完成电流采样→坐标变换→状态机驱动→电压输出→死区补偿→PWM更新的完整信号链。

```c
hpm_mcl_stat_t hpm_mcl_detect_offline_para(mcl_loop_t *loop)
{
    float ia, ib, ic;
    float theta;
    float alpha, beta;
    float sinx, cosx;
    float sens_d, sens_q;
    float ref_d = 0, ref_q = 0;
    float ud, uq;
    hpm_mcl_stat_t status;
    mcl_control_svpwm_duty_t duty;
    mcl_control_dead_area_pwm_offset_t duty_offset;
#if !defined(MCL_CFG_EN_DEAD_AREA_COMPENSATION) || (MCL_CFG_EN_DEAD_AREA_COMPENSATION == 0)
    return mcl_fail;
#endif

    MCL_ASSERT_OPT(loop != NULL, mcl_invalid_pointer);
    if (loop->cfg->enable_offline_param_detection) {
        MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_a_current, &ia), loop->status, loop_status_fail);
        MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_b_current, &ib), loop->status, loop_status_fail);
        if (loop->rundata.offline_detection.mode == offline_param_detection_mode_flux) {
            theta = hpm_mcl_encoder_get_theta(loop->encoder);
        } else {
            theta = 0;
        }
        ic = -(ia + ib);
        sinx = loop->control->method.sin_x(theta);
        cosx = loop->control->method.cos_x(theta);
        loop->control->method.clarke(ia, ib, ic, &alpha, &beta);
        loop->control->method.park(alpha, beta, sinx, cosx, &sens_d, &sens_q);
        uq = 0;
        ud = 0;
        switch (loop->rundata.offline_detection.mode) {
        case offline_param_detection_mode_init:
            loop->rundata.offline_detection.tick_count = 0;
            loop->rundata.offline_detection.last_mode = 0;
            loop->rundata.offline_detection.last_ualpha = 0;
            loop->rundata.offline_detection.last_ubeta = 0;
            loop->control->method.offline_param_detection_init(&loop->control->cfg->offline_param_detection_cfg);
            loop->rundata.offline_detection.mode = offline_param_detection_mode_wait;
            loop->rundata.offline_detection.last_mode = offline_param_detection_mode_init;
            break;
        case offline_param_detection_mode_wait:
            loop->rundata.offline_detection.tick_count++;
            if (loop->rundata.offline_detection.tick_count > loop->const_time.offline_detection_wait_ts) {
                loop->rundata.offline_detection.tick_count = 0;
                switch (loop->rundata.offline_detection.last_mode) {
                case offline_param_detection_mode_init:
                    loop->rundata.offline_detection.mode = offline_param_detection_mode_rs;
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                    break;
                case offline_param_detection_mode_rs:
                    loop->rundata.offline_detection.mode = offline_param_detection_mode_ld;
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                    break;
                case offline_param_detection_mode_ld:
                    loop->rundata.offline_detection.mode = offline_param_detection_mode_lq;
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                    break;
                case offline_param_detection_mode_lq:
                    loop->rundata.offline_detection.mode = offline_param_detection_mode_ls;
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                    break;
                case offline_param_detection_mode_ls:
                    loop->rundata.offline_detection.mode = offline_param_detection_mode_flux;
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                    break;
                case offline_param_detection_mode_flux:
                    loop->rundata.offline_detection.result = loop->control->cfg->offline_param_detection_cfg.result;
                    if (MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.rs) ||
                        MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.ls) ||
                        MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.flux) ||
                        MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.lq) ||
                        MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.ld) ||
                        MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.ld) ||
                        MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.flux) ||
                        MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.lq) ||
                        MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.rs) ||
                        MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.ls)) {
                            loop->rundata.offline_detection.mode = offline_param_detection_mode_error;
                    } else {
                        loop->rundata.offline_detection.mode = offline_param_detection_mode_end;
                    }
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                    break;
                default:
                    loop->rundata.offline_detection.mode = offline_param_detection_mode_init;
                    loop->rundata.offline_detection.last_mode = offline_param_detection_mode_wait;
                break;
                }
            }
            break;
        case offline_param_detection_mode_rs:
            status = loop->control->method.offline_param_detection_rs(&loop->control->cfg->offline_param_detection_cfg, alpha, beta, &ud, &uq);
            if (status == mcl_success) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_wait;
                loop->rundata.offline_detection.last_mode = offline_param_detection_mode_rs;
            } else if (status == status_fail) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_error;
            }
            break;
        case offline_param_detection_mode_ld:
            if (loop->control->method.offline_param_detection_ld(&loop->control->cfg->offline_param_detection_cfg, alpha, beta, &ud, &uq) == mcl_success) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_wait;
                loop->rundata.offline_detection.last_mode = offline_param_detection_mode_ld;
            }
            break;
        case offline_param_detection_mode_lq:
            if (loop->control->method.offline_param_detection_lq(&loop->control->cfg->offline_param_detection_cfg, alpha, beta, &ud, &uq) == mcl_success) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_wait;
                loop->rundata.offline_detection.last_mode = offline_param_detection_mode_lq;
            }
            break;
        case offline_param_detection_mode_ls:
            if (loop->control->method.offline_param_detection_ls(&loop->control->cfg->offline_param_detection_cfg) == mcl_success) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_wait;
                loop->rundata.offline_detection.last_mode = offline_param_detection_mode_ls;
            }
            break;
        case offline_param_detection_mode_flux:
            status = loop->control->method.offline_param_detection_flux(&loop->control->cfg->offline_param_detection_cfg, alpha, beta,
                loop->rundata.offline_detection.last_ualpha, loop->rundata.offline_detection.last_ubeta, &ref_d, &ref_q);
            if (status == mcl_success) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_wait;
                loop->rundata.offline_detection.last_mode = offline_param_detection_mode_flux;
            } else if (status == status_fail) {
                loop->rundata.offline_detection.mode = offline_param_detection_mode_error;
            }
            loop->control->method.currentd_pid(ref_d, sens_d, &loop->control->cfg->currentd_pid_cfg, &ud);
            loop->control->method.currentq_pid(ref_q, sens_q, &loop->control->cfg->currentq_pid_cfg, &uq);
            break;
        case offline_param_detection_mode_end:
        case offline_param_detection_mode_error:
            break;
        default:
            loop->rundata.offline_detection.mode = offline_param_detection_mode_init;
            break;
        }
        loop->control->method.invpark(ud, uq, sinx, cosx, &alpha, &beta);
        loop->control->method.svpwm(alpha, beta, *loop->const_vbus, &duty);
        loop->rundata.offline_detection.last_ualpha = alpha;
        loop->rundata.offline_detection.last_ubeta = beta;
        loop->control->method.dead_area_polarity_detection(&loop->control->cfg->dead_area_compensation_cfg, sens_d, sens_q, theta,
                                                loop->const_time.dead_area_ts, *loop->const_time.current_ts, &duty_offset);
        duty.a += duty_offset.a_offset;
        duty.b += duty_offset.b_offset;
        duty.c += duty_offset.c_offset;
    } else {
        duty.a = 0;
        duty.b = 0;
        duty.c = 0;
    }
    hpm_mcl_drivers_update_bldc_duty(loop->drivers, duty.a, duty.b, duty.c);
    return  mcl_success;
}
```

#### 逐块解析

**① 编译期死区补偿强制检查（L132-L138）**

```c
#if !defined(MCL_CFG_EN_DEAD_AREA_COMPENSATION) || (MCL_CFG_EN_DEAD_AREA_COMPENSATION == 0)
    return mcl_fail;
#endif
```

若宏 `MCL_CFG_EN_DEAD_AREA_COMPENSATION` 未定义或为0，函数直接返回 `mcl_fail`。这是**编译期硬约束**——参数检测必须启用死区补偿，否则逆变器非线性导致的电压误差将使检测结果完全不可信。

**② 电流采样与角度选择（L142-L153）**

```c
MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_a_current, &ia), loop->status, loop_status_fail);
MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_b_current, &ib), loop->status, loop_status_fail);
if (loop->rundata.offline_detection.mode == offline_param_detection_mode_flux) {
    theta = hpm_mcl_encoder_get_theta(loop->encoder);
} else {
    theta = 0;
}
```

- 读取A相、B相电流，C相由基尔霍夫电流定律 `ic = -(ia + ib)` 计算
- **关键分支**：仅 Flux 阶段读取编码器角度（电机旋转），其余阶段 θ=0（电机静止，d轴锁定在α轴方向）

**③ Clarke→Park坐标变换（L149-L153）**

```c
ic = -(ia + ib);
sinx = loop->control->method.sin_x(theta);
cosx = loop->control->method.cos_x(theta);
loop->control->method.clarke(ia, ib, ic, &alpha, &beta);
loop->control->method.park(alpha, beta, sinx, cosx, &sens_d, &sens_q);
```

标准 FOC 前向通道：三相电流 → αβ静止坐标系 → dq旋转坐标系。θ=0 时 Park 变换退化为恒等变换（d=α, q=β），简化了 Rs/Ld/Lq 阶段的分析。

**④ 状态机 switch 分支（L156-L261）**

状态机流程如下：

```
init → wait → rs → wait → ld → wait → lq → wait → ls → wait → flux → wait → end/error
```

各分支行为：

| 状态 | 行为 | 成功后跳转 | 失败后跳转 |
|------|------|-----------|-----------|
| `init` | 清零计数器、调用 `detection_init`、转入 wait | wait(last=init) | — |
| `wait` | tick_count++，超时后根据 last_mode 决定下一阶段 | 下一检测阶段 | — |
| `rs` | 调用 `offline_param_detection_rs` | wait(last=rs) | error |
| `ld` | 调用 `offline_param_detection_ld` | wait(last=ld) | — |
| `lq` | 调用 `offline_param_detection_lq` | wait(last=lq) | — |
| `ls` | 调用 `offline_param_detection_ls`（纯计算） | wait(last=ls) | — |
| `flux` | 调用 `offline_param_detection_flux` + PID电流环 | wait(last=flux) | error |
| `end`/`error` | 空操作，保持当前状态 | — | — |

**wait 状态的作用：** 每个检测阶段完成后，注入的电压/电流需要时间泄放。wait 状态持续 `delay_times` 个控制周期，确保：
1. 上阶段残余电流衰减至零（LR时间常数 τ = L/R）
2. 电机回到机械静止（Rs/Ld/Lq 阶段要求）
3. 避免残余磁场/电流干扰下一阶段测量

**⑤ 结果有效性校验（L193-L206）**

```c
if (MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.rs) ||
    MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.ls) ||
    MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.flux) ||
    MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.lq) ||
    MCL_MATH_IS_ZERO(loop->rundata.offline_detection.result.ld) ||
    MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.ld) ||
    MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.flux) ||
    MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.lq) ||
    MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.rs) ||
    MCL_FLOAT_IS_INFINITY(loop->rundata.offline_detection.result.ls)) {
        loop->rundata.offline_detection.mode = offline_param_detection_mode_error;
} else {
    loop->rundata.offline_detection.mode = offline_param_detection_mode_end;
}
```

在 Flux 检测完成后的 wait 状态中，对所有5个结果做零值/无穷大检测：
- `MCL_MATH_IS_ZERO(x)`：检测 `|x| < 0.000001f`（近零值，说明检测失败或除零）
- `MCL_FLOAT_IS_INFINITY(x)`：检测 `x == ±INFINITY`（说明除零或数值溢出）

任一结果异常即判定整个检测流程失败。

**⑥ Flux 阶段的特殊处理——PID电流环（L252-L253）**

```c
loop->control->method.currentd_pid(ref_d, sens_d, &loop->control->cfg->currentd_pid_cfg, &ud);
loop->control->method.currentq_pid(ref_q, sens_q, &loop->control->cfg->currentq_pid_cfg, &uq);
```

Flux 是唯一使用 PID 电流环的阶段。原因是 Flux 阶段电机在旋转，需要闭环控制电流以维持稳定的 Id=0、Iq=Vbus 的运行状态。而 Rs/Ld/Lq 阶段直接开环输出电压（ud/uq 由检测算法直接设置），因为电机静止，开环即可。

**⑦ 电压输出链（L262-L270）**

```c
loop->control->method.invpark(ud, uq, sinx, cosx, &alpha, &beta);
loop->control->method.svpwm(alpha, beta, *loop->const_vbus, &duty);
loop->rundata.offline_detection.last_ualpha = alpha;
loop->rundata.offline_detection.last_ubeta = beta;
loop->control->method.dead_area_polarity_detection(..., &duty_offset);
duty.a += duty_offset.a_offset;
duty.b += duty_offset.b_offset;
duty.c += duty_offset.c_offset;
```

- `invpark`：dq → αβ 逆变换
- `svpwm`：αβ电压 → 三相占空比
- `last_ualpha/last_ubeta`：保存上一周期输出电压，供 Flux 检测积分使用
- 死区补偿：无条件执行（编译期已保证宏开启），在占空比上叠加偏移量

**⑧ 关闭输出（L271-L275）**

```c
} else {
    duty.a = 0;
    duty.b = 0;
    duty.c = 0;
}
```

若 `enable_offline_param_detection` 为 false，三相占空比清零，逆变器输出关闭。

---

### 7.2 定子电阻 Rs 检测实现 (`hpm_mcl_control_offline_param_detection_rs`)

**源码位置：** [hpm_mcl_control.c:L359-L377](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L359-L377)

**原理公式：**

$$R_s = \frac{U_d}{I_s}$$

其中 $U_d$ 为 d 轴累积施加电压，$I_s = \sqrt{I_\alpha^2 + I_\beta^2}$ 为 αβ 轴合成电流幅值。由于 θ=0 时 d 轴与 α 轴重合，d 轴电压等效于在 α 轴施加直流电压，电流仅沿 d 轴流动，此时电机静止无反电动势，欧姆定律严格成立。

```c
hpm_mcl_stat_t hpm_mcl_control_offline_param_detection_rs(mcl_control_offline_param_detection_t *detection, float ialpha, float ibeta, float *ud, float *uq)
{
    float is;

    *uq = 0;
    is = MCL_SUM_OF_SQUARE_MODE(ialpha, ibeta);
    if (is >= detection->cfg.current_half_rated) {
        detection->result.rs = detection->rs.ud_mem / is;
        return mcl_success;
    } else {
        detection->rs.ud_mem += detection->cfg.ud_delta;
        *ud = detection->rs.ud_mem;
        if (*ud > detection->cfg.vbus) {
            return mcl_fail;
        }
    }

    return mcl_running;
}
```

#### 逐行解析

| 行 | 代码 | 解析 |
|----|------|------|
| L363 | `*uq = 0;` | q轴电压置零，确保电流仅沿d轴建立 |
| L364 | `is = MCL_SUM_OF_SQUARE_MODE(ialpha, ibeta);` | 计算电流幅值 $I_s = \sqrt{I_\alpha^2 + I_\beta^2}$，宏定义为 `sqrtf(a*a + b*b)` |
| L365-367 | `if (is >= current_half_rated)` | 电流达到半额定电流时触发计算：$R_s = U_{d,mem} / I_s$，返回 `mcl_success` |
| L369 | `ud_mem += ud_delta;` | 每周期电压步进递增，缓慢建立电流避免冲击 |
| L370 | `*ud = ud_mem;` | 输出当前累积电压值给d轴 |
| L371-373 | `if (*ud > vbus)` | 电压超过母线电压仍未达到目标电流 → 检测失败（电机可能断路或参数配置错误） |
| L376 | `return mcl_running;` | 检测进行中，状态机继续调用 |

#### 工程要点

1. **步进式注入而非阶跃**：`ud_delta` 典型值 0.0001（标幺值），在 20kHz 控制频率下约 0.2s 达到半额定电流，避免电流冲击损坏驱动器
2. **半额定电流选择**：`current_half_rated = i_rated / 2`，兼顾信噪比（电流太小ADC噪声大）和安全性（不超过额定电流）
3. **θ=0消除反电动势**：转子静止时 $e = -\omega_e \psi_f = 0$，电压方程退化为纯电阻 $U = R_s \cdot I$，无需考虑运动反电动势
4. **ud_mem 的含义**：它是从零开始逐步累加的 d 轴电压值，代表实际施加到电机上的电压。当电流达到目标值时，此电压即为稳态电压，可直接用于欧姆定律计算

---

### 7.3 d 轴电感 Ld 检测实现 (`hpm_mcl_control_offline_param_detection_ld`)

**源码位置：** [hpm_mcl_control.c:L379-L402](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L379-L402)

**原理公式：**

$$L_d = \frac{V_{bus}/2}{\Delta I_s / \Delta t}$$

对 d 轴施加满幅母线电压阶跃，测量电流变化率 $\Delta I_s / \Delta t$，由电感基本方程 $V = L \cdot di/dt$ 计算电感。其中 $V_{bus}/2$ 是 SVPWM 调制下单相最大输出电压的近似值。

```c
hpm_mcl_stat_t hpm_mcl_control_offline_param_detection_ld(mcl_control_offline_param_detection_t *detection, float ialpha, float ibeta, float *ud, float *uq)
{
    float is;

    *uq = 0;
    *ud = detection->cfg.vbus;
    if (detection->tick_count == 0) {
        detection->ls.is_last = MCL_SUM_OF_SQUARE_MODE(ialpha, ibeta);
    }
    detection->tick_count++;

    if (detection->tick_count > detection->cfg.inductor_detection_times) {
        detection->tick_count = 0;
        is = MCL_SUM_OF_SQUARE_MODE(ialpha, ibeta);
        if (MCL_FLOAT_IS_ZERO(is - detection->ls.is_last)) {
            return mcl_fail;
        }
        detection->result.ld = detection->cfg.vbus / 2 / ((is - detection->ls.is_last) /
                        ((detection->cfg.inductor_detection_times - 1) * detection->cfg.detection_loop_ts));
        return mcl_success;
    }

    return mcl_running;
}
```

#### 逐行解析

| 行 | 代码 | 解析 |
|----|------|------|
| L383-384 | `*uq = 0; *ud = vbus;` | d轴施加满幅母线电压，q轴为零——电流仅沿d轴上升 |
| L385-387 | `if (tick_count == 0)` | 首个周期记录初始电流 $I_{s,0}$ 作为基准 |
| L388 | `tick_count++;` | 周期计数递增 |
| L390 | `if (tick_count > inductor_detection_times)` | 检测时间到达，计算结果 |
| L392 | `is = MCL_SUM_OF_SQUARE_MODE(...)` | 读取当前电流幅值 $I_{s,N}$ |
| L393-395 | `if (MCL_FLOAT_IS_ZERO(is - is_last))` | 电流无变化（$\Delta I \approx 0$）→ 电感无穷大或断路，返回失败 |
| L396-397 | `result.ld = vbus/2 / (ΔIs / Δt)` | 核心计算：$L_d = \frac{V_{bus}/2}{(I_{s,N} - I_{s,0}) / ((N-1) \cdot T_s)}$ |
| L401 | `return mcl_running;` | 检测进行中 |

**时间间隔计算**：$\Delta t = (inductor\_detection\_times - 1) \times detection\_loop\_ts$。使用 `-1` 是因为 N 个采样点之间有 N-1 个间隔，这是离散数值微分的标准处理。

#### d 轴锁定方向的原因

θ=0 时，Park 变换退化为 d=α, q=β。此时 Ud=Vbus 施加在 d 轴（即 α 轴），电流沿 d 轴方向线性上升。由于 d 轴对齐转子磁极方向，测得的电感即为 d 轴电感 Ld。这是 PMSM 磁路各向异性的直接体现——d 轴磁路经过永磁体，磁导率低，电感较小。

#### 工程要点

1. **阶跃响应法 vs 高频注入法**：本实现采用阶跃响应法（V=L·di/dt），实现简单但对磁路饱和敏感。大电流下电感值会偏小（磁饱和效应）
2. **inductor_detection_times 选择**：典型值 20（20kHz 下对应 1ms）。太短则 ΔI 小、噪声大；太长则电流可能过大
3. **Vbus/2 近似**：SVPWM 线性调制区单相最大相电压峰值为 $V_{bus}/\sqrt{3}$，过调制区可达 $V_{bus}/2$。此处使用 $V_{bus}/2$ 是一种工程近似，在满幅注入时接近实际输出电压

---

### 7.4 q 轴电感 Lq 检测实现 (`hpm_mcl_control_offline_param_detection_lq`)

**源码位置：** [hpm_mcl_control.c:L404-L424](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L404-L424)

**原理公式：**

$$L_q = \frac{V_{bus}/2}{\Delta I_s / \Delta t}$$

与 Ld 检测完全对称，唯一区别是注入方向从 d 轴旋转 90° 到 q 轴。

```c
hpm_mcl_stat_t hpm_mcl_control_offline_param_detection_lq(mcl_control_offline_param_detection_t *detection, float ialpha, float ibeta, float *ud, float *uq)
{
    float is;

    *ud = 0;
    *uq = detection->cfg.vbus;
    if (detection->tick_count == 0) {
        detection->ls.is_last = MCL_SUM_OF_SQUARE_MODE(ialpha, ibeta);
    }
    detection->tick_count++;

    if (detection->tick_count > detection->cfg.inductor_detection_times) {
        detection->tick_count = 0;
        is = MCL_SUM_OF_SQUARE_MODE(ialpha, ibeta);
        detection->result.lq = detection->cfg.vbus / 2 / ((is - detection->ls.is_last) /
                        ((detection->cfg.inductor_detection_times - 1) * detection->cfg.detection_loop_ts));
        return mcl_success;
    }

    return mcl_running;
}
```

#### 逐行解析

| 行 | 代码 | 与 Ld 的差异 |
|----|------|-------------|
| L408-409 | `*ud = 0; *uq = vbus;` | 电压从 d 轴切换到 q 轴——d 轴为零，q 轴满压 |
| L410-412 | `if (tick_count == 0)` | 同 Ld：记录初始电流 |
| L418-419 | `result.lq = vbus/2 / (ΔIs / Δt)` | 计算公式相同，结果写入 `result.lq` |

**注意**：Lq 检测没有像 Ld 那样做 `MCL_FLOAT_IS_ZERO(is - is_last)` 的零增量保护。这是一个代码不对称之处——如果 q 轴电流也不上升（极端情况），Lq 将计算为无穷大，最终被结果校验捕获。

#### 凸极电机 Ld ≠ Lq 的物理原因

对于内置式永磁同步电机（IPMSM）：

- **d 轴**：磁路经过永磁体，永磁体磁导率接近空气（$\mu_r \approx 1$），磁阻大，电感 $L_d$ 较小
- **q 轴**：磁路经过铁芯硅钢片，磁导率高（$\mu_r \approx 2000 \sim 5000$），磁阻小，电感 $L_q$ 较大

因此 IPMSM 满足 $L_q > L_d$（凸极比 $\rho = L_q / L_d > 1$），典型凸极比 2~5。而表贴式永磁同步电机（SPMSM）$L_d \approx L_q$。

从示例输出可以验证：
```
ld: 0.000968, lq: 0.001808  →  Lq/Ld ≈ 1.87
```
该电机具有明显的凸极效应。

#### 工程要点

1. **θ=0 下 q 轴注入的合理性**：虽然 θ=0 时 d 轴对齐 α 轴，但 q 轴注入的电压经过 invpark 变换后映射到 β 轴方向。由于电机静止且无反电动势，电流沿 β 轴（即 q 轴）线性上升，测得的确实是 q 轴电感
2. **Lq 检测在 Ld 之后**：必须先完成 Ld 检测再检测 Lq，因为两者共用 `ls.is_last` 和 `tick_count`，且 Ls = Ld + Lq 的计算依赖两者

---

### 7.5 磁链 Flux 检测实现 (`hpm_mcl_control_offline_param_detection_flux`)

**源码位置：** [hpm_mcl_control.c:L433-L457](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L433-L457)

**原理公式（反电动势积分法）：**

$$\psi_\alpha = \int (U_\alpha - I_\alpha R_s) \, dt - I_\alpha L_s$$

$$\psi_\beta = \int (U_\beta - I_\beta R_s) \, dt - I_\beta L_s$$

$$\psi_f = \sqrt{\psi_\alpha^2 + \psi_\beta^2}$$

这是从 PMSM 电压方程出发，减去电阻压降和电感压降后，对剩余部分（即反电动势）积分得到磁链。稳态下磁链幅值恒定，通过低通滤波+峰值保持提取。

```c
hpm_mcl_stat_t hpm_mcl_control_offline_param_detection_flux(mcl_control_offline_param_detection_t *detection, float ialpha, float ibeta,
            float ualpha, float ubeta, float *ref_d, float *ref_q)
{
    float alpha, beta, flux;

    *ref_d = 0;
    *ref_q = detection->cfg.vbus;
    detection->tick_count++;
    detection->flux.val0_mem += (ualpha - ialpha * detection->result.rs) * detection->cfg.detection_loop_ts;
    detection->flux.val1_mem += (ubeta - ibeta * detection->result.rs) * detection->cfg.detection_loop_ts;
    alpha = detection->flux.val0_mem - (ialpha * detection->result.ls);
    beta = detection->flux.val1_mem - (ibeta * detection->result.ls);
    flux = MCL_SUM_OF_SQUARE_MODE(alpha, beta);
    detection->flux.val_filter = detection->flux.val_filter * (1 - detection->cfg.lowpass_k) +
                                    flux * detection->cfg.lowpass_k;
    if (detection->flux.val_filter > detection->flux.val_filter_mem) {
        detection->flux.val_filter_mem = detection->flux.val_filter;
    }
    if (detection->tick_count > detection->cfg.flux_detection_times) {
        detection->tick_count = 0;
        detection->result.flux = detection->flux.val_filter_mem;
        return mcl_success;
    }
    return mcl_running;
}
```

#### 逐行解析

| 行 | 代码 | 解析 |
|----|------|------|
| L438-439 | `*ref_d = 0; *ref_q = vbus;` | 设定电流参考值：Id=0, Iq=Vbus（满压驱动电机旋转） |
| L440 | `tick_count++;` | 周期计数递增 |
| L441 | `val0_mem += (Uα - Iα·Rs)·Ts` | α轴反电动势积分：$\psi_{\alpha,int} = \sum (U_\alpha - I_\alpha R_s) \cdot T_s$ |
| L442 | `val1_mem += (Uβ - Iβ·Rs)·Ts` | β轴反电动势积分：$\psi_{\beta,int} = \sum (U_\beta - I_\beta R_s) \cdot T_s$ |
| L443 | `alpha = val0_mem - Iα·Ls` | 减去电感压降：$\psi_\alpha = \psi_{\alpha,int} - I_\alpha L_s$ |
| L444 | `beta = val1_mem - Iβ·Ls` | 减去电感压降：$\psi_\beta = \psi_{\beta,int} - I_\beta L_s$ |
| L445 | `flux = sqrt(α² + β²)` | 计算瞬时磁链幅值 |
| L446-447 | `val_filter = val_filter·(1-k) + flux·k` | 一阶低通滤波：$y[n] = (1-k) \cdot y[n-1] + k \cdot x[n]$ |
| L448-450 | `if (val_filter > val_filter_mem)` | 峰值保持：记录滤波后磁链的最大值 |
| L451-454 | `if (tick_count > flux_detection_times)` | 检测时间到达，取峰值作为最终磁链值 |

#### 需要编码器配合的原因

Flux 检测是唯一需要电机旋转的阶段。在调度函数中：

```c
if (loop->rundata.offline_detection.mode == offline_param_detection_mode_flux) {
    theta = hpm_mcl_encoder_get_theta(loop->encoder);
} else {
    theta = 0;
}
```

电机旋转时需要实时角度进行 Park/InvPark 变换，确保 dq 轴坐标系跟随转子旋转。同时 Flux 阶段使用 PID 电流环（而非开环电压），也需要准确的 dq 轴反馈电流，这依赖于正确的角度。

此外，`ualpha` 和 `ubeta` 使用的是**上一周期**的输出电压值（`last_ualpha/last_ubeta`），而非当前周期的参考值。这是因为实际施加到电机上的电压存在一拍延迟，使用上一拍输出更接近真实值。

#### 工程要点

1. **误差累积传递**：Flux 计算使用 Rs 和 Ls（= Ld + Lq），前级检测误差会直接传递到磁链结果。典型误差链：Rs 偏大 → 积分值偏小 → Flux 偏小
2. **积分漂移问题**：纯积分器存在直流偏置累积问题。本实现未加积分抗漂移措施（如高通滤波或限幅），长时间运行可能导致积分饱和。`flux_detection_times` 不宜设置过大
3. **低通滤波系数 `lowpass_k`**：典型值 0.01，截止频率约 $f_c = k \cdot f_s / (2\pi) \approx 32$ Hz（20kHz 采样），能有效滤除高频噪声同时保留磁链基频分量
4. **峰值保持策略**：取整个检测过程中的最大滤波值作为磁链幅值。这假设电机已达到稳态转速，磁链幅值恒定。若电机尚未稳速，峰值可能偏低

---

### 7.6 死区补偿在检测中的作用

**相关源码位置：**
- 编译期强制检查：[hpm_mcl_loop.c:L132-L138](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L132-L138)
- 运行时补偿执行：[hpm_mcl_loop.c:L266-L270](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L266-L270)
- 补偿算法实现：[hpm_mcl_control.c:L257-L300](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L257-L300)

#### 为什么参数检测必须启用死区补偿

**编译期硬约束：**

```c
#if !defined(MCL_CFG_EN_DEAD_AREA_COMPENSATION) || (MCL_CFG_EN_DEAD_AREA_COMPENSATION == 0)
    return mcl_fail;
#endif
```

死区时间（Dead Time）是逆变器上下桥臂切换时为防止直通而插入的延迟。在此期间，输出电压由电流方向决定（续流二极管导通），而非由 PWM 占空比决定。这导致**实际输出电压偏离指令电压**：

$$\Delta U = \frac{2 \cdot T_d \cdot U_{dc}}{T_s} \cdot \text{sign}(I)$$

其中 $T_d$ 为死区时间，$T_s$ 为 PWM 周期，$\text{sign}(I)$ 为电流极性。

**死区对参数检测的影响：**

| 检测项 | 死区影响 | 误差方向 |
|--------|---------|---------|
| Rs | 实际 Ud 偏离指令值 → Rs 计算偏差 | 取决于电流极性，可能偏大或偏小 |
| Ld | Vbus/2 近似失效 → di/dt 计算偏差 | 通常使电感偏大 |
| Lq | 同 Ld | 同 Ld |
| Flux | 积分用 Uα/Uβ 偏离实际 → 磁链积分漂移 | 累积误差，方向不确定 |

死区误差在低电流时尤为严重（误差占比大），而 Rs 检测恰好工作在半额定电流的低值区域，电感检测的电流变化量也较小，因此死区补偿对检测精度至关重要。

#### 运行时补偿执行

```c
loop->control->method.dead_area_polarity_detection(&loop->control->cfg->dead_area_compensation_cfg, sens_d, sens_q, theta,
                                        loop->const_time.dead_area_ts, *loop->const_time.current_ts, &duty_offset);
duty.a += duty_offset.a_offset;
duty.b += duty_offset.b_offset;
duty.c += duty_offset.c_offset;
```

在检测模式下，死区补偿**无条件执行**（不像 FOC 模式有 `enable_dead_area_compensation` 开关）。补偿值直接叠加到 SVPWM 输出的三相占空比上。

#### 补偿算法实现

```c
hpm_mcl_stat_t hpm_mcl_control_dead_area_polarity_detection(mcl_control_dead_area_compensation_t *dead_area,
                float id, float iq, float theta,
                float deadtime, float ts, mcl_control_dead_area_pwm_offset_t *pwm_out)
{
    float out_d, out_q, sens_theta;
    int8_t ia = 0, ib = 0, ic = 0;

    out_q = hpm_mcl_control_lowpass_filter(iq, &dead_area->q_mem, dead_area->cfg.lowpass_k);
    out_d = hpm_mcl_control_lowpass_filter(id, &dead_area->d_mem, dead_area->cfg.lowpass_k);
    sens_theta = atan2f(out_q, out_d);
    theta += sens_theta;
    theta = MCL_ANGLE_MOD_X(0, MCL_2PI, theta);

    if ((theta >= (MCL_PI * (11.0f / 6))) || (theta < (MCL_PI * (1.0f / 6)))) {
        ia = 1; ib = -1; ic = -1;
    } else if ((theta >= (MCL_PI * (1.0f / 6))) && (theta < (MCL_PI * (1.0f / 2)))) {
        ia = 1; ib = 1; ic = -1;
    } else if ((theta >= (MCL_PI * (1.0f / 2))) && (theta < (MCL_PI * (5.0f / 6)))) {
        ia = -1; ib = 1; ic = -1;
    } else if ((theta >= (MCL_PI * (5.0f / 6))) && (theta < (MCL_PI * (7.0f / 6)))) {
        ia = -1; ib = 1; ic = 1;
    } else if ((theta >= (MCL_PI * (7.0f / 6))) && (theta < (MCL_PI * (3.0f / 2)))) {
        ia = -1; ib = -1; ic = 1;
    } else if ((theta >= (MCL_PI * (3.0f / 2))) && (theta < (MCL_PI * (11.0f / 6)))) {
        ia = 1; ib = -1; ic = 1;
    }
    pwm_out->a_offset = 2.0f * deadtime * ia / ts;
    pwm_out->b_offset = 2.0f * deadtime * ib / ts;
    pwm_out->c_offset = 2.0f * deadtime * ic / ts;

    return mcl_success;
}
```

#### 逐块解析

**① 电流方向检测（L264-L268）**

```c
out_q = hpm_mcl_control_lowpass_filter(iq, &dead_area->q_mem, dead_area->cfg.lowpass_k);
out_d = hpm_mcl_control_lowpass_filter(id, &dead_area->d_mem, dead_area->cfg.lowpass_k);
sens_theta = atan2f(out_q, out_d);
theta += sens_theta;
```

对 Id/Iq 做低通滤波后，用 `atan2` 计算电流空间矢量相对于 d 轴的偏角，叠加到电角度 θ 上，得到电流空间矢量在静止坐标系下的绝对角度。这个角度决定了三相电流的极性。

**② 六扇区极性判定（L270-L294）**

将 0~2π 分为6个扇区（每60°一个），根据电流矢量所在扇区确定三相电流极性（±1）。这是基于六步换相的原理——在每个扇区内，总有一相电流绝对值最大且方向确定。

**③ PWM偏移量计算（L295-L297）**

```c
pwm_out->a_offset = 2.0f * deadtime * ia / ts;
```

补偿量 = $\frac{2 \cdot T_d \cdot \text{sign}(I_x)}{T_s}$，其中 $T_s$ 为 PWM 周期。系数 2 是因为每个开关周期内上下桥臂各产生一次死区误差。

#### 工程要点

1. **检测模式下死区补偿不可关闭**：FOC 模式下可通过 `enable_dead_area_compensation` 开关控制，但检测模式在编译期强制要求。这是合理的设计——检测精度直接依赖电压精度
2. **低通滤波的必要性**：电流采样含开关噪声，直接用瞬时值判断极性会导致频繁误判。低通滤波平滑了电流方向，避免补偿振荡
3. **补偿精度限制**：本方法仅补偿死区时间引起的电压误差，未考虑管压降（Vce_sat/Vf）、开关时间等非线性因素。在低电压/低电流工况下，管压降占比不可忽略
4. **对 Rs 检测的影响最大**：Rs 检测使用步进式电压注入，电流从零缓慢上升。在电流过零点附近，死区误差占比最大（可能超过50%），补偿质量直接决定 Rs 精度

---

## 代码文件索引

| 文件 | 职责 |
|------|------|
| [hpm_mcl_control.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h) | 检测配置结构体、结果结构体、算法函数指针声明 |
| [hpm_mcl_control.c](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c) | 五种检测算法实现 + 初始化函数 |
| [hpm_mcl_loop.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h) | 状态机枚举、运行数据结构、用户接口inline函数 |
| [hpm_mcl_loop.c](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c) | 状态机调度、电流采样→Clarke→Park→逆变→PWM |
| [hpm_mcl_physical.h](../../../hpm_MC/middleware/hpm_mcl_v2/hpm_mcl_physical.h) | `physical_motor_t`：检测结果最终写入目标 |
| [bldc_offline_param_detection.c](../../../hpm_MC/samples/motor_ctrl/bldc_offline_param_detection/src/bldc_offline_param_detection.c) | 完整示例应用代码 |
| [hpm_bldc_define.h](../../../hpm_MC/middleware/hpm_mcl/inc/hpm_bldc_define.h) | v1手动参数定义（对照：无检测功能） |

---

## 参考文献

- HPMicro 知识库：[电机离线参数辨识](https://kb.hpmicro.com/2024/08/07/%e7%94%b5%e6%9c%ba%e7%a6%bb%e7%ba%bf%e5%8f%82%e6%95%b0%e8%be%a8%e8%af%86/)
- 示例 README：[README_zh.rst](../../../hpm_MC/samples/motor_ctrl/bldc_offline_param_detection/README_zh.rst)
- 关联文档：[SDK-02-HPM-MC-v2-Core-Loop.md](SDK-02-HPM-MC-v2-Core-Loop.md)

---

*文档更新时间: 2026-05-26*