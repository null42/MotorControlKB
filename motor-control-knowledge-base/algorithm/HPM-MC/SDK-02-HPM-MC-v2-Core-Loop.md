# HPM-MC v2 核心控制循环深度解析 ★★★★★ 🔰📚🔧

**文档版本：** v1.0  
**生成日期：** 2026-05-23  
**源码位置：** `hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h`
**中间件版本：** v1.10.0+

---

## 目录

1. [模块概述](#1-模块概述)
2. [模块组成与依赖](#2-模块组成与依赖)
3. [控制链核心 — hpm_mcl_control.h](#3-控制链核心--hpm_mcl_controlh)
4. [主循环调度 — hpm_mcl_loop.h](#4-主循环调度--hpm_mcl_looph)
5. [传感器处理](#5-传感器处理)
6. [驱动层抽象 — hpm_mcl_drivers.h](#6-驱动层抽象--hpm_mcl_driversh)
7. [故障检测 — hpm_mcl_detect.h](#7-故障检测--hpm_mcl_detecth)
8. [完整控制流程（以bldc_foc.c为例）](#8-完整控制流程以bldc_focc为例)
9. [与MC_LIB控制环对比](#9-与mc_lib控制环对比)
10. [硬件加速深度分析（HPM官方资料补充）](#10-硬件加速深度分析hpm官方资料补充)
11. [v2 核心代码实现分析](#11-v2-核心代码实现分析)

---

## 1. 模块概述

### 1.1 功能定位

`mcl_loop` 是 HPM MCL V2 中间件的**核心调度器**。它不直接实现控制算法，而是聚合所有子系统（传感器、控制算法、驱动输出、路径规划、故障检测、硬件加速环路），在统一的调度周期下协调各模块运行。

与 MC_LIB 不同，MCL V2 采用了**回调驱动的插件架构**，用户通过注册回调函数注入硬件操作实现，中间件只负责算法调度和控制流编排。

### 1.2 控制环架构总览

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                          MCL V2 控制环架构                                     │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌──────────────────────────────────────────────────────────────────────┐     │
│  │                      mcl_loop_t (核心调度器)                           │     │
│  │                                                                        │     │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐              │     │
│  │  │ encoder  │  │ analog   │  │ control  │  │ drivers  │              │     │
│  │  │ (角度/   │  │ (Ia,Ib,  │  │ (FOC/    │  │ (PWM     │              │     │
│  │  │  速度)   │  │  Vbus)   │  │  PID/SMC)│  │  输出)   │              │     │
│  │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘              │     │
│  │       │             │             │             │                     │     │
│  │  ┌────┴─────────────┴─────────────┴─────────────┴────┐               │     │
│  │  │              hpm_mcl_loop() 周期调度                │               │     │
│  │  │   sensor → transform → PID → SVPWM → duty update   │               │     │
│  │  └────────────────────────────────────────────────────┘               │     │
│  │                                                                        │     │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐                            │     │
│  │  │ detect   │  │ path     │  │hw_loop   │(v1.10.0+)                  │     │
│  │  │ (故障    │  │ (步进    │  │ (VSC/CLC/ │                            │     │
│  │  │  监控)   │  │  规划)   │  │  QEO加速)│                            │     │
│  │  └──────────┘  └──────────┘  └──────────┘                            │     │
│  └──────────────────────────────────────────────────────────────────────┘     │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

### 1.3 六种运行模式

| 枚举值 | 模式名称 | 适用场景 |
|--------|---------|---------|
| `mcl_mode_foc` (1) | 软件FOC | 通用FOC控制，纯软件实现 Clarke/Park/PID/SVPWM |
| `mcl_mode_block` (2) | 六步方波 | BLDC方波控制，基于Hall传感器的换相控制 |
| `mcl_mode_hardware_foc` (3) | 硬件FOC | 使用VSC/CLC/QEO硬件模块完成完整电流环 |
| `mcl_mode_step_foc` (4) | 步进电机FOC | 支持梯形曲线路径规划的步进电机闭环控制 |
| `mcl_mode_offline_param_detection` (5) | 离线参数检测 | 自动检测电机 Rs/Ld/Lq/Ls/Flux 参数 |
| `mcl_mode_hybrid_foc` (6) | 混合硬件FOC | 部分硬件加速(VSC/CLC/QEO可选)，软硬件协同 |

> **模式定义位置：** [hpm_mcl_loop.h:L31-L37](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L31-L37)

### 1.4 模块特点

```
┌─────────────────────────────────────────────────────────────────┐
│                    MCL V2 Core Loop 特点                         │
├─────────────────────────────────────────────────────────────────┤
│  ✓ 回调驱动的插件架构：硬件操作通过回调注册，中间件与平台解耦    │
│  ✓ 函数指针表机制 (mcl_control_method_t)：所有算法可替换         │
│  ✓ 多级控制环：电流环→速度环→位置环，可独立使能/禁用            │
│  ✓ 硬件加速分层：soft-FOC → hybrid-FOC → hardware-FOC          │
│  ✓ 可编译裁剪：DQ解耦/死区补偿/SMC通过 MCL_CFG 宏条件编译       │
│  ✓ 离线参数自整定：Rs/Ld/Lq/Ls/Flux 自动检测                    │
│  ✓ 三阶段电机对齐：粗对齐→精对齐→稳定化                          │
│  ✓ API版本演进：v1.10.0引入 hw_loop 参数，宏重载实现向后兼容     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 模块组成与依赖

### 2.1 mcl_loop_t 结构体分析

```c
typedef struct {
    mcl_loop_status_t status;          // ───── 循环状态：null/init/run/fail
    mcl_loop_cfg_t *cfg;               // ───── 模式配置：FOC/Block/Step/Hybrid
    mcl_encoder_t *encoder;            // ───── 编码器子系统（角度/速度/UVW）
    mcl_analog_t *analog;              // ───── 模拟量采集（Ia/Ib/Ic/Vbus）
    mcl_control_t *control;            // ───── 控制算法子系统（函数指针表）
    mcl_drivers_t *drivers;            // ───── PWM驱动抽象层
    mcl_debug_t *debug;                // ───── 调试FIFO
    mcl_path_plan_t *path;             // ───── 路径规划（步进电机梯形曲线）
    hpm_mcl_type_t *const_vbus;        // ───── 母线电压常量引用
    hpm_mcl_type_t *lq;               // ───── q轴电感引用
    hpm_mcl_type_t *ld;               // ───── d轴电感引用
    hpm_mcl_type_t *flux;             // ───── 磁链引用
    mcl_hw_loop_t *hybrid_hw_loop;    // ───── 硬件混合环路 (v1.10.0+)
    struct {
        float *current_ts;             // ───── 电流环周期
        float *speed_ts;               // ───── 速度环周期
        float *position_ts;            // ───── 位置环周期
        float dead_area_ts;            // ───── 死区补偿时间
        float offline_detection_wait_ts; // ── 离线检测等待时间
    } const_time;
    struct { /* rundata */ } rundata;   // ───── 运行数据（block方向/离线检测/电流tick）
    mcl_user_value_t ref_id;           // ───── 用户设定 d轴电流
    mcl_user_value_t ref_iq;           // ───── 用户设定 q轴电流
    mcl_user_value_t ref_speed;        // ───── 用户设定 速度
    mcl_user_value_t ref_position;     // ───── 用户设定 位置
    void *hardware;                    // ───── 硬件CLC配置指针
    bool enable;                       // ───── 循环使能标志
} mcl_loop_t;
```

> **完整定义：** [hpm_mcl_loop.h:L107-L151](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L107-L151)

### 2.2 子系统依赖关系

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           子系统依赖关系图                                 │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│                    ┌───────────────────────┐                              │
│                    │      mcl_cfg_t        │  ← 物理参数集合              │
│                    │  (physical/电机/板级)  │     (mcl_physical_para_t)    │
│                    └──────────┬────────────┘                              │
│                               │                           ┌──────────┐   │
│              ┌────────────────┼───────────────────────────┤  detect  │   │
│              │                │                           │ (故障检测)│   │
│              ▼                ▼                           └────┬─────┘   │
│  ┌──────────┴──────┐  ┌──────┴───────┐  ┌──────────┐          │         │
│  │    analog       │  │    encoder    │  │  drivers  │          │ 监控    │
│  │ (ADC采样/Ia,Ib) │  │ (QEI/ABZ/Hall)│  │ (PWM输出) │          │         │
│  └────────┬────────┘  └──────┬────────┘  └─────┬──────┘          │         │
│           │                  │                 │                 │         │
│           │    Ia, Ib, Vbus  │  theta, speed   │                 │         │
│           ▼                  ▼                 │                 ▼         │
│  ┌────────────────────────────────────────────┴──────────────────┐        │
│  │                       mcl_loop_t                               │        │
│  │                  ┌───────────┐                                  │        │
│  │                  │  control  │ ← mcl_control_method_t函数指针表 │        │
│  │                  │ Clarke/   │   (Park/Clarke/SVPWM/PID/       │        │
│  │                  │ Park/PID  │    SMC/离线检测/Block换相)     │        │
│  │                  └─────┬─────┘                                  │        │
│  │                        │ Ud, Uq                                │        │
│  │              ┌─────────▼─────────┐                              │        │
│  │              │     hw_loop       │ ← (v1.10.0+) 可选硬件加速    │        │
│  │              │  VSC → CLC → QEO  │    混合FOC模式               │        │
│  │              └─────────┬─────────┘                              │        │
│  │                        │ duty_a/b/c                            │        │
│  └────────────────────────┼───────────────────────────────────────┘        │
│                           │                                                │
│                           ▼                                                │
│                    ┌──────────────┐                                        │
│                    │   drivers    │ ← pwm_duty_set(channel, duty)          │
│                    │  (物理PWM)   │                                        │
│                    └──────────────┘                                        │
│                                                                           │
│  ┌──────────────────────────────────────────────────────────────┐         │
│  │  path (步进电机路径规划) ── 仅在 mcl_mode_step_foc 下使用     │         │
│  │  debug (调试FIFO)       ── 可选的运行时数据记录               │         │
│  └──────────────────────────────────────────────────────────────┘         │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

> **依赖关系：** `analog` 和 `encoder` 必须先于 `control` 初始化；`drivers` 在 `control` 之后配置；`loop` 作为聚合体最后初始化。

### 2.3 初始化顺序（严格时序）

```
step 1:  hpm_mcl_analog_init()     → ADC采样通道配置，电流零点/量程标定
step 2:  hpm_mcl_filter_iir_df1_init() → 编码器信号IIR滤波器初始化
step 3:  hpm_mcl_encoder_init()    → 编码器类型/精度/速度计算方法配置
step 4:  hpm_mcl_drivers_init()    → PWM模块初始化（通过回调）
step 5:  hpm_mcl_control_init()    → 控制算法函数指针表挂载 + PID参数配置
step 6:  hpm_mcl_hw_loop_init()    → (可选) VSC/CLC/QEO硬件初始化
step 7:  hpm_mcl_loop_init()       → 聚合所有子系统，建立调度上下文
step 8:  hpm_mcl_detect_init()     → 故障检测模块，监控上述子系统状态
```

> **参考实现：** [bldc_foc.c:L398-L411](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c#L398-L411)

---

## 3. 控制链核心 — hpm_mcl_control.h

### 3.1 函数指针表机制 — mcl_control_method_t

MCL V2 的核心设计精髓在于将所有控制算法抽象为一个函数指针表，用户可自由替换任一算法实现：

```c
typedef struct {
    hpm_mcl_type_t (*sin_x)(hpm_mcl_type_t x);                          // 正弦函数
    hpm_mcl_type_t (*cos_x)(hpm_mcl_type_t x);                          // 余弦函数
    hpm_mcl_type_t (*arctan_x)(hpm_mcl_type_t y, hpm_mcl_type_t x);     // 反正切
    hpm_mcl_stat_t (*park)(...);         // ── Park 变换 (αβ → dq)
    hpm_mcl_stat_t (*clarke)(...);       // ── Clarke 变换 (abc → αβ)
    hpm_mcl_stat_t (*currentd_pid)(...); // ── d轴电流PID
    hpm_mcl_stat_t (*currentq_pid)(...); // ── q轴电流PID
    hpm_mcl_stat_t (*speed_pid)(...);    // ── 速度环PID
    hpm_mcl_stat_t (*position_pid)(...); // ── 位置环PID
    hpm_mcl_stat_t (*invpark)(...);      // ── 逆Park变换 (dq → αβ)
    hpm_mcl_stat_t (*svpwm)(...);        // ── SVPWM生成 (3通道)
    hpm_mcl_stat_t (*step_svpwm)(...);   // ── SVPWM生成 (4通道步进)
    hpm_mcl_stat_t (*get_block_sector)(...); // ── 方波换相扇区
    hpm_mcl_stat_t (*dead_area_polarity_detection)(...); // ── 死区补偿
    void (*smc_init)(...);               // ── SMC初始化
    void (*smc_process)(...);            // ── SMC滑模观测器处理
    hpm_mcl_stat_t (*offline_param_detection_rs)(...);  // ── 离线: 电阻检测
    hpm_mcl_stat_t (*offline_param_detection_ld)(...);  // ── 离线: Ld检测
    hpm_mcl_stat_t (*offline_param_detection_lq)(...);  // ── 离线: Lq检测
    hpm_mcl_stat_t (*offline_param_detection_ls)(...);  // ── 离线: Ls检测
    hpm_mcl_stat_t (*offline_param_detection_flux)(...);// ── 离线: 磁链检测
    hpm_mcl_stat_t (*offline_param_detection_init)(...);// ── 离线: 初始化
} mcl_control_method_t;
```

> **完整定义：** [hpm_mcl_control.h:L214-L243](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L214-L243)

**架构意义：** 对比 MC_LIB v1 的 `hpm_foc.h` 中全局函数式 API（如 `hpm_mcl_bldc_foc_clarke()`、`hpm_mcl_bldc_foc_park()`），V2 将所有算法组织在统一结构体中，实现**算法与调度分离**。换用不同的控制算法（如从PID切换到SMC）只需修改函数指针赋值。

### 3.2 PID 控制器

#### 3.2.1 配置结构体

```c
typedef struct {
    hpm_mcl_type_t kp;              // 比例系数
    hpm_mcl_type_t ki;              // 积分系数
    hpm_mcl_type_t kd;              // 微分系数
    hpm_mcl_type_t integral_max;    // 积分上限（抗饱和）
    hpm_mcl_type_t integral_min;    // 积分下限
    hpm_mcl_type_t output_max;      // 输出上限
    hpm_mcl_type_t output_min;      // 输出下限
} mcl_control_pid_cfg_t;
```

> **定义位置：** [hpm_mcl_control.h:L66-L74](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L66-L74)

**电流环PI参数整定公式（从bldc_foc.c中提取）：**

```
Kp = Ls * (ωbw * 2π)^2 * Ts * 1.5
Ki = Rs * (ωbw * 2π)^2 * Ts * 1.5
其中 ωbw = 电流环带宽 (典型 500 Hz)
     Ts = 电流环周期 = 1/PWM频率 = 1/20000 = 50μs
```

> **实际代码：** [bldc_foc.c:L307-L321](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c#L307-L321)

#### 3.2.2 两种PID实现

| API | 类型 | 适用场景 |
|-----|------|---------|
| `hpm_mcl_delta_pid()` | 增量式PID | 电流环（响应快，无积分饱和累积） |
| `hpm_mcl_position_pid()` | 位置式PID | 速度环/位置环（稳态精度高） |

> **API定义：** [hpm_mcl_control.h:L315](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L315) / [hpm_mcl_control.h:L330](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L330)

#### 3.2.3 PID配置层级

```
mcl_control_cfg_t {
    callback.init                          // 用户自定义初始化回调
    callback.method                        // 用户自定义算法函数表
    currentd_pid_cfg: mcl_control_pid_t    // d轴电流PID (cfg + integral + error_n1/N2)
    currentq_pid_cfg: mcl_control_pid_t    // q轴电流PID
    speed_pid_cfg: mcl_control_pid_t       // 速度环PID
    position_pid_cfg: mcl_control_pid_t    // 位置环PID
    dead_area_compensation_cfg             // 死区补偿配置
    smc_cfg: mcl_control_smc_t             // SMC观测器配置
    offline_param_detection_cfg            // 离线参数检测配置
}
```

> **定义位置：** [hpm_mcl_control.h:L258-L267](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L258-L267)

### 3.3 SMC 滑模观测器配置

```c
typedef struct {
    struct {
        float res;          // 定子电阻 (Ω)
        float inductor;     // 定子电感
        float sample_ts;    // 电流采样周期
        float loop_ts;      // 控制周期 (s)
    } const_data;
    struct {
        float smc_f;        // 滑模系数 F
        float smc_g;        // 滑模系数 G
        float zero;         // 滑模收敛阈值
        float ksmc;         // 滑模增益
        float filter_coeff; // 低通滤波器系数
    } factor;
    float theta0;           // 初始角度
    mcl_control_pid_t pll;  // PLL锁相环（从反电势提取角度）
} mcl_control_smc_cfg_t;
```

> **定义位置：** [hpm_mcl_control.h:L119-L135](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L119-L135)

SMC 通过 `MCL_CFG_EN_SENSORLESS_SMC` 宏条件编译，运行时由 `loop_cfg.enable_smc` 控制使能。

### 3.4 离线参数检测

检测流程按 8 个阶段顺序执行：

```
init → rs(电阻) → ld(d轴电感) → lq(q轴电感) → ls(绕组电感) → flux(磁链) → wait → end
                                   ↑ 检测出错 → error
```

> **状态枚举：** [hpm_mcl_loop.h:L83-L92](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L83-L92)

配置参数：

```c
typedef struct {
    float current_half_rated;         // 半额定电流
    float ud_delta;                   // 每次电压增量步长
    float vbus;                       // 母线电压
    float inductor_detection_times;   // 电感检测时长 (典型1ms)
    float flux_detection_times;       // 磁链检测时长
    float delay_times;                // 阶段间延迟（泄放电流）
    float detection_loop_ts;          // 检测循环周期
    float lowpass_k;                  // 磁链求解低通滤波系数
} mcl_control_offline_param_detection_cfg_t;
```

> **定义位置：** [hpm_mcl_control.h:L159-L168](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h#L159-L168)

---

## 4. 主循环调度 — hpm_mcl_loop.h

### 4.1 循环状态机

```
                    ┌──────────┐
                    │   null   │ (初始状态)
                    └────┬─────┘
                         │ hpm_mcl_loop_init()
                         ▼
                    ┌──────────┐
              ┌─────│   init   │
              │     └────┬─────┘
              │          │ hpm_mcl_loop_enable() + 首次 hpm_mcl_loop()
              │          ▼
              │     ┌──────────┐
              │     │   run    │◄───────── 每PWM周期调用 hpm_mcl_loop()
              │     └────┬─────┘
              │          │ 故障检测触发 / 用户手动 disable
              │          ▼
              │     ┌──────────┐
              │     │   fail   │──▶ 触发 hpm_mcl_detect 输出禁用
              │     └──────────┘
              │
              └── 重复周期调用：使能 & 状态=run 时执行控制迭代
```

### 4.2 运行模式切换

```
  mcl_mode_foc (1)          mcl_mode_block (2)         mcl_mode_hardware_foc (3)
  ┌──────────────────┐      ┌──────────────────┐      ┌──────────────────┐
  │ Clarke → Park    │      │ Hall扇区判断    │      │ VSC: Clarke+Park  │
  │ PID(d) PID(q)    │      │ PWM六步换相     │      │ CLC: 电流环PID    │
  │ 逆Park → SVPWM   │      │                  │      │ QEO: 逆Park+SVPWM │
  └──────────────────┘      └──────────────────┘      └──────────────────┘

  mcl_mode_step_foc (4)     mcl_mode_offline (5)       mcl_mode_hybrid_foc (6)
  ┌──────────────────┐      ┌──────────────────┐      ┌──────────────────┐
  │ 梯形曲线路径规划 │      │ Rs → Ld → Lq    │      │ VSC/CLC/QEO可选   │
  │ 4通道SVPWM输出   │      │ → Ls → Flux     │      │ 软硬件数据交换    │
  │ 闭环位置/速度    │      │ → 结果输出      │      │ 动态使能/禁用    │
  └──────────────────┘      └──────────────────┘      └──────────────────┘
```

### 4.3 电机对齐算法

MCL V2 提供三种对齐算法（`mcl_motor_alignment_algorithm_t`）：

| 算法 | 枚举值 | 原理 |
|------|--------|------|
| basic | 0 | 单阶段：给定固定d轴电流，持续指定时间，转子自然对齐到d轴 |
| three_stage | 1 | 三阶段：大电流粗对齐(含q轴扰动) → 中电流精对齐 → 小电流稳定化 |
| adaptive | 2 | 预留：自适应对齐算法 |

**三阶段对齐配置：**

```c
mcl_motor_alignment_cfg_t alignment_cfg;
alignment_cfg.algorithm = mcl_alignment_algorithm_three_stage;
// Stage1: 粗对齐，克服静摩擦，带q轴扰动打破可能卡住的位置
alignment_cfg.config.three_stage.stage1 = {.d_current=8.0f, .q_current=0.5f, .delay_ms=500};
// Stage2: 精对齐，降低电流减少过冲
alignment_cfg.config.three_stage.stage2 = {.d_current=5.0f, .delay_ms=800};
// Stage3: 稳定化，最小电流维持位置
alignment_cfg.config.three_stage.stage3 = {.d_current=3.0f, .delay_ms=400};
alignment_cfg.config.three_stage.final_delay_ms = 100;  // 对齐后稳定延时
```

> **API定义：** [hpm_mcl_loop.h:L228](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L228)  
> **实际调用：** [bldc_foc.c:L1356-L1403](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c#L1356-L1403)

### 4.4 API版本演进（v1.9.0 → v1.10.0）

v1.10.0 新增 `mcl_hw_loop_t *hw_loop` 参数以支持混合硬件加速模式。通过 C 宏"函数重载"技巧实现向后兼容：

```c
#define hpm_mcl_loop_init(...) \
    GET_MACRO(__VA_ARGS__, hpm_mcl_loop_init_v2, hpm_mcl_loop_init_v1, ...)(__VA_ARGS__)
```

- 传入 8 个参数 → 自动调用 `hpm_mcl_loop_init_v1()`（传NULL给hw_loop）
- 传入 9 个参数 → 自动调用 `hpm_mcl_loop_init_v2()`（传入hw_loop）

> **实现位置：** [hpm_mcl_loop.h:L264-L273](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h#L264-L273)

---

## 5. 传感器处理

### 5.1 编码器模块 — hpm_mcl_encoder.h

#### 5.1.1 四种速度计算方法

| 方法 | 枚举值 | 原理 | 适用速度段 |
|------|--------|------|-----------|
| **T法** | `encoder_method_t` | 测量两次角度变化之间的时间间隔 | 低速 (< speed_abs_switch_m_t) |
| **M法** | `encoder_method_m` | 单位时间内统计角度变化量 | 高速 (> speed_abs_switch_m_t) |
| **M-T法** | `encoder_method_m_t` | 自适应切换：低速T法，高速M法 | 全速域 |
| **PLL法** | `encoder_method_pll` | 锁相环跟踪角度变化率 | 全速域（平滑性好） |
| **用户自定义** | `encoder_method_user` | 用户自行实现速度计算 | 特殊需求 |

> **枚举定义：** [hpm_mcl_encoder.h:L32-L38](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h#L32-L38)

**M-T法切换阈值配置：** `speed_abs_switch_m_t` — 速度高于此值使用M法，低于此值使用T法。

#### 5.1.2 IIR 滤波器

编码器输出支持 IIR DF1 滤波器级联，bldc_foc示例中使用 2 级 IIR 滤波器（`section=2`），每级带独立系数矩阵。滤波器类型完全可配置：

```c
mcl_filter_iir_df1_t encoder_iir;
mcl_filter_iir_df1_memory_t encoder_iir_mem[2];  // 2级滤波器状态存储
```

#### 5.1.3 角度预测机制 (theta_forecast)

通过 `MCL_EN_THETA_FORECAST` 配置开关控制（默认开启）。原理：利用当前速度外推一个控制周期的角度增量，消除因采样延迟导致的角度滞后误差。

```
theta_forecast = theta_current + speed * Ts
```

> **配置宏：** [hpm_mcl_cfg.h:L14-L18](../../../hpm_MC/middleware/hpm_mcl_v2/hpm_mcl_cfg.h#L14-L18)

#### 5.1.4 PLL 锁相环配置

```c
typedef struct {
    float kp;   // 比例系数
    float ki;   // 积分系数
} mcl_encoder_cal_speed_pll_cfg_t;
```

> **定义位置：** [hpm_mcl_encoder.h:L94-L97](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h#L94-L97)

### 5.2 模拟量采集 — hpm_mcl_analog.h

#### 5.2.1 10 通道定义

```c
typedef enum {
    analog_a_current = 0,  // A相电流
    analog_b_current = 1,  // B相电流
    analog_c_current = 2,  // C相电流
    analog_a_voltage = 3,  // A相电压
    analog_b_voltage = 4,  // B相电压
    analog_c_voltage = 5,  // C相电压
    analog_vbus = 6,       // 母线电压
    // 7~9 保留
} mcl_analog_chn_t;
```

> **枚举定义：** [hpm_mcl_analog.h:L31-L39](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_analog.h#L31-L39)  
> **通道总数：** [hpm_mcl_physical.h:L11](../../../hpm_MC/middleware/hpm_mcl_v2/hpm_mcl_physical.h#L11) `MCL_ANALOG_CHN_NUM = 10`

#### 5.2.2 每通道可独立配置 IIR 滤波器

```c
mcl_filter_iir_df1_t *iir[MCL_ANALOG_CHN_NUM];  // 每通道一个IIR滤波器实例
```

#### 5.2.3 ADC 采样到物理值转换

通过 `physical_board_analog_t` 完成：

```
物理值 = (ADC_raw - 零点偏移) * (Vref / 采样精度) / (运放增益 * 采样电阻)
```

---

## 6. 驱动层抽象 — hpm_mcl_drivers.h

### 6.1 通道定义

```c
typedef enum {
    /* BLDC三通道（互补输出） */
    mcl_drivers_chn_a,   mcl_drivers_chn_b,   mcl_drivers_chn_c,
    /* 步进电机四通道 */
    mcl_drivers_chn_a0,  mcl_drivers_chn_a1,  mcl_drivers_chn_b0,  mcl_drivers_chn_b1,
    /* BLDC六通道（非互补，支持独立上下桥控制） */
    mcl_drivers_chn_ah,  mcl_drivers_chn_al,  mcl_drivers_chn_bh,
    mcl_drivers_chn_bl,  mcl_drivers_chn_ch,  mcl_drivers_chn_cl,
} mcl_drivers_channel_t;
```

> **枚举定义：** [hpm_mcl_drivers.h:L32-L59](../../../hpm_MC/middleware/hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h#L32-L59)

### 6.2 回调接口

```c
typedef struct {
    void (*init)(void);                                   // PWM模块初始化
    hpm_mcl_stat_t (*update_duty_cycle)(chn, duty);       // 更新占空比（核心）
    _FUNC_OPTIONAL_ hpm_mcl_stat_t (*update_frequency)(chn, freqc);   // 更新频率
    _FUNC_OPTIONAL_ hpm_mcl_stat_t (*update_phase_offset)(chn, tick); // 更新相位偏移
    hpm_mcl_stat_t (*disable_all_drivers)(void);          // 紧急关断所有输出
    hpm_mcl_stat_t (*enable_all_drivers)(void);           // 使能所有输出
    hpm_mcl_stat_t (*disable_drivers)(chn);               // 禁用指定通道
    hpm_mcl_stat_t (*enable_drivers)(chn);                // 使能指定通道
} mcl_drivers_callback_t;
```

> **定义位置：** [hpm_mcl_drivers.h:L65-L74](../../../hpm_MC/middleware/hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h#L65-L74)

其中 `update_frequency` 和 `update_phase_offset` 标记为 `_FUNC_OPTIONAL_`，表示可选实现，主要是步进电机场景使用。

---

## 7. 故障检测 — hpm_mcl_detect.h

### 7.1 四子系统监控

```c
typedef struct {
    mcl_loop_status_t *control_loop;     // 监控 loop 状态
    mcl_encoder_status_t *encoder;       // 监控 encoder 状态
    mcl_analog_status_t *analog;         // 监控 analog 状态
    mcl_drivers_status_t *drivers;       // 监控 drivers 状态
} mcl_detect_status;
```

> **定义位置：** [hpm_mcl_detect.h:L17-L22](../../../hpm_MC/middleware/hpm_mcl_v2/core/detect/hpm_mcl_detect.h#L17-L22)

### 7.2 故障检测流程

```
每1ms (独立Timer中断)
         │
         ▼
  hpm_mcl_detect_loop()
         │
         ├─ check loop.status    == loop_status_fail ?  → 输出禁用
         ├─ check encoder.status == encoder_status_fail ? → 输出禁用
         ├─ check analog.status  == analog_status_fail ?  → 输出禁用
         ├─ check drivers.status == drivers_status_fail ? → 输出禁用
         │
         ▼
  故障触发:
    callback.disable_output()    → 立即关断PWM输出（拉低/高阻态）
    callback.user_process()      → 用户自定义故障处理（记录/告警）
```

> **实际调用：** [bldc_foc.c:L977-L978](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c#L977-L978) 在 1ms GPTMR 中断中独立于控制环运行

### 7.3 可配置的子模块检测

```c
typedef struct {
    bool enable_detect;              // 总使能开关
    struct {
        bool loop;                   // 单独控制各子系统检测
        bool encoder;
        bool analog;
        bool drivers;
    } en_submodule_detect;
    mcl_detect_callback_t callback;  // 故障回调
} mcl_detect_cfg_t;
```

---

## 8. 完整控制流程（以bldc_foc.c为例）

### 8.1 初始化阶段

```
main()
 │
 ├─ board_init()                            // 板级时钟/引脚
 ├─ motor_clock_hz = clock_get_frequency()  // 获取系统时钟
 │
 ├─ motor_init()                            // ← 核心初始化
 │   ├─ 填充 physical 参数                  // 电机: Rs/Ls/Ld/Lq/Pole/Flux
 │   │                                      // 板级: 采样电阻/增益/ADC精度/PWM频率
 │   │                                      // 时间: Ts_current/Ts_speed/Ts_position
 │   ├─ 配置 analog      (通道/滤波器)       // hpm_mcl_analog_init()
 │   ├─ 配置 encoder_iir (2级低通滤波器)     // hpm_mcl_filter_iir_df1_init()
 │   ├─ 配置 encoder     (QEI/M法/PLL)       // hpm_mcl_encoder_init()
 │   ├─ 配置 drivers     (PWM回调注册)       // hpm_mcl_drivers_init()
 │   ├─ 配置 control     (PID/SMC/离线检测)  // hpm_mcl_control_init()
 │   │   ├─ currentd_pid: Kp, Ki, integral_max, output_max
 │   │   ├─ currentq_pid: 同上
 │   │   ├─ speed_pid:    Kp, Ki
 │   │   └─ position_pid: Kp, Ki
 │   ├─ 配置 hw_loop (可选)                  // hpm_mcl_hw_loop_init()
 │   ├─ 配置 loop       (mode/使能speed_loop) // hpm_mcl_loop_init()
 │   └─ 配置 detect     (监控4子系统)        // hpm_mcl_detect_init()
 │
 ├─ adc_isr_enable()                         // 使能ADC中断
 ├─ timer_init()                             // 1ms故障检测定时器
 │
 ├─ motor_adc_midpoint()                     // 采集ADC零点（200ms平均）
 │
 ├─ hpm_mcl_loop_enable()                    // 使能控制循环
 │
 ├─ [硬件FOC模式] vsc/clc/qeo μnit()         // 硬件模块初始化+TRGM矩阵配置
 │
 ├─ motor_angle_align()                      // 三阶段电机角度对齐
 │
 ├─ hpm_mcl_loop_set_speed()                 // 设定初始速度
 │
 └─ while(1) { 交互循环 }                    // 用户通过串口切换速度/位置模式
```

> **完整main函数：** [bldc_foc.c:L1429-L1559](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c#L1429-L1559)

### 8.2 ADC中断中的周期控制（PWM同步）

```
PWM触发ADC采样 → ADC转换完成 → ADC中断 ISR
                                    │
                                    ▼
                            hpm_mcl_encoder_process()  // 1. 更新角度/速度 (含IIR滤波)
                                    │
                                    ▼
                            hpm_mcl_loop()             // 2. 控制迭代
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
              sensor读取      control计算       drivers输出
              Ia,Ib,theta     Clarke→Park      SVPWM→duty
              →scale转换      PID(d)→PID(q)    update_duty_cycle()
                              →逆Park
```

> **ISR实现：** [bldc_foc.c:L1006-L1017](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c#L1006-L1017)

### 8.3 硬件FOC模式下的特殊流程

硬件FOC模式将电流环完全移交硬件处理：

```
软件侧 (main/ISR)                    硬件侧 (VSC→CLC→QEO流水线)
─────────────────                    ──────────────────────────
1. 设定 Id_ref/Iq_ref                 
   → clc_set_expect_adc_value()      ┌─ VSC: ADC数据→ Clarke→Park→ Id_fbk, Iq_fbk
                                     ├─ CLC: Id_fbk vs Id_ref → 3p3z PID → Ud
                                     │       Iq_fbk vs Iq_ref → 3p3z PID → Uq
2. 速度环PID (软件)                  └─ QEO: Ud,Uq,θ → 逆Park→ SVPWM→ PWM占空比
   → 输出 Iq_ref 给硬件
```

> 数据流通过 TRGM 矩阵互联：ADC→VSC→CLC→QEO→PWM，全程硬件处理，无需CPU干预电流环。

---

## 9. 与MC_LIB控制环对比

| 对比维度 | MC_LIB (ST MCFOC) | HPM MCL V2 |
|---------|-------------------|------------|
| **调度模式** | 显式状态机：定位→IF启动→闭环→弱磁，状态切换由函数调用完成 | 模式枚举驱动：`mcl_loop_mode_t` 决定调度路径，模式间独立 |
| **算法组织** | 全局函数式 API：`MCFOC_CurrentLoop_F()` / `MCFOC_SpeedLoop_F()` | 函数指针表 `mcl_control_method_t`：所有算法可运行时替换 |
| **初始化复杂度** | 单一函数调用，参数在结构体中预设 | 7步顺序初始化，每步需用户配置物理参数和回调 |
| **硬件耦合** | 纯软件库，通过用户回调对接硬件 | 内置回调机制，支持三种硬件加速级别（soft/hybrid/hardware） |
| **参数检测** | 无内置离线检测 | `mcl_mode_offline_param_detection` 内置5参数自动检测 |
| **故障保护** | 无独立故障检测模块 | 独立 `detect` 模块，1ms周期监控4个子系统 |
| **编码器支持** | 依赖用户提供角度 | 内置 4 种速度计算方法 + IIR滤波 + 角度预测 |
| **电机类型** | 仅PMSM/BLDC FOC | BLDC FOC / 步进FOC / 方波Block / 通用FOC |
| **启动对齐** | 单阶段 d轴电流对齐 | 三阶段对齐（粗→精→稳定）+ 可扩展自适应算法 |
| **可裁剪性** | 编译时用宏条件排除 | 编译时 MCL_CFG 宏（DQ解耦/死区补偿/SMC）+ 运行时使能/禁用 |
| **API兼容** | 版本间函数签名固定 | 宏重载实现跨版本兼容（8参数→9参数自动适配） |

### 9.1 状态机对比

```
MC_LIB 状态机:                     MCL V2 状态机:
                                  
  ALIGN ──▶ IF_START ──▶ CLOSED       Mode选择 ──▶ Align ──▶ 周期迭代
    │                      │              │
    └── 弱磁（并行） ◀──────┘          mcl_mode_foc / block / step / hybrid
                                         └── 每PWM周期一个 hpm_mcl_loop()
```

**核心差异：** MC_LIB 内置完整状态转移逻辑（含IF启动和闭环切换判断），MCL V2 将状态转移权交给用户，只保证在选定模式下正确执行控制运算。这使得 MCL V2 对自定义启动策略更加灵活（如可以直接使用三阶段对齐后切入闭环）。

---

## 11. v2 核心代码实现分析

> **源码位置：** `hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c` / `hpm_mcl_control.c` / `hpm_mcl_filter.c`

本章节从源码层面逐函数、逐段解析 MCL V2 核心控制循环的实现细节，将第1~10章的架构设计与数据结构映射到真实的 C 代码执行路径上。

---

### 11.1 FOC 电流环核心函数 (`hpm_mcl_current_foc_loop`)

这是 MCL V2 最核心的函数——**每个 PWM 周期调用一次**，完成从传感器读取到 PWM 占空比输出的全链路 FOC 控制。它同时承载了位置环/速度环的降频调度、三种 FOC 模式（纯软件/硬件/混合）的分支处理。

#### 完整代码

```c
hpm_mcl_stat_t hpm_mcl_current_foc_loop(mcl_loop_t *loop)
{
    float ref_speed = 0;
    float sens_speed;
    float ref_position = 0;
    float ia, ib, ic;
    float theta;
    float alpha, beta;
    float sinx, cosx;
    float sens_d, sens_q;
    float ref_d = 0, ref_q = 0;
    float ud, uq;
    float theta_abs;
#if defined(MCL_EN_LOOP_TIME_COUNT) && MCL_EN_LOOP_TIME_COUNT
    uint64_t delta_time;
#endif
    mcl_hardware_clc_cfg_t *clc_cfg;
    mcl_control_svpwm_duty_t duty;
#if defined(MCL_CFG_EN_DEAD_AREA_COMPENSATION) && MCL_CFG_EN_DEAD_AREA_COMPENSATION
    mcl_control_dead_area_pwm_offset_t duty_offset;
#endif

#if defined(MCL_CFG_EN_THETA_FORECAST) && MCL_CFG_EN_THETA_FORECAST
    float sinx_, cosx_;
    float theta_forecast;
#endif

    MCL_ASSERT_OPT(loop != NULL, mcl_invalid_pointer);
    theta = hpm_mcl_encoder_get_theta(loop->encoder);
#if defined(MCL_CFG_EN_THETA_FORECAST) && MCL_CFG_EN_THETA_FORECAST
    theta_forecast = hpm_mcl_encoder_get_forecast_theta(loop->encoder);
#endif
    if (loop->enable) {
        if (loop->cfg->enable_position_loop) {
            loop->time.position_ts += *loop->const_time.current_ts;
            MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_position.enable, ref_position, loop->ref_position.value, loop->exec_ref.position);
            if (loop->time.position_ts >= *loop->const_time.position_ts) {
                loop->time.position_ts = 0;
                MCL_ASSERT_EXEC_CODE_AND_RETURN(hpm_mcl_encoder_get_absolute_theta(loop->encoder, &theta_abs) == mcl_success,
                loop->status = loop_status_fail, mcl_fail);
                loop->control->method.position_pid(ref_position, theta_abs, &loop->control->cfg->position_pid_cfg, &loop->exec_ref.speed);
            }
        } else {
            loop->exec_ref.speed = 0;
            loop->time.position_ts = 0;
        }
        if (loop->cfg->enable_speed_loop) {
            loop->time.speed_ts += *loop->const_time.current_ts;
            MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_speed.enable, ref_speed, loop->ref_speed.value, loop->exec_ref.speed);
            if (loop->time.speed_ts >= *loop->const_time.speed_ts) {
                loop->time.speed_ts = 0;
                sens_speed = hpm_mcl_encoder_get_speed(loop->encoder);
                loop->control->method.speed_pid(ref_speed, sens_speed,
                &loop->control->cfg->speed_pid_cfg, &loop->exec_ref.iq);
            }
        } else {
            loop->time.speed_ts = 0;
            loop->exec_ref.iq = 0;
        }
        MCL_VALUE_SET_IF_TRUE(loop->ref_id.enable, ref_d, loop->ref_id.value);
        MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_iq.enable, ref_q, loop->ref_iq.value, loop->exec_ref.iq);
        switch (loop->cfg->mode) {
        case mcl_mode_foc:
            MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_a_current, &ia), loop->status, loop_status_fail);
            MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_b_current, &ib), loop->status, loop_status_fail);
#if defined(MCL_EN_LOOP_TIME_COUNT) && MCL_EN_LOOP_TIME_COUNT
            delta_time = hpm_csr_get_core_mcycle();
#endif
            ic = -(ia + ib);
            loop->control->method.clarke(ia, ib, ic, &alpha, &beta);
#if defined(MCL_CFG_EN_SENSORLESS_SMC) && MCL_CFG_EN_SENSORLESS_SMC
            loop->control->method.smc_process(&loop->control->cfg->smc_cfg, loop->control->cfg->smc_cfg.ualpha,
                loop->control->cfg->smc_cfg.ubeta, alpha, beta);
#endif
            sinx = loop->control->method.sin_x(theta);
            cosx = loop->control->method.cos_x(theta);
            loop->control->method.park(alpha, beta, sinx, cosx, &sens_d, &sens_q);
            loop->control->method.currentd_pid(ref_d, sens_d, &loop->control->cfg->currentd_pid_cfg, &ud);
            loop->control->method.currentq_pid(ref_q, sens_q, &loop->control->cfg->currentq_pid_cfg, &uq);
#if defined(MCL_CFG_EN_DQ_AXIS_DECOUPLING) && MCL_CFG_EN_DQ_AXIS_DECOUPLING
            if (loop->cfg->enable_dq_axis_decoupling) {
                if (loop->cfg->enable_speed_loop) {
                    sens_speed = hpm_mcl_encoder_get_speed(loop->encoder);
                    ud -= sens_q * sens_speed * (*loop->encoder->pole_num) * (*loop->lq);
                    uq += sens_speed * (*loop->encoder->pole_num) * (*loop->ld * sens_q + *loop->flux);
                }
            }
#endif
#if defined(MCL_CFG_EN_THETA_FORECAST) && MCL_CFG_EN_THETA_FORECAST
            sinx_ = loop->control->method.sin_x(theta_forecast);
            cosx_ = loop->control->method.cos_x(theta_forecast);
            loop->control->method.invpark(ud, uq, sinx_, cosx_, &alpha, &beta);
#else
            loop->control->method.invpark(ud, uq, sinx, cosx, &alpha, &beta);
#endif

#if defined(MCL_CFG_EN_SENSORLESS_SMC) && MCL_CFG_EN_SENSORLESS_SMC
            loop->control->cfg->smc_cfg.ualpha = alpha;
            loop->control->cfg->smc_cfg.ubeta = beta;
#endif
            loop->control->method.svpwm(alpha, beta, *loop->const_vbus, &duty);
#if defined(MCL_CFG_EN_DEAD_AREA_COMPENSATION) && MCL_CFG_EN_DEAD_AREA_COMPENSATION
            if (loop->cfg->enable_dead_area_compensation) {
                loop->control->method.dead_area_polarity_detection(&loop->control->cfg->dead_area_compensation_cfg, sens_d, sens_q, theta, loop->const_time.dead_area_ts,
                                                                    *loop->const_time.current_ts, &duty_offset);
                duty.a += duty_offset.a_offset;
                duty.b += duty_offset.b_offset;
                duty.c += duty_offset.c_offset;
                MCL_VALUE_LIMIT(duty.a, 0, 1);
                MCL_VALUE_LIMIT(duty.b, 0, 1);
                MCL_VALUE_LIMIT(duty.c, 0, 1);
            }
#endif
#if defined(MCL_EN_LOOP_TIME_COUNT) && MCL_EN_LOOP_TIME_COUNT
            loop->rundata.current_loop_tick = hpm_csr_get_core_mcycle() - delta_time;
#endif
            hpm_mcl_drivers_update_bldc_duty(loop->drivers, duty.a, duty.b, duty.c);
            break;
        case mcl_mode_hardware_foc:
            clc_cfg = (mcl_hardware_clc_cfg_t *)loop->hardware;
            clc_cfg->clc_set_val(loop_chn_id, clc_cfg->convert_float_to_clc_val(ref_d));
            clc_cfg->clc_set_val(loop_chn_iq, clc_cfg->convert_float_to_clc_val(ref_q));
            break;
        case mcl_mode_hybrid_foc:
            MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_a_current, &ia), loop->status, loop_status_fail);
            MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_b_current, &ib), loop->status, loop_status_fail);
            loop->hybrid_hw_loop->hw_loop_data.software.ia = ia;
            loop->hybrid_hw_loop->hw_loop_data.software.ib = ib;
            loop->hybrid_hw_loop->hw_loop_data.software.ic = -(ia + ib);
            loop->hybrid_hw_loop->hw_loop_data.software.position = theta;
            if (loop->hybrid_hw_loop->hw_loop_status.vsc_enabled) {
                if (loop->hybrid_hw_loop->cfg->callback.vsc_pre_process != NULL) {
                    loop->hybrid_hw_loop->cfg->callback.vsc_pre_process(loop->hybrid_hw_loop);
                }
                hpm_mcl_vsc_run(loop->hybrid_hw_loop);
                if (loop->hybrid_hw_loop->cfg->callback.vsc_post_process != NULL) {
                    loop->hybrid_hw_loop->cfg->callback.vsc_post_process(loop->hybrid_hw_loop);
                }
            } else {
                sinx = loop->control->method.sin_x(theta);
                cosx = loop->control->method.cos_x(theta);
                loop->control->method.clarke(loop->hybrid_hw_loop->hw_loop_data.software.ia, loop->hybrid_hw_loop->hw_loop_data.software.ib, loop->hybrid_hw_loop->hw_loop_data.software.ic, &alpha, &beta);
                loop->control->method.park(alpha, beta, sinx, cosx, &loop->hybrid_hw_loop->hw_loop_data.software.d, &loop->hybrid_hw_loop->hw_loop_data.software.q);
            }
            if (loop->hybrid_hw_loop->hw_loop_status.clc_enabled) {
                if (loop->hybrid_hw_loop->cfg->callback.clc_pre_process != NULL) {
                    loop->hybrid_hw_loop->cfg->callback.clc_pre_process(loop->hybrid_hw_loop);
                }
                hpm_mcl_clc_run(loop->hybrid_hw_loop, ref_d, ref_q);
                if (loop->hybrid_hw_loop->cfg->callback.clc_post_process != NULL) {
                    loop->hybrid_hw_loop->cfg->callback.clc_post_process(loop->hybrid_hw_loop);
                }
            } else {
                loop->control->method.currentd_pid(ref_d, loop->hybrid_hw_loop->hw_loop_data.software.d, &loop->control->cfg->currentd_pid_cfg, &loop->hybrid_hw_loop->hw_loop_data.software.ud);
                loop->control->method.currentq_pid(ref_q, loop->hybrid_hw_loop->hw_loop_data.software.q, &loop->control->cfg->currentq_pid_cfg, &loop->hybrid_hw_loop->hw_loop_data.software.uq);
            }
            if (loop->hybrid_hw_loop->hw_loop_status.qeo_enabled) {
                if (loop->hybrid_hw_loop->cfg->callback.qeo_pre_process != NULL) {
                    loop->hybrid_hw_loop->cfg->callback.qeo_pre_process(loop->hybrid_hw_loop);
                }
                hpm_mcl_qeo_run(loop->hybrid_hw_loop);
                if (loop->hybrid_hw_loop->cfg->callback.qeo_post_process != NULL) {
                    loop->hybrid_hw_loop->cfg->callback.qeo_post_process(loop->hybrid_hw_loop);
                }
            } else {
                sinx = loop->control->method.sin_x(theta);
                cosx = loop->control->method.cos_x(theta);
                loop->control->method.invpark(loop->hybrid_hw_loop->hw_loop_data.software.ud, loop->hybrid_hw_loop->hw_loop_data.software.uq, sinx, cosx, &alpha, &beta);
                loop->control->method.svpwm(alpha, beta, *loop->const_vbus, &duty);
                loop->hybrid_hw_loop->hw_loop_data.software.duty_u = duty.a;
                loop->hybrid_hw_loop->hw_loop_data.software.duty_v = duty.b;
                loop->hybrid_hw_loop->hw_loop_data.software.duty_w = duty.c;
                hpm_mcl_drivers_update_bldc_duty(loop->drivers, loop->hybrid_hw_loop->hw_loop_data.software.duty_u, loop->hybrid_hw_loop->hw_loop_data.software.duty_v, loop->hybrid_hw_loop->hw_loop_data.software.duty_w);
            }
            break;
        default:
            break;
        }
    } else {
        duty.a = 0;
        duty.b = 0;
        duty.c = 0;
        hpm_mcl_drivers_update_bldc_duty(loop->drivers, duty.a, duty.b, duty.c);
    }
    return  mcl_success;
}
```

> **源码位置：** [hpm_mcl_loop.c:L280-L469](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L280-L469)

#### 逐段解析

**① 角度获取（函数入口）**

```c
theta = hpm_mcl_encoder_get_theta(loop->encoder);
#if defined(MCL_CFG_EN_THETA_FORECAST) && MCL_CFG_EN_THETA_FORECAST
    theta_forecast = hpm_mcl_encoder_get_forecast_theta(loop->encoder);
#endif
```

- 在 `loop->enable` 判断之前就获取角度，确保即使循环未使能也能读取编码器数据
- `theta_forecast` 通过条件编译控制，公式为 $\theta_{forecast} = \theta_{current} + \omega \cdot T_s$，补偿采样延迟导致的角度滞后

**② 位置环调度逻辑（时间累加→降频执行）**

```c
if (loop->cfg->enable_position_loop) {
    loop->time.position_ts += *loop->const_time.current_ts;
    MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_position.enable, ref_position, loop->ref_position.value, loop->exec_ref.position);
    if (loop->time.position_ts >= *loop->const_time.position_ts) {
        loop->time.position_ts = 0;
        MCL_ASSERT_EXEC_CODE_AND_RETURN(hpm_mcl_encoder_get_absolute_theta(loop->encoder, &theta_abs) == mcl_success,
        loop->status = loop_status_fail, mcl_fail);
        loop->control->method.position_pid(ref_position, theta_abs, &loop->control->cfg->position_pid_cfg, &loop->exec_ref.speed);
    }
} else {
    loop->exec_ref.speed = 0;
    loop->time.position_ts = 0;
}
```

**降频执行原理：** 位置环和速度环的执行频率低于电流环。实现方式是**时间累加器**——每次电流环调用时累加 $T_s$，当累加值达到位置环周期 $T_{position}$ 时执行一次位置环 PID 并清零累加器。

$$\text{position\_ts} \mathrel{+}= T_s, \quad \text{当 } \text{position\_ts} \geq T_{position} \text{ 时执行位置环}$$

- `MCL_FUNCTION_SET_IF_ELSE_TRUE` 宏：若用户直接设定了位置参考值（`ref_position.enable == true`），使用用户值；否则使用上一次位置环输出的 `exec_ref.position`
- 位置环输出写入 `exec_ref.speed`，作为速度环的输入参考——**环间数据传递通过 `exec_ref` 结构体**

**③ 速度环调度逻辑**

```c
if (loop->cfg->enable_speed_loop) {
    loop->time.speed_ts += *loop->const_time.current_ts;
    MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_speed.enable, ref_speed, loop->ref_speed.value, loop->exec_ref.speed);
    if (loop->time.speed_ts >= *loop->const_time.speed_ts) {
        loop->time.speed_ts = 0;
        sens_speed = hpm_mcl_encoder_get_speed(loop->encoder);
        loop->control->method.speed_pid(ref_speed, sens_speed,
        &loop->control->cfg->speed_pid_cfg, &loop->exec_ref.iq);
    }
} else {
    loop->time.speed_ts = 0;
    loop->exec_ref.iq = 0;
}
```

与位置环相同的降频机制。速度环输出写入 `exec_ref.iq`，作为电流环 q 轴参考值。

**④ 电流参考值合成**

```c
MCL_VALUE_SET_IF_TRUE(loop->ref_id.enable, ref_d, loop->ref_id.value);
MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_iq.enable, ref_q, loop->ref_iq.value, loop->exec_ref.iq);
```

- d 轴参考：仅当用户直接设定时使用用户值，否则保持 0（标准 FOC 的 d 轴零电流控制）
- q 轴参考：用户直接设定优先，否则使用速度环输出 `exec_ref.iq`

**⑤ 电流采样与 Clarke 变换（`mcl_mode_foc` 分支）**

```c
MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_a_current, &ia), loop->status, loop_status_fail);
MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_b_current, &ib), loop->status, loop_status_fail);
ic = -(ia + ib);
loop->control->method.clarke(ia, ib, ic, &alpha, &beta);
```

- 三相电流采样：仅采样 A、B 两相，C 相由基尔霍夫电流定律 $i_c = -(i_a + i_b)$ 计算
- Clarke 变换通过函数指针调用，默认实现：

$$\alpha = i_a, \quad \beta = \frac{\sqrt{3}}{3} i_a + \frac{2\sqrt{3}}{3} i_b$$

**⑥ Park 变换（含角度获取）**

```c
sinx = loop->control->method.sin_x(theta);
cosx = loop->control->method.cos_x(theta);
loop->control->method.park(alpha, beta, sinx, cosx, &sens_d, &sens_q);
```

Park 变换将 αβ 坐标系电流投影到 dq 旋转坐标系：

$$i_d = \cos\theta \cdot i_\alpha + \sin\theta \cdot i_\beta$$
$$i_q = -\sin\theta \cdot i_\alpha + \cos\theta \cdot i_\beta$$

**⑦ 电流环 PI 控制**

```c
loop->control->method.currentd_pid(ref_d, sens_d, &loop->control->cfg->currentd_pid_cfg, &ud);
loop->control->method.currentq_pid(ref_q, sens_q, &loop->control->cfg->currentq_pid_cfg, &uq);
```

默认挂载 `hpm_mcl_control_pi`，即位置式 PI 控制器：

$$u = K_p \cdot e + K_i \cdot \sum e, \quad e = i_{ref} - i_{sens}$$

含积分限幅和输出限幅。

**⑧ DQ 轴解耦补偿（条件编译）**

```c
#if defined(MCL_CFG_EN_DQ_AXIS_DECOUPLING) && MCL_CFG_EN_DQ_AXIS_DECOUPLING
    if (loop->cfg->enable_dq_axis_decoupling) {
        if (loop->cfg->enable_speed_loop) {
            sens_speed = hpm_mcl_encoder_get_speed(loop->encoder);
            ud -= sens_q * sens_speed * (*loop->encoder->pole_num) * (*loop->lq);
            uq += sens_speed * (*loop->encoder->pole_num) * (*loop->ld * sens_q + *loop->flux);
        }
    }
#endif
```

基于 PMSM 电压方程的交叉耦合项补偿：

$$u_d' = u_d - \omega_e \cdot L_q \cdot i_q$$
$$u_q' = u_q + \omega_e \cdot (L_d \cdot i_q + \psi_f)$$

其中 $\omega_e = \text{speed} \times P_n$（电角速度 = 机械角速度 × 极对数）。解耦后 d/q 轴电流环可独立整定。

**⑨ 角度预测补偿（条件编译）**

```c
#if defined(MCL_CFG_EN_THETA_FORECAST) && MCL_CFG_EN_THETA_FORECAST
    sinx_ = loop->control->method.sin_x(theta_forecast);
    cosx_ = loop->control->method.cos_x(theta_forecast);
    loop->control->method.invpark(ud, uq, sinx_, cosx_, &alpha, &beta);
#else
    loop->control->method.invpark(ud, uq, sinx, cosx, &alpha, &beta);
#endif
```

关键设计：**角度预测仅作用于逆 Park 变换**，而非 Park 变换。原因：Park 变换使用的是采样时刻的实际角度，而逆 Park 变换产生的电压将在下一个 PWM 周期生效，因此需要使用预测角度 $\theta_{forecast}$ 来补偿一个周期的延迟。

**⑩ 逆 Park 变换与 SVPWM**

```c
loop->control->method.svpwm(alpha, beta, *loop->const_vbus, &duty);
```

逆 Park 变换：

$$u_\alpha = \cos\theta \cdot u_d - \sin\theta \cdot u_q$$
$$u_\beta = \sin\theta \cdot u_d + \cos\theta \cdot u_q$$

SVPWM 将 αβ 电压归一化后按扇区计算三相占空比，输出范围 [0, 1]。

**⑪ 死区补偿（条件编译）**

```c
#if defined(MCL_CFG_EN_DEAD_AREA_COMPENSATION) && MCL_CFG_EN_DEAD_AREA_COMPENSATION
    if (loop->cfg->enable_dead_area_compensation) {
        loop->control->method.dead_area_polarity_detection(..., &duty_offset);
        duty.a += duty_offset.a_offset;
        duty.b += duty_offset.b_offset;
        duty.c += duty_offset.c_offset;
        MCL_VALUE_LIMIT(duty.a, 0, 1);
        MCL_VALUE_LIMIT(duty.b, 0, 1);
        MCL_VALUE_LIMIT(duty.c, 0, 1);
    }
#endif
```

死区补偿通过检测电流极性确定补偿方向，补偿量为 $\Delta d = \frac{2 \cdot T_{dead}}{T_s} \cdot \text{sign}(i_x)$，补偿后必须重新限幅防止占空比越界。

**⑫ 硬件 FOC 模式（CLC 硬件加速）**

```c
case mcl_mode_hardware_foc:
    clc_cfg = (mcl_hardware_clc_cfg_t *)loop->hardware;
    clc_cfg->clc_set_val(loop_chn_id, clc_cfg->convert_float_to_clc_val(ref_d));
    clc_cfg->clc_set_val(loop_chn_iq, clc_cfg->convert_float_to_clc_val(ref_q));
    break;
```

硬件 FOC 模式下，软件仅负责将 $i_d^{ref}$ 和 $i_q^{ref}$ 写入 CLC 硬件寄存器。电流环的 Clarke→Park→PID→逆Park→SVPWM 全链路由 VSC→CLC→QEO 硬件流水线完成，CPU 开销趋近于零。

**⑬ 混合 FOC 模式（VSC+CLC+QEO 硬件协同）**

```c
case mcl_mode_hybrid_foc:
    // 1. 电流采样 → 写入共享数据区
    loop->hybrid_hw_loop->hw_loop_data.software.ia = ia;
    loop->hybrid_hw_loop->hw_loop_data.software.ib = ib;
    loop->hybrid_hw_loop->hw_loop_data.software.ic = -(ia + ib);
    loop->hybrid_hw_loop->hw_loop_data.software.position = theta;

    // 2. VSC 阶段（可选硬件/软件 Clarke+Park）
    if (loop->hybrid_hw_loop->hw_loop_status.vsc_enabled) {
        hpm_mcl_vsc_run(loop->hybrid_hw_loop);    // 硬件 VSC
    } else {
        // 软件 Clarke + Park
        loop->control->method.clarke(...);
        loop->control->method.park(...);
    }

    // 3. CLC 阶段（可选硬件/软件电流环 PID）
    if (loop->hybrid_hw_loop->hw_loop_status.clc_enabled) {
        hpm_mcl_clc_run(loop->hybrid_hw_loop, ref_d, ref_q);  // 硬件 CLC
    } else {
        loop->control->method.currentd_pid(...);  // 软件 PID
        loop->control->method.currentq_pid(...);
    }

    // 4. QEO 阶段（可选硬件/软件逆Park+SVPWM）
    if (loop->hybrid_hw_loop->hw_loop_status.qeo_enabled) {
        hpm_mcl_qeo_run(loop->hybrid_hw_loop);    // 硬件 QEO
    } else {
        loop->control->method.invpark(...);        // 软件逆Park
        loop->control->method.svpwm(...);          // 软件SVPWM
    }
    break;
```

混合模式的核心设计是**三级可选硬件加速**，每级独立使能/禁用：

| 阶段 | 硬件使能 | 硬件执行 | 软件回退 |
|------|---------|---------|---------|
| Clarke+Park | `vsc_enabled` | VSC 硬件模块 | `method.clarke` + `method.park` |
| 电流环 PID | `clc_enabled` | CLC 硬件模块 | `method.currentd_pid` + `method.currentq_pid` |
| 逆Park+SVPWM | `qeo_enabled` | QEO 硬件模块 | `method.invpark` + `method.svpwm` |

每级前后均有 pre/post_process 回调钩子，允许用户在硬件处理前后插入自定义逻辑（如数据格式转换、缩放等）。

#### 三种 FOC 模式对比

| 维度 | `mcl_mode_foc` | `mcl_mode_hardware_foc` | `mcl_mode_hybrid_foc` |
|------|----------------|------------------------|----------------------|
| Clarke+Park | 软件 | 硬件(VSC) | 可选硬件/软件 |
| 电流环 PID | 软件(PI) | 硬件(CLC/3P3Z) | 可选硬件/软件 |
| 逆Park+SVPWM | 软件 | 硬件(QEO) | 可选硬件/软件 |
| CPU 开销 | 全量（~1μs@600MHz） | 极低（仅写寄存器） | 按配置递减 |
| 灵活性 | 最高 | 最低 | 中等（逐级选择） |
| 适用场景 | 通用/调试 | 追求极致性能 | 渐进式迁移 |

---

### 11.2 主循环调度函数 (`hpm_mcl_loop`)

#### 代码

```c
hpm_mcl_stat_t hpm_mcl_loop(mcl_loop_t *loop)
{
    MCL_ASSERT_OPT(loop != NULL, mcl_invalid_pointer);
    switch (loop->cfg->mode) {
    case mcl_mode_foc:
    case mcl_mode_hardware_foc:
    case mcl_mode_hybrid_foc:
        return hpm_mcl_current_foc_loop(loop);
    case mcl_mode_block:
        return hpm_mcl_block_loop(loop);
        break;
    case mcl_mode_step_foc:
        return hpm_mcl_step_foc_loop(loop);
        break;
    case mcl_mode_offline_param_detection:
        return hpm_mcl_detect_offline_para(loop);
    default:
        MCL_ASSERT_EXEC_CODE_AND_RETURN(false, loop->status = loop_status_fail, mcl_fail);
        break;
    }
    return mcl_fail;
}
```

> **源码位置：** [hpm_mcl_loop.c:L625-L646](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L625-L646)

#### 解析

**模式分发机制：** `hpm_mcl_loop` 是一个纯分发函数，根据 `loop->cfg->mode` 枚举值将控制流路由到对应的循环实现：

| mode 枚举值 | 路由目标 | 说明 |
|-------------|---------|------|
| `mcl_mode_foc` / `mcl_mode_hardware_foc` / `mcl_mode_hybrid_foc` | `hpm_mcl_current_foc_loop` | 三种 FOC 模式共用同一个函数，内部再 switch 分支 |
| `mcl_mode_block` | `hpm_mcl_block_loop` | 方波控制独立循环 |
| `mcl_mode_step_foc` | `hpm_mcl_step_foc_loop` | 步进电机 FOC 独立循环 |
| `mcl_mode_offline_param_detection` | `hpm_mcl_detect_offline_para` | 离线参数检测独立循环 |

**与 v1 的架构差异：**

- **v1 (MC_LIB)：** 控制循环是单一的大函数（如 `MCFOC_CurrentLoop_F()`），内部通过全局状态机切换 ALIGN→IF_START→CLOSED_LOOP，模式间强耦合
- **v2 (MCL V2)：** 模式间完全解耦，每种模式有独立的循环函数，由 `hpm_mcl_loop` 统一调度。模式切换只需修改 `loop->cfg->mode` 枚举值，无需管理复杂的状态转移

---

### 11.3 方波控制循环 (`hpm_mcl_block_loop`)

#### 代码

```c
hpm_mcl_stat_t hpm_mcl_block_loop(mcl_loop_t *loop)
{
    hpm_mcl_type_t speed_pi_out = {0};
    float speed_pi_out_fp;
    float ref_speed = 0;
    mcl_control_svpwm_duty_t duty;

    MCL_ASSERT_OPT(loop != NULL, mcl_invalid_pointer);
    if (loop->enable) {
        if (loop->cfg->enable_position_loop) {
            MCL_ASSERT_EXEC_CODE_AND_RETURN(false, loop->status = loop_status_fail, mcl_invalid_argument);
        }
        if (loop->cfg->enable_speed_loop) {
            loop->time.speed_ts += *loop->const_time.speed_ts;
            MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_speed.enable, ref_speed, loop->ref_speed.value, loop->exec_ref.speed);
            if (loop->time.speed_ts >= *loop->const_time.speed_ts) {
                loop->time.speed_ts = 0;
                loop->control->method.speed_pid(ref_speed, hpm_mcl_encoder_get_speed(loop->encoder),
                &loop->control->cfg->speed_pid_cfg, &speed_pi_out);
            }
        } else {
            MCL_ASSERT_EXEC_CODE_AND_RETURN(false, loop->status = loop_status_fail, mcl_invalid_argument);
        }
        speed_pi_out_fp = MCL_MATH_CONVERT_FLOAT(speed_pi_out);
        if (speed_pi_out_fp < 0) {
            speed_pi_out_fp = -speed_pi_out_fp;
            loop->rundata.block.dir = motor_dir_forward;
        } else {
            loop->rundata.block.dir = motor_dir_back;
        }
        duty.a = speed_pi_out_fp;
        duty.b = speed_pi_out_fp;
        duty.c = speed_pi_out_fp;
        hpm_mcl_drivers_update_bldc_duty(loop->drivers, duty.a, duty.b, duty.c);
    } else {
        duty.a = 0;
        duty.b = 0;
        duty.c = 0;
        hpm_mcl_drivers_update_bldc_duty(loop->drivers, duty.a, duty.b, duty.c);
    }
    return  mcl_success;
}
```

> **源码位置：** [hpm_mcl_loop.c:L558-L599](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L558-L599)

#### 解析

**控制流程：** 速度环 PID → 方向判断 → 占空比输出

1. **位置环禁用检查：** 方波控制不支持位置环，若误使能则直接报错 `mcl_invalid_argument`
2. **速度环必须使能：** 方波控制必须开启速度环，否则报错——因为方波控制没有电流环，速度环是唯一的闭环
3. **方向判断：** 速度环输出为负值时取绝对值作为占空比，同时设置电机方向 `motor_dir_forward`；正值时设为 `motor_dir_back`
4. **三相等占空比：** `duty.a = duty.b = duty.c = speed_pi_out_fp`，三相输出相同占空比，具体哪相导通由 Hall 传感器扇区决定（通过 `hpm_mcl_loop_refresh_block` 在 Hall 中断中更新换相）

**工程要点：** 方波控制的换相逻辑不在本函数中，而是由外部 Hall 中断调用 `hpm_mcl_loop_refresh_block` → `get_block_sector` → `hpm_mcl_drivers_block_update` 完成。本函数仅负责速度闭环和占空比计算。

---

### 11.4 步进电机 FOC 循环 (`hpm_mcl_step_foc_loop`)

#### 代码

```c
hpm_mcl_stat_t hpm_mcl_step_foc_loop(mcl_loop_t *loop)
{
    float ref_speed = 0;
    float sens_speed;
    float ref_position = 0;
    float theta_abs;
    float ia, ib;
    float theta;
    float alpha, beta;
    float sinx, cosx;
    float sens_d, sens_q;
    float ref_d = 0;
    float ref_q = 0;
    float ud, uq;
    float sinx_, cosx_;
    float theta_;
    mcl_control_svpwm_duty_t duty;

    MCL_ASSERT_OPT(loop != NULL, mcl_invalid_pointer);
    if (loop->cfg->enable_step_motor_closed_loop) {
        theta = hpm_mcl_encoder_get_theta(loop->encoder);
        theta_ = hpm_mcl_encoder_get_forecast_theta(loop->encoder);
    } else {
        theta =  hpm_mcl_path_get_current_theta(loop->path);
        theta_ = MCL_ANGLE_MOD_X(0, MCL_PI * 2, (hpm_mcl_path_get_next_theta(loop->path) - (0.25f * MCL_PI)));
    }
    if (loop->enable) {
        MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_a_current, &ia), loop->status, loop_status_fail);
        MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_get_value(loop->analog, analog_b_current, &ib), loop->status, loop_status_fail);
        MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_step_convert(loop->analog, ia, analog_a_current, theta, &ia, loop->cfg->enable_step_motor_closed_loop), loop->status, loop_status_fail);
        MCL_STATUS_SET_IF_TRUE(mcl_success != hpm_mcl_analog_step_convert(loop->analog, ib, analog_b_current, theta, &ib, loop->cfg->enable_step_motor_closed_loop), loop->status, loop_status_fail);
        if (loop->cfg->enable_step_motor_closed_loop) {
            // ... 位置环 + 速度环（同FOC） ...
            MCL_VALUE_SET_IF_TRUE(loop->ref_id.enable, ref_d, loop->ref_id.value);
            MCL_FUNCTION_SET_IF_ELSE_TRUE(loop->ref_iq.enable, ref_q, loop->ref_iq.value, loop->exec_ref.iq);
        } else {
            theta =  MCL_ANGLE_MOD_X(0, MCL_PI * 2, (theta - (0.25f * MCL_PI)));
            MCL_VALUE_SET_IF_TRUE(loop->ref_id.enable, ref_d, loop->ref_id.value);
            ref_q = 0;
        }
        alpha = ia;
        beta = ib;
        sinx = loop->control->method.sin_x(theta);
        cosx = loop->control->method.cos_x(theta);
        loop->control->method.park(alpha, beta, sinx, cosx, &sens_d, &sens_q);
        loop->control->method.currentd_pid(ref_d, sens_d, &loop->control->cfg->currentd_pid_cfg, &ud);
        loop->control->method.currentq_pid(ref_q, sens_q, &loop->control->cfg->currentq_pid_cfg, &uq);
        sinx_ = loop->control->method.sin_x(theta_);
        cosx_ = loop->control->method.cos_x(theta_);
        loop->control->method.invpark(ud, uq, sinx_, cosx_, &alpha, &beta);
        loop->control->method.step_svpwm(alpha, beta, *loop->const_vbus, &duty);
        hpm_mcl_drivers_update_step_duty(loop->drivers, duty.a0, duty.a1, duty.b0, duty.b1);
    } else {
        duty.a0 = 0; duty.a1 = 0; duty.b0 = 0; duty.b1 = 0;
        hpm_mcl_drivers_update_step_duty(loop->drivers, duty.a0, duty.a1, duty.b0, duty.b1);
    }
    return  mcl_success;
}
```

> **源码位置：** [hpm_mcl_loop.c:L471-L556](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L471-L556)

#### 解析

**开环/闭环两种模式的核心区别：**

| 维度 | 开环模式 (`enable_step_motor_closed_loop = false`) | 闭环模式 (`enable_step_motor_closed_loop = true`) |
|------|---------------------------------------------------|---------------------------------------------------|
| 角度来源 | 路径规划器 `hpm_mcl_path_get_current_theta` | 编码器 `hpm_mcl_encoder_get_theta` |
| 速度环/位置环 | 不使能 | 使能，同标准 FOC |
| q 轴参考 | 固定为 0 | 速度环输出 |
| d 轴参考 | 用户设定 | 用户设定 |

**步进电机特殊的电流采样转换：**

```c
hpm_mcl_analog_step_convert(loop->analog, ia, analog_a_current, theta, &ia, loop->cfg->enable_step_motor_closed_loop);
```

步进电机的两相绕组（A/B相）与三相 BLDC 的电流采样不同，需要根据当前电角度将采样值映射到正确的坐标系。闭环模式下还需要考虑角度偏移。

**角度偏移 $\frac{\pi}{4}$ 的工程意义：**

```c
theta = MCL_ANGLE_MOD_X(0, MCL_PI * 2, (theta - (0.25f * MCL_PI)));
```

开环模式下，路径规划器输出的角度是 A 相电角度，而 Park 变换需要的是 d 轴对齐角度。步进电机两相绕组空间相差 90°，d 轴方向相对于 A 相偏移 45°（$\frac{\pi}{4}$），因此需要减去这个偏移量。

**四通道 SVPWM 输出：**

步进电机使用 `step_svpwm` 而非标准 `svpwm`，输出四路占空比（a0/a1/b0/b1），分别对应两相绕组的正反电流方向：

```c
if (alpha > 0) ta1on = alpha; else ta0on = -alpha;
if (beta > 0) tb1on = beta; else tb0on = -beta;
```

---

### 11.5 v2 控制算法函数指针机制 (`hpm_mcl_control_init`)

#### 代码

```c
hpm_mcl_stat_t hpm_mcl_control_init(mcl_control_t *control, mcl_control_cfg_t *cfg)
{
    MCL_ASSERT(control != NULL, mcl_invalid_pointer);
    MCL_ASSERT(cfg != NULL, mcl_invalid_pointer);

    control->cfg = cfg;
    MCL_ASSERT(hpm_mcl_pid_init(&control->cfg->currentd_pid_cfg) == mcl_success, mcl_fail);
    MCL_ASSERT(hpm_mcl_pid_init(&control->cfg->currentq_pid_cfg) == mcl_success, mcl_fail);
    MCL_ASSERT(hpm_mcl_pid_init(&control->cfg->position_pid_cfg) == mcl_success, mcl_fail);
    MCL_ASSERT(hpm_mcl_pid_init(&control->cfg->speed_pid_cfg) == mcl_success, mcl_fail);

    control->cfg->dead_area_compensation_cfg.d_mem = 0;
    control->cfg->dead_area_compensation_cfg.q_mem = 0;
    control->cfg->smc_cfg.ialpha_mem = 0;
    control->cfg->smc_cfg.ibeta_mem = 0;
    control->cfg->smc_cfg.cfg.pll.integral = 0;

    control->method.arctan_x = &hpm_mcl_control_arctan;
    control->method.clarke = &hpm_mcl_control_clarke;
    control->method.cos_x = &hpm_mcl_control_cos;
    control->method.currentd_pid = &hpm_mcl_control_pi;
    control->method.currentq_pid = &hpm_mcl_control_pi;
    control->method.invpark = &hpm_mcl_control_inv_park;
    control->method.park = &hpm_mcl_control_park;
    control->method.position_pid = &hpm_mcl_control_pi;
    control->method.sin_x = &hpm_mcl_control_sin;
    control->method.speed_pid = &hpm_mcl_control_pi;
    control->method.svpwm = &hpm_mcl_control_svpwm;
    control->method.step_svpwm = &hpm_mcl_control_step_svpwm;
    control->method.get_block_sector = &hpm_mcl_control_get_block_sector;
    control->method.dead_area_polarity_detection = &hpm_mcl_control_dead_area_polarity_detection;
    control->method.smc_init = &hpm_mcl_control_smc_init;
    control->method.smc_process = &hpm_mcl_control_smc_process;
    control->method.offline_param_detection_flux = &hpm_mcl_control_offline_param_detection_flux;
    control->method.offline_param_detection_ld = &hpm_mcl_control_offline_param_detection_ld;
    control->method.offline_param_detection_lq = &hpm_mcl_control_offline_param_detection_lq;
    control->method.offline_param_detection_rs = &hpm_mcl_control_offline_param_detection_rs;
    control->method.offline_param_detection_ls = &hpm_mcl_control_offline_param_detection_ls;
    control->method.offline_param_detection_init = &hpm_mcl_control_detection_init;
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.arctan_x, control->cfg->callback.method.arctan_x);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.clarke, control->cfg->callback.method.clarke);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.cos_x, control->cfg->callback.method.cos_x);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.currentd_pid, control->cfg->callback.method.currentd_pid);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.invpark, control->cfg->callback.method.invpark);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.park, control->cfg->callback.method.park);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.position_pid, control->cfg->callback.method.position_pid);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.sin_x, control->cfg->callback.method.sin_x);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.speed_pid, control->cfg->callback.method.speed_pid);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.svpwm, control->cfg->callback.method.svpwm);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.step_svpwm, control->cfg->callback.method.step_svpwm);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.get_block_sector, control->cfg->callback.method.get_block_sector);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.dead_area_polarity_detection, control->cfg->callback.method.dead_area_polarity_detection);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.smc_init, control->cfg->callback.method.smc_init);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.smc_process, control->cfg->callback.method.smc_process);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.offline_param_detection_rs, control->cfg->callback.method.offline_param_detection_rs);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.offline_param_detection_ls, control->cfg->callback.method.offline_param_detection_ls);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.offline_param_detection_ld, control->cfg->callback.method.offline_param_detection_ld);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.offline_param_detection_lq, control->cfg->callback.method.offline_param_detection_lq);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.offline_param_detection_flux, control->cfg->callback.method.offline_param_detection_flux);
    MCL_FUNCTION_INIT_IF_NO_EMPTY(control->method.offline_param_detection_init, control->cfg->callback.method.offline_param_detection_init);

    control->method.offline_param_detection_init(&control->cfg->offline_param_detection_cfg);
    control->cfg->callback.init();

    return mcl_success;
}
```

> **源码位置：** [hpm_mcl_control.c:L563-L636](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.c#L563-L636)

#### 解析

**两阶段挂载机制：**

1. **第一阶段：默认实现挂载** — 将所有函数指针指向中间件内置的默认实现（如 `hpm_mcl_control_clarke`、`hpm_mcl_control_pi` 等）
2. **第二阶段：用户覆盖** — 通过 `MCL_FUNCTION_INIT_IF_NO_EMPTY` 宏，若用户在 `cfg->callback.method` 中提供了自定义实现，则覆盖默认函数指针

```c
#define MCL_FUNCTION_INIT_IF_NO_EMPTY(default_func, user_func) \
    if ((user_func) != NULL) { default_func = user_func; }
```

**回调驱动架构的实现原理：**

```
用户层:  cfg->callback.method.clarke = my_custom_clarke;  // 用户自定义
           ↓ MCL_FUNCTION_INIT_IF_NO_EMPTY
中间件层: control->method.clarke = my_custom_clarke;       // 覆盖默认值
           ↓ 运行时调用
控制环:   loop->control->method.clarke(ia, ib, ic, &alpha, &beta);  // 执行用户实现
```

**与 v1 直接函数调用的差异：**

| 维度 | v1 (MC_LIB) | v2 (MCL V2) |
|------|-------------|-------------|
| 调用方式 | 直接调用 `hpm_mcl_bldc_foc_clarke()` | 函数指针 `control->method.clarke()` |
| 算法替换 | 需修改源码或重编译 | 运行时通过 callback 覆盖，无需改中间件 |
| 测试友好性 | 难以 mock 硬件依赖 | 可注入测试桩函数 |
| 间接调用开销 | 无（直接跳转） | 极小（一次指针解引用，<1ns@600MHz） |

**工程要点：** 默认实现中，所有 PID 控制器（电流d/q轴、速度、位置）均挂载 `hpm_mcl_control_pi`（位置式 PI），而非增量式 `hpm_mcl_delta_pid`。若需使用增量式 PID，需在 `callback.method` 中显式覆盖。

---

### 11.6 IIR 直接 I 型滤波器 (`hpm_mcl_filter_iir_df1`)

#### 原理

IIR 直接 I 型（Direct Form I）二阶节级联结构，传递函数为：

$$H(z) = \prod_{i=1}^{N} \frac{b_{0,i} + b_{1,i} z^{-1} + b_{2,i} z^{-2}}{1 + a_{1,i} z^{-1} + a_{2,i} z^{-2}} \cdot S_i$$

其中 $N$ 为级联节数（`cfg->section`），$S_i$ 为每级的增益缩放因子。

#### 代码

```c
float hpm_mcl_filter_iir_df1(mcl_filter_iir_df1_t *iir, float input)
{
    uint8_t i = 0;
    float y = 0;

    MCL_ASSERT_OPT(iir != NULL, mcl_invalid_pointer);
    iir->mem[0].x0 = input;
    for (i = 0; i < iir->cfg->section; i++) {
        y = iir->cfg->matrix[i].b0 * iir->mem[i].x0 +
            iir->cfg->matrix[i].b1 * iir->mem[i].x1 +
            iir->cfg->matrix[i].b2 * iir->mem[i].x2 -
            iir->cfg->matrix[i].a1 * iir->mem[i].y1 -
            iir->cfg->matrix[i].a2 * iir->mem[i].y2;
        iir->mem[i].x2 = iir->mem[i].x1;
        iir->mem[i].x1 = iir->mem[i].x0;
        iir->mem[i].y2 = iir->mem[i].y1;
        iir->mem[i].y1 = y;
        y *= iir->cfg->matrix[i].scale;
        if ((i+1) < iir->cfg->section) {
            iir->mem[i+1].x0 = y;
        }
    }

    return y;
}
```

> **源码位置：** [hpm_mcl_filter.c:L23-L47](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_filter.c#L23-L47)

#### 逐行解析

**差分方程：**

$$y[n] = b_0 \cdot x[n] + b_1 \cdot x[n-1] + b_2 \cdot x[n-2] - a_1 \cdot y[n-1] - a_2 \cdot y[n-2]$$

**状态变量含义与更新顺序：**

| 变量 | 含义 | 更新方向 |
|------|------|---------|
| `x0` | 当前输入 $x[n]$ | 由外部赋值（首级为 input，后续级为上一级输出） |
| `x1` | 上一次输入 $x[n-1]$ | `x1 ← x0`（当前输入变为历史） |
| `x2` | 上上次输入 $x[n-2]$ | `x2 ← x1`（最老的历史被覆盖） |
| `y1` | 上一次输出 $y[n-1]$ | `y1 ← y`（当前输出变为历史） |
| `y2` | 上上次输出 $y[n-2]$ | `y2 ← y1`（最老的历史被覆盖） |

**更新顺序至关重要：** 必须先更新 `x2 ← x1`，再 `x1 ← x0`；先 `y2 ← y1`，再 `y1 ← y`。代码中严格遵循此顺序，避免历史数据被提前覆盖。

**级联机制：** 上一级输出 `y` 乘以缩放因子 `scale` 后，作为下一级的 `x0` 输入。bldc_foc 示例中使用 2 级级联（`section=2`），实现 4 阶 IIR 滤波器。

---

### 11.7 Type-I PLL 锁相环 (`hpm_mcl_filter_pll_type_i`)

#### 原理

Type-I PLL 采用 **PI 控制器 + 低通滤波器 + 积分器** 的级联结构，从正余弦输入信号中提取角度和速度。

结构框图：

```
sin_val, cos_val
       │
       ▼
  ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
  │ 相位检测 │────▶│ PI控制器 │────▶│ 低通滤波 │────▶│ 积分器   │──▶ θ
  │ (a0计算) │     │ (x0+kp)  │     │ (x1)     │     │ (x2)     │
  └──────────┘     └──────────┘     └──────────┘     └──────────┘
                                                         │
                                                         ▼
                                                        speed
```

#### 代码

```c
void hpm_mcl_filter_pll_type_i(mcl_filter_pll_type_i_t *pll, float sin_val, float cos_val, mcl_filter_pll_output_t *output)
{
    float a0, a1;

    a0 = cosf(pll->x2) * pll->cfg.k1 * sin_val - sinf(pll->x2) * pll->cfg.k1 * cos_val;

    pll->x0 += pll->cfg.ki * a0;
    a1 = pll->x0 + a0 * pll->cfg.kp;

    pll->x1 += pll->cfg.lowpass_k * (a1 - pll->x1);

    pll->x2 += a1 * pll->cfg.c;
    pll->x2 = MCL_ANGLE_MOD_X(0, MCL_2PI, pll->x2);

    output->theta = pll->x2;
    output->speed = pll->x1;
}
```

> **源码位置：** [hpm_mcl_filter.c:L49-L70](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_filter.c#L49-L70)

#### 逐行解析

**① 相位检测（a0 计算）：**

$$a_0 = k_1 \cdot (\cos\hat\theta \cdot \sin\theta_{in} - \sin\hat\theta \cdot \cos\theta_{in}) = k_1 \cdot \sin(\theta_{in} - \hat\theta)$$

当相位误差 $\Delta\theta = \theta_{in} - \hat\theta$ 很小时，$a_0 \approx k_1 \cdot \Delta\theta$，即相位误差的线性近似。

**② PI 控制器：**

$$x_0 \mathrel{+}= k_i \cdot a_0 \quad \text{(积分项)}$$
$$a_1 = x_0 + k_p \cdot a_0 \quad \text{(比例+积分)}$$

**③ 低通滤波器：**

$$x_1 \mathrel{+}= k_{lp} \cdot (a_1 - x_1)$$

一阶 IIR 低通滤波，滤除 PI 输出中的高频纹波，$x_1$ 即为估计速度。

**④ 积分器（角度更新）：**

$$x_2 \mathrel{+}= a_1 \cdot c$$

$c$ 为积分增益，将频率估计值积分为角度。$x_2$ 即为估计角度。

**⑤ 角度归一化 `MCL_ANGLE_MOD_X`：**

```c
pll->x2 = MCL_ANGLE_MOD_X(0, MCL_2PI, pll->x2);
```

将角度限制在 $[0, 2\pi)$ 范围内，防止浮点溢出。这是实时控制中**必须**的操作——若角度持续累加，浮点精度在大数值时会急剧下降（float 在 $10^6$ 量级时有效位数仅剩个位），导致角度分辨率丧失。

---

### 11.8 Type-II PLL 锁相环 (`hpm_mcl_filter_pll_type_ii`)

#### 原理

Type-II PLL 采用**双积分器**结构，相比 Type-I 增加了一个积分环节，可实现更高的稳态精度（对加速度输入无稳态误差）。

结构框图：

```
sin_val, cos_val
       │
       ▼
  ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
  │ 相位检测 │────▶│ 积分器1  │────▶│ 滤波器   │────▶│ 积分器2  │──▶ θ
  │ (a0计算) │     │ (x0)     │     │ (a2,a3)  │     │ (x2→x3) │
  └──────────┘     └──────────┘     └──────────┘     └──────────┘
                                                         │
                                                         ▼
                                                       speed
```

#### 代码

```c
void hpm_mcl_filter_pll_type_ii(mcl_filter_pll_type_ii_t *pll, float sin_val, float cos_val, mcl_filter_pll_output_t *output)
{
    float a0, a1, a2, a3, a4;

    a0 = cosf(pll->x3) * pll->cfg.k1 * sin_val - sinf(pll->x3) * pll->cfg.k1 * cos_val;

    pll->x0 += a0;
    a1 = pll->x0 * pll->cfg.c;

    a2 = a1 - pll->x1 * pll->cfg.a;
    a3 = a2 * pll->cfg.b + a2;

    pll->x2 += a3;
    a4 = pll->x2 * pll->cfg.c;

    pll->x1 = a2;
    pll->x3 = MCL_ANGLE_MOD_X(0, MCL_2PI, a4);

    output->theta = pll->x3;
    output->speed = a3;
}
```

> **源码位置：** [hpm_mcl_filter.c:L72-L99](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_filter.c#L72-L99)

#### 逐行解析

**① 相位检测：** 与 Type-I 相同，$a_0 = k_1 \cdot \sin(\theta_{in} - \hat\theta)$

**② 第一积分器：**

$$x_0 \mathrel{+}= a_0, \quad a_1 = x_0 \cdot c$$

直接积分相位误差（无比例项），$c$ 为增益缩放。

**③ 滤波器（零极点结构）：**

$$a_2 = a_1 - x_1 \cdot a, \quad a_3 = a_2 \cdot b + a_2 = a_2 \cdot (1 + b)$$

参数 $a$ 为零点系数，$b$ 为极点系数，构成一阶零极点滤波器。

**④ 第二积分器：**

$$x_2 \mathrel{+}= a_3, \quad a_4 = x_2 \cdot c$$

将滤波后的频率估计积分为角度。

**⑤ 状态更新：**

$$x_1 = a_2, \quad x_3 = \text{MOD}(a_4, 2\pi)$$

#### Type-I vs Type-II 对比

| 维度 | Type-I PLL | Type-II PLL |
|------|-----------|-------------|
| 积分器数量 | 1 个 | 2 个 |
| 速度估计来源 | 低通滤波后的 PI 输出 | 滤波器输出 $a_3$ |
| 角度估计来源 | 单积分器 $x_2$ | 双积分器 $x_2 \rightarrow x_3$ |
| 对恒速输入的稳态误差 | 零（一型系统） | 零（二型系统） |
| 对加速度输入的稳态误差 | 非零（有差） | 零（无差） |
| 参数数量 | 5 个（k1, kp, ki, c, lowpass_k） | 4 个（k1, a, b, c） |
| 适用场景 | 恒速或缓变速 | 存在加速度的应用（如伺服快速加减速） |

---

### 11.9 电机角度对齐 (`hpm_mcl_motor_angle_alignment`)

#### 原理

电机角度对齐的目的是**确定编码器零点与电机 d 轴的对应关系**。方法：给定 d 轴电流 $I_d$，使转子磁极被强制吸引到 d 轴方向，此时编码器的读数即为电角度零点。

$$\text{对齐原理：} I_d > 0 \Rightarrow \text{转子对齐到 } d \text{ 轴} \Rightarrow \theta_{encoder} = 0$$

#### 基础对齐代码

```c
hpm_mcl_stat_t hpm_mcl_motor_angle_alignment_basic(mcl_loop_t *loop, mcl_motor_alignment_basic_cfg_t *cfg)
{
    mcl_user_value_t id, iq;

    MCL_ASSERT(loop != NULL, mcl_invalid_pointer);
    MCL_ASSERT(cfg != NULL, mcl_invalid_pointer);
    MCL_ASSERT(loop->encoder != NULL, mcl_invalid_pointer);

    iq.enable = true;
    iq.value = cfg->q_current;
    hpm_mcl_loop_set_current_q(loop, iq);

    hpm_mcl_encoder_force_theta(loop->encoder, 0, true);

    id.enable = true;
    id.value = cfg->d_current;
    hpm_mcl_loop_set_current_d(loop, id);

    MCL_DELAY_MS(cfg->delay_ms);

    hpm_mcl_encoder_set_initial_theta(loop->encoder, hpm_mcl_encoder_get_raw_theta(loop->encoder));
    hpm_mcl_encoder_force_theta(loop->encoder, 0, false);

    id.enable = true;
    id.value = 0;
    hpm_mcl_loop_set_current_d(loop, id);
    iq.enable = false;
    iq.value = 0;
    hpm_mcl_loop_set_current_q(loop, iq);

    return mcl_success;
}
```

> **源码位置：** [hpm_mcl_loop.c:L648-L685](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L648-L685)

#### 基础对齐流程解析

1. **设置 q 轴电流：** 可配置小量 q 轴电流扰动，帮助克服静摩擦
2. **强制编码器角度为零：** `hpm_mcl_encoder_force_theta(encoder, 0, true)` — 锁定编码器输出为 0，使 FOC 控制环以 $\theta = 0$ 运行
3. **施加 d 轴电流：** 转子被吸引到 d 轴方向（$\theta = 0$）
4. **等待对齐完成：** `MCL_DELAY_MS(cfg->delay_ms)` — 阻塞延时等待转子稳定
5. **标定编码器零点：** `hpm_mcl_encoder_set_initial_theta` — 将当前编码器原始读数记录为零点偏移
6. **释放编码器强制：** `hpm_mcl_encoder_force_theta(encoder, 0, false)` — 恢复编码器正常跟踪
7. **撤除对齐电流：** 将 d/q 轴电流归零

#### 三阶段对齐代码

```c
hpm_mcl_stat_t hpm_mcl_motor_angle_alignment_three_stage(mcl_loop_t *loop, mcl_motor_alignment_three_stage_cfg_t *cfg)
{
    mcl_user_value_t id, iq;
    int stage;

    MCL_ASSERT(loop != NULL, mcl_invalid_pointer);
    MCL_ASSERT(cfg != NULL, mcl_invalid_pointer);
    MCL_ASSERT(loop->encoder != NULL, mcl_invalid_pointer);

    iq.enable = true;
    iq.value = 0;
    hpm_mcl_loop_set_current_q(loop, iq);

    hpm_mcl_encoder_force_theta(loop->encoder, 0, true);

    for (stage = 0; stage < 3; stage++) {
        id.enable = true;

        switch (stage) {
        case 0:
            id.value = cfg->stage1.d_current;
            hpm_mcl_loop_set_current_d(loop, id);
            iq.enable = true;
            iq.value = cfg->stage1.q_current;
            hpm_mcl_loop_set_current_q(loop, iq);
            MCL_DELAY_MS(cfg->stage1.delay_ms);
            break;

        case 1:
            id.value = cfg->stage2.d_current;
            hpm_mcl_loop_set_current_d(loop, id);
            iq.enable = true;
            iq.value = 0;
            hpm_mcl_loop_set_current_q(loop, iq);
            MCL_DELAY_MS(cfg->stage2.delay_ms);
            break;

        case 2:
            id.value = cfg->stage3.d_current;
            hpm_mcl_loop_set_current_d(loop, id);
            iq.enable = true;
            iq.value = 0;
            hpm_mcl_loop_set_current_q(loop, iq);
            MCL_DELAY_MS(cfg->stage3.delay_ms);
            break;
        }
    }

    hpm_mcl_encoder_set_initial_theta(loop->encoder, hpm_mcl_encoder_get_raw_theta(loop->encoder));
    hpm_mcl_encoder_force_theta(loop->encoder, 0, false);
    MCL_DELAY_MS(cfg->final_delay_ms);

    id.enable = true;
    id.value = 0;
    hpm_mcl_loop_set_current_d(loop, id);
    iq.enable = false;
    iq.value = 0;
    hpm_mcl_loop_set_current_q(loop, iq);

    return mcl_success;
}
```

> **源码位置：** [hpm_mcl_loop.c:L687-L759](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.c#L687-L759)

#### 三阶段对齐流程解析

| 阶段 | d 轴电流 | q 轴电流 | 目的 |
|------|---------|---------|------|
| Stage 1: 粗对齐 | 大电流（如 8A） | 小扰动（如 0.5A） | 克服静摩擦力矩，快速将转子拉到 d 轴附近 |
| Stage 2: 精对齐 | 中电流（如 5A） | 0 | 降低电流减少过冲，提高对齐精度 |
| Stage 3: 稳定化 | 小电流（如 3A） | 0 | 最小电流维持位置，避免电流突变导致转子振荡 |

**三阶段对齐的工程意义：**

1. **克服静摩擦：** 电机从静止启动时静摩擦力矩大，单阶段小电流可能无法驱动转子。Stage 1 的大电流确保转子可靠转动
2. **q 轴扰动打破卡点：** Stage 1 的 q 轴扰动电流产生切向力，帮助转子脱离可能卡住的齿槽位置
3. **渐进式收敛：** 电流逐级降低，避免大电流突然撤除时转子因惯性过冲，导致对齐角度偏差
4. **最终稳定延时：** `final_delay_ms` 确保在释放编码器强制之前转子已完全稳定

**统一入口函数：**

```c
hpm_mcl_stat_t hpm_mcl_motor_angle_alignment(mcl_loop_t *loop, mcl_motor_alignment_cfg_t *cfg)
{
    switch (cfg->algorithm) {
    case mcl_alignment_algorithm_basic:
        return hpm_mcl_motor_angle_alignment_basic(loop, &cfg->config.basic);
    case mcl_alignment_algorithm_three_stage:
        return hpm_mcl_motor_angle_alignment_three_stage(loop, &cfg->config.three_stage);
    case mcl_alignment_algorithm_adaptive:
        return mcl_invalid_argument;
    default:
        return mcl_invalid_argument;
    }
}
```

通过 `mcl_motor_alignment_algorithm_t` 枚举和 `mcl_motor_alignment_algorithm_cfg_t` 联合体实现策略模式，用户选择算法后自动路由到对应实现。`adaptive` 算法为预留接口，尚未实现。

---

## 总结

### 核心知识点

1. **`mcl_loop_t`** 是聚合调度器，不实现算法，只协调子系统运行。
2. **`mcl_control_method_t`** 函数指针表是 V2 架构的最大创新点——算法完全可替换。
3. **三种硬件加速梯度**：soft-FOC（全软件）→ hybrid-FOC（VSC/CLC/QEO可选）→ hardware-FOC（硬件电流环）。
4. **独立故障检测** `detect` 模块以独立Timer运行，与PWM同步的 `hpm_mcl_loop()` 解耦。
5. **三阶段对齐** 通过 `hpm_mcl_motor_angle_alignment()` API 统一调用，硬件/软件模式下电流值不同。
6. **API兼容机制**：宏重载使同一函数名 `hpm_mcl_loop_init()` 兼容 v1.9.0 和 v1.10.0+。

### 关键文件索引

| 文件 | 职责 |
|------|------|
| [hpm_mcl_loop.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/loop/hpm_mcl_loop.h) | 核心调度器定义 + 对齐算法 + API |
| [hpm_mcl_control.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/control/hpm_mcl_control.h) | 函数指针表 + PID/SMC/离线检测配置 |
| [hpm_mcl_encoder.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h) | 编码器抽象 + 4种速度计算 + 角度预测 |
| [hpm_mcl_analog.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/sensor/hpm_mcl_analog.h) | 10通道ADC + 每通道独立IIR滤波 |
| [hpm_mcl_drivers.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h) | PWM驱动抽象 + BLDC/步进/非对称通道 |
| [hpm_mcl_detect.h](../../../hpm_MC/middleware/hpm_mcl_v2/core/detect/hpm_mcl_detect.h) | 4子系统故障监控 + 输出禁用保护 |
| [hpm_mcl_cfg.h](../../../hpm_MC/middleware/hpm_mcl_v2/hpm_mcl_cfg.h) | 编译时特性开关 (SMC/DQ解耦/死区补偿/角度预测) |
| [hpm_mcl_hw_loop.h](../../../hpm_MC/middleware/hpm_mcl_v2/drivers/hpm_mcl_hw_loop.h) | 硬件混合环路 (VSC/CLC/QEO 动态使能) |
| [hpm_mcl_physical.h](../../../hpm_MC/middleware/hpm_mcl_v2/hpm_mcl_physical.h) | 电机/板级/时间 三大物理参数结构体 |
| [bldc_foc.c](../../../hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c) | BLDC FOC完整示例：初始化→对齐→速度/位置模式 |

### 下一步

- [ALG-03-PI-Current-Regulator](../ALG-03-PI-Current-Regulator.md)：电流环PI调节器理论
- [ALG-07-Sensorless-Observers](../ALG-07-Sensorless-Observers.md)：SMC无感观测器理论
- [SDK-03-HPM-MC-v2-Detect](SDK-03-HPM-MC-v2-Detect.md)：HPM离线参数检测模块
- [SDK-04-HPM-MC-v2-Hybrid-Ctrl](SDK-04-HPM-MC-v2-Hybrid-Ctrl.md)：力位混合控制模块

---

*文档更新时间: 2026-05-23*

---

## 10. 硬件加速深度分析（HPM 官方资料补充）

### 10.1 1μs 软件环路

HPM_MCLV2 支持 **1μs 级别的软件电流环**（主频 ≥ 600MHz 的先楫 MCU 可保证）。

**实现机制**:
- 控制算法放置在 **fast 内存空间**（紧耦合内存 ITCM/DTCM），降低访存延迟
- 底层驱动高度优化，减少数据传输开销
- 控制算法精简，确保在 1μs 内完成坐标变换 → PID → SVPWM 全链路

**复现方法**（来自官方 `bldc_foc` 示例）:
1. 关闭死区补偿、dq 轴解耦等非必需特性
2. 确认编译优化级别为 `-O2` 或更高
3. 使用逻辑分析仪监测 PWM 中断间隔

**带宽配置**: 通过宏 `MOTOR0_CURRENT_LOOP_BANDWIDTH` 调节电流环带宽

### 10.2 PID → 3P-3Z 控制器转换

HPM 硬件电流环（CLC 外设）使用 **3P-3Z（三极点三零点）控制器** 而非传统 PID。

**3P-3Z 结构**: 2P2Z 控制器（高频段）级联 1P1Z 控制器（低频段）

离散传递函数：
```
u(z)/e(z) = (b0 + b1·z⁻¹ + b2·z⁻²)/(1 - a0·z⁻¹ - a1·z⁻²) · (1 + b3·z⁻¹)/(1 - a2·z⁻¹)
```

**转换 API** (`hpm_mcl_control.h`):
```
hpm_mcl_stat_t hpm_mcl_pid_to_3p3z(mcl_control_pid_cfg_t *cfg_pid, mcl_clc_coeff_cfg_t *cfg_3p3z)
```
- 输入: kp, ki, kd（增量式 PID 参数）
- 输出: b0, b1, b2, b3, a0, a1, a2（3P-3Z 系数）
- **仅支持增量式 PID 转换**（位置式 PID 转换后稳态误差不一致）

**性能验证**: Matlab 仿真显示增量式 PID → 3P-3Z 转换后系统响应完全重合，可直接替代。

参考: [HPM KB: PID切换到3P-3Z控制器](https://kb.hpmicro.com/2024/10/16/)