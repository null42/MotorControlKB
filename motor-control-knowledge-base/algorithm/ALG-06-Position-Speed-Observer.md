# ALG-06 位置与速度观测器

> **📌 文档定位：** 本文是观测器的**入门基础篇**。首先从有感场景（编码器/Hall）入手，介绍位置提取与速度计算的基本方法（T法、M法、M-T法）；随后以概要形式引入无感观测器的核心思想（SMO、Luenberger、PLL），建立对"如何从电气量反推机械量"的整体认知。**深入学习无传感器场景下各类观测器的数学推导、代码实现与参数整定，请参考 [ALG-07 无感FOC观测器](./ALG-07-Sensorless-Observers.md)。**

---

## 1. 为什么要观测器？

无传感器FOC通过电压、电流估算转子位置和速度，省去编码器/旋变，降低成本、提高可靠性。核心挑战：**从电气量反推机械量**。

## 2. 滑模观测器 (SMO) — 入门概要

> ⚠️ 本节及后续第3、4节为无感观测器的**概要介绍**，旨在建立基本概念。完整数学推导、代码实现与参数整定见 **[ALG-07 无感FOC观测器](./ALG-07-Sensorless-Observers.md)**。

**原理**：构造一个滑模面 $s = \hat{i}_s - i_s$（估算电流-实测电流），用开关函数强制系统在滑模面上滑动。

**αβ坐标系下的SMO：**

$$\frac{d\hat{i}_\alpha}{dt} = -\frac{R}{L}\hat{i}_\alpha + \frac{1}{L}v_\alpha - \frac{k}{L} \cdot \text{sign}(\hat{i}_\alpha - i_\alpha)$$

$$\frac{d\hat{i}_\beta}{dt} = -\frac{R}{L}\hat{i}_\beta + \frac{1}{L}v_\beta - \frac{k}{L} \cdot \text{sign}(\hat{i}_\beta - i_\beta)$$

其中 $\text{sign}(x)$ 为符号函数，k为滑模增益。

**反电势提取**（低通滤波后）：$\hat{e}_{\alpha\beta} = k \cdot \text{LPF}\{\text{sign}(\hat{i}_s - i_s)\}$

**位置估计**：$\hat{\theta}_e = \text{atan2}(-\hat{e}_\alpha, \hat{e}_\beta)$

**优点**：对参数变化鲁棒性强；**缺点**：开关函数引入高频抖动（chattering）

## 3. 龙伯格观测器 (Luenberger)

**原理**：线性全阶/降阶状态观测器，通过极点配置实现误差收敛。

$$\frac{d\hat{x}}{dt} = A\hat{x} + Bu + L(y - C\hat{x})$$

其中L为观测器增益矩阵，决定收敛速度。零极点配置在电机极点的2~5倍频率处。

**优点**：平滑无抖动；**缺点**：依赖电机参数（R, L），参数失配时有稳态误差

## 4. PLL锁相环位置跟踪

从反电势提取位置后，用PLL平滑和跟踪：

$$
\varepsilon = \hat{e}_\alpha \cos\hat{\theta} + \hat{e}_\beta \sin\hat{\theta} \approx k_E \omega_e (\hat{\theta}_e - \theta_e)
$$

PLL结构：鉴相器(ε) → PI控制器 → 积分器 → θ̂

## 5. 三种观测器对比

| 特性 | SMO | Luenberger | PLL |
|------|-----|-----------|-----|
| 鲁棒性 | ★★★★★ | ★★★ | ★★★★ |
| 平滑度 | ★★★ | ★★★★★ | ★★★★★ |
| 低速性能 | ★★★ | ★★★★ | ★★★★ |
| 参数敏感性 | 低 | 高 | 中 |
| 实现复杂度 | 中 | 高 | 低 |

## 6. 关键工程问题

- **低速区**：反电势小→信噪比低→SMO失效；需切换到开环或HFI
- **过渡区**：开环→闭环平滑切换（加权融合）
- **初始位置**：静止时反电势=0，需高频注入或预定位

> **仿真体验**：比较三种观测器在不同转速和初始误差下的收敛表现。

:::sim observer

### 🔗 hpm_MC 代码实现参考

**v2 编码器速度计算** (`hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h`):
- 四种速度计算方法：
  - **T法**: 测两个脉冲间时间 → 适合低速
  - **M法**: 固定时间内计脉冲数 → 适合高速
  - **M-T法**: T法+M法混合 → 全速域
  - **PLL法**: 锁相环提取速度 → 平滑低延迟
- `mcl_encoder_t` 内建 IIR 速度滤波器

**角度预测**:
- `theta_forecast` 机制：根据当前速度预测下一周期角度，补偿计算延迟

**编码器驱动**:
- `hpm_mcl_abz.h` — ABZ/QEI 正交编码器（θ = phase_count × 2π / encoder_resolution）
- `hpm_mcl_uvw.h` — UVW/Hall 三路霍尔（θ = hall_state × 60° + 30°偏移）

**参考**: `SDK-02-HPM-MC-v2-Core-Loop.md` 第5节「传感器处理」

#### PLL 锁相环滤波器 — HPM 官方资料补充

HPM MCL 提供两种 PLL 滤波器：

| 特性 | Type I PLL | Type II PLL |
|------|-----------|-------------|
| **传递函数** | H(s) = Kd·Kv / [s(1+s·Tf)] | 含双积分器 + 补偿滤波器 |
| **速度跟踪** | 跟踪恒定速度存在固有相位误差 | **跟踪恒定速度无静差** |
| **噪声抑制** | 一般 | **更优**（额外低通滤波级） |
| **适用场景** | 简单速度估计 | 高精度位置/速度观测 |

**Type II PLL 结构**:
- 积分器1: 生成与速度成比例的信号，离散传递函数 Hint₁(z) = Ts·z/(z-1)
- 补偿滤波器: 提供相位裕量 + 抑制高频噪声，Hcomp(z) = k1·c·(1-a·z⁻¹)/(1-b·z⁻¹)
- 积分器2: 生成位置估计

**配置 API** (`hpm_mcl_filter.h`):
```c
typedef struct {
    float k1;  // ADC 增益
    float a;   // 补偿器零点系数（典型: -0.99996948）
    float b;   // 补偿器极点系数（典型: -0.99966431）
    float c;   // 积分器增益（典型: 1e-4）
} mcl_filter_pll_type_ii_cfg_t;
```

**结论**: HPM_MCLV2 推荐使用 Type II PLL（更好的跟踪精度和噪声抑制）

参考: [HPM KB: 锁相环滤波器性能分析](https://kb.hpmicro.com/2024/10/31/)


## 🧪 仿真验证
> 本模块的理论可在 [C 语言仿真](../simulation/SIM-00-C-Simulation-Overview.md) 中验证。
> 对应仿真模式：MODE_SELECT_FOC_SENSORLESS (31)，关键操作：对比 OBSV.theta_d（观测角度）和 ACM.theta_d（真实角度），观察观测器收敛过程