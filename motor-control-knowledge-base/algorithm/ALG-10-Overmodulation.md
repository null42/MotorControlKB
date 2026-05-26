# ALG-10 过调制与六阶梯波

**模块编号：** ALG-10
**模块名称：** 过调制与六阶梯波（Overmodulation & Six-Step Wave）
**文档版本：** v2.0
**适用对象：** 电机控制工程师、嵌入式开发者
**前置知识：** ALG-09 SVPWM 空间矢量调制、Park/Clarke 坐标变换、逆变器拓扑基础

---

## 1. 📌 核心摘要 ★★★★☆ 🔰📚

过调制（Overmodulation）是 SVPWM 在线性调制区之外进一步提升电压利用率的必要手段。当调制比 $m$ 超过 1.0（内切圆极限）后，参考电压矢量 $V_{ref}$ 的圆形轨迹被逆变器六边形电压边界截断——我们被迫在"电压失真"和"电压幅值"之间做权衡。本模块的核心认知锚点：**过调制不是故障，而是一个受控的非线性工作区域**，理解它需要掌握从"圆形→截断圆→六边形顶点→六阶梯波"的完整演变路径。

```
    SVPWM 调制区域演变示意：

    m ≤ 1.0              1.0 < m < 1.155        m = 1.155
  ┌───────────┐       ┌───────────┐       ┌───────────┐
  │   ╭───╮  ╱│       │  ╱─────╲  ╲│       │  ●─────●  │
  │  │  ○  │ ╱ │  →   │ ╱   ⌒   ╲ ╲│  →   │ ╱       ╲ │
  │  │     │╱  │       │╱         ╲╲│       │●         ●│
  │   ╰───╯   │       │            │       │ ╲       ╱ │
  └───────────┘       └───────────┘       └──●─────●───┘
   线性调制区          过调制 I 区          六阶梯波区
   Vref 圆形完整      Vref 弧段被截断       Vref 仅走六顶点
```

| 关键概念 | 说明 |
|----------|------|
| 调制比 m | $m = V_{ref} / (V_{dc}/\sqrt{3})$，m=1 对应内切圆，m≈1.155 对应外接圆 |
| 过调制 I 区 | $1.0 < m < 1.155$，部分弧段被六边形截断，基波增大但引入低次谐波 |
| 六阶梯波 | $m \approx 1.155$，$V_{ref}$ 仅走 6 个基本矢量顶点，基波利用率达理论最大值 |
| 电压利用率 | 六阶梯波基波 $V_1 = \frac{2}{\pi}V_{dc} \approx 0.637V_{dc}$，对 SPWM 提升 27.3% |

---

## 2. 📐 原理推导 ★★★★☆ 📚

### 2.1 调制区域划分

SVPWM的电压输出能力受限于六边形边界。根据调制比 m 的不同，可分为三个区域：

| 区域 | m 范围 | Vref轨迹 | 输出电压特征 |
|------|--------|----------|-------------|
| 线性区 | m ≤ 1.0 | 完整圆形（内切圆内） | 正弦基波 |
| 过调制I区 | 1.0 < m < 1.155 | 部分弧段被六边形截断 | 基波增大，低次谐波出现 |
| 六阶梯波 | m = 1.155 | 六边形顶点（外接圆） | 基波最大，谐波丰富 |

其中 m = Vref / (Vdc/√3)，m = 1.0 对应内切圆半径，m = 1.155 对应外接圆（六边形顶点）半径。

### 2.2 过调制I区 (1.0 < m < 1.155)

Vref超出六边形的弧段被限幅到六边形边界：

- **限幅算法**：若 Vref 超出六边形，沿角度方向将其缩放到六边形边线上
- **结果**：基波幅值高于线性区最大值，但同时引入5次、7次等低次谐波
- **典型应用**：短暂加速/爬坡时提升电压利用率

### 2.3 过调制II区 / 六阶梯波

当 m = 1.155 时，Vref始终落在六边形顶点（6个基本矢量）：

- 输出电压波形变为6拍阶梯波
- 基波利用率达到理论最大值：$V_1 = \frac{2}{\pi}V_{dc} \approx 0.637 V_{dc}$
- 对SPWM的利用率提升：$0.637 / 0.5 = 1.273$ 倍
- **代价**：谐波含量极大（5次、7次、11次、13次…）

### 2.4 过调制策略选择

| 策略 | 描述 | 优点 | 缺点 |
|------|------|------|------|
| 最小相位误差 | 保持Vref角度不变，仅缩小幅值 | 相位连续 | 幅值失真 |
| 最小幅值误差 | 保持Vref幅值不变，改变角度 | 基波最大 | 角度跳变 |
| 混合策略 | 低速用最小相位，高速逼近六步 | 过渡平滑 | 实现复杂 |

### 2.5 工程实践

- **弱磁区**：天然需要过调制，因为反电势超过Vdc限制
- **转矩精度**：过调制区转矩-电流关系非线性，需查表或模型补偿
- **电流环稳定性**：过调制引入的谐波可能激发LC谐振

> **仿真体验**：拖动调制比 m 滑块，观察Vref轨迹从圆形→被截断→六边形顶点→六阶梯波的演变过程。

:::sim overmodulation

---

## 3. 🔢 数学建模 ★★★★☆ 📚

### 3.1 调制比与电压极限

调制比 $m$ 的精确定义：

$$m = \frac{|V_{ref}|}{V_{dc}/\sqrt{3}} = \frac{\sqrt{V_\alpha^2 + V_\beta^2}}{V_{dc}/\sqrt{3}}$$

其中：
- $V_{ref}$：参考电压矢量幅值，单位 V
- $V_{dc}$：直流母线电压，单位 V
- $V_\alpha, V_\beta$：静止坐标系电压分量，单位 V

线性调制区最大输出相电压基波幅值：
$$V_{1,\max}^{\text{linear}} = \frac{V_{dc}}{\sqrt{3}} \approx 0.577V_{dc} \quad [\text{V}]$$

六阶梯波时最大输出相电压基波幅值（傅里叶分解）：
$$V_{1,\max}^{\text{six-step}} = \frac{2}{\pi}V_{dc} \approx 0.637V_{dc} \quad [\text{V}]$$

电压利用率提升比：
$$\eta_{boost} = \frac{0.637}{0.577} \approx 1.103 \quad (\text{相对于SVPWM线性区提升 10.3%})$$

### 3.2 过调制I区电压增益特性

过调制I区中，实际基波电压 $V_1$ 与指令调制比 $m_i$ 呈非线性关系。根据"等面积补偿"原理，截断区域的基波损失需要通过剩余导通区域的幅值增大来补偿。近似关系：

$$V_1(m) = \frac{2}{\pi}V_{dc} \cdot f(m), \quad f(m) = \begin{cases} m & m \leq 1.0 \\ \frac{2}{\pi}\left[m \cdot \arcsin\left(\frac{1}{m}\right) + \sqrt{m^2 - 1}\right] & 1.0 < m \leq \frac{2}{\sqrt{3}} \end{cases}$$

其中 $f(m)$ 为过调制增益函数，无量纲。

### 3.3 谐波分析

六阶梯波相电压的傅里叶级数展开：

$$v_{an}(t) = \frac{2V_{dc}}{\pi}\left(\sin\omega t + \frac{1}{5}\sin 5\omega t + \frac{1}{7}\sin 7\omega t + \frac{1}{11}\sin 11\omega t + \frac{1}{13}\sin 13\omega t + \cdots\right)$$

其中 $\omega = 2\pi f_e$ 为基波电角频率，单位 rad/s。

各次谐波相对于基波的幅值比：
$$\frac{V_n}{V_1} = \frac{1}{n}, \quad n = 5, 7, 11, 13, 17, 19, \cdots \quad (n = 6k \pm 1, k \in \mathbb{Z}^+)$$

六阶梯波电压 THD 理论值：
$$\text{THD}_{\text{six-step}} = \frac{\sqrt{\sum_{n=5,7,11,\cdots}^{\infty} (1/n)^2}}{1} \times 100\% \approx 31.1\%$$

---

## 4. 💻 代码实现 ★★★☆☆ 🔧

### 4.1 过调制限幅算法

以下为基于扇区判别的过调制限幅 C 代码，实现最小相位误差策略——当 $V_{ref}$ 超出六边形时沿径向缩回边界：

```c
#include <math.h>

#define SQRT3       1.7320508f
#define ONE_OVER_SQRT3  0.5773503f

typedef struct {
    float alpha;
    float beta;
} alpha_beta_t;

typedef enum {
    SECTOR_1, SECTOR_2, SECTOR_3,
    SECTOR_4, SECTOR_5, SECTOR_6
} sector_t;

static sector_t svpwm_get_sector(float alpha, float beta)
{
    float v_ref1 = beta;
    float v_ref2 = -0.5f * beta + 0.8660254f * alpha;
    float v_ref3 = -0.5f * beta - 0.8660254f * alpha;

    uint8_t sec = 0;
    if (v_ref1 > 0.0f) sec |= 0x01;
    if (v_ref2 > 0.0f) sec |= 0x02;
    if (v_ref3 > 0.0f) sec |= 0x04;

    switch (sec) {
        case 3: return SECTOR_1;
        case 1: return SECTOR_2;
        case 5: return SECTOR_3;
        case 4: return SECTOR_4;
        case 6: return SECTOR_5;
        case 2: return SECTOR_6;
        default: return SECTOR_1;
    }
}

alpha_beta_t overmodulation_clip(float valpha, float vbeta, float vdc)
{
    float vdc_div_sqrt3 = vdc * ONE_OVER_SQRT3;
    float mag = sqrtf(valpha * valpha + vbeta * vbeta);

    if (mag <= vdc_div_sqrt3) {
        alpha_beta_t out = {valpha, vbeta};
        return out;
    }

    if (mag >= vdc * (2.0f / 3.0f)) {
        alpha_beta_t out = {
            valpha / mag * vdc * (2.0f / 3.0f),
            vbeta / mag * vdc * (2.0f / 3.0f)
        };
        return out;
    }

    sector_t sec = svpwm_get_sector(valpha, vbeta);

    float vdc_div_3 = vdc / 3.0f;
    float scale;

    switch (sec) {
        case SECTOR_1:
            scale = vdc_div_sqrt3 / (0.5f * SQRT3 * valpha + 0.5f * vbeta);
            break;
        case SECTOR_2:
            scale = vdc_div_sqrt3 / (0.5f * SQRT3 * valpha - 0.5f * vbeta);
            break;
        case SECTOR_3:
            scale = vdc_div_sqrt3 / vbeta;
            break;
        case SECTOR_4:
            scale = vdc_div_sqrt3 / (-0.5f * SQRT3 * valpha - 0.5f * vbeta);
            break;
        case SECTOR_5:
            scale = vdc_div_sqrt3 / (-0.5f * SQRT3 * valpha + 0.5f * vbeta);
            break;
        case SECTOR_6:
            scale = vdc_div_sqrt3 / (-vbeta);
            break;
        default:
            scale = 1.0f;
            break;
    }

    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.0f) scale = 0.0f;

    alpha_beta_t out = {valpha * scale, vbeta * scale};
    return out;
}
```

### 4.2 过调制使能判断

```c
typedef struct {
    float modulation_index;
    uint8_t overmod_enable;
    uint8_t six_step_enable;
} overmod_state_t;

overmod_state_t overmod_state_update(float valpha, float vbeta, float vdc)
{
    overmod_state_t state = {0};
    float mag = sqrtf(valpha * valpha + vbeta * vbeta);
    float m = mag / (vdc * ONE_OVER_SQRT3);

    state.modulation_index = m;

    if (m > 1.0f) {
        state.overmod_enable = 1;
    }
    if (m >= 1.15f) {
        state.six_step_enable = 1;
    }

    return state;
}
```

---

### 🔗 hpm_MC 代码实现参考

**v1 SVPWM 实现** (`hpm_mcl/inc/hpm_foc.h`):
- `hpm_foc_svpwm()` 标准七段式 SVPWM（0/7分段）
- 输出占空比 d_a, d_b, d_c 直接写入 PWM 比较寄存器

**v2 SVPWM 实现** (`hpm_mcl_v2/core/control/hpm_mcl_control.h`):
- `mcl_control_method_t` 中包含 svpwm 函数指针
- 占空比通过 `mcl_drivers_channel_t` 统一传递给 PWM 模块

**过调制处理**:
- hpm_mcl 中 SVPWM 内建电压限幅（clip to inverter max）
- 参考: 路径规划中的弱磁策略可在高速时自动降低磁链 → 等效扩展调制范围

**与 MC_LIB 对比**:
- MC_LIB: `MCFOC_SVPWM_F()` 支持三电阻/单电阻两种模式，含过调制处理
- hpm_MCL: 标准 SVPWM，过调制依赖弱磁策略间接处理

---

## 5. 🔧 参数整定 ★★★☆☆ 🔧

### 5.1 过调制使能时机

过调制的使能应基于调制比阈值而非转速阈值，因为同一转速在不同母线电压下调制比不同。

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 进入过调制阈值 $m_{enter}$ | 0.95 ~ 0.98 | 留滞回裕量，避免在线性区边界反复切换 |
| 退出过调制阈值 $m_{exit}$ | 0.90 ~ 0.93 | 滞回宽度建议 0.03 ~ 0.05 |
| 六阶梯波阈值 $m_{six}$ | 1.14 ~ 1.15 | 接近理论最大值时切换为六步模式 |

**滞回控制伪代码：**

```
if (m > m_enter && !overmod_active):
    overmod_active = true
elif (m < m_exit && overmod_active):
    overmod_active = false
```

### 5.2 过渡策略

从线性区平滑过渡到过调制区的要点：

1. **渐进限幅**：当 $m$ 在 $[1.0, 1.05]$ 区间时，限幅算法采用部分限幅（如 80% 限幅 + 20% 原始），避免电压矢量突变
2. **电流环增益补偿**：过调制区电压-电流关系非线性，FOC 电流环 PI 输出需要乘以补偿系数 $K_{comp}(m)$，该系数可通过离线标定查表获得
3. **积分器抗饱和**：过调制限幅后 $V_d, V_q$ 实际值与指令值有差，需要对 PI 积分器做 back-calculation 抗饱和处理

### 5.3 典型参数配置

| 应用场景 | 过调制策略 | 最大调制比 | 备注 |
|----------|-----------|-----------|------|
| 伺服定位 | 禁用 | 1.0 | 转矩精度优先 |
| 通用变频器 | 最小相位误差 | 1.10 | 兼顾电压利用率和电流质量 |
| 高速主轴 | 混合策略 → 六阶梯波 | 1.155 | 极速优先，容忍谐波 |
| 电动工具 | 六阶梯波 | 1.155 | 成本敏感，效率优先 |

---

## 6. 🔗 硬件约束 ★★★★☆ ⚠️

### 6.1 直流母线电压跌落

过调制区电流增大导致母线电压纹波加剧，实际 $V_{dc}$ 低于标称值，影响调制比计算精度。

- **纹波幅值估算**：$\Delta V_{dc} \approx \frac{I_{peak} \cdot T_s}{C_{dc}}$，其中 $I_{peak}$ 为相电流峰值（A），$T_s$ 为 PWM 周期（s），$C_{dc}$ 为母线电容值（F）
- **应对**：调制比计算应使用实时采样的 $V_{dc}$ 而非额定值；或在过调制区预留 3% ~ 5% 的电压裕量
- **相关模块**：[HW-03 直流母线电容选型]()

### 6.2 死区时间对高调制比的影响

调制比越高，有效导通时间占比越大，死区的影响相对减弱。但在六阶梯波模式下，电流换相时刻的死区会产生电压畸变：

- **死区时间** $t_{dead}$ 通常为 1 ~ 3 μs
- 影响体现在低调制比区域更大（$V_{out}/V_{dc}$ 越小，死区占比越高）
- 过调制区死区补偿仍然必要，但补偿量相对线性区减少
- **相关模块**：[ALG-11 死区补偿]()

### 6.3 功率器件约束

六阶梯波模式功率器件承受的电流应力最大，需校核：

| 约束项 | 六阶梯波 vs SVPWM 线性区 |
|--------|--------------------------|
| IGBT/MOSFET 导通损耗 | ↑ 约 10% ~ 15%（电流增大） |
| 开关损耗 | ↓（开关次数减少） |
| 结温 | 需在六阶梯波满载工况下校核 |
| di/dt 应力 | 取决于电机电感，与调制方式直接关系不大 |

### 6.4 电流采样时序约束

过调制区 PWM 占空比接近 100%，有效矢量时间极短，可能低于电流采样所需的最小窗口时间 $T_{min\_sample}$（通常 3 ~ 5 μs）：

- 当非零矢量持续时间 $< T_{min\_sample}$ 时，该相电流无法在此 PWM 周期内准确采样
- **应对**：在过调制区切换为"双电阻采样 + 推算第三相"或使用相电流观测器补偿
- **相关模块**：[HW-05 电流采样电路]()

---

## 7. 🚀 前沿拓展 ★★★★★ 💡

### 7.1 单模式过调制（Single-Mode Overmodulation）

传统双区过调制（I 区 + II 区）存在模式切换时的过渡不平滑问题。单模式过调制在整个过调制区间采用统一的参考电压修正公式，无需分区：

- 通过傅里叶级数预补偿生成参考电压，使实际基波输出与指令值线性对应
- 简化实现，适合对实时性要求高的 MCU 平台
- 代表性方法：文献中的"统一过调制算法"（Unified Overmodulation Algorithm）

### 7.2 特定谐波消除 PWM（SHE-PWM）

在过调制/六阶梯波区，SHE-PWM 通过预计算开关角来选择性消除特定次谐波（如 5 次、7 次），而非像 SVPWM 那样均匀分布谐波：

- 需要离线求解非线性方程组获得开关角，存储为查找表
- 优点：可在保持高电压利用率的同时抑制转矩脉动相关谐波
- 缺点：开关角计算依赖精确的 $V_{dc}$，母线电压波动会导致消除效果下降
- 在高压大功率牵引领域（如高铁牵引逆变器）有成熟应用

### 7.3 不连续 PWM 在过调制区的应用

DPWM（Discontinuous PWM）在过调制区有独特优势：

- **DPWM1/DPWMMIN/DPWMMAX**：每个基波周期有 120° 不开关区间，开关损耗降低 1/3
- 过调制区 DPWM 可自然过渡到六阶梯波（因为 DPWM 的本质是"每个扇区钳位 60°"）
- 结合过调制的 DPWM 称为 **ODPWM（Overmodulation DPWM）**，在电动压缩机等高速轻载应用中效果显著

### 7.4 基于模型预测的过调制

MPC（Model Predictive Control）天然支持过调制——它直接将逆变器离散电压矢量作为有限控制集，无需调制器：

- 六阶梯波对应 MPC 在每个扇区只选 6 个基本矢量（也是最长持续时间的矢量）
- FCS-MPC 在过调制区天然平滑过渡，无调制比概念
- 计算量大，需高性能处理器（FPGA 或高主频 DSP）

### 7.5 过调制与无位置传感器控制的兼容性

过调制引起的电压/电流波形畸变对无位置传感器算法（如滑模观测器、高频注入）的影响是当前研究热点：

- 电压模型法：过调制区电压误差增大，导致反电势估计偏差
- 高频注入法：注入信号可能与过调制谐波产生交叉调制
- 解决方案：过调制区切换为基于 $V_{dc}$ 和占空比的前馈电压重建，绕过电流采样误差

:::sim overmodulation