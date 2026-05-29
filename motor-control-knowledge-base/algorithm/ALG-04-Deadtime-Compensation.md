# ALG-04 死区补偿策略

**模块编号：** ALG-04
**模块名称：** 死区补偿策略（Deadtime Compensation）
**文档版本：** v2.0
**适用对象：** 电机控制工程师、功率电子工程师
**前置知识：** 三相逆变器拓扑（六开关桥臂）、PWM 调制原理、续流二极管工作机制、SVPWM 空间矢量调制

---

## 1. 📌 核心摘要 ★★★★☆ 🔰

**一句话总结：** 死区补偿是避免逆变器上下桥臂直通短路的前提下，消除输出电压畸变和电流过零钳位效应的关键技术，补偿精度直接影响低速转矩平稳性和噪音水平。

**认知钩子：** 死区就像电梯门的安全间隔——门不能同时开合（防止短路），但每次开关之间的"等待时间"会让电梯的实际运行时间偏离预期。如果每次等待都少跑一点距离（电压损失），累积下来电梯就会在楼层间错位（电流过零钳位）。更糟糕的是，等待时间的方向取决于你"进还是出"（电流方向），这让补偿变得微妙且容易出错。

```
逆变器单桥臂:
         ┌─────┬─────┐
  Vdc ───┤ Q1  │ Q2  ├─── GND
         └──┬──┴──┬──┘
  PWM_H ───┘     └─── PWM_L
        ←  T_dead →
  Q1:  ──┐        ┌──
         └────────┘
  Q2:  ──────┐  ┌────
              └──┘
  死区期间: 两管均关断, 续流二极管导通
```

| 关键概念 | 说明 |
|---------|------|
| 死区时间 $T_{dead}$ | 上下管切换间强制插入的安全间隔（典型 1~3 μs） |
| 电压误差 $\Delta V$ | $V_{dc} \cdot T_{dead} / T_{PWM}$，极性取决于电流方向 |
| 过零钳位 | 电流过零时误差极性反转，导致电流"卡"在零点附近 |
| 补偿本质 | 根据电流极性，在指令电压上叠加符号正确的 $\Delta V$ |

---

## 2. 📐 原理推导 ★★★★☆ 📚

### 2.1 死区效应

为防止逆变器上下桥臂直通短路（shoot-through），上下管切换间必须插入死区时间 $T_{dead}$（典型值：1~3μs）：

- **上管关断** → 等待 $T_{dead}$ → **下管开通**
- 在这段死区时间内，两管均关断，续流二极管导通

死区时间的物理来源是功率器件（IGBT/MOSFET）的关断延迟——关断过程远慢于开通过程。若不放"延迟开通"，就会出现瞬间直通短路，电流急剧上升导致器件损坏。

### 2.2 死区引起的电压误差

死区导致逆变器输出电压偏离指令值：

$$\Delta V = V_{dc} \cdot \frac{T_{dead}}{T_{PWM}}$$

误差的**极性取决于相电流方向**：

- $i > 0$（流出逆变器）→ 实际电压偏低 $- \Delta V$
- $i < 0$（流入逆变器）→ 实际电压偏高 $+ \Delta V$
- $i \approx 0$ → 电流方向不明确，电压误差为非线性过渡

**典型数值**：$V_{dc}=300V$, $T_{dead}=2\mu s$, $f_{PWM}=10kHz \rightarrow \Delta V = 300 \times 2/100 = 6V$（=2% Vdc，低速时严重）

低速时输出电压仅几伏到十几伏，$6V$ 的误差占比极大，是低速性能劣化的首要因素。

### 2.3 电流过零钳位效应

死区误差在电流过零时极性反转，导致**电流波形在过零点被"钳位"成六拍阶梯状**：

- 原因：电流过零时控制器试图增加电压 → 死区吃掉这部分电压 → 电流无法上升 → 平顶
- **后果**：转矩脉动增大、低速抖振、噪音增加

过零钳位的本质是一个"死区非线性区"——当 $\lvert i\rvert$ 很小时，$\text{sign}(i)$ 的取值因噪声和采样误差而不确定，导致补偿量在 $+\Delta V$ 和 $-\Delta V$ 之间反复跳变，加剧了电流畸变。

---

## 3. 🔢 数学建模 ★★★★☆ 📚

### 3.1 平均电压误差模型

单相桥臂在一个 PWM 周期内的平均电压误差：

$$\Delta V_{phase} = -\text{sign}(i_{phase}) \cdot V_{dc} \cdot \frac{T_{dead}}{T_{PWM}}$$

其中：
- $\Delta V_{phase}$ —— 单相平均电压误差（V）
- $i_{phase}$ —— 该相瞬时电流（A），符号定义：正 = 流出逆变器
- $V_{dc}$ —— 直流母线电压（V）
- $T_{dead}$ —— 死区时间（s）
- $T_{PWM}$ —— PWM 周期（s）

### 3.2 三相电压误差矢量

将三相误差变换至 $\alpha\beta$ 静止坐标系（等幅值 Clarke 变换）：

$$\begin{bmatrix} \Delta V_\alpha \\\\ \Delta V_\beta \end{bmatrix} = \frac{2}{3} \begin{bmatrix} 1 & -\frac{1}{2} & -\frac{1}{2} \\\\ 0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \end{bmatrix} \begin{bmatrix} -\text{sign}(i_a) \\\\ -\text{sign}(i_b) \\\\ -\text{sign}(i_c) \end{bmatrix} \cdot \Delta V$$

误差矢量的大小和方向随电流极性扇区变化，形成六边形分布的扰动矢量。

### 3.3 观测器扰动估计模型

将死区误差视为未知扰动 $d$，通过一阶扰动观测器在线估计：

$$\frac{d\hat{d}}{dt} = \gamma \cdot (i - \hat{i})$$

其中：
- $\hat{d}$ —— 估计扰动（V），包含死区误差及模型不匹配项
- $\gamma$ —— 观测器增益（$\Omega$/H·s），决定估计收敛速度
- $i$ —— 实测电流（A）
- $\hat{i}$ —— 模型预测电流（A）

观测器模型：$\hat{i}$ 由标称电机模型 $\frac{d\hat{i}}{dt} = \frac{1}{L}(v - R\hat{i} + \hat{d})$ 计算。

---

## 4. 💻 代码实现 ★★★☆☆ 🔧

### 4.1 补偿方法一：平均电压前馈

最常用的方法——根据电流极性前馈补偿电压：

$$v_{comp} = \text{sign}(i) \cdot \Delta V$$

```c
#define DEADTIME_NS       2000
#define PWM_PERIOD_NS     100000
#define VDC               300.0f

float deadtime_comp_voltage(float i_phase)
{
    float delta_v = VDC * (float)DEADTIME_NS / (float)PWM_PERIOD_NS;
    float threshold = 0.05f;

    if (i_phase > threshold)
        return delta_v;
    else if (i_phase < -threshold)
        return -delta_v;
    else
        return 0.0f;
}
```

- 实现简单，仅需判断电流极性
- **关键难点**：电流过零点附近极性判断容易误判（需加滞环或滤波）

### 4.2 补偿方法二：PWM 占空比修正

直接修改每相 PWM 比较值：

$$T_{cmp}' = T_{cmp} \pm T_{dead}$$

```c
void pwm_duty_deadtime_comp(float *duty_a, float *duty_b, float *duty_c,
                             float ia, float ib, float ic)
{
    float t_dead_ticks = DEADTIME_NS * PWM_TIMER_CLK_HZ / 1e9f;
    int sign_a = (ia > 0.05f) ? 1 : ((ia < -0.05f) ? -1 : 0);
    int sign_b = (ib > 0.05f) ? 1 : ((ib < -0.05f) ? -1 : 0);
    int sign_c = (ic > 0.05f) ? 1 : ((ic < -0.05f) ? -1 : 0);

    *duty_a += sign_a * t_dead_ticks;
    *duty_b += sign_b * t_dead_ticks;
    *duty_c += sign_c * t_dead_ticks;

    clamp_duty(duty_a); clamp_duty(duty_b); clamp_duty(duty_c);
}
```

- 优点：不修改控制算法，仅在 PWM 生成层修正
- 缺点：需要知道确切电流方向

### 4.3 补偿方法三：观测器扰动估计

将死区误差视为未知扰动，通过扰动观测器在线估计：

```c
typedef struct {
    float gamma;
    float R;
    float L_inv;
    float Ts;
    float d_hat;
    float i_hat;
} disturbance_observer_t;

float dob_update(disturbance_observer_t *obs, float v_cmd, float i_meas)
{
    obs->i_hat += obs->Ts * obs->L_inv *
                  (v_cmd - obs->R * obs->i_hat + obs->d_hat);
    obs->d_hat += obs->Ts * obs->gamma * (i_meas - obs->i_hat);
    return obs->d_hat;
}
```

其中 $\hat{d}$ 为估计扰动（包含死区误差），$\gamma$ 为观测器增益。

- 优点：平滑、无需精确电流极性、可补偿其他扰动
- 缺点：实现复杂，需整定观测器增益

---

## 5. 🔧 参数整定 ★★★★☆ 🔧

### 5.1 四种策略对比

| 方法 | 实现难度 | 过零区表现 | 转矩脉动改善 |
|------|---------|-----------|-------------|
| 无补偿 | — | 六拍畸变 | 基准 |
| 平均电压前馈 | ★ | 可能过补偿 | ~70% |
| PWM 占空比修正 | ★★ | 滞环边界抖动 | ~80% |
| 观测器估计 | ★★★★ | 平滑过渡 | ~95% |

### 5.2 工程实践要点

- **电流极性检测**：使用指令值（$i_d^*, i_q^*$）比实测值（$i_d, i_q$）更可靠（噪声小）。指令值经低通滤波后极性稳定，且不受 ADC 采样噪声影响
- **最小脉冲限制**：补偿后需确保 PWM 占空比 > 死区占比（否则该相完全不输出），即 $duty_{min} > T_{dead} / T_{PWM}$
- **自适应死区**：谐振型栅极驱动可动态调整死区，平衡安全和效率
- **过零滞环**：设置电流滞环区间 $\pm i_{hyst}$（典型值 2~5% 额定电流），在滞环内保持补偿量不变，避免极性频繁翻转

### 5.3 典型参数参考值

| 参数 | 典型值 | 说明 |
|------|--------|------|
| $T_{dead}$ | 1~3 μs | 取决于功率器件开关速度 |
| $\Delta V$ 占比 | 1~3% $V_{dc}$ | 10kHz PWM, $2\mu s$ 死区 |
| 电流滞环阈值 | 2~5% 额定电流 | 防止过零附近反复跳变 |
| 观测器增益 $\gamma$ | 500~2000 | 根据电流环带宽整定 |
| 最小占空比限制 | > $T_{dead}/T_{PWM}$ + 5% | 保证有效电压输出 |

---

## 6. 🔗 硬件约束 ★★★★☆ ⚠️

### 6.1 功率器件开关速度与死区关系

死区时间的下限由功率器件的关断特性决定：

$$T_{dead,min} = t_{d(off),max} - t_{d(on),min} + t_{f,max} + t_{margin}$$

其中：
- $t_{d(off),max}$ —— 最大关断延迟时间（s），IGBT 典型 $0.5 \sim 2\mu s$
- $t_{d(on),min}$ —— 最小开通延迟时间（s），IGBT 典型 $0.1 \sim 0.5\mu s$
- $t_{f,max}$ —— 最大下降时间（s），$0.2 \sim 1\mu s$
- $t_{margin}$ —— 安全裕量（s），$0.2 \sim 0.5\mu s$

IGBT 的拖尾电流（tail current）使得关断过程远慢于开通，因此死区主要由关断特性决定。MOSFET 由于无拖尾电流，死区可大幅缩短。

> 相关硬件模块：[HW-02 功率逆变器设计](file:///e:/gitee_CodeStorage/学习/MotorControl-main/motor-control-knowledge-base/hardware/HW-02-Power-Inverter.md)

### 6.2 栅极驱动 IC 死区设置

栅极驱动 IC（如 IR2110、UCC21520）通常内置可编程死区或固定死区：

- **固定死区型**：IC 内部固化了死区时间（如 $500ns$），不可通过软件调整。补偿时需以 IC 实际死区为准
- **可编程死区型**：通过外部电阻或 MCU 配置死区时间。注意 IC 手册中死区精度（通常 ±10~15%），此误差将直接引入补偿误差
- **互锁型驱动**：硬件直接互锁上下管 PWM，不允许两管同时导通。补偿算法需考虑硬件互锁的额外延迟

### 6.3 温度对死区的影响

功率器件的开关速度随温度变化，间接影响有效死区时间：

- **IGBT**：结温 $T_j$ 从 25°C 升至 125°C 时，$t_{d(off)}$ 增加约 20~50%（拖尾电流延长），等效死区增大
- **MOSFET**：温度影响较小（$R_{DS(on)}$ 变化为主），$t_{d(off)}$ 变化 ≤ 10%
- **工程实践**：在高温工况下实际有效死区比冷态更大，若补偿量未随温度修正，将导致低速性能恶化。高端驱动器（如车规级）需监控结温并查表修正 $T_{dead}$ 的等效值

---

## 7. 🚀 前沿拓展 ★★★★★ 💡

### 7.1 自适应死区调节

根据相电流幅值动态调整死区时间——大电流时功率器件开关速度快（栅极驱动能力强），死区可适当缩短以减小电压误差；小电流时恢复标称死区保证安全性：

$$T_{dead}(\lvert i\rvert) = T_{dead,nom} - k_{adapt} \cdot \lvert i\rvert$$

约束条件：$T_{dead}(\lvert i\rvert) \geq T_{dead,min}$（由硬件关断特性决定的下限）。

这要求栅极驱动 IC 支持动态死区配置（通常通过 SPI 或 I²C 配置），典型的如 TI UCC5870-Q1 智能栅极驱动器。

### 7.2 基于在线参数辨识的补偿

死区误差 $\Delta V$ 的精确值受 $V_{dc}$ 波动、温度漂移等因素影响。通过在线参数辨识（如 RLS 递归最小二乘）实时估计等效死区误差：

$$\hat{\theta} = [\hat{R}_s, \hat{\Delta V}]^T$$

在不增加额外传感器的前提下，利用电流环稳态误差反推死区误差，实现无模型的自适应补偿。该方法在部分工业伺服产品中已有落地。

### 7.3 SiC/GaN 宽禁带器件的低死区优势

SiC MOSFET 和 GaN HEMT 的开关速度比传统 Si IGBT 快 5~10 倍：

- **SiC MOSFET**：$t_{d(off)}$ 约 $20 \sim 50ns$，死区可缩短至 $100 \sim 300ns$
- **GaN HEMT**：$t_{d(off)}$ 约 $10 \sim 30ns$，死区可缩短至 $50 \sim 100ns$

以 GaN 为例，$T_{dead}=100ns$、$f_{PWM}=20kHz$（$T_{PWM}=50\mu s$）时：
$$\Delta V = V_{dc} \cdot \frac{100ns}{50\mu s} = 0.2\% \cdot V_{dc}$$

死区误差降低一个数量级，使得死区补偿在某些场景下不再是必须项。这是 GaN/SiC 推动高频化、高功率密度化的关键技术优势之一。

### 7.4 基于状态观测器的无传感器补偿

将死区误差建模为周期性扰动（频率 = 6 倍电气频率），通过谐振观测器在多同步旋转坐标系中估计并补偿特定谐波分量。该方法能同时处理死区和逆变器非线性（如管压降、寄生电容效应）的复合扰动。

---

> **仿真体验**：调节死区时间，对比未补偿→平均电压→PWM修正→观测器估计四种方法的电流波形改善效果。

:::sim deadtime-comp

### 🔗 hpm_MC 代码实现参考

**死区补偿配置** (`hpm_mcl_v2/hpm_mcl_cfg.h`):
- 编译宏 `HPM_MCL_ENABLE_DEAD_AREA_COMPENSATION` 控制死区补偿使能
- 补偿量基于电流极性自动计算
- 参考示例 `hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c` 中宏配置

**v1 死区处理** (`hpm_mcl/inc/hpm_foc.h`):
- v1 中死区补偿未抽象为独立模块，由 `hpm_foc_svpwm()` 在占空比计算时考虑

**驱动层支持** (`hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h`):
- `mcl_drivers_channel_t` 中 update_duty 回调支持相位补偿
- PWM 死区时间在 HAL 层配置，mcl 层不解耦

**与 MC_LIB 对比**:
- MC_LIB: `MCFOC_DeadTime_COMP_F()` 独立死区补偿函数
- hpm_MCL: 编译宏控制使能，补偿量集成在控制链中


## 🧪 仿真验证
> 本模块的理论可在 [C 语言仿真](../simulation/SIM-00-C-Simulation-Overview.md) 中验证。
> 对应仿真模式：MODE_SELECT_INVERTER_NONLINEARITY_SENSORLESS (49)，关键操作：在 ACMSim.h 中改 `__INVERTER_NONLINEARITY` 宏（0=理想, 1=Sul1996, 2=Sigmoid, 3=LUT），对比电流波形畸变程度