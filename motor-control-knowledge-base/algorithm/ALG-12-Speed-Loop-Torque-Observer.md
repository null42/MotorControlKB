# ALG-12 速度环与负载转矩观测器

## 1. 速度环架构

FOC 典型级联控制结构：**速度环（外环）→ 电流环（内环）**。速度环输出为 q 轴电流指令 $I_{q\_ref}$，作为电流环的给定值。

速度环带宽通常设为电流环的 1/5 ~ 1/10：

$$\omega_{speed} \leq \frac{\omega_{current}}{5} \sim \frac{\omega_{current}}{10}$$

这样设计是为了保证内环动态远快于外环，使级联分析时可将电流环近似等效为一阶惯性环节或单位增益。

## 2. 速度环 PI 参数整定

### 2.1 运动方程

电机的机械运动方程为：

$$J\frac{d\omega}{dt} = T_e - T_L - B\omega$$

其中：
- $J$：转动惯量（kg·m²）
- $T_e = K_t \cdot i_q$：电磁转矩（Nm），$K_t = \frac{3}{2} P \psi_f$ 为转矩常数
- $T_L$：负载转矩（Nm）
- $B$：粘滞摩擦系数（Nm·s/rad）

### 2.2 简化模型

忽略 $B\omega$ 项（通常远小于 $T_e$ 和 $T_L$），从 $i_q$ 到 $\omega$ 的传递函数为：

$$G(s) = \frac{K_t}{Js + B} \approx \frac{K_t}{Js}$$

这是一个纯积分环节，无自平衡能力。

### 2.3 对称最优法 (Symmetrical Optimum)

对 $G(s) = \frac{K_t}{Js}$ 这类积分型被控对象，使用**对称最优法 (SO)** 整定 PI 参数：

$$K_p = \frac{J \cdot \omega_c}{K_t}$$

$$K_i = \frac{K_p \cdot \omega_c}{a^2} = \frac{J \cdot \omega_c^2}{K_t \cdot a^2}$$

其中 $a = 2 \sim 4$（推荐 $a = 2$ 时相角裕度最大），$\omega_c$ 为期望的穿越频率。

SO 法得到的开环传递函数为：

$$G_{ol}(s) = K_t \cdot \left(K_p + \frac{K_i}{s}\right) \cdot \frac{1}{Js}$$

闭环系统对阶跃响应的超调约为 43%（$a=2$ 时），需在指令通道加**前置滤波器**减小超调。

### 2.4 抗积分饱和 (Anti-windup)

速度环在加减速或堵转时 $i_q$ 指令极易达到限幅值，积分器必须做抗饱和处理：

- **Back-calculation（反计算法）**：积分值减去饱和前后差值 $\times$ 跟踪增益 $K_b$

$$u_i[k] = u_i[k-1] + K_i T_s e[k] - K_b T_s (u_{sat}[k-1] - u_{unsat}[k-1])$$

- **Clamping（条件积分）**：仅在输出未饱和时累积积分

$$u_i[k] = \begin{cases} u_i[k-1] + K_i T_s e[k], & \text{if } u \in [-u_{max}, u_{max}] \\ u_i[k-1], & \text{otherwise} \end{cases}$$

工程上常用 **Clamping + Back-calculation 组合策略**：条件积分防止正常运行时 windup，反计算在退出饱和时快速拉回积分值。

## 3. 负载转矩观测器

负载转矩 $T_L$ 作为外部扰动直接影响速度调节品质。若能实时估计 $T_L$ 并作为**前馈补偿**，可大幅提升抗扰性能。

### 3.1 增广状态方程

将 $T_L$ 作为状态变量，假设其变化缓慢（$\dot{T}_L \approx 0$）：

$$\begin{bmatrix} \dot{\omega} \\ \dot{T}_L \end{bmatrix} = \begin{bmatrix} -\frac{B}{J} & -\frac{1}{J} \\ 0 & 0 \end{bmatrix} \begin{bmatrix} \omega \\ T_L \end{bmatrix} + \begin{bmatrix} \frac{K_t}{J} \\ 0 \end{bmatrix} i_q$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} \omega \\ T_L \end{bmatrix}$$

### 3.2 转矩前馈补偿

将估计的 $\hat{T}_L$ 折算为等效 q 轴电流前馈：

$$i_{q\_ff} = \frac{\hat{T}_L}{K_t}$$

速度环输出加上前馈：

$$i_{q\_ref} = u_{PI} + i_{q\_ff}$$

## 4. Luenberger 全阶转矩观测器

完整的 Luenberger 观测器形式：

$$\begin{bmatrix} \dot{\hat{\omega}} \\ \dot{\hat{T}}_L \end{bmatrix} = \begin{bmatrix} -\frac{B}{J} & -\frac{1}{J} \\ 0 & 0 \end{bmatrix} \begin{bmatrix} \hat{\omega} \\ \hat{T}_L \end{bmatrix} + \begin{bmatrix} \frac{K_t}{J} \\ 0 \end{bmatrix} i_q + \begin{bmatrix} l_1 \\ l_2 \end{bmatrix} (\omega - \hat{\omega})$$

其中 $L = [l_1, l_2]^T$ 为观测器增益矩阵。

### 4.1 极点配置

观测器极点 $\lambda_{1,2}$ 配置在速度环闭环极点的 $k_{obs}$ 倍处（$k_{obs} = 5 \sim 10$）：

$$\lambda_{1,2} = -k_{obs} \cdot \omega_{speed}$$

由特征方程 $\det(sI - (A - LC)) = 0$ 可解得：

$$l_1 = (k_{obs1} + k_{obs2}) \cdot \omega_{speed} - \frac{B}{J}$$

$$l_2 = k_{obs1} \cdot k_{obs2} \cdot \omega_{speed}^2 \cdot J$$

工程上常取重极点 $\lambda_1 = \lambda_2 = -k_{obs}\omega_{speed}$，简化增益计算：

$$l_1 = 2k_{obs}\omega_{speed} - \frac{B}{J}, \quad l_2 = J \cdot (k_{obs}\omega_{speed})^2$$

## 5. 基于 Gopinath 方法的降阶观测器

将 $T_L$ 建模为缓慢变化的扰动，$\dot{T}_L \approx 0 + w$（$w$ 为过程噪声）。Gopinath 降阶观测器仅估计不可测状态 $T_L$：

$$\dot{z} = -\frac{l}{J} z + \frac{l}{J}(K_t i_q - l\omega)$$

$$\hat{T}_L = z + l \cdot \omega$$

其中 $l$ 为标量观测器增益，$l < 0$ 且 $|l|$ 越大收敛越快（但噪声放大也越严重）。

**优点**：单增益、实现极简，适合入式资源受限场景。

### 5.1 离散化实现

在 DSP/MCU 中的迭代公式：

```c
float gopinath_update(float omega_meas, float iq_ref, float Kt, float J, float l, float Ts) {
    static float z = 0.0f;
    float omega_est = (z + l * omega_meas);
    float dz = -(l / J) * z + (l / J) * (Kt * iq_ref - l * omega_meas);
    z += dz * Ts;
    return (z + l * omega_meas);  // TL_hat
}
```

观测器极点 $\lambda = l/J$，给定期望时间常数 $\tau_{obs}$，则 $l = -J / \tau_{obs}$（负增益保证稳定性）。

## 6. 惯量辨识

准确的 $J$ 是速度环 PI 整定和转矩观测器的基础。工程上常用的两种辨识方法：

### 6.1 加减速法

在无负载条件下，施加恒定转矩 $T_{acc}$，测量角加速度 $\alpha$：

$$J = \frac{T_e}{\alpha} = \frac{K_t \cdot i_q}{\Delta\omega / \Delta t}$$

**步骤**：
1. 速度开环，施加恒定的 $i_q = i_{acc}$
2. 记录 $\omega$ 从 $\omega_1$ 到 $\omega_2$ 的时间 $\Delta t$
3. 计算 $\alpha = (\omega_2 - \omega_1) / \Delta t$
4. 计算 $J = K_t \cdot i_{acc} / \alpha$

### 6.2 递推最小二乘法 (RLS)

在线辨识 $J$，适应变负载场景。将运动方程离散化：

$$\omega[k] = \omega[k-1] + \frac{T_s}{J}\left(K_t i_q[k-1] - T_L[k-1]\right)$$

RLS 迭代公式：

$$\hat{\theta}[k] = \hat{\theta}[k-1] + K[k]\left(y[k] - \varphi^T[k]\hat{\theta}[k-1]\right)$$

$$K[k] = \frac{P[k-1]\varphi[k]}{\lambda + \varphi^T[k]P[k-1]\varphi[k]}$$

$$P[k] = \frac{1}{\lambda}\left(P[k-1] - K[k]\varphi^T[k]P[k-1]\right)$$

其中 $\hat{\theta} = 1/\hat{J}$，$\varphi = T_s \cdot K_t i_q[k-1]$，$\lambda$ 为遗忘因子（0.95~0.99）。

## 7. 速度环 + TL 前馈的性能提升

| 对比项 | 无前馈 | 有负载转矩前馈 |
|--------|--------|---------------|
| 突加负载速度跌落 | 大（仅靠PI积分响应） | 减小 60~80% |
| 恢复时间 | 慢（ms~十ms级） | 快 3~5 倍 |
| 稳态误差 | 有（积分缓慢消除） | 几乎为零 |
| 电流环负担 | 大 | 小（前馈分担） |
| 对惯量辨识误差敏感度 | 不敏感 | 较敏感（$J$ 误差 → 前馈误差） |

**前馈补偿的本质**：将负载扰动从反馈控制回路中"剥离"出来，由前馈通道直接抵消，反馈通道（PI）只需处理模型误差和高频扰动。

## 8. 工程实现要点

1. **观测器启动**：初始阶段 $\hat{T}_L$ 不准确，前馈应逐步切入（增益从 0 渐变到 1）
2. **低通滤波**：$\hat{T}_L$ 估计值需经适当的低通滤波，去除高频噪声
3. **限幅保护**：$i_{q\_ff}$ 必须有上限，防止异常估计值导致过流
4. **摩擦补偿**：低速段摩擦非线性显著，可额外加入 Stribeck 摩擦模型补偿
5. **参数鲁棒性**：$J$ 辨识误差 30% 以内，前馈仍能提供 50% 以上的扰动抑制效果

> **仿真体验**：调整惯量、PI参数、负载阶跃，观察速度响应波形及转矩观测器的跟踪精度，对比有/无前馈的性能差异。

:::sim speed-loop

### 🔗 hpm_MC 代码实现参考

**路径规划中的前馈力矩** (`hpm_mcl_v2/core/control/hpm_mcl_path_plan.h`):
- 梯形速度曲线输出的加速度可转化为前馈力矩：τ_ff = J × acc + B × speed
- 混合控制中：`τ = τ_ff + kp*(q_des-q) + kd*(dq_des-dq)` 前馈+反馈联合

**混合控制中的力矩限幅** (`hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h`):
- `torque_limit` 防止过大力矩指令
- `deadzone` 防止微小误差引起高频抖动

**参考**:
- `SDK-05-HPM-MC-v2-Path-Plan.md` — 路径规划
- `SDK-04-HPM-MC-v2-Hybrid-Ctrl.md` — 混合控制