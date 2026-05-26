# ALG-15 前沿研究：离散时间非线性磁链观测器

**模块编号：** ALG-15  
**模块名称：** 前沿研究——离散时间非线性磁链观测器（Discrete-Time Nonlinear Flux Observer）  
**文档版本：** v2.0  
**适用对象：** 电机控制高级工程师、算法研究者  
**前置知识：** ALG-07 无感FOC观测器、数字控制理论、Z变换  
**论文来源：** Su M., Li Z., Dan H., et al., "Discrete-Time Nonlinear Flux Observer for Sensorless SPMSM Drives with Low Carrier Ratios," IEEE Trans. Power Electronics, 2026. DOI: 10.1109/TPEL.2026.3687870

---

## 1. 📌 核心摘要 ★★★★☆ 📚

**一句话：** 当PWM频率与电机基波频率之比（载波比 $f_{ratio}$）较低时，传统连续时间磁链观测器经前向欧拉法离散化后极点向单位圆边界迁移甚至溢出，导致观测器性能退化甚至失稳；本文提出基于ZOH一致离散化的离散时间非线性磁链观测器，并通过电流预测模型最小化误差函数优化增益，在载波比低至5.2时仍保持稳定。

**认知挂钩：** 如果说传统磁链观测器是"用连续时间的地图导航"，那么在低载波比下这张地图就失真了——采样点太少，连续模型离散化后"地形"变了。本文的方法是"直接画一张离散时间的地图"（ZOH离散化），并"根据路况调校导航参数"（增益优化），确保在"稀疏路网"（低载波比）下也能精确导航。

**核心问题链：**

```
高速/大功率 → PWM频率受限 → 载波比fratio低 → 前向欧拉离散化失真 → 极点迁移→失稳
                                                                    ↓
                                              ZOH一致离散化 + 增益优化 → 全速域稳定
```

**载波比定义：**

$$
f_{ratio} = \frac{f_{PWM}}{f_e}
$$

| 应用场景 | 典型 $f_e$ | 典型 $f_{PWM}$ | $f_{ratio}$ | 问题严重程度 |
|---------|-----------|---------------|-------------|------------|
| 低压伺服 | 100Hz | 20kHz | 200 | 无问题 |
| 高速电机 | 1kHz | 10kHz | 10 | 严重 |
| 大功率驱动 | 50Hz | 500Hz | 10 | 严重 |
| 超高速电机 | 2kHz | 10kHz | 5 | 极严重 |

---

## 2. 📐 原理推导 ★★★★★ 📚

### 2.1 低载波比问题分析

#### 2.1.1 问题起源

传统无感控制方法在连续时间域设计观测器，然后用前向欧拉法离散化实现。当 $f_{ratio}$ 较低时：

1. **ZOH延迟被忽略：** PWM逆变器输出电压在采样周期内保持恒定（零阶保持），前向欧拉法未考虑这一特性
2. **计算延迟被忽略：** ADC采样到PWM更新之间存在1~2个采样周期的延迟
3. **低阶近似引入误差：** 前向欧拉法 $\frac{dx}{dt} \approx \frac{x(k+1)-x(k)}{T}$ 在 $T$ 较大时精度不足

#### 2.1.2 极点迁移机制

前向欧拉离散化将连续时间极点 $s = \sigma + j\omega$ 映射为：

$$
z = 1 + sT = 1 + \sigma T + j\omega T
$$

当 $T\omega_e$ 增大（即 $f_{ratio}$ 降低）时：
- 离散极点向单位圆边界移动
- 稳定裕度减小
- 当 $T\omega_e$ 超过临界值时，极点逸出单位圆 → 观测器失稳

**关键参数：** $T\omega_e$（采样周期×电角速度），$f_{ratio} = \frac{2\pi}{T\omega_e}$

#### 2.1.3 前向欧拉法的局限性

前向欧拉法是条件稳定的显式方法：

$$
\hat{\psi}_{\alpha\beta}(k+1) = \hat{\psi}_{\alpha\beta}(k) + T \cdot f_{\alpha\beta}(k)
$$

当 $T$ 足够大时，即使连续时间观测器稳定，离散化实现也可能失去渐近稳定性。这不是连续时间观测器设计的缺陷，而是离散化方法的固有限制。

### 2.2 SPMSM连续时间模型

#### 2.2.1 αβ坐标系电压方程

$$
\frac{d\psi_{s,\alpha\beta}}{dt} = u_{\alpha\beta} - R_s i_{\alpha\beta}
$$

定子磁链和转子磁链复矢量：

$$
\psi_{s,\alpha\beta} = L_s i_{\alpha\beta} + \psi_m e^{j\theta_e}
$$

$$
\psi_{r,\alpha\beta} = \psi_m e^{j\theta_e}
$$

### 2.3 传统非线性磁链观测器

#### 2.3.1 梯度搜索法

利用转子磁链幅值恒定 $\|\psi_{r,\alpha\beta}\|^2 = \psi_m^2$ 作为约束，通过梯度搜索同时估计磁链幅值和位置：

$$
\frac{d\hat{\psi}_{s,\alpha\beta}}{dt} = u_{\alpha\beta} - R_s i_{\alpha\beta} + \left[\lambda + \frac{jq}{2}\right]\left(\hat{\psi}_{r,\alpha\beta} - \psi_m \frac{\hat{\psi}_{r,\alpha\beta}}{\|\hat{\psi}_{r,\alpha\beta}\|}\right)
$$

其中：
- $\lambda$：幅值梯度搜索增益
- $q$：位置梯度搜索增益
- $q = 0$ 退化为传统非线性磁链观测器

#### 2.3.2 位置计算

$$
\hat{\theta}_e = -\arctan\left(\frac{\hat{\psi}_{s\beta} - L_s i_\beta}{\hat{\psi}_{s\alpha} - L_s i_\alpha}\right)
$$

### 2.4 与ALG-3的衔接

**传统连续时间磁链观测器**（ALG-07中的磁链观测器）的局限性：

| 特性 | 传统磁链观测器（ALG-07） | 离散时间磁链观测器（ALG-15） |
|------|------------------------|--------------------------|
| 设计域 | 连续时间设计+离散化 | 直接离散时间域设计 |
| 离散化方法 | 前向欧拉 | ZOH一致离散化 |
| 低载波比稳定性 | 差（极点迁移） | 好（极点位置可控） |
| 电感交叉耦合 | 忽略 | 考虑 |
| 增益设计 | 经验整定 | 误差函数最小化优化 |
| 适用载波比 | >20 | 低至5.2 |

**演进路径：** ALG-07磁链观测器 → 增加位置梯度项($q$) → ZOH离散化 → 增益优化 → ALG-15离散时间观测器

---

## 3. 🔢 数学建模 ★★★★★ 📚

### 3.1 ZOH一致离散时间磁链模型

#### 3.1.1 连续时间模型重写

将SPMSM电压方程重写为适合ZOH离散化的形式：

$$
\frac{d\psi_{s,\alpha\beta}}{dt} + \left(\frac{R_s}{L_s} + j\omega_e\right)\psi_{s,\alpha\beta} = u_{\alpha\beta} + \frac{R_s}{L_s}\psi_m e^{j\theta_e}
$$

#### 3.1.2 ZOH离散化求解

假设电角速度在 $[kT, (k+1)T)$ 内恒定，$\theta_e(t) = \theta_e(k) + \omega_e(t-kT)$，对上式求解：

$$
\psi_{s,\alpha\beta}(k+1) = e^{-\left(\frac{R_s}{L_s}+j\omega_e\right)T}\psi_{s,\alpha\beta}(k) + \int_0^T e^{-\left(\frac{R_s}{L_s}+j\omega_e\right)\tau}d\tau \cdot u_{\alpha\beta}(k) + \text{磁链耦合项}
$$

**关键：** ZOH离散化精确考虑了PWM电压在采样周期内的保持特性，而非简单的前向欧拉近似。

#### 3.1.3 复矢量离散模型

定义离散时间状态转移矩阵元素：

$$
\Phi = e^{-\left(\frac{R_s}{L_s}+j\omega_e\right)T}
$$

$$
\Gamma = \int_0^T e^{-\left(\frac{R_s}{L_s}+j\omega_e\right)\tau}d\tau = \frac{1-e^{-\left(\frac{R_s}{L_s}+j\omega_e\right)T}}{\frac{R_s}{L_s}+j\omega_e}
$$

离散时间SPMSM磁链模型：

$$
\psi_{s,\alpha\beta}(k+1) = \Phi \cdot \psi_{s,\alpha\beta}(k) + \Gamma \cdot u_{\alpha\beta}(k) + \Lambda \cdot \psi_m e^{j\theta_e(k)}
$$

其中 $\Lambda$ 为磁链交叉耦合项，由延迟导致的d-q轴电感交叉耦合产生。

#### 3.1.4 电感交叉耦合

在低载波比下，采样延迟导致d轴和q轴之间产生交叉耦合：

$$
\Lambda = \Phi \cdot L_s \cdot (e^{j\omega_e T} - 1) \cdot i_{\alpha\beta}(k)
$$

**物理意义：** 在一个采样周期内，转子旋转了 $\omega_e T$ 角度，导致d-q轴坐标系发生旋转，原本解耦的d轴和q轴产生耦合。载波比越低，$\omega_e T$ 越大，耦合越强。

### 3.2 离散时间非线性磁链观测器

#### 3.2.1 观测器方程

基于ZOH离散模型，直接在离散时间域设计观测器：

$$
\hat{\psi}_{s,\alpha\beta}(k+1) = \Phi \cdot \hat{\psi}_{s,\alpha\beta}(k) + \Gamma \cdot u_{\alpha\beta}(k) + \hat{\Lambda}(k) + G_{\alpha\beta}(k)
$$

其中 $G_{\alpha\beta}(k)$ 为非线性校正项：

$$
G_{\alpha\beta}(k) = \left[\lambda + \frac{jq}{2}\right]\left(\hat{\psi}_{r,\alpha\beta}(k) - \hat{\psi}_m(k) \frac{\hat{\psi}_{r,\alpha\beta}(k)}{\|\hat{\psi}_{r,\alpha\beta}(k)\|}\right)
$$

#### 3.2.2 小信号稳定性分析

在dq同步坐标系下建立误差动态方程，线性化后得到小信号模型：

$$
\begin{bmatrix} \tilde{\psi}_{sd}(k+1) \\ \tilde{\psi}_{sq}(k+1) \end{bmatrix} = \mathbf{H} \begin{bmatrix} \tilde{\psi}_{sd}(k) \\ \tilde{\psi}_{sq}(k) \end{bmatrix}
$$

其中矩阵 $\mathbf{H}$ 的元素 $h_{11}, h_{12}, h_{21}, h_{22}$ 取决于三个关键量：
- 观测器增益 $q$ 和 $\lambda$
- $T\omega_e$（采样周期×电角速度）

**稳定性判据：** 矩阵 $\mathbf{H}$ 的特征值（离散极点）必须位于单位圆内。

#### 3.2.3 极点迁移特性

随着 $T\omega_e$ 增大：
- 前向欧拉法：极点快速向单位圆边界移动，最终逸出 → 失稳
- ZOH离散化：极点迁移幅度显著减小，在更大 $T\omega_e$ 范围内保持稳定

### 3.3 增益优化设计

#### 3.3.1 电流预测模型

从磁链估计值预测电流：

$$
\hat{i}_{\alpha\beta}^{pred}(k+1) = \frac{\hat{\psi}_{s,\alpha\beta}(k+1) - \hat{\psi}_m e^{j\hat{\theta}_e(k+1)}}{L_s}
$$

#### 3.3.2 误差函数定义

定义电流预测误差：

$$
e_i(k) = i_{\alpha\beta}(k) - \hat{i}_{\alpha\beta}^{pred}(k)
$$

误差函数（磁链幅值误差的度量）：

$$
J(q, \lambda) = \sum_k \left\|e_i(k)\right\|^2
$$

#### 3.3.3 增益优化求解

最小化误差函数 $J(q, \lambda)$，对 $q$ 和 $\lambda$ 求偏导并令其为零：

$$
\frac{\partial J}{\partial q} = 0, \quad \frac{\partial J}{\partial \lambda} = 0
$$

**优化结果：** 增益 $q$ 和 $\lambda$ 是运行状态（$T\omega_e$、电流幅值）的函数，而非固定常数。

**增益随速度变化规律：**
- 低速（$T\omega_e$ 小）：增益可取较大值，收敛快
- 高速（$T\omega_e$ 大）：增益需适当减小，保证稳定性

#### 3.3.4 全速域稳定性保证

通过增益优化，使得在所有运行点：
1. 矩阵 $\mathbf{H}$ 的谱半径 $\rho(\mathbf{H}) < 1$
2. 极点远离单位圆边界，有足够的稳定裕度
3. 动态性能（收敛速度、阻尼）满足要求

---

## 4. 💻 代码实现 ★★★★☆ 🔧

### 4.1 ZOH离散化参数计算

```c
typedef struct {
    float Phi_real, Phi_imag;
    float Gamma_real, Gamma_imag;
    float Lambda_real, Lambda_imag;
    float Rs_over_Ls;
    float omega_e;
    float Ts;
} dt_flux_model_t;

void dt_flux_model_update(dt_flux_model_t *m, float omega_e)
{
    float alpha = m->Rs_over_Ls;
    float beta = omega_e;
    float Ts = m->Ts;
    
    float exp_real = expf(-alpha * Ts) * cosf(beta * Ts);
    float exp_imag = expf(-alpha * Ts) * sinf(-beta * Ts);
    
    m->Phi_real = exp_real;
    m->Phi_imag = exp_imag;
    
    float denom = alpha * alpha + beta * beta;
    float coeff = (1.0f - exp_real) * alpha - exp_imag * beta;
    float coeff_imag = (1.0f - exp_real) * beta + exp_imag * alpha;
    
    m->Gamma_real = coeff / denom;
    m->Gamma_imag = coeff_imag / denom;
    
    m->omega_e = omega_e;
}
```

### 4.2 离散时间非线性磁链观测器

```c
typedef struct {
    float psi_alpha, psi_beta;
    float theta_est;
    float psi_m_est;
    float speed_est;
    float gain_lambda;
    float gain_q;
    dt_flux_model_t model;
} dt_flux_obs_t;

void dt_flux_obs_update(dt_flux_obs_t *obs, float i_alpha, float i_beta,
                         float u_alpha, float u_beta)
{
    dt_flux_model_t *m = &obs->model;
    
    float psi_r_alpha = obs->psi_alpha - Ls * i_alpha;
    float psi_r_beta = obs->psi_beta - Ls * i_beta;
    
    float psi_r_mag = sqrtf(psi_r_alpha * psi_r_alpha + psi_r_beta * psi_r_beta);
    
    float err_alpha = psi_r_alpha - obs->psi_m_est * psi_r_alpha / psi_r_mag;
    float err_beta = psi_r_beta - obs->psi_m_est * psi_r_beta / psi_r_mag;
    
    float G_alpha = obs->gain_lambda * err_alpha - obs->gain_q * err_beta / 2.0f;
    float G_beta = obs->gain_lambda * err_beta + obs->gain_q * err_alpha / 2.0f;
    
    float psi_alpha_new = m->Phi_real * obs->psi_alpha - m->Phi_imag * obs->psi_beta
                        + m->Gamma_real * u_alpha - m->Gamma_imag * u_beta
                        + G_alpha;
    float psi_beta_new = m->Phi_imag * obs->psi_alpha + m->Phi_real * obs->psi_beta
                       + m->Gamma_imag * u_alpha + m->Gamma_real * u_beta
                       + G_beta;
    
    obs->psi_alpha = psi_alpha_new;
    obs->psi_beta = psi_beta_new;
    
    obs->psi_m_est = psi_r_mag;
    
    obs->theta_est = atan2f(psi_r_beta, psi_r_alpha);
}
```

### 4.3 增益优化计算

```c
void dt_flux_obs_gain_optimize(dt_flux_obs_t *obs, float Two_e)
{
    float rho_target = 0.85f;
    
    if (Two_e < 0.1f)
    {
        obs->gain_lambda = 0.2f;
        obs->gain_q = 2.0f;
    }
    else if (Two_e < 0.5f)
    {
        obs->gain_lambda = 0.15f;
        obs->gain_q = 1.5f;
    }
    else
    {
        obs->gain_lambda = 0.1f / (1.0f + Two_e);
        obs->gain_q = 1.0f / (1.0f + Two_e);
    }
}
```

### 4.4 速度估算（PLL）

```c
void dt_flux_obs_speed_est(dt_flux_obs_t *obs, float dir)
{
    static float theta_last = 0.0f;
    static float speed_lpf = 0.0f;
    
    float dtheta = obs->theta_est - theta_last;
    if (dtheta > M_PI) dtheta -= 2.0f * M_PI;
    if (dtheta < -M_PI) dtheta += 2.0f * M_PI;
    
    float speed_raw = dtheta / obs->model.Ts;
    speed_lpf = 0.95f * speed_lpf + 0.05f * speed_raw;
    
    obs->speed_est = dir * speed_lpf;
    theta_last = obs->theta_est;
}
```

### 4.5 参数选择流程

```
1. 确定系统参数：Rs, Ls, ψm, T(采样周期)
2. 确定运行范围：ωe_min ~ ωe_max
3. 计算载波比范围：fratio_min = 2π/(T·ωe_max)
4. 若 fratio_min > 20：传统观测器即可
5. 若 fratio_min < 20：使用离散时间观测器
   a. 预计算Φ, Γ查找表（按ωe索引）
   b. 设计增益调度表（按Tωe索引）
   c. 验证全速域极点位置
6. 实测验证：阶跃响应、负载突变、低速稳定性
```

### 4.6 🔗 hpm_MC 代码实现参考

**v2 离散时间观测器** (`hpm_mcl_v2/core/control/hpm_mcl_control.h`):
- `mcl_control_smc_cfg_t` 中 SMC 实现基于离散时间滑模观测器理论
- 反电动势经 `hpm_mcl_filter` 低通滤波后送入 PLL
- 支持调试模式 trace 数据输出（`mcl_control_debug_info`）

**混合控制** (`hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h`):
- 力位混合控制 τ = τ_ff + kp*(q_des-q) + kd*(dq_des-dq)
- 代表学术界阻抗控制/导纳控制的工业实现
- 速度 LPF + 死区 + 力矩限幅保证工程鲁棒性

**离线参数辨识** (`hpm_mcl_v2/core/detect/`):
- Rs/Ld/Lq/Ls/Flux 自动检测
- 基于状态机的顺序注入策略
- 前沿课题：在线参数辨识 vs 离线检测

**参考**:
- `SDK-03-HPM-MC-v2-Detect.md` — 离线参数检测
- `SDK-04-HPM-MC-v2-Hybrid-Ctrl.md` — 混合控制
- `SDK-05-HPM-MC-v2-Path-Plan.md` — 路径规划

---

## 5. 🔧 参数整定 ★★★★★ 🔧

### 5.1 关键参数

| 参数 | 说明 | 影响范围 |
|------|------|---------|
| $\lambda$ | 幅值梯度增益 | 磁链幅值估计收敛速度 |
| $q$ | 位置梯度增益 | 角度估计收敛速度和精度 |
| $\Phi, \Gamma$ | ZOH离散化矩阵 | 模型精度，需按速度更新 |
| $R_s$ | 定子电阻 | 低速精度，需温度补偿 |
| $L_s$ | 定子电感 | 电流预测精度 |
| $\psi_m$ | 永磁体磁链 | 幅值约束精度 |

### 5.2 增益整定指南

**原则：** 增益是 $T\omega_e$ 的函数，需随速度自适应调整。

| 运行区域 | $T\omega_e$ | $\lambda$ | $q$ | 说明 |
|---------|-----------|----------|-----|------|
| 低速 | <0.1 | 0.2 | 2.0 | 增益可大，收敛快 |
| 中速 | 0.1~0.5 | 0.15 | 1.5 | 适中增益 |
| 高速/低载波比 | >0.5 | $\frac{0.1}{1+T\omega_e}$ | $\frac{1.0}{1+T\omega_e}$ | 增益需减小保稳 |

### 5.3 ZOH参数预计算

**离线预计算查找表：**

```c
#define OMEGA_TABLE_SIZE 100
float Phi_real_table[OMEGA_TABLE_SIZE];
float Phi_imag_table[OMEGA_TABLE_SIZE];
float Gamma_real_table[OMEGA_TABLE_SIZE];
float Gamma_imag_table[OMEGA_TABLE_SIZE];

void precompute_zoh_tables(float Rs, float Ls, float Ts, float omega_max)
{
    float alpha = Rs / Ls;
    for (int i = 0; i < OMEGA_TABLE_SIZE; i++)
    {
        float omega = omega_max * i / OMEGA_TABLE_SIZE;
        float exp_r = expf(-alpha * Ts) * cosf(omega * Ts);
        float exp_i = expf(-alpha * Ts) * sinf(-omega * Ts);
        
        Phi_real_table[i] = exp_r;
        Phi_imag_table[i] = -exp_i;
        
        float d = alpha * alpha + omega * omega;
        Gamma_real_table[i] = ((1 - exp_r) * alpha + exp_i * omega) / d;
        Gamma_imag_table[i] = ((1 - exp_r) * omega - exp_i * alpha) / d;
    }
}
```

### 5.4 常见问题与解决方案

| 问题 | 现象 | 可能原因 | 解决方案 |
|------|------|---------|---------|
| 高速角度发散 | 角度估算突然失效 | 增益过大/载波比过低 | 减小增益/提高PWM频率 |
| 低速收敛慢 | 启动后角度长时间不准确 | 增益过小 | 增大低速区增益 |
| 电流预测误差大 | 观测电流与实际电流偏差大 | 电阻参数不准 | 在线Rs辨识 |
| 极点接近单位圆 | 观测器振荡 | ZOH参数计算错误 | 检查Φ,Γ计算 |
| 磁链幅值估计偏移 | ψ_m估计持续偏离真实值 | λ增益不足 | 增大λ |

---

## 6. 🔗 硬件约束 ★★★★☆ ⚠️

### 6.1 PWM频率→载波比

🔗 **硬件约束：载波比由PWM频率和电机基波频率共同决定，PWM频率受开关损耗限制**

- **开关损耗限制：** MOSFET开关频率可达100kHz以上，但IGBT在中大功率应用中通常限制在几kHz。开关损耗 $P_{sw} \propto f_{PWM} \cdot I \cdot V$，提高PWM频率直接增加损耗和发热
- **CPU资源限制：** 控制周期 $T = 1/f_{PWM}$ 内需完成ADC采样、坐标变换、PI计算、观测器更新、SVPWM计算。20kHz下仅50μs，观测器计算时间约5~10μs
- **载波比临界值：** 实验验证本方法在 $f_{ratio} = 5.2$ 时仍有效，但低于此值需进一步验证 → [HW-05 功率器件与栅极驱动](../hardware/HW-05-Power-Devices-Gate-Drivers.md#45-开关损耗计算)

### 6.2 ADC同步→电流采样时序

🔗 **硬件约束：ZOH离散化模型假设电压和电流在同一时刻采样，实际存在延迟**

- **采样延迟：** ADC采样时刻与PWM更新时刻之间存在1~2个采样周期的延迟。延迟导致实际电压与模型假设不一致
- **双采样策略：** 在PWM周期中心点采样电流（对称采样），可减少纹波影响但增加时序复杂度
- **延迟补偿：** 需在观测器中补偿1拍延迟，将 $u_{\alpha\beta}(k)$ 替换为 $u_{\alpha\beta}(k-1)$ 或使用预测电压 → [HW-04 MCU外设与通信](../hardware/HW-04-MCU-Peripherals.md#422-双adc同步采样)

### 6.3 电流采样精度→观测器精度

🔗 **硬件约束：磁链观测器精度直接受电流采样精度影响**

- **量化误差：** 12位ADC在额定电流5%以下时，量化噪声严重影响磁链计算 $\psi = \int(u - R_s i)dt$ 中的 $R_s i$ 项
- **偏置漂移：** 运放偏置随温度漂移，导致积分漂移。ZOH离散化虽改善了极点位置，但未消除偏置影响
- **噪声放大：** 磁链观测器本质是积分器，高频噪声通过积分累积 → [HW-02 电流采样电路](../hardware/HW-02-Current-Sensing.md#441-adc关键参数)

### 6.4 电阻参数→低速精度

🔗 **硬件约束：磁链观测器低速精度高度依赖电阻参数准确性**

- **温度漂移：** 铜绕组温度系数0.393%/°C，温升80°C时Rs增大约31%。磁链积分中 $R_s \cdot i$ 项误差随时间累积
- **在线辨识需求：** 高精度应用需在线辨识Rs变化。本方法中电流预测模型可辅助Rs辨识
- **参数敏感性：** ZOH离散化相比前向欧拉降低了参数敏感性，但Rs误差仍是低速主要误差源 → [HW-01 电机本体基础](../hardware/HW-01-Motor-Basics.md#56-参数测量方法)

---

## 7. 🚀 前沿拓展 ★★★★★ 💡

### 7.1 扩展到IPMSM

本文方法针对SPMSM（$L_d = L_q$），扩展到IPMSM需要：
- 修改磁链模型：$\psi_s = L_d i_d + \psi_m + jL_q i_q$
- 考虑凸极效应对ZOH离散化的影响
- 增益优化需考虑d-q轴电感差异

### 7.2 与深度学习结合

- **神经网络增益调度：** 用神经网络学习最优增益映射 $q^*, \lambda^* = f(T\omega_e, I_s)$
- **端到端观测器：** 用物理信息神经网络（PINN）替代解析观测器，将ZOH约束嵌入网络
- **在线参数辨识：** 利用LSTM网络学习Rs、Ls的时变特性

### 7.3 多采样率控制

- **内插外推：** 在低载波比下，使用多个电流采样点提高观测器更新频率
- **异步采样：** 电流采样频率高于PWM频率，提高磁链估计精度
- **预测控制集成：** 将离散时间观测器与模型预测控制（MPC）统一在同一离散框架下

### 7.4 功能安全考量

- **双观测器冗余：** 同时运行离散时间磁链观测器和SMO，交叉验证
- **残差监测：** 监测电流预测误差 $\|e_i\|$ 作为观测器健康指标
- **安全降级：** 观测器失效时切换到IF启动或安全停机

### 7.5 与ALG-07观测器的融合策略

| 速度区域 | 推荐观测器 | 理由 |
|---------|-----------|------|
| 0~5%额定 | HFI（ALG-09） | 零速有效 |
| 5~30%额定 | 传统磁链观测器（ALG-07） | 载波比高，传统方法足够 |
| 30~100%额定 | 离散时间磁链观测器（ALG-15） | 低载波比下更稳定 |
| >100%额定（弱磁） | 离散时间磁链观测器（ALG-15） | 极低载波比，必须用ZOH方法 |

---

## 关键公式速查表

| 名称 | 公式 | 说明 |
|------|------|------|
| 载波比 | $f_{ratio} = f_{PWM}/f_e = 2\pi/(T\omega_e)$ | 核心参数 |
| ZOH状态转移 | $\Phi = e^{-(R_s/L_s + j\omega_e)T}$ | 精确离散化 |
| ZOH输入矩阵 | $\Gamma = \frac{1-\Phi}{R_s/L_s + j\omega_e}$ | 电压影响 |
| 非线性校正 | $G = [\lambda + jq/2](\hat{\psi}_r - \hat{\psi}_m \hat{\psi}_r/\|\hat{\psi}_r\|)$ | 幅值+位置修正 |
| 位置估算 | $\hat{\theta}_e = \arctan(\hat{\psi}_{r\beta}/\hat{\psi}_{r\alpha})$ | 角度计算 |
| 稳定性判据 | $\rho(\mathbf{H}) < 1$ | 极点在单位圆内 |
| 增益优化 | $\min_{q,\lambda} J = \sum \|i - \hat{i}^{pred}\|^2$ | 误差最小化 |

## 实验验证结果（论文数据）

| 测试条件 | 载波比 | 传统观测器 | 本文方法 |
|---------|--------|-----------|---------|
| 额定转速 | 20 | 稳定 | 稳定 |
| 2倍额定转速 | 10 | 振荡 | 稳定 |
| 4倍额定转速 | 5.2 | 失稳 | 稳定 |
| 阶跃负载 | 10 | 角度误差大 | 角度误差小 |
| 速度反转 | 10 | 过渡过程差 | 平滑过渡 |
