# ALG-09 高频注入法

**模块编号：** ALG-09  
**模块名称：** 高频注入法（High Frequency Injection, HFI）  
**文档版本：** v2.0  
**适用对象：** 电机控制工程师、嵌入式开发者  
**前置知识：** ALG-01 FOC理论基础、ALG-05 有感FOC实现、ALG-07 无感FOC观测器

---

## 1. ?? 核心摘要 ★★★☆☆ ????

**一句话：** 高频注入法利用IPM电机的凸极效应（$L_d \neq L_q$），通过注入高频电压信号检测电流响应来估算转子位置，是零速/低速无感FOC的核心技术，与高速观测器（SMO/磁链）配合实现全速域无感控制。

**认知挂钩：** 如果说反电动势观测器是"听电机的声音"来判断位置（低速时声音太小听不到），高频注入就是"主动敲击电机"——注入一个高频信号作为"敲击"，通过"回声"（电流响应）来判断位置。凸极效应越强，"回声"越清晰。

**核心原理链：**

```text
凸极效应(Ld≠Lq) → 注入高频电压 → 电流响应含位置信息 → 解调提取角度误差 → PLL跟踪
```

**高频注入法类型总览：**

| 注入方式 | 注入信号 | 解调方法 | 优点 | 缺点 |
|---------|---------|---------|------|------|
| 脉振高频注入 | d轴正弦波 | 乘法解调+LPF | 实现简单 | 带宽受限 |
| 方波高频注入 | d轴方波 | 差分解调 | 带宽宽、精度高 | 需要高速采样 |
| 旋转高频注入 | αβ轴旋转电压 | 同步解调 | 适用于SPMSM | 计算量大 |

**适用条件：**

| 条件 | 要求 | 说明 |
|------|------|------|
| 电机类型 | IPMSM/SynRM | 必须有凸极效应 |
| 凸极率 | $\xi > 0.1$ | 太小则信噪比不足 |
| 速度范围 | 0~15%额定转速 | 高速时基波干扰大 |
| SPMSM | 不适用 | $L_d = L_q$，无凸极效应 |

---

## 2. ?? 原理推导 ★★★★☆ ??

### 2.1 凸极效应物理原理

#### 2.1.1 IPM电机磁路差异

IPM电机的转子结构导致d轴和q轴磁路截然不同：

**d轴磁路：**
- 穿过永磁体区域
- 永磁体磁导率接近空气（$\mu_r \approx 1$）
- 磁阻大 → 电感小 → $L_d$ 小

**q轴磁路：**
- 不穿过永磁体，通过铁芯
- 铁芯磁导率高
- 磁阻小 → 电感大 → $L_q$ 大

**结果：** $L_d < L_q$，典型比值 $L_q/L_d \approx 2 \sim 3$

#### 2.1.2 凸极率定义

$$
\xi = \frac{L_q - L_d}{L_d + L_q}
$$

其中：
- $\xi$：凸极率，无量纲，值越大表示d/q轴电感差异越大
- $L_d$：d轴电感 ($H$)，穿过永磁体方向，磁阻大故电感小
- $L_q$：q轴电感 ($H$)，通过铁芯方向，磁阻小故电感大

凸极率越大，高频注入的位置估算信噪比越高。典型IPM电机 $\xi \approx 0.3 \sim 0.5$。

### 2.2 高频模型推导

#### 2.2.1 高频电压方程

在高频下，电阻压降可忽略（$\omega_h L \gg R_s$）：

$$
\begin{cases}
u_{dh} = L_{dh} \frac{di_{dh}}{dt} \\
u_{qh} = L_{qh} \frac{di_{qh}}{dt}
\end{cases}
$$

其中：
- $u_{dh}, u_{qh}$：d轴、q轴高频电压分量 ($V$)
- $i_{dh}, i_{qh}$：d轴、q轴高频电流分量 ($A$)
- $L_{dh}, L_{qh}$：d轴、q轴高频电感 ($H$)，与基波电感略有差异（因高频下磁路饱和程度不同）

#### 2.2.2 估计坐标系下的电感

在估计的dq'坐标系下（与真实dq坐标系存在角度误差 $\theta_{err}$）：

$$
\begin{bmatrix} L_{d'} \\ L_{q'} \end{bmatrix} = \begin{bmatrix} \frac{L_d + L_q}{2} + \frac{L_d - L_q}{2}\cos(2\theta_{err}) \\ \frac{L_d + L_q}{2} - \frac{L_d - L_q}{2}\cos(2\theta_{err}) \end{bmatrix}
$$

其中：
- $L_{d'}, L_{q'}$：估计dq'坐标系下的d轴、q轴视在电感 ($H$)
- $L_d, L_q$：真实dq坐标系下的d轴、q轴电感 ($H$)
- $\theta_{err}$：估计角度与真实角度之间的误差 ($rad$)，$\theta_{err} = \theta_{est} - \theta_{real}$

**关键结论：** 估计坐标系下的电感包含角度误差 $\theta_{err}$ 的信息！

当 $\theta_{err} = 0$ 时：$L_{d'} = L_d$, $L_{q'} = L_q$（完全对齐）  
当 $\theta_{err} = 45°$ 时：$L_{d'} = L_{q'} = \frac{L_d + L_q}{2}$（无法区分）

### 2.3 脉振高频注入原理

#### 2.3.1 注入方式

在估计的d'轴注入高频正弦电压：

$$
u_{dh} = U_h \cos(\omega_h t)
$$

其中：
- $U_h$：注入电压幅值（典型值：5~15V）
- $\omega_h$：注入角频率（典型值：500~2000 Hz）

#### 2.3.2 电流响应

q轴高频电流响应：

$$
i_{qh} \approx \frac{U_h}{\omega_h L_{avg}} \left[ 1 + \xi \cos(2\theta_{err}) \right] \sin(\omega_h t)
$$

其中 $L_{avg} = \frac{L_d + L_q}{2}$。

**关键：** q轴高频电流包含角度误差信息 $\cos(2\theta_{err})$！

#### 2.3.3 信号解调

将q轴高频电流乘以 $\sin(\omega_h t)$：

$$
i_{qh} \cdot \sin(\omega_h t) = \frac{U_h}{2\omega_h L_{avg}} \left[ 1 + \xi \cos(2\theta_{err}) \right] + \text{高频分量}
$$

低通滤波后得到角度误差信号：

$$
\varepsilon = K \cdot \xi \cos(2\theta_{err}) \approx 2K\xi \theta_{err} \quad (\theta_{err} \text{很小时})
$$

其中：
- $\varepsilon$：解调后的角度误差信号 ($A$)
- $K$：解调增益系数，$K = \frac{U_h}{2\omega_h L_{avg}}$
- $\xi$：凸极率
- $\theta_{err}$：估计角度与真实角度之间的误差 ($rad$)

### 2.4 方波高频注入原理

#### 2.4.1 注入方式

在d轴注入方波电压：

$$
u_{dh} = \begin{cases} +U_h & \text{前半周期} \\ -U_h & \text{后半周期} \end{cases}
$$

#### 2.4.2 差分解调

利用方波前后半周期的电流差分提取角度误差：

$$
\Delta i_{qh} \propto \frac{1}{L_{avg} - \Delta L \cos(2\theta_{err})} \approx \frac{1}{L_{avg}} (1 + \xi \cos(2\theta_{err}))
$$

其中：
- $\Delta i_{qh}$：前后半周期q轴高频电流差分值 ($A$)，包含角度误差信息
- $L_{avg}$：平均电感 ($H$)，$L_{avg} = \frac{L_d + L_q}{2}$
- $\Delta L$：电感差 ($H$)，$\Delta L = \frac{L_q - L_d}{2}$
- $\xi$：凸极率
- $\theta_{err}$：角度估计误差 ($rad$)

**方波注入优势：**
- 信号幅值恒定，不随时间衰减
- 差分解调简单，无需乘法器
- 带宽更宽，适合快速动态响应

### 2.5 N/S极判断原理

#### 2.5.1 180°模糊问题

高频注入只能确定转子位置到±90°，存在N/S极模糊：

$$
\theta_{est} = \theta_{real} \quad \text{或} \quad \theta_{est} = \theta_{real} + 180°
$$

原因：$\cos(2\theta_{err})$ 中 $\theta_{err}$ 和 $\theta_{err} + 180°$ 产生相同的电流响应。

#### 2.5.2 判断方法

| 方法 | 原理 | 优点 | 缺点 |
|------|------|------|------|
| 脉冲注入法 | 注入6扇区电压矢量，比较峰值电流 | 可靠性高 | 启动时间增加 |
| Id偏置法 | 注入直流偏置，比较Id正负半周 | 实现简单 | 需要稳定条件 |
| 磁饱和法 | 利用磁饱和非线性 | 无需额外注入 | 对电机参数敏感 |

---

## 3. ?? 数学建模 ★★★★★ ??

### 3.1 脉振注入完整数学模型

#### 3.1.1 高频电流响应推导

在估计dq'坐标系下，注入 $u_{dh} = U_h \cos(\omega_h t)$ 后：

$$
\begin{cases}
i_{dh} = \frac{U_h}{\omega_h L_{d'}} \sin(\omega_h t) \\
i_{qh} = \frac{U_h}{\omega_h L_{q'}} \sin(\omega_h t)
\end{cases}
$$

展开 $L_{q'}$：

$$
i_{qh} = \frac{U_h}{\omega_h \left[\frac{L_d+L_q}{2} - \frac{L_d-L_q}{2}\cos(2\theta_{err})\right]} \sin(\omega_h t)
$$

当 $\xi$ 较小时，近似展开：

$$
i_{qh} \approx \frac{U_h}{\omega_h L_{avg}} \left[ 1 + \xi \cos(2\theta_{err}) \right] \sin(\omega_h t)
$$

#### 3.1.2 解调信号推导

乘法解调：

$$
i_{qh} \cdot \sin(\omega_h t) = \frac{U_h}{\omega_h L_{avg}} \left[ 1 + \xi \cos(2\theta_{err}) \right] \cdot \frac{1 - \cos(2\omega_h t)}{2}
$$

低通滤波后：

$$
\varepsilon = \frac{U_h \xi}{2\omega_h L_{avg}} \cos(2\theta_{err})
$$

小角度近似：

$$
\varepsilon \approx \frac{U_h \xi}{\omega_h L_{avg}} \theta_{err} \quad (|\theta_{err}| < 15°)
$$

### 3.2 方波注入完整数学模型

#### 3.2.1 方波电流响应

方波注入时，q轴高频电流线性上升：

$$
i_{qh}(t) = \frac{U_h}{L_{q'}} \cdot t \quad \text{(前半周期)}
$$

半周期末电流值：

$$
i_{qh}\left(\frac{T}{2}\right) = \frac{U_h T}{2 L_{q'}}
$$

#### 3.2.2 差分信号

前后半周期差分：

$$
\Delta i_{qh} = \frac{U_h T}{2} \left(\frac{1}{L_{q',\text{前}}} + \frac{1}{L_{q',\text{后}}}\right)
$$

代入 $L_{q'}$ 表达式：

$$
\Delta i_{qh} \propto \frac{1}{L_{avg} - \Delta L \cos(2\theta_{err})} \approx \frac{1}{L_{avg}} (1 + \xi \cos(2\theta_{err}))
$$

### 3.3 PLL角度跟踪数学模型

#### 3.3.1 PLL结构

$$
\omega_{est} = K_p \cdot \varepsilon + K_i \int \varepsilon \, dt
$$

$$
\theta_{est} = \int \omega_{est} \, dt
$$

其中：
- $\omega_{est}$：估算的电角速度 ($rad/s$)
- $\theta_{est}$：估算的电角度 ($rad$)
- $K_p$：PLL比例增益
- $K_i$：PLL积分增益
- $\varepsilon$：解调后的角度误差信号

#### 3.3.2 PLL闭环传递函数

$$
G_{PLL}(s) = \frac{K_p s + K_i}{s^2 + K_p s + K_i}
$$

自然频率和阻尼比：

$$
\omega_n = \sqrt{K_i}, \quad \zeta = \frac{K_p}{2\sqrt{K_i}}
$$

其中：
- $G_{PLL}(s)$：PLL闭环传递函数
- $K_p$：PLL比例增益
- $K_i$：PLL积分增益
- $\omega_n$：PLL自然频率 ($rad/s$)
- $\zeta$：阻尼比，通常取0.707

**设计准则：** $\zeta = 0.707$（临界阻尼），$\omega_n$ 远小于注入频率 $\omega_h$。

### 3.4 全速域切换数学模型

#### 3.4.1 线性加权过渡

$$
\theta = (1 - K_{smo}) \cdot \theta_{HFI} + K_{smo} \cdot \theta_{SMO}
$$

其中：

$$
K_{smo} = \frac{\omega - \omega_{Lo}}{\omega_{Hi} - \omega_{Lo}}, \quad K_{smo} \in [0, 1]
$$

#### 3.4.2 切换速度阈值设计

| 参数 | 典型值 | 说明 |
|------|--------|------|
| $\omega_{Lo}$ | 10%额定转速 | HFI开始退出 |
| $\omega_{Hi}$ | 15%额定转速 | SMO完全接管 |
| 滞后宽度 | 3~5%额定转速 | 防止频繁切换 |

### 3.5 注入频率选择准则

$$
f_{base} \ll f_h \ll f_{sw}/10
$$

| 参数 | 典型范围 | 说明 |
|------|---------|------|
| 基波频率 $f_{base}$ | 0~100Hz | 电机运行频率 |
| 注入频率 $f_h$ | 500~2000Hz | 避开基波和载波 |
| 开关频率 $f_{sw}$ | 10~20kHz | PWM频率 |

---

## 4. ?? 代码实现 ★★★★☆ ??

### 4.1 脉振高频注入实现

**代码位置：** [high_frequency_injection.c:87-143](../../脉振和方波高频注入+观测器_无感全速域运行/F4脉振高频注入+磁链观测器/motor/high_frequency_injection.c#L87)

```c
void HFI_Process(CURRENT_DQ_DEF current_dq_temp, CURRENT_DQ_DEF* current_dq_lpf, HFI_DEF* hfi_temp)
{
    float hfi_cos, hfi_sin;
    float q_current_bpf;
    float hfi_pid_error;
    float hfi_pid_out;
    
    hfi_cos = arm_cos_f32((temp1 / temp2) * 2.0f * PI);
    hfi_sin = arm_sin_f32((temp1 / temp2) * 2.0f * PI);
    
    hfi_temp->hfi_inj_volt = hfi_cos * HFI_inj_volt_amp_temp;
    
    IIR_Butterworth(current_dq_temp.Id, &current_dq_lpf->Id, &D_IIR_LPF_Par);
    IIR_Butterworth(current_dq_temp.Iq, &current_dq_lpf->Iq, &Q_IIR_LPF_Par);
    
    IIR_Butterworth(current_dq_temp.Iq, &q_current_bpf, &Q_IIR_BPF_Par);
    
    temp1 = q_current_bpf * hfi_sin;
    
    IIR_Butterworth(temp1, &hfi_pid_error, &ERR_IIR_LPF_Par);
    
    hfi_pid_out = HFI_PID.P_Gain * hfi_pid_error + HFI_PID.I_Sum;
    hfi_integrator += FOC_PERIOD * hfi_pid_out;
    
    if(hfi_integrator < 0.0f)
        hfi_integrator += 2.0f * PI;
    else if(hfi_integrator > (2.0f * PI))
        hfi_integrator -= 2.0f * PI;
}
```

### 4.2 方波高频注入实现

**代码位置：** [hfi.c:149-169](../../脉振和方波高频注入+观测器_无感全速域运行/基于DSP脉振方波高频注入与增强型滑模/hfi +esmo/Source/hfi.c#L149)

```c
if (++squ_cnt >= Squ_PRD)
{
    squ_cnt = 0;
    dIq = SIGN(h->dutyMax) * (iq->Out - IqOld);
    IqOld = iq->Out;
    h->dutyMax = -h->dutyMax;
}
```

### 4.3 高通滤波器实现

**代码位置：** [hfi.c:46-72](../../脉振和方波高频注入+观测器_无感全速域运行/基于DSP脉振方波高频注入与增强型滑模/hfi +esmo/Source/hfi.c#L46)

```c
void HPF_INIT(HPF_COEFF_handle k)
{
    k->wTby2 = _IQmpy(k->PiT, k->freq);
    k->N     = _IQdiv(_IQ(1.0), _IQ(1.0) + k->wTby2);
    k->D     = _IQmpy(_IQ(1.0) - k->wTby2, k->N);
}

void HPF_MODULE(HPF_handle f, HPF_COEFF_handle k)
{
    f->Out = _IQmpy(k->N, f->In1 - f->In0) + _IQmpy(k->D, f->Out);
    f->In0 = f->In1;
}
```

### 4.4 N/S极判断实现

**代码位置：** [hfi.c:231-459](../../脉振和方波高频注入+观测器_无感全速域运行/基于DSP脉振方波高频注入与增强型滑模/hfi +esmo/Source/hfi.c#L231)

```c
void NS_Determination(NS_ID *n, HFI *h, CLARKE *i)
{
    for(vector = 1; vector < 7; vector++)
    {
        PeakCur = PeakCurTable[vector-1];
        if(PeakCurMax[0] < PeakCur)
        {
            PeakCurMax[1] = PeakCurMax[0];
            PeakSector[1] = PeakSector[0];
            PeakCurMax[0] = PeakCur;
            PeakSector[0] = vector;
        }
    }
    
    if ((angle >= angLo) && (angle <= angHi))
    {
        h->thetaEst = angle;
        NS_Status = SUCCESS;
    }
    else
    {
        angle += _IQ(0.5);
    }
}
```

**Id偏置法实现：** [high_frequency_injection.c:146-191](../../脉振和方波高频注入+观测器_无感全速域运行/F4脉振高频注入+磁链观测器/motor/high_frequency_injection.c#L146)

```c
void d_direction_process(float in_temp, uint8_t* out_temp)
{
    if(in_temp > id_max)
        id_max = in_temp;
    if(in_temp < id_min)
        id_min = in_temp;
    
    if(id_max > (-id_min))
    {
        *out_temp = 1;
    }
    else
    {
        hfi_integrator = hfi_integrator + PI;
        *out_temp = 2;
    }
}
```

### 4.5 HFI状态机

**代码位置：** [hfi.c:114-223](../../脉振和方波高频注入+观测器_无感全速域运行/基于DSP脉振方波高频注入与增强型滑模/hfi +esmo/Source/hfi.c#L114)

```c
void ZLSPD(HPF *iq, HPF_COEFF *k, HFI *h, NS_ID *n, CLARKE *i, PHASEVOLTAGE *v)
{
    switch(h->HFI_Status)
    {
    case HFI_RESET_STATE:
        h->HFI_Status = HFI_IPD_STATE;
        break;
        
    case HFI_IPD_STATE:
        if (NS_cnt >= h->HFI_Time2)
            h->HFI_Status = HFI_NSID_STATE;
        break;
        
    case HFI_NSID_STATE:
        NS_Determination(n, h, i);
        break;
        
    case HFI_RUN_STATE:
        break;
    }
}
```

### 4.6 全速域切换实现

```c
void ANGLE_TRANSIT(TRANSITION *v) {
    _iq Ksmo, spd = _IQabs(v->spd);
    
    if (spd >= v->spdHi)
        v->angle = v->angleSMO;
    
    else if (spd >= v->spdLo) {
        Ksmo = (spd - v->spdLo) * v->scale;
        v->angle = _IQmpy(v->angleHFI, _IQ(1.0) - Ksmo) + 
                   _IQmpy(v->angleSMO, Ksmo);
    }
    else
        v->angle = v->angleHFI;
}
```

### 4.7 ?? hpm_MC 代码实现参考

**v1 HFI高频注入** (`hpm_mcl/inc/hpm_hfi.h`):
- `hpm_hfi_para_t` 注入参数：vh（注入电压幅值）、hf_current_coeff（高频电流系数）
- `hpm_hfi_pole_detect_para_t` N/S极辨识参数：detect_time、pole_current_threshold
- `hpm_hfi_pll_para_t` PLL参数：kp/ki、低通滤波器系数

**注入策略**:
- 方波电压注入法：在 d 轴注入高频方波电压 vh
- 响应电流解调：提取 q 轴高频电流分量，经 PLL 提取转速和角度
- 极辨识阶段：注入反向脉冲，通过电流响应区分 N/S 极

**参数选择指南**（从示例 `bldc_hfi` 提取）:
- vh = 0.625 × PWM_RELOAD（典型值）
- PLL kp = 10, ki = 1.0
- 最高运行频率 1.1r/s（极低速零速FOC）

**示例应用**: `hpm_MC/samples/motor_ctrl/bldc_hfi/`

**参考**: `SDK-01-HPM-MC-Architecture.md` 第3节「v1 HFI分析」

---

## 5. ?? 参数整定 ★★★★★ ??

### 5.1 注入电压参数整定

| 参数 | 说明 | 整定方法 |
|------|------|---------|
| $U_h$ | 注入电压幅值 | 从小值开始，逐步增大直到角度估算稳定 |
| $f_h$ | 注入频率 | 满足 $f_{base} \ll f_h \ll f_{sw}/10$ |

**注入电压选择原则：**

$$
U_h \geq \frac{2\omega_h L_{avg} I_{detect}}{\xi}
$$

其中：
- $U_h$：注入电压幅值 ($V$)
- $\omega_h$：注入角频率 ($rad/s$)
- $L_{avg}$：平均电感 ($H$)，$L_{avg} = \frac{L_d + L_q}{2}$
- $I_{detect}$：最小可检测电流 ($A$)，取决于ADC分辨率和噪声水平
- $\xi$：凸极率

**权衡：**
- $U_h$ 太小：信噪比低，角度估算不稳定
- $U_h$ 太大：产生可听噪声、额外转矩脉动、影响基波控制

### 5.2 PLL参数整定

| 参数 | 说明 | 整定方法 |
|------|------|---------|
| $K_p$ | PLL比例增益 | 影响角度跟踪速度 |
| $K_i$ | PLL积分增益 | 影响稳态精度 |

**设计步骤：**

1. 确定PLL带宽 $\omega_{PLL}$（典型10~50Hz）
2. 计算 $K_i = \omega_{PLL}^2$
3. 计算 $K_p = 2\zeta\omega_{PLL}$，取 $\zeta = 0.707$
4. 实测调整：先Kp后Ki，观察角度跟踪性能

**典型参数：**

```c
HFI_PID.P_Gain = 50.0f;
HFI_PID.I_Gain = 500.0f;
```

### 5.3 滤波器参数整定

| 滤波器 | 目的 | 截止频率设计 |
|--------|------|-------------|
| 带通滤波器(BPF) | 提取高频分量 | 中心频率 = $f_h$，带宽 = $0.1 f_h$ |
| 低通滤波器(LPF) | 基波分离 | $f_c < 0.1 f_h$ |
| 高通滤波器(HPF) | 去除基波 | $f_c > 2 f_{base,max}$ |
| 误差LPF | 解调后滤波 | $f_c \ll f_h$ |

### 5.4 方波注入参数整定

| 参数 | 说明 | 典型值 |
|------|------|--------|
| 方波周期 | 一个方波周期的PWM计数 | 10~50个PWM周期 |
| 注入占空比 | 注入电压/母线电压 | 5%~15% |

### 5.5 常见问题与解决方案

| 问题 | 现象 | 可能原因 | 解决方案 |
|------|------|---------|---------|
| 角度估算抖动 | 估算角度高频振荡 | PLL带宽过高/注入电压过大 | 降低PLL带宽/减小注入幅值 |
| N/S极判断错误 | 电机反转 | Id偏置法判断条件不满足 | 使用脉冲注入法/增加判断延时 |
| 可听噪声 | 电机发出嗡嗡声 | 注入频率在听觉范围 | 提高注入频率>2kHz |
| 高速性能差 | 角度误差增大 | 基波干扰注入信号 | 切换到SMO/磁链观测器 |
| 切换冲击 | HFI→SMO切换时电流突变 | 角度不连续 | 线性加权过渡/角度预对齐 |
| SPMSM失效 | 无法估算位置 | 无凸极效应 | 使用反电动势观测器+IF启动 |

---

## 6. ?? 硬件约束 ★★★★☆ ??

### 6.1 注入信号→逆变器非线性

?? **硬件约束：注入信号精度受逆变器死区和管压降严重影响**

- **死区效应：** 逆变器死区导致实际输出电压偏离给定值。在HFI中，注入电压幅值通常仅5~15V，而死区电压误差可达2~5V，占比极大。死区产生的谐波与注入信号混叠，严重干扰解调
- **管压降：** IGBT/MOSFET的饱和压降（1~2V）在低调制比下占比显著，导致注入电压实际幅值与给定值不一致
- **应对措施：** 需要死区补偿算法修正注入电压 → [HW-05 功率器件与栅极驱动](../hardware/HW-05-Power-Devices-Gate-Drivers.md#411-死区时间)

### 6.2 凸极效应→电机结构

?? **硬件约束：高频注入法的适用性完全取决于电机凸极效应**

- **凸极率要求：** $\xi = \frac{L_q - L_d}{L_d + L_q} > 0.1$ 才能获得足够的信噪比。表贴式SPMSM（$L_d = L_q$）完全不适用
- **磁饱和影响：** 大电流时铁芯饱和导致 $L_d, L_q$ 变化，凸极率随负载变化，影响位置估算精度
- **交叉耦合：** 实际电机存在d-q轴交叉耦合电感，在高频模型中被忽略，导致估算误差 → [HW-01 电机本体基础](../hardware/HW-01-Motor-Basics.md#421-电感参数)

### 6.3 信噪比→电流采样噪声

?? **硬件约束：高频注入位置估算的信噪比直接受限于电流采样系统质量**

- **采样噪声：** 高频电流响应幅值通常仅几十到几百毫安，ADC量化噪声和开关噪声可能淹没有用信号。12位ADC在额定电流1%以下时量化噪声严重影响解调
- **采样同步：** 注入信号与电流采样必须精确同步，采样时刻偏差导致解调相位误差
- **抗混叠滤波：** ADC前端抗混叠滤波器不能过度衰减注入频率分量 → [HW-02 电流采样电路](../hardware/HW-02-Current-Sensing.md#433-rc滤波器设计)

### 6.4 注入频率→PWM频率

?? **硬件约束：注入频率受PWM频率限制，影响位置估算带宽**

- **频率上限：** $f_h < f_{sw}/10$，否则一个注入周期内PWM脉冲数不足，注入波形失真
- **采样率限制：** 方波注入的差分解调需要在一个方波半周期内采样多次，PWM频率决定采样率上限
- **计算延迟：** 滤波器计算（BPF+LPF）需要在每个PWM周期内完成，影响最小PWM周期选择 → [HW-04 MCU外设与通信](../hardware/HW-04-MCU-Peripherals.md#412-互补pwm与死区)

---

## 7. ?? 前沿拓展 ★★★★★ ??

### 7.1 旋转高频注入

在αβ坐标系注入旋转电压矢量：

$$
\begin{cases}
u_{\alpha h} = U_h \cos(\omega_h t) \\
u_{\beta h} = U_h \sin(\omega_h t)
\end{cases}
$$

其中：
- $u_{\alpha h}, u_{\beta h}$：αβ轴高频注入电压分量 ($V$)
- $U_h$：注入电压幅值 ($V$)
- $\omega_h$：注入角频率 ($rad/s$)

**优势：** 对SPMSM也可使用（利用磁饱和引起的微小凸极效应）  
**挑战：** 解调算法更复杂，需要负序分量提取

### 7.2 自适应注入策略

- **注入幅值自适应：** 根据信噪比动态调整注入电压，低速大注入、高速小注入
- **注入频率自适应：** 根据电机运行频率动态调整，避免与基波谐波重叠
- **多频注入：** 同时注入多个频率，提高鲁棒性

### 7.3 基于深度学习的HFI

- **端到端位置估算：** 用神经网络替代传统解调+PLL流程
- **噪声抑制：** 用自编码器从含噪电流中提取位置信息
- **迁移学习：** 在仿真中训练，迁移到不同电机

### 7.4 无注入传感器less技术

- **INFORM方法：** 利用PWM开关动作本身产生的电流变化，无需额外注入
- **低速反电动势增强：** 通过特殊调制策略增强低速反电动势可观测性
- **全速域统一观测器：** 将HFI和SMO统一在同一数学框架下，避免切换

### 7.5 与ALG-07观测器的协同

HFI与高速观测器的全速域组合是当前工业主流方案：

| 方案 | 低速 | 高速 | 切换策略 |
|------|------|------|---------|
| HFI + SMO | HFI | SMO | 线性加权 |
| HFI + 磁链观测器 | HFI | 磁链 | 滞后切换 |
| HFI + EKF | HFI | EKF | 软切换 |

详见 [ALG-07 无感FOC观测器](./ALG-07-Sensorless-Observers.md#7-前沿拓展)

---

## 高频注入法性能对比

| 特性 | 脉振注入 | 方波注入 | 旋转注入 |
|------|---------|---------|---------|
| 适用电机 | IPMSM | IPMSM | IPMSM/SPMSM |
| 位置精度 | ★★★☆☆ | ★★★★☆ | ★★★☆☆ |
| 动态响应 | ★★★☆☆ | ★★★★☆ | ★★★☆☆ |
| 计算量 | ★★★☆☆ | ★★☆☆☆ | ★★★★☆ |
| 噪声 | ★★★☆☆ | ★★★★☆ | ★★★☆☆ |
| 实现难度 | ★★☆☆☆ | ★★★☆☆ | ★★★★☆ |

## 关键公式速查表

| 名称 | 公式 | 说明 |
|------|------|------|
| 凸极率 | $\xi = \frac{L_q - L_d}{L_d + L_q}$ | 电感差异度量 |
| q轴高频电流 | $i_{qh} \propto \cos(2\theta_{err})$ | 角度误差信息 |
| 解调误差信号 | $\varepsilon \approx 2K\xi\theta_{err}$ | 小角度近似 |
| PLL方程 | $\dot{\theta} = K_p \varepsilon + K_i \int \varepsilon$ | 角度跟踪 |
| 全速域切换 | $\theta = (1-K)\theta_{HFI} + K\theta_{SMO}$ | 线性加权 |
| 注入频率选择 | $f_{base} \ll f_h \ll f_{sw}/10$ | 频率约束 |