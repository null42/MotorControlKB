# MC_LIB - 观测器模块详解

**文档版本：** v1.0  
**生成日期：** 2026-04-26  
**源码位置：** `MC_LIB/3_MC/31_FOC/310_FOC_F/MCFOC_EST_F.c/h`

---

## 目录

1. [模块概述](#1-模块概述)
2. [滑模观测器(SMO)](#2-滑模观测器smo)
3. [磁链观测器(FLUX)](#3-磁链观测器flux)
4. [反电动势观测器(EMF)](#4-反电动势观测器emf)
5. [PLL角度跟踪](#5-pll角度跟踪)
6. [角度补偿机制](#6-角度补偿机制)
7. [参数自适应](#7-参数自适应)
8. [API参考](#8-api参考)

---

## 1. 模块概述

### 1.1 功能说明

MCFOC_EST_F模块实现了多种无感观测器算法：

| 函数 | 功能 | 适用场景 |
|------|------|---------|
| `MCFOC_EST_SMO_F` | 滑模观测器 | 中高速，全速域 |
| `MCFOC_EST_FLUX_F` | 磁链观测器 | 中低速，高精度 |
| `MCFOC_EST_EMF_F` | 反电动势观测器 | 高速，简单实现 |
| `MCFOC_EST_PLL_F` | PLL角度跟踪 | 所有观测器的角度提取 |

### 1.2 观测器对比

| 特性 | SMO | FLUX | EMF |
|------|-----|------|-----|
| 低速性能 | 中等 | 优秀 | 较差 |
| 高速性能 | 优秀 | 良好 | 优秀 |
| 计算量 | 中等 | 较高 | 较低 |
| 参数敏感性 | 中等 | 较高 | 较低 |
| 鲁棒性 | 优秀 | 良好 | 中等 |

### 1.3 模块特点

```
┌─────────────────────────────────────────────────────────────┐
│                   MCFOC_EST_F模块特点                        │
├─────────────────────────────────────────────────────────────┤
│  ✓ 多观测器支持：SMO、FLUX、EMF三种观测器                    │
│  ✓ PLL角度跟踪：平滑的角度和速度估计                         │
│  ✓ 角度补偿：根据频率和电流自适应补偿延迟                    │
│  ✓ 参数自适应：根据运行状态调整观测器增益                    │
│  ✓ 平滑切换：观测器输出平滑过渡                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 滑模观测器(SMO)

### 2.1 数学原理

**滑模观测器**通过构造电流观测器，利用滑模控制律提取反电动势信息。

**PMSM电压方程（αβ坐标系）：**

$$
\begin{cases}
u_\alpha = R_s i_\alpha + L_s \frac{di_\alpha}{dt} + e_\alpha \\
u_\beta = R_s i_\beta + L_s \frac{di_\beta}{dt} + e_\beta
\end{cases}
$$

其中：
- $u_\alpha, u_\beta$：αβ轴定子电压 ($V$)
- $R_s$：定子电阻 ($\Omega$)
- $i_\alpha, i_\beta$：αβ轴定子电流 ($A$)
- $L_s$：定子电感 ($H$)
- $e_\alpha, e_\beta$：αβ轴反电动势 ($V$)

**电流观测器：**

$$
\begin{cases}
\frac{d\hat{i}_\alpha}{dt} = -\frac{R_s}{L_s}\hat{i}_\alpha + \frac{1}{L_s}(u_\alpha - z_\alpha) \\
\frac{d\hat{i}_\beta}{dt} = -\frac{R_s}{L_s}\hat{i}_\beta + \frac{1}{L_s}(u_\beta - z_\beta)
\end{cases}
$$

其中：
- $\hat{i}_\alpha, \hat{i}_\beta$：αβ轴观测电流 ($A$)
- $R_s$：定子电阻 ($\Omega$)
- $L_s$：定子电感 ($H$)
- $u_\alpha, u_\beta$：αβ轴定子电压 ($V$)
- $z_\alpha, z_\beta$：滑模控制律输出，等效反电动势 ($V$)

**滑模控制律：**

$$
\begin{cases}
z_\alpha = H_1 \cdot \text{sign}(\hat{i}_\alpha - i_\alpha) \\
z_\beta = H_1 \cdot \text{sign}(\hat{i}_\beta - i_\beta)
\end{cases}
$$

其中：
- $z_\alpha, z_\beta$：滑模控制律输出 ($V$)
- $H_1$：滑模增益 ($V$)，需满足 $H_1 > \max(|e_\alpha|, |e_\beta|)$ 以保证滑模面可达
- $\hat{i}_\alpha, \hat{i}_\beta$：αβ轴观测电流 ($A$)
- $i_\alpha, i_\beta$：αβ轴实际电流 ($A$)
- $\text{sign}(\cdot)$：符号函数

**反电动势提取（低通滤波）：**

$$
\begin{cases}
\hat{e}_\alpha = \frac{1}{1 + s/\omega_c} z_\alpha \\
\hat{e}_\beta = \frac{1}{1 + s/\omega_c} z_\beta
\end{cases}
$$

其中：
- $\hat{e}_\alpha, \hat{e}_\beta$：滤波后的αβ轴反电动势估计值 ($V$)
- $\omega_c$：低通滤波器截止角频率 ($rad/s$)，需折中选择：过低则相位延迟大，过高则纹波大
- $z_\alpha, z_\beta$：滑模控制律输出 ($V$)

### 2.2 数据结构

```c
typedef struct
{
    // ================== 角度三角函数 ==================
    ST_TRIG_F   TG_SMO_Triangle_Est;      // 估计角度三角函数
    ST_TRIG_F   TG_SMO_Triangle_Comp;     // 补偿角度三角函数
    
    // ================== 观测电流 ==================
    float       _V_F_SMO_Ialfa;           // 观测α轴电流
    float       _V_F_SMO_Ibeta;           // 观测β轴电流
    
    // ================== 滑模控制量 ==================
    float       _V_F_SMO_Zalfa;           // α轴滑模控制量
    float       _V_F_SMO_Zbeta;           // β轴滑模控制量
    
    // ================== 反电动势估计 ==================
    ST_LPF_F    LPF_SMO_Ealfa;            // α轴反电动势滤波器
    ST_LPF_F    LPF_SMO_Ebeta;            // β轴反电动势滤波器
    float       _V_F_SMO_Ealfa;           // α轴反电动势
    float       _V_F_SMO_Ebeta;           // β轴反电动势
    
    // ================== PLL控制器 ==================
    ST_PID_POS_F    PID_SMO_PLL;          // PLL PID控制器
    float           _V_F_SMO_PLL_Err;     // PLL误差
    float           _V_F_SMO_PLL_W;       // PLL估计角速度
    float           _V_F_SMO_PLL_Theta;   // PLL估计角度
    
    // ================== 参数 ==================
    float       _P_F_SMO_H1;              // 滑模增益H1
    float       _P_F_SMO_H2;              // 滑模增益H2
    float       _P_F_SMO_Kslide;          // 滑模系数
    
    // ================== 补偿角度查找表 ==================
    TABLE_2D_F  TAB_SMO_Angle_Comp;       // 角度补偿表
    
}ST_SMO_CONTROL_F;
```

### 2.3 代码实现

```c
void MCFOC_EST_SMO_F(ST_SMO_CONTROL_F* pSMO, 
                      ST_PMSM_ELEC_F* pPMSMe, 
                      ST_PMSM_PARA_F* pPMSMa)
{
    float F_Err_Ialfa = 0.0f, F_Err_Ibeta = 0.0f;
    float F_Delta_Ialfa = 0.0f, F_Delta_Ibeta = 0.0f;
    
    // ================== 步骤1：计算电流误差 ==================
    F_Err_Ialfa = pSMO->_V_F_SMO_Ialfa - pPMSMe->_V_F_Ialfa;
    F_Err_Ibeta = pSMO->_V_F_Ibeta - pPMSMe->_V_F_Ibeta;
    
    // ================== 步骤2：滑模控制律 ==================
    // Z = H1 * sign(I_est - I) + H2 * (I_est - I)
    pSMO->_V_F_SMO_Zalfa = pSMO->_P_F_SMO_H1 * MATH_SIGN_F(F_Err_Ialfa)
                         + pSMO->_P_F_SMO_H2 * F_Err_Ialfa;
    pSMO->_V_F_SMO_Zbeta = pSMO->_P_F_SMO_H1 * MATH_SIGN_F(F_Err_Ibeta)
                         + pSMO->_P_F_SMO_H2 * F_Err_Ibeta;
    
    // ================== 步骤3：电流观测器更新 ==================
    // dI_est/dt = -Rs/Ls * I_est + 1/Ls * (U - Z)
    F_Delta_Ialfa = -pPMSMa->_O_F_Rs / pPMSMa->_O_F_Ls * pSMO->_V_F_SMO_Ialfa
                  + 1.0f / pPMSMa->_O_F_Ls * (pPMSMe->_V_F_Ualfa_Pre - pSMO->_V_F_SMO_Zalfa);
    F_Delta_Ibeta = -pPMSMa->_O_F_Rs / pPMSMa->_O_F_Ls * pSMO->_V_F_SMO_Ibeta
                  + 1.0f / pPMSMa->_O_F_Ls * (pPMSMe->_V_F_Ubeta_Pre - pSMO->_V_F_SMO_Zbeta);
    
    // 欧拉积分
    pSMO->_V_F_SMO_Ialfa += F_Delta_Ialfa * pPMSMa->_O_F_Ts;
    pSMO->_V_F_SMO_Ibeta += F_Delta_Ibeta * pPMSMa->_O_F_Ts;
    
    // ================== 步骤4：反电动势提取 ==================
    // 通过低通滤波器提取反电动势
    LPF_Cal_F(&pSMO->LPF_SMO_Ealfa, pSMO->_V_F_SMO_Zalfa);
    LPF_Cal_F(&pSMO->LPF_SMO_Ebeta, pSMO->_V_F_SMO_Zbeta);
    pSMO->_V_F_SMO_Ealfa = pSMO->LPF_SMO_Ealfa.F_Out;
    pSMO->_V_F_SMO_Ebeta = pSMO->LPF_SMO_Ebeta.F_Out;
    
    // ================== 步骤5：PLL角度跟踪 ==================
    MCFOC_EST_PLL_F(pSMO, pPMSMe, pPMSMa);
}
```

### 2.4 算法分析

**关键点：**

1. **滑模增益选择**：
   - H1必须足够大以克服反电动势：$H_1 > \max(|e_\alpha|, |e_\beta|)$
   - H2用于平滑切换，减小抖振

2. **低通滤波器截止频率**：
   - 太低：相位滞后大
   - 太高：高频噪声大
   - 典型值：$\omega_c = 2\pi \times 50 \sim 200$ rad/s

3. **计算量**：
   - 电流观测器：4次乘法 + 4次加法
   - 滑模控制律：4次乘法 + 2次加法 + 2次符号函数
   - 低通滤波：2次乘法 + 2次加法
   - PLL：约10次乘法 + 8次加法

---

## 3. 磁链观测器(FLUX)

### 3.1 数学原理

**磁链观测器**通过积分电压方程估计磁链，进而计算转子位置。

**磁链方程：**

$$
\begin{cases}
\psi_\alpha = \int (u_\alpha - R_s i_\alpha) dt \\
\psi_\beta = \int (u_\beta - R_s i_\beta) dt
\end{cases}
$$

其中：
- $\psi_\alpha, \psi_\beta$：αβ轴定子总磁链 ($Wb$)
- $u_\alpha, u_\beta$：αβ轴定子电压 ($V$)
- $R_s$：定子电阻 ($\Omega$)
- $i_\alpha, i_\beta$：αβ轴定子电流 ($A$)

**转子磁链：**

$$
\begin{cases}
\psi_{f\alpha} = \psi_\alpha - L_s i_\alpha \\
\psi_{f\beta} = \psi_\beta - L_s i_\beta
\end{cases}
$$

其中：
- $\psi_{f\alpha}, \psi_{f\beta}$：αβ轴转子（永磁体）磁链分量 ($Wb$)
- $\psi_\alpha, \psi_\beta$：αβ轴定子总磁链 ($Wb$)
- $L_s$：定子电感 ($H$)
- $i_\alpha, i_\beta$：αβ轴定子电流 ($A$)

**角度计算：**

$$
\theta = \arctan\left(\frac{\psi_{f\beta}}{\psi_{f\alpha}}\right)
$$

其中：
- $\theta$：估计的转子电角度 ($rad$)
- $\psi_{f\alpha}, \psi_{f\beta}$：αβ轴转子磁链分量 ($Wb$)

### 3.2 数据结构

```c
typedef struct
{
    // ================== 角度三角函数 ==================
    ST_TRIG_F   TG_FLUX_Triangle_Est;     // 估计角度三角函数
    ST_TRIG_F   TG_FLUX_Triangle_Comp;    // 补偿角度三角函数
    
    // ================== 磁链估计 ==================
    float       _V_F_FLUX_Psi_alfa;       // α轴磁链
    float       _V_F_FLUX_Psi_beta;       // β轴磁链
    float       _V_F_FLUX_Psi_f_alfa;     // α轴转子磁链
    float       _V_F_FLUX_Psi_f_beta;     // β轴转子磁链
    
    // ================== PLL控制器 ==================
    ST_PID_POS_F    PID_FLUX_PLL;         // PLL PID控制器
    float           _V_F_FLUX_PLL_Err;    // PLL误差
    float           _V_F_FLUX_PLL_W;      // PLL估计角速度
    float           _V_F_FLUX_PLL_Theta;  // PLL估计角度
    
    // ================== 参数 ==================
    float       _P_F_FLUX_Ls;             // 电感
    float       _P_F_FLUX_Rs;             // 电阻
    
    // ================== 补偿角度查找表 ==================
    TABLE_2D_F  TAB_FLUX_Angle_Comp;      // 角度补偿表
    
}ST_FLUX_CONTROL_F;
```

### 3.3 代码实现

```c
void MCFOC_EST_FLUX_F(ST_FLUX_CONTROL_F* pFLUX, 
                       ST_PMSM_ELEC_F* pPMSMe, 
                       ST_PMSM_PARA_F* pPMSMa)
{
    // ================== 步骤1：磁链积分 ==================
    // Ψ = ∫(U - R*I)dt
    pFLUX->_V_F_FLUX_Psi_alfa += (pPMSMe->_V_F_Ualfa_Pre 
                                  - pPMSMa->_O_F_Rs * pPMSMe->_V_F_Ialfa) 
                                 * pPMSMa->_O_F_Ts;
    pFLUX->_V_F_FLUX_Psi_beta += (pPMSMe->_V_F_Ubeta_Pre 
                                  - pPMSMa->_O_F_Rs * pPMSMe->_V_F_Ibeta) 
                                 * pPMSMa->_O_F_Ts;
    
    // ================== 步骤2：转子磁链计算 ==================
    // Ψf = Ψ - Ls*I
    pFLUX->_V_F_FLUX_Psi_f_alfa = pFLUX->_V_F_FLUX_Psi_alfa 
                                  - pPMSMa->_O_F_Ls * pPMSMe->_V_F_Ialfa;
    pFLUX->_V_F_FLUX_Psi_f_beta = pFLUX->_V_F_FLUX_Psi_beta 
                                  - pPMSMa->_O_F_Ls * pPMSMe->_V_F_Ibeta;
    
    // ================== 步骤3：PLL角度跟踪 ==================
    MCFOC_EST_FLUX_PLL_F(pFLUX, pPMSMe, pPMSMa);
}
```

### 3.4 磁链漂移问题

**问题：** 纯积分器存在直流漂移问题。

**解决：** 使用低通滤波器代替纯积分器：

$$
\Psi = \frac{1}{s + \omega_c}(U - R \cdot I)
$$

---

## 4. 反电动势观测器(EMF)

### 4.1 数学原理

**反电动势观测器**直接估计反电动势，适用于高速运行。

**反电动势与角度关系：**

$$
\begin{cases}
e_\alpha = -\omega_e \psi_f \sin\theta \\
e_\beta = \omega_e \psi_f \cos\theta
\end{cases}
$$

其中：
- $e_\alpha, e_\beta$：αβ轴反电动势 ($V$)
- $\omega_e$：电角速度 ($rad/s$)
- $\psi_f$：永磁体磁链 ($Wb$)
- $\theta$：转子电角度 ($rad$)

**角度计算：**

$$
\theta = -\arctan\left(\frac{e_\alpha}{e_\beta}\right)
$$

其中：
- $\theta$：估计的转子电角度 ($rad$)
- $e_\alpha, e_\beta$：αβ轴反电动势 ($V$)

### 4.2 代码实现

```c
void MCFOC_EST_EMF_F(ST_EMF_CONTROL_F* pEMF, 
                      ST_PMSM_ELEC_F* pPMSMe, 
                      ST_PMSM_PARA_F* pPMSMa)
{
    // ================== 步骤1：反电动势估计 ==================
    // E = U - R*I - L*dI/dt
    pEMF->_V_F_EMF_Ealfa = pPMSMe->_V_F_Ualfa_Pre 
                         - pPMSMa->_O_F_Rs * pPMSMe->_V_F_Ialfa
                         - pPMSMa->_O_F_Ls * pEMF->_V_F_EMF_Di_alfa;
    pEMF->_V_F_EMF_Ebeta = pPMSMe->_V_F_Ubeta_Pre 
                         - pPMSMa->_O_F_Rs * pPMSMe->_V_F_Ibeta
                         - pPMSMa->_O_F_Ls * pEMF->_V_F_EMF_Di_beta;
    
    // ================== 步骤2：电流微分计算 ==================
    pEMF->_V_F_EMF_Di_alfa = (pPMSMe->_V_F_Ialfa - pEMF->_V_F_EMF_Ialfa_Last) 
                             / pPMSMa->_O_F_Ts;
    pEMF->_V_F_EMF_Di_beta = (pPMSMe->_V_F_Ibeta - pEMF->_V_F_EMF_Ibeta_Last) 
                             / pPMSMa->_O_F_Ts;
    
    // 保存当前电流
    pEMF->_V_F_EMF_Ialfa_Last = pPMSMe->_V_F_Ialfa;
    pEMF->_V_F_EMF_Ibeta_Last = pPMSMe->_V_F_Ibeta;
    
    // ================== 步骤3：PLL角度跟踪 ==================
    MCFOC_EST_EMF_PLL_F(pEMF, pPMSMe, pPMSMa);
}
```

---

## 5. PLL角度跟踪

### 5.1 数学原理

**PLL（锁相环）**用于平滑地跟踪角度和估计速度。

$$
\varepsilon = -\hat{e}_\alpha \cos\hat{\theta} - \hat{e}_\beta \sin\hat{\theta}
$$

其中：
- $\varepsilon$：PLL相位误差信号 ($V$)
- $\hat{e}_\alpha, \hat{e}_\beta$：滤波后的αβ轴反电动势估计值 ($V$)
- $\hat{\theta}$：估计的电角度 ($rad$)

**PI调节器：**

$$
\dot{\hat{\omega}} = K_i \cdot \varepsilon
$$

$$
\hat{\omega} = K_p \cdot \varepsilon + \int K_i \cdot \varepsilon \, dt
$$

其中：
- $\hat{\omega}$：估计的电角速度 ($rad/s$)
- $K_p$：PLL比例增益
- $K_i$：PLL积分增益
- $\varepsilon$：相位误差信号 ($V$)

**角度积分：**

$$
\hat{\theta} = \int \hat{\omega} \, dt
$$

其中：
- $\hat{\theta}$：估计的电角度 ($rad$)
- $\hat{\omega}$：估计的电角速度 ($rad/s$)

### 5.2 代码实现

```c
void MCFOC_EST_PLL_F(ST_SMO_CONTROL_F* pSMO, 
                      ST_PMSM_ELEC_F* pPMSMe, 
                      ST_PMSM_PARA_F* pPMSMa)
{
    // ================== 步骤1：计算PLL误差 ==================
    // ε = -Eα*cos(θ̂) - Eβ*sin(θ̂)
    pSMO->_V_F_SMO_PLL_Err = -pSMO->_V_F_SMO_Ealfa * pSMO->TG_SMO_Triangle_Est.F_Cos
                           - pSMO->_V_F_SMO_Ebeta * pSMO->TG_SMO_Triangle_Est.F_Sin;
    
    // ================== 步骤2：PI控制器 ==================
    PID_Pos_Cal_F(&pSMO->PID_SMO_PLL, pSMO->_V_F_SMO_PLL_Err, 0.0f);
    pSMO->_V_F_SMO_PLL_W = pSMO->PID_SMO_PLL.F_Out;
    
    // ================== 步骤3：角度积分 ==================
    pSMO->_V_F_SMO_PLL_Theta += pSMO->_V_F_SMO_PLL_W * pPMSMa->_O_F_Ts;
    
    // ================== 步骤4：角度归一化 ==================
    MATH_ANGLE_MOD_F(pSMO->_V_F_SMO_PLL_Theta);
    
    // ================== 步骤5：更新三角函数 ==================
    pSMO->TG_SMO_Triangle_Est.F_Angle = pSMO->_V_F_SMO_PLL_Theta;
    Math_SinCos_F(&pSMO->TG_SMO_Triangle_Est);
}
```

### 5.3 PLL参数整定

| 参数 | 计算公式 | 说明 |
|------|---------|------|
| Kp | $\frac{2\zeta\omega_n}{\psi_f}$ | 比例增益，$\zeta$为阻尼比（通常取0.707），$\omega_n$为自然频率 ($rad/s$)，$\psi_f$为永磁体磁链 ($Wb$) |
| Ki | $\frac{\omega_n^2}{\psi_f}$ | 积分增益，$\omega_n$为自然频率 ($rad/s$)，$\psi_f$为永磁体磁链 ($Wb$) |
| $\omega_n$ | $\omega_n = \frac{\omega_{bw}}{2\zeta}$ | 自然频率，$\omega_{bw}$为期望的PLL带宽 ($rad/s$) |

**典型值：**
- $\zeta = 0.707$（阻尼比）
- $\omega_n = 2\pi \times 10 \sim 50$ rad/s（自然频率）

---

## 6. 角度补偿机制

### 6.1 补偿来源

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          观测器角度补偿来源                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  1. 观测器固有延迟                                                       │
│     ├── SMO：低通滤波器相位滞后                                          │
│     ├── FLUX：积分器/低通滤波器滞后                                      │
│     └── EMF：电流微分计算延迟                                            │
│                                                                          │
│  2. 数字控制延迟                                                         │
│     ├── ADC采样延迟                                                      │
│     ├── 计算延迟                                                         │
│     └── PWM更新延迟                                                      │
│                                                                          │
│  3. 补偿方法                                                             │
│     └── 通过二维查表获得补偿角度                                         │
│         表索引：频率、电流                                               │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 6.2 补偿角度计算

```c
void MCFOC_EST_SMO_Adapt_F(ST_SMO_CONTROL_F* pSMO, 
                            ST_PMSM_ELEC_F* pPMSMe, 
                            ST_PMSM_PARA_F* pPMSMa)
{
    float F_Freq_tmp = MATH_ABS_F(pPMSMe->_O_F_Freq);
    float F_Is_tmp = MATH_ABS_F(pPMSMe->_O_F_Is);
    
    // 通过二维查表获得补偿角度
    pSMO->TG_SMO_Triangle_Comp.F_Angle = TABLE_2D_Inter_F(
        &pSMO->TAB_SMO_Angle_Comp, F_Freq_tmp, F_Is_tmp);
    
    // 角度归一化
    MATH_ANGLE_MOD_F(pSMO->TG_SMO_Triangle_Comp.F_Angle);
    
    // 计算sin/cos
    Math_SinCos_F(&pSMO->TG_SMO_Triangle_Comp);
}
```

### 6.3 补偿角度表示例

**SMO观测器补偿角度表（单位：弧度）：**

| 电流\频率 | 10Hz | 50Hz | 100Hz | 200Hz |
|----------|------|------|-------|-------|
| 1A | 0.02 | 0.08 | 0.15 | 0.25 |
| 5A | 0.03 | 0.10 | 0.18 | 0.28 |
| 10A | 0.04 | 0.12 | 0.20 | 0.30 |
| 20A | 0.05 | 0.15 | 0.22 | 0.32 |

---

## 7. 参数自适应

### 7.1 滑模增益自适应

```c
void MCFOC_EST_SMO_Gain_Adapt_F(ST_SMO_CONTROL_F* pSMO, 
                                 ST_PMSM_ELEC_F* pPMSMe, 
                                 ST_PMSM_PARA_F* pPMSMa)
{
    float F_Freq_tmp = MATH_ABS_F(pPMSMe->_O_F_Freq);
    float F_Es_tmp = MATH_ABS_F(pPMSMe->_O_F_Es);
    
    // H1自适应：根据反电动势大小调整
    // H1 > |E|，确保滑模可达
    pSMO->_P_F_SMO_H1 = 1.5f * F_Es_tmp + 5.0f;
    
    // H2自适应：根据频率调整
    // 低速时减小H2，减小抖振
    if(F_Freq_tmp < 20.0f)
    {
        pSMO->_P_F_SMO_H2 = 100.0f;
    }
    else if(F_Freq_tmp < 50.0f)
    {
        pSMO->_P_F_SMO_H2 = 200.0f;
    }
    else
    {
        pSMO->_P_F_SMO_H2 = 500.0f;
    }
}
```

### 7.2 PLL参数自适应

```c
void MCFOC_EST_PLL_Adapt_F(ST_SMO_CONTROL_F* pSMO, 
                            ST_PMSM_ELEC_F* pPMSMe, 
                            ST_PMSM_PARA_F* pPMSMa)
{
    float F_Freq_tmp = MATH_ABS_F(pPMSMe->_O_F_Freq);
    
    // 低速时降低PLL带宽，提高稳定性
    if(F_Freq_tmp < 20.0f)
    {
        pSMO->PID_SMO_PLL._P_F_Kp = 500.0f;
        pSMO->PID_SMO_PLL._P_F_Ki = 5000.0f;
    }
    else if(F_Freq_tmp < 50.0f)
    {
        pSMO->PID_SMO_PLL._P_F_Kp = 1000.0f;
        pSMO->PID_SMO_PLL._P_F_Ki = 10000.0f;
    }
    else
    {
        pSMO->PID_SMO_PLL._P_F_Kp = 2000.0f;
        pSMO->PID_SMO_PLL._P_F_Ki = 20000.0f;
    }
}
```

---

## 8. API参考

### 8.1 初始化函数

```c
void MCFOC_EST_SMO_Init_F(ST_SMO_CONTROL_F* pSMO);
void MCFOC_EST_FLUX_Init_F(ST_FLUX_CONTROL_F* pFLUX);
void MCFOC_EST_EMF_Init_F(ST_EMF_CONTROL_F* pEMF);
```

**功能：** 初始化观测器结构体

### 8.2 观测器计算函数

```c
void MCFOC_EST_SMO_F(ST_SMO_CONTROL_F* pSMO, 
                      ST_PMSM_ELEC_F* pPMSMe, 
                      ST_PMSM_PARA_F* pPMSMa);

void MCFOC_EST_FLUX_F(ST_FLUX_CONTROL_F* pFLUX, 
                       ST_PMSM_ELEC_F* pPMSMe, 
                       ST_PMSM_PARA_F* pPMSMa);

void MCFOC_EST_EMF_F(ST_EMF_CONTROL_F* pEMF, 
                      ST_PMSM_ELEC_F* pPMSMe, 
                      ST_PMSM_PARA_F* pPMSMa);
```

**调用时机：** 在电流环周期调用（通常100μs）

### 8.3 参数自适应函数

```c
void MCFOC_EST_SMO_Adapt_F(ST_SMO_CONTROL_F* pSMO, 
                            ST_PMSM_ELEC_F* pPMSMe, 
                            ST_PMSM_PARA_F* pPMSMa);
```

**调用时机：** 在速度环周期调用（通常1ms）

---

## 总结

### 核心知识点

1. **SMO观测器**：通过滑模控制律提取反电动势，适用于全速域
2. **FLUX观测器**：通过磁链积分估计角度，低速性能好
3. **EMF观测器**：直接估计反电动势，高速性能好
4. **PLL角度跟踪**：平滑地跟踪角度和估计速度
5. **角度补偿**：根据频率和电流自适应补偿延迟

### 关键参数配置

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `_P_F_SMO_H1` | 10~50 | 滑模增益H1 |
| `_P_F_SMO_H2` | 100~500 | 滑模增益H2 |
| `LPF_SMO_Ealfa.F_Cutoff` | 50~200Hz | 反电动势滤波截止频率 |
| `PID_SMO_PLL._P_F_Kp` | 500~2000 | PLL比例增益 |
| `PID_SMO_PLL._P_F_Ki` | 5000~20000 | PLL积分增益 |

### 下一步

- [MC-LIB-Control-Loop](MC-LIB-Control-Loop.md)：深入分析电流环和速度环
- [MC-LIB-Porting-Guide](MC-LIB-Porting-Guide.md)：学习如何移植和使用

---

## 🆚 与 hpm_MC 观测器对比

| 维度 | MC_LIB | hpm_MCL |
|------|--------|---------|
| **滑模观测器(SMO)** | `MCFOC_EST_SMO_F()` | v1: `hpm_smc_para_t` + `hpm_smc_pll_para_t`; v2: `mcl_control_smc_cfg_t` |
| **磁链观测器(FLUX)** | `MCFOC_EST_FLUX_F()` | 未实现（v2 用编码器融合替代） |
| **反电动势观测器(EMF)** | `MCFOC_EST_EMF_F()` | v1 over_zero 过零检测（方波BLDC） |
| **高频注入(HFI)** | 未实现 | v1: `hpm_hfi_para_t` 方波电压注入（凸极电机专用） |
| **PLL锁相环** | 集成在 SMO/FLUX 内部 | v1: 独立 `hpm_smc_pll_para_t`; v2: encoder 内置 PLL 速度计算 |
| **编码器融合** | 无 | v2: T/M/MT/PLL 四种速度方法 + theta_forecast |
| **零/低速策略** | IF拖动 | HFI（高频注入）+ 编码器混合 |

参考: `算法/ALG-01-Sensorless-Observers.md` + `算法/HPM-MC/SDK-01-HPM-MC-Architecture.md`

---

*文档更新时间: 2026-04-26*
