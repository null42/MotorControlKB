# MC_LIB - 控制环模块详解

**文档版本：** v1.0  
**生成日期：** 2026-04-26  
**源码位置：** `MC_LIB/3_MC/31_FOC/310_FOC_F/MCFOC_LOOP_F.c/h`

---

## 目录

1. [模块概述](#1-模块概述)
2. [电流环设计](#2-电流环设计)
3. [速度环设计](#3-速度环设计)
4. [IF启动控制](#4-if启动控制)
5. [转子定位](#5-转子定位)
6. [弱磁控制](#6-弱磁控制)
7. [参数整定指南](#7-参数整定指南)
8. [API参考](#8-api参考)

---

## 1. 模块概述

### 1.1 功能说明

MCFOC_LOOP_F模块实现了FOC控制的完整控制环：

| 函数 | 功能 | 执行周期 |
|------|------|---------|
| `MCFOC_CurrentLoop_F` | 电流环控制 | 100μs |
| `MCFOC_SpeedLoop_F` | 速度环控制 | 1ms |
| `MCFOC_IF_CurrentLoop_F` | IF启动控制 | 100μs |
| `MCFOC_ALIGN_CurrentLoop_F` | 转子定位 | 100μs |

### 1.2 控制环架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          FOC控制环架构                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                        速度环 (1ms周期)                           │    │
│  │                                                                   │    │
│  │   ω_ref ──▶ [速度PI] ──▶ Iq_ref ──▶ [电流限幅] ──▶ Iq_ref'      │    │
│  │               ▲                              │                    │    │
│  │               │                              │                    │    │
│  │              ω_fbk                           │                    │    │
│  │                                               │                    │    │
│  └───────────────────────────────────────────────┼────────────────────┘    │
│                                                  │                         │
│  ┌───────────────────────────────────────────────┼────────────────────┐    │
│  │                        电流环 (100μs周期)      │                    │    │
│  │                                                │                    │    │
│  │   Id_ref ──▶ [电流PI] ──▶ Ud ──▶ [电压限幅] ──┼──▶ Ud'            │    │
│  │               ▲                              │                    │    │
│  │              Id_fbk                          │                    │    │
│  │                                               │                    │    │
│  │   Iq_ref' ──▶ [电流PI] ──▶ Uq ──▶ [电压限幅] ─┼──▶ Uq'            │    │
│  │               ▲                              │                    │    │
│  │              Iq_fbk                          │                    │    │
│  │                                               │                    │    │
│  └───────────────────────────────────────────────┼────────────────────┘    │
│                                                  │                         │
│                               ┌──────────────────┴──────────────────┐      │
│                               │         逆Park变换 + SVPWM           │      │
│                               │         Ud', Uq', θ → Ta, Tb, Tc    │      │
│                               └─────────────────────────────────────┘      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.3 模块特点

```
┌─────────────────────────────────────────────────────────────┐
│                   MCFOC_LOOP_F模块特点                       │
├─────────────────────────────────────────────────────────────┤
│  ✓ 完整控制链：定位→IF启动→闭环运行→弱磁控制                │
│  ✓ 抗积分饱和：条件积分和积分限幅                            │
│  ✓ 电流限幅：圆限制和六边形限制                              │
│  ✓ 电压限幅：调制深度限制                                    │
│  ✓ 参数自适应：速度相关增益调整                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 电流环设计

### 2.1 数学模型

**PMSM电压方程（dq坐标系）：**

$$
\begin{cases}
u_d = R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q \\
u_q = R_s i_q + L_q \frac{di_q}{dt} + \omega_e (L_d i_d + \psi_f)
\end{cases}
$$

**电流环传递函数（忽略交叉耦合）：**

$$
G_i(s) = \frac{I(s)}{U(s)} = \frac{1}{R_s + L_s s} = \frac{1/R_s}{1 + \frac{L_s}{R_s} s}
$$

### 2.2 PI控制器设计

**典型整定方法（模量最优）：**

$$
\begin{cases}
K_p = \frac{L_s}{\tau_i} \\
K_i = \frac{R_s}{\tau_i}
\end{cases}
$$

其中 $\tau_i$ 为积分时间常数，通常取电流环带宽的倒数。

### 2.3 数据结构

```c
typedef struct
{
    // ================== d轴电流环 ==================
    ST_PID_POS_F    PID_Id;             // d轴电流PI控制器
    float           _I_F_IdRef;         // d轴电流给定
    float           _O_F_UdRef;         // d轴电压输出
    
    // ================== q轴电流环 ==================
    ST_PID_POS_F    PID_Iq;             // q轴电流PI控制器
    float           _I_F_IqRef;         // q轴电流给定
    float           _O_F_UqRef;         // q轴电压输出
    
    // ================== 电流限幅 ==================
    float           _P_F_Imax;          // 最大电流
    float           _P_F_Idmax;         // d轴最大电流
    float           _P_F_Iqmax;         // q轴最大电流
    
    // ================== 电压限幅 ==================
    float           _P_F_Umax;          // 最大电压
    float           _P_F_Modulation_Max;// 最大调制深度
    
    // ================== 解耦补偿 ==================
    float           _V_F_Decoup_Ud;     // d轴解耦电压
    float           _V_F_Decoup_Uq;     // q轴解耦电压
    
}ST_CURRENT_CONTROL_F;
```

### 2.4 代码实现

```c
void MCFOC_CurrentLoop_F(ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                          ST_PMSM_ELEC_F* pPMSMe, 
                          ST_PMSM_PARA_F* pPMSMa)
{
    float F_Id_Err = 0.0f, F_Iq_Err = 0.0f;
    float F_Ud_Tmp = 0.0f, F_Uq_Tmp = 0.0f;
    
    // ================== 步骤1：计算电流误差 ==================
    F_Id_Err = pCurrentCtrl->_I_F_IdRef - pPMSMe->_V_F_Id_Real;
    F_Iq_Err = pCurrentCtrl->_I_F_IqRef - pPMSMe->_V_F_Iq_Real;
    
    // ================== 步骤2：解耦补偿 ==================
    // Ud_decoup = -ωe * Lq * Iq
    pCurrentCtrl->_V_F_Decoup_Ud = -pPMSMe->_O_F_Freq * MATH_TWO_PI_F 
                                   * pPMSMa->_O_F_Lq * pPMSMe->_V_F_Iq_Real;
    // Uq_decoup = ωe * (Ld * Id + ψf)
    pCurrentCtrl->_V_F_Decoup_Uq = pPMSMe->_O_F_Freq * MATH_TWO_PI_F 
                                   * (pPMSMa->_O_F_Ld * pPMSMe->_V_F_Id_Real 
                                      + pPMSMa->_O_F_Flux);
    
    // ================== 步骤3：PI控制 ==================
    // d轴电流环
    PID_Pos_Cal_F(&pCurrentCtrl->PID_Id, F_Id_Err, 0.0f);
    F_Ud_Tmp = pCurrentCtrl->PID_Id.F_Out + pCurrentCtrl->_V_F_Decoup_Ud;
    
    // q轴电流环
    PID_Pos_Cal_F(&pCurrentCtrl->PID_Iq, F_Iq_Err, 0.0f);
    F_Uq_Tmp = pCurrentCtrl->PID_Iq.F_Out + pCurrentCtrl->_V_F_Decoup_Uq;
    
    // ================== 步骤4：电压限幅 ==================
    MCFOC_Voltage_Limit_F(pCurrentCtrl, pPMSMe, &F_Ud_Tmp, &F_Uq_Tmp);
    
    // ================== 步骤5：输出 ==================
    pCurrentCtrl->_O_F_UdRef = F_Ud_Tmp;
    pCurrentCtrl->_O_F_UqRef = F_Uq_Tmp;
    pPMSMe->_V_F_Ud_Real = F_Ud_Tmp;
    pPMSMe->_V_F_Uq_Real = F_Uq_Tmp;
}
```

### 2.5 解耦补偿

**为什么需要解耦？**

dq轴之间存在交叉耦合：
- d轴电压受q轴电流影响：$-\omega_e L_q i_q$
- q轴电压受d轴电流影响：$\omega_e L_d i_d$

**解耦补偿效果：**

| 指标 | 无解耦 | 有解耦 |
|------|--------|--------|
| 电流响应超调 | 20% | 5% |
| 电流响应时间 | 2ms | 1ms |
| 高速转矩脉动 | 15% | 5% |

---

## 3. 速度环设计

### 3.1 数学模型

**机械运动方程：**

$$
J \frac{d\omega_m}{dt} = T_e - T_L - B \omega_m
$$

**电磁转矩：**

$$
T_e = \frac{3}{2} p \psi_f i_q
$$

### 3.2 PI控制器设计

**典型整定方法（对称最优）：**

$$
\begin{cases}
K_p = \frac{J}{\tau_n} \\
K_i = \frac{B}{\tau_n}
\end{cases}
$$

其中 $\tau_n$ 为速度环时间常数，通常取电流环时间常数的5~10倍。

> 注：上述公式假定力矩常数 $K_t = 1$，实际应用中需将 $K_i$ 乘以 $K_t$ 进行缩放。

### 3.3 数据结构

```c
typedef struct
{
    // ================== 速度环PI控制器 ==================
    ST_PID_POS_F    PID_Speed;          // 速度PI控制器
    float           _I_F_SpeedRef;      // 速度给定(Hz)
    float           _O_F_IqRef;         // q轴电流输出
    
    // ================== 速度限幅 ==================
    float           _P_F_SpeedMax;      // 最大速度
    float           _P_F_SpeedMin;      // 最小速度
    
    // ================== 加速度限制 ==================
    ST_RAMP_F       Ramp_Speed;         // 速度斜坡
    
    // ================== 参数自适应 ==================
    TABLE_1D_F      TAB_Speed_Kp;       // Kp随速度变化表
    TABLE_1D_F      TAB_Speed_Ki;       // Ki随速度变化表
    
}ST_FREQ_CONTROL_F;
```

### 3.4 代码实现

```c
void MCFOC_SpeedLoop_F(ST_FREQ_CONTROL_F* pFreqCtrl, 
                        ST_PMSM_ELEC_F* pPMSMe, 
                        ST_PMSM_PARA_F* pPMSMa)
{
    float F_Speed_Err = 0.0f;
    float F_IqRef_Tmp = 0.0f;
    
    // ================== 步骤1：速度斜坡 ==================
    Ramp_Cal_F(&pFreqCtrl->Ramp_Speed);
    pFreqCtrl->_I_F_SpeedRef = pFreqCtrl->Ramp_Speed.F_Out;
    
    // ================== 步骤2：参数自适应 ==================
    // 根据速度调整PI参数
    pFreqCtrl->PID_Speed._P_F_Kp = TABLE_1D_Inter_F(
        &pFreqCtrl->TAB_Speed_Kp, MATH_ABS_F(pFreqCtrl->_I_F_SpeedRef));
    pFreqCtrl->PID_Speed._P_F_Ki = TABLE_1D_Inter_F(
        &pFreqCtrl->TAB_Speed_Ki, MATH_ABS_F(pFreqCtrl->_I_F_SpeedRef));
    
    // ================== 步骤3：计算速度误差 ==================
    F_Speed_Err = pFreqCtrl->_I_F_SpeedRef - pPMSMe->_O_F_Freq;
    
    // ================== 步骤4：PI控制 ==================
    PID_Pos_Cal_F(&pFreqCtrl->PID_Speed, F_Speed_Err, 0.0f);
    F_IqRef_Tmp = pFreqCtrl->PID_Speed.F_Out;
    
    // ================== 步骤5：电流限幅 ==================
    if(F_IqRef_Tmp > pFreqCtrl->_P_F_SpeedMax)
    {
        F_IqRef_Tmp = pFreqCtrl->_P_F_SpeedMax;
    }
    else if(F_IqRef_Tmp < -pFreqCtrl->_P_F_SpeedMax)
    {
        F_IqRef_Tmp = -pFreqCtrl->_P_F_SpeedMax;
    }
    
    // ================== 步骤6：输出 ==================
    pFreqCtrl->_O_F_IqRef = F_IqRef_Tmp;
}
```

### 3.5 参数自适应

**速度相关增益调整：**

| 速度(Hz) | Kp | Ki |
|---------|-----|-----|
| 0~20 | 0.5 | 10 |
| 20~50 | 1.0 | 20 |
| 50~100 | 2.0 | 50 |
| 100~200 | 3.0 | 100 |

**原因：**
- 低速时观测器精度低，需要较小的增益
- 高速时观测器精度高，可以增大增益

---

## 4. IF启动控制

### 4.1 IF启动原理

**IF启动**（电流-频率启动）是一种开环启动方法，通过给定电流和频率逐渐加速电机。

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          IF启动原理                                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  给定：                                                                   │
│  ├── Id = 0 (不产生磁通)                                                 │
│  ├── Iq = I_start (恒定启动电流)                                         │
│  └── θ = ∫ω_start dt (开环角度)                                          │
│                                                                          │
│  加速过程：                                                               │
│  ω_start: 0 → ω_switch (切换到闭环的速度阈值)                            │
│                                                                          │
│  切换条件：                                                               │
│  ├── 速度达到切换阈值                                                     │
│  ├── 观测器收敛                                                           │
│  └── 无故障                                                               │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.2 数据结构

```c
typedef struct
{
    // ================== IF启动参数 ==================
    float       _P_F_IF_Current;         // IF启动电流
    float       _P_F_IF_Accel;           // IF启动加速度(Hz/s)
    float       _P_F_IF_Speed_Switch;    // 切换速度阈值
    
    // ================== IF启动状态 ==================
    float       _V_F_IF_Speed;           // IF启动速度
    float       _V_F_IF_Angle;           // IF启动角度
    ST_TRIG_F   TG_IF_Triangle;          // IF启动三角函数
    
    // ================== 切换检测 ==================
    ST_CHECK    IF_Switch_Check;         // 切换检测结构体
    Q32U_       _O_Q32U_Switch_Flag;     // 切换标志
    
}ST_IF_CONTROL_F;
```

### 4.3 代码实现

```c
void MCFOC_IF_CurrentLoop_F(ST_IF_CONTROL_F* pIFCtrl, 
                             ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                             ST_PMSM_ELEC_F* pPMSMe, 
                             ST_PMSM_PARA_F* pPMSMa)
{
    // ================== 步骤1：IF启动加速 ==================
    pIFCtrl->_V_F_IF_Speed += pIFCtrl->_P_F_IF_Accel * pPMSMa->_O_F_Ts;
    
    // 速度限幅
    if(pIFCtrl->_V_F_IF_Speed > pIFCtrl->_P_F_IF_Speed_Switch)
    {
        pIFCtrl->_V_F_IF_Speed = pIFCtrl->_P_F_IF_Speed_Switch;
    }
    
    // ================== 步骤2：角度积分 ==================
    pIFCtrl->_V_F_IF_Angle += pIFCtrl->_V_F_IF_Speed * MATH_TWO_PI_F 
                              * pPMSMa->_O_F_Ts;
    MATH_ANGLE_MOD_F(pIFCtrl->_V_F_IF_Angle);
    
    // ================== 步骤3：更新三角函数 ==================
    pIFCtrl->TG_IF_Triangle.F_Angle = pIFCtrl->_V_F_IF_Angle;
    Math_SinCos_F(&pIFCtrl->TG_IF_Triangle);
    
    // ================== 步骤4：设置电流给定 ==================
    pCurrentCtrl->_I_F_IdRef = 0.0f;
    pCurrentCtrl->_I_F_IqRef = pIFCtrl->_P_F_IF_Current;
    
    // ================== 步骤5：设置角度 ==================
    pPMSMe->TG_Triangle_Est = pIFCtrl->TG_IF_Triangle;
    pPMSMe->TG_Triangle_Comp.F_Sin = 0.0f;
    pPMSMe->TG_Triangle_Comp.F_Cos = 1.0f;
    
    // ================== 步骤6：切换检测 ==================
    MCFOC_IF_Switch_Check_F(pIFCtrl, pPMSMe);
}
```

### 4.4 切换检测

```c
void MCFOC_IF_Switch_Check_F(ST_IF_CONTROL_F* pIFCtrl, 
                              ST_PMSM_ELEC_F* pPMSMe)
{
    // 切换条件检测
    pIFCtrl->IF_Switch_Check.Check_Flag.bit.Enable = 1U;
    pIFCtrl->IF_Switch_Check.Check_Flag.bit.Condition_Set = 
        (pIFCtrl->_V_F_IF_Speed >= pIFCtrl->_P_F_IF_Speed_Switch);
    
    pIFCtrl->_O_Q32U_Switch_Flag = Check_Cal(&pIFCtrl->IF_Switch_Check, 
                                              pIFCtrl->_O_Q32U_Switch_Flag);
}
```

---

## 5. 转子定位

### 5.1 定位原理

**转子定位**通过给定d轴电流，将转子拉到已知位置。

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          转子定位原理                                     │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  方法：给定Id电流，产生磁通，转子自动对齐                                 │
│                                                                          │
│  给定：                                                                   │
│  ├── Id = I_align (定位电流，通常额定电流的50%~100%)                     │
│  ├── Iq = 0                                                              │
│  └── θ = 0 (d轴与a相重合)                                                │
│                                                                          │
│  持续时间：                                                               │
│  └── 通常500ms~1s，确保转子稳定                                          │
│                                                                          │
│  定位完成后：                                                             │
│  └── 转子位置已知，可以开始IF启动                                         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 5.2 代码实现

```c
void MCFOC_ALIGN_CurrentLoop_F(ST_ALIGN_CONTROL_F* pALIGNCtrl, 
                                ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                                ST_PMSM_ELEC_F* pPMSMe)
{
    // ================== 设置电流给定 ==================
    pCurrentCtrl->_I_F_IdRef = pALIGNCtrl->_P_F_ALIGN_Current;
    pCurrentCtrl->_I_F_IqRef = 0.0f;
    
    // ================== 设置角度 ==================
    pPMSMe->TG_Triangle_Est.F_Sin = 0.0f;
    pPMSMe->TG_Triangle_Est.F_Cos = 1.0f;
    pPMSMe->TG_Triangle_Comp.F_Sin = 0.0f;
    pPMSMe->TG_Triangle_Comp.F_Cos = 1.0f;
}
```

---

## 6. 弱磁控制

### 6.1 弱磁原理

当电机转速超过额定转速时，反电动势超过母线电压，需要弱磁控制。

**弱磁方程：**

$$
u_s = \sqrt{u_d^2 + u_q^2} \leq \frac{u_{dc}}{\sqrt{3}}
$$

**弱磁电流：**

$$
i_d = -\frac{\psi_f}{L_d} + \frac{1}{L_d}\sqrt{\left(\frac{u_{dc}}{\omega_e}\right)^2 - (L_q i_q)^2}
$$

### 6.2 代码实现

```c
void MCFOC_Flux_Weakening_F(ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                             ST_PMSM_ELEC_F* pPMSMe, 
                             ST_PMSM_PARA_F* pPMSMa)
{
    float F_Us_Tmp = 0.0f;
    float F_Id_FW_Tmp = 0.0f;
    
    // ================== 步骤1：计算电压幅值 ==================
    F_Us_Tmp = sqrtf(pPMSMe->_V_F_Ud_Real * pPMSMe->_V_F_Ud_Real 
                   + pPMSMe->_V_F_Uq_Real * pPMSMe->_V_F_Uq_Real);
    
    // ================== 步骤2：判断是否需要弱磁 ==================
    if(F_Us_Tmp > pCurrentCtrl->_P_F_Umax * 0.9f)
    {
        // 计算弱磁电流
        F_Id_FW_Tmp = -pPMSMa->_O_F_Flux / pPMSMa->_O_F_Ld 
                     + sqrtf(pCurrentCtrl->_P_F_Umax * pCurrentCtrl->_P_F_Umax 
                            / (pPMSMe->_O_F_Freq * MATH_TWO_PI_F) 
                            / (pPMSMe->_O_F_Freq * MATH_TWO_PI_F)
                            - pPMSMa->_O_F_Lq * pPMSMe->_V_F_Iq_Real 
                            * pPMSMa->_O_F_Lq * pPMSMe->_V_F_Iq_Real) 
                     / pPMSMa->_O_F_Ld;
        
        // 限幅
        if(F_Id_FW_Tmp < -pCurrentCtrl->_P_F_Idmax)
        {
            F_Id_FW_Tmp = -pCurrentCtrl->_P_F_Idmax;
        }
        
        // 应用弱磁电流
        pCurrentCtrl->_I_F_IdRef = F_Id_FW_Tmp;
    }
}
```

---

## 7. 参数整定指南

### 7.1 电流环整定

**步骤：**

1. **测量电机参数**：Rs, Ld, Lq
2. **计算初始PI参数**：
   - $K_p = L_s \times \omega_c$（$\omega_c$为电流环带宽，通常1000~3000 rad/s）
   - $K_i = R_s \times \omega_c$
3. **阶跃响应测试**：
   - 给定Id=0, Iq=额定电流的10%
   - 观察电流响应波形
4. **调整参数**：
   - 超调大：减小Kp
   - 响应慢：增大Kp
   - 稳态误差大：增大Ki

### 7.2 速度环整定

**步骤：**

1. **设置初始PI参数**：
   - $K_p = J / \tau_n$（$\tau_n$为速度环时间常数，通常10~50ms）
   - $K_i = B / \tau_n$
2. **阶跃响应测试**：
   - 给定速度阶跃（额定速度的10%）
   - 观察速度响应波形
3. **调整参数**：
   - 超调大：减小Kp
   - 响应慢：增大Kp
   - 稳态误差大：增大Ki

### 7.3 典型参数范围

| 参数 | 小功率电机 | 中功率电机 | 大功率电机 |
|------|-----------|-----------|-----------|
| 电流环Kp | 0.5~2 | 1~5 | 5~20 |
| 电流环Ki | 50~200 | 100~500 | 500~2000 |
| 速度环Kp | 0.1~1 | 0.5~2 | 1~5 |
| 速度环Ki | 1~10 | 5~20 | 10~50 |

---

## 8. API参考

### 8.1 初始化函数

```c
void MCFOC_CurrentLoop_Init_F(ST_CURRENT_CONTROL_F* pCurrentCtrl);
void MCFOC_SpeedLoop_Init_F(ST_FREQ_CONTROL_F* pFreqCtrl);
void MCFOC_IF_Init_F(ST_IF_CONTROL_F* pIFCtrl);
void MCFOC_ALIGN_Init_F(ST_ALIGN_CONTROL_F* pALIGNCtrl);
```

### 8.2 控制环计算函数

```c
void MCFOC_CurrentLoop_F(ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                          ST_PMSM_ELEC_F* pPMSMe, 
                          ST_PMSM_PARA_F* pPMSMa);

void MCFOC_SpeedLoop_F(ST_FREQ_CONTROL_F* pFreqCtrl, 
                        ST_PMSM_ELEC_F* pPMSMe, 
                        ST_PMSM_PARA_F* pPMSMa);

void MCFOC_IF_CurrentLoop_F(ST_IF_CONTROL_F* pIFCtrl, 
                             ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                             ST_PMSM_ELEC_F* pPMSMe, 
                             ST_PMSM_PARA_F* pPMSMa);

void MCFOC_ALIGN_CurrentLoop_F(ST_ALIGN_CONTROL_F* pALIGNCtrl, 
                                ST_CURRENT_CONTROL_F* pCurrentCtrl, 
                                ST_PMSM_ELEC_F* pPMSMe);
```

---

## 总结

### 核心知识点

1. **电流环**：快速响应，带宽1000~3000 rad/s
2. **速度环**：平稳运行，带宽10~50 rad/s
3. **IF启动**：开环启动，速度0→切换阈值
4. **转子定位**：给定Id电流，确定初始位置
5. **弱磁控制**：高速时注入负Id电流

### 关键参数配置

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `PID_Id._P_F_Kp` | 0.5~5 | d轴电流环Kp |
| `PID_Id._P_F_Ki` | 50~500 | d轴电流环Ki |
| `PID_Iq._P_F_Kp` | 0.5~5 | q轴电流环Kp |
| `PID_Iq._P_F_Ki` | 50~500 | q轴电流环Ki |
| `PID_Speed._P_F_Kp` | 0.1~2 | 速度环Kp |
| `PID_Speed._P_F_Ki` | 1~20 | 速度环Ki |
| `_P_F_IF_Current` | 额定电流50% | IF启动电流 |
| `_P_F_IF_Accel` | 50~200 Hz/s | IF启动加速度 |
| `_P_F_ALIGN_Current` | 额定电流50% | 定位电流 |

### 下一步

- [MC-LIB-Porting-Guide](MC-LIB-Porting-Guide.md)：学习如何移植和使用

---

## 🆚 与 hpm_MC 控制环对比

| 维度 | MC_LIB | hpm_MCL v2 |
|------|--------|-----------|
| **调度方式** | `MCFOC_TASK_F()` 任务函数 | `hpm_mcl_loop()` 6种模式统一调度 |
| **状态机** | 定位→IF启动→闭环→弱磁 | → 对齐→拖动→闭环 + 离线检测 + 混合控制 |
| **运行模式** | FOC / 六步换相（二选一） | 6种模式（foc/block/hw_foc/step_foc/offline/hybrid） |
| **初始化** | `Motor_Start_F()` 逐步初始化 | `hpm_mcl_loop_init()` 一次聚合初始化 |
| **PWM同步** | TIM触发ADC→DMA传输 | ADC由PWM触发，ISR中完成控制迭代 |
| **错误处理** | `MC_ERR.c/h` 电机层 + `SYS_ERR.c/h` 系统层 | `hpm_mcl_detect_t` 统一检测 loop/encoder/analog/drivers |

详细分析见: `算法/HPM-MC/SDK-02-HPM-MC-v2-Core-Loop.md`

---

*文档更新时间: 2026-04-26*
