# ADV-HW-02 ADC深度配置与DMA数据搬运：从寄存器到控制环的精密桥梁

**模块编号：** ADV-HW-02
**模块名称：** ADC深度配置与DMA数据搬运（ADC Deep Configuration & DMA Data Transfer）
**文档版本：** v2.0
**适用对象：** 已掌握基础ADC/DMA概念和FOC控制环路的嵌入式工程师
**前置知识：** HW-02 电流采样电路、HW-04 MCU外设与通信、ADV-HW-01 PWM深度配置
**关联模块：** ADV-HW-01 PWM深度配置与电流采样时序联动
**副标题：** 注入组、PWM触发、DMA双缓冲——理解电流采样的完整数据通路
**难度等级：** ★★★★☆

---

## 1. 核心摘要

**一句话讲清楚**：ADC不是"读一个值"那么简单——注入组保证时序确定性，PWM触发保证采样时刻精确，DMA双缓冲保证数据搬运零CPU开销，数据对齐保证算法精度。整条链路任何一个环节配置错误，电流环性能直接崩塌。

**认知挂钩**：很多工程师以为ADC配置就是CubeMX点几下，**这是严重误区！** 电机控制中的ADC配置是一个精密的时序系统：PWM中心对齐模式下的特定时刻触发ADC，注入组打断规则组保证电流采样优先级，DMA在后台搬运数据而CPU专注FOC计算。这不仅仅是"配置外设"，而是**构建一条从物理电流到算法输入的确定性数据通路**。

**与算法的关联**：
- ADC分辨率与量化误差 → 电流环控制精度下限 → 1 LSB误差对应多少安培？
- 采样时刻精度 → 电流重建准确性 → 采样偏移导致d/q轴交叉耦合
- 注入组优先级 → 控制周期确定性 → 规则组不能打断电流采样
- DMA双缓冲 → CPU与数据搬运并行 → 零拷贝减少延迟
- 数据对齐与标幺值 → 算法数值稳定性 → Q15格式直接用于定点PID

**前置知识**：
- HW-02 电流采样电路（信号链：电流→电压→放大→滤波→ADC输入）
- HW-04 MCU外设与通信（PWM基本配置、ADC基本概念）
- ADV-HW-01 PWM深度配置（中心对齐模式、TRGO机制）

---

## 2. 问题引入

### 工程师的真实困惑

**场景1：电流采样值跳变**
```
工程师A:"我的电流采样值偶尔会跳变，导致电流环输出尖峰..."
问题现象:
- ADC值偶尔出现异常跳变
- 电流波形有随机毛刺
- 调PI参数无法消除
根因: ADC触发时序不正确，采样时刻落在PWM开关切换瞬间
```

**场景2：三相电流之和不等于零**
```
工程师B:"我采样了三相电流，但Ia+Ib+Ic不等于零，差了0.3A..."
问题现象:
- KCL约束不满足
- Id始终有偏置
- 低速时电流环不稳定
根因: 三相ADC未同步采样，采样时刻存在时间差
```

**场景3：DMA数据偶尔错位**
```
工程师C:"用DMA搬运ADC数据，偶尔数据会错位一个通道..."
问题现象:
- 电流值和温度值交换了
- 偶尔出现，重启后可能正常
- 高速运行时更容易出现
根因: DMA配置不当，未正确处理半传输/全传输中断
```

**场景4：控制周期不稳定**
```
工程师D:"我的FOC计算应该在PWM周期内完成，但偶尔会超时..."
问题现象:
- 控制周期有抖动
- 电流环偶尔失控
- 通信任务运行时更明显
根因: ADC中断优先级不够高，被其他中断抢占
```

### 核心问题

这些问题的根本原因是什么？

**答案**：ADC-DMA-PWM联动配置存在缺陷！

- 采样值跳变 → ADC触发时序与PWM开关切换冲突
- 三相不同步 → 双ADC同步模式未正确配置
- DMA错位 → 双缓冲机制理解错误
- 周期抖动 → 中断优先级安排不合理

### 学习目标

读完本模块，你将能够：

- 理解ADC分辨率、量化误差与电流测量精度的关系
- 掌握注入组与规则组的区别及电机控制中的正确用法
- 精确配置PWM触发ADC的完整触发链
- 理解DMA Circular模式与双缓冲机制
- 完成从PWM→TRGO→ADC→DMA→中断→FOC的完整联动配置
- 直接将配置应用于STM32G474生产项目

---

## 3. ADC分辨率与量化误差

### 3.1 12位ADC的基本参数

STM32G474内置ADC为12位逐次逼近型（SAR ADC），量化级数为：

$$N_{level} = 2^{12} = 4096$$

量化步长（LSB，Least Significant Bit）：

$$V_{LSB} = \frac{V_{REF}}{2^{12}} = \frac{V_{REF}}{4096}$$

对于常见的 $V_{REF} = 3.3V$：

$$V_{LSB} = \frac{3.3}{4096} \approx 0.806 \text{ mV}$$

### 3.2 有效位数（ENOB）

**标称12位不等于真正12位精度！** 实际有效位数（ENOB, Effective Number of Bits）受以下因素影响：

$$ENOB = \frac{SINAD - 1.76}{6.02}$$

其中SINAD（Signal-to-Noise And Distortion）是ADC的实际信噪失真比。

影响ENOB的关键因素：

| 因素 | 影响 | 典型值 |
|------|------|--------|
| INL（积分非线性） | 全量程最大偏差 | STM32G474: +/-1 LSB |
| DNL（差分非线性） | 相邻码宽偏差 | STM32G474: +/-0.5 LSB |
| 量化噪声 | 固有噪声 | $\pm 0.5$ LSB |
| 电路噪声 | PCB布局、电源纹波 | 视设计而定 |
| 时钟抖动 | 采样时刻不确定性 | ADCCLK抖动 |

**工程经验**：在良好PCB设计下，STM32G474的ADC通常能达到10.5~11.5 ENOB。这意味着实际可用精度约为10~11位，而非标称的12位。

### 3.3 电流采样精度要求

FOC控制对电流采样精度的典型要求：

$$\Delta I_{error} < 1\% \times I_{rated}$$

以一个额定电流 $I_{rated} = 10A$ 的电机为例：

$$\Delta I_{error} < 0.1A$$

**分辨率与电流测量范围的关系**：

假设电流采样电路的参数：
- 采样电阻 $R_s = 0.01\Omega$
- 运放增益 $G = 20$
- ADC参考电压 $V_{REF} = 3.3V$
- ADC中点电压 $V_{mid} = 1.65V$（双极性电流偏置到单极性）

电流到ADC值的转换链：

$$I \xrightarrow{R_s} V_{sense} = I \times R_s \xrightarrow{G} V_{adc} = V_{mid} + I \times R_s \times G \xrightarrow{ADC} D = \frac{V_{adc}}{V_{REF}} \times 4096$$

反向推导——1 LSB对应的电流值：

$$\Delta I_{1LSB} = \frac{V_{REF}}{4096 \times R_s \times G} = \frac{3.3}{4096 \times 0.01 \times 20} \approx 4.03 \text{ mA}$$

可测量的最大电流（单极性方向）：

$$I_{max} = \frac{V_{REF}/2}{R_s \times G} = \frac{1.65}{0.01 \times 20} = 8.25A$$

**量化误差占额定电流的比例**：

$$\frac{\Delta I_{1LSB}}{I_{rated}} = \frac{0.00403}{10} = 0.04\%$$

这个精度远好于1%的要求，看起来很充裕。但**注意**：实际误差还包括运放偏移、温漂、噪声等，这些叠加后往往远大于量化误差本身。

### 3.4 过采样提升分辨率

如果12位分辨率不够，STM32G474支持硬件过采样：

$$ENOB_{os} = ENOB_{base} + \frac{1}{2}\log_2(N_{os})$$

其中 $N_{os}$ 是过采样倍数。例如256倍过采样可提升4位有效分辨率：

$$ENOB_{os} = 12 + 4 = 16 \text{ bit（理论值）}$$

但代价是转换时间成倍增加，电机控制中通常不使用过采样（时序不允许），而是通过硬件滤波和软件校准来提升精度。

---

## 4. ADC采样时间计算

### 4.1 采样时间公式

ADC总转换时间由采样时间和逐次逼近转换时间组成：

$$T_{conv} = T_{samp} + T_{sar} = (SMP + 0.5) \times T_{ADCCLK} + 12.5 \times T_{ADCCLK}$$

其中：
- $SMP$：采样周期数，由SMPR1/SMPR2寄存器配置
- $T_{ADCCLK}$：ADC时钟周期
- 12.5：12位SAR ADC固有的逐次逼近周期数

STM32G474 ADC时钟最高60MHz（$T_{ADCCLK} = 16.67ns$），各采样时间配置：

| SMP[2:0] | 采样周期数 | 采样时间（60MHz ADCCLK） |
|-----------|-----------|------------------------|
| 000 | 2.5 cycles | 41.7 ns |
| 001 | 6.5 cycles | 108.3 ns |
| 010 | 12.5 cycles | 208.3 ns |
| 011 | 24.5 cycles | 408.3 ns |
| 100 | 47.5 cycles | 791.7 ns |
| 101 | 92.5 cycles | 1541.7 ns |
| 110 | 247.5 cycles | 4125.0 ns |
| 111 | 640.5 cycles | 10675.0 ns |

### 4.2 采样时间与信号源阻抗的关系

采样时间的物理本质是：ADC内部采样电容 $C_{adc}$ 通过外部信号源阻抗 $R_s$ 充电到输入电压所需的时间。

$$\tau = R_{total} \times C_{adc}$$

其中 $R_{total} = R_{external} + R_{switch}$（外部信号源输出阻抗 + ADC内部开关导通电阻）。

STM32G474的 $C_{adc} \approx 5pF$，$R_{switch} \approx 1k\Omega$。

采样电容充电到 $N$ 个时间常数后的误差：

$$\epsilon = e^{-N}$$

| 时间常数数 $N$ | 充电误差 | 精度 |
|---------------|---------|------|
| 5 | 0.67% | ~7 bit |
| 7 | 0.09% | ~10 bit |
| 9 | 0.01% | ~12 bit |
| 11 | 0.002% | >12 bit |

**对于12位精度，需要至少9个时间常数**：

$$T_{samp} \geq 9 \times R_{total} \times C_{adc}$$

### 4.3 电流采样电路的采样时间选择

典型电流采样电路输出阻抗分析：

```
运放输出 ──┬── R_filter (100~470Ω) ──┬── ADC输入
           │                          │
           │                     C_filter (1~10nF)
           │                          │
           └──────────────────────────┴── GND
```

- 运放输出阻抗：通常 < 1Ω（闭环）
- RC滤波电阻：100~470Ω
- ADC内部开关电阻：~1kΩ
- 总阻抗：$R_{total} \approx 1100 \sim 1470\Omega$

所需最小采样时间：

$$T_{samp,min} = 9 \times 1470 \times 5 \times 10^{-12} \approx 66 \text{ ns}$$

对应60MHz ADCCLK下需要约4个采样周期，选择 `SMP = 6.5 cycles`（108.3ns）留有充足裕量。

**工程实践**：在AxDr项目中，ADC注入组采样时间配置为 `ADC_SAMPLETIME_2CYCLES_5`（41.7ns），这是因为该项目的运放输出直接连接ADC输入，无额外RC滤波，输出阻抗极低。但如果你的电路有RC滤波，**必须增大采样时间**，否则采样值将不准确。

### 4.4 多通道扫描转换的总时间

注入组配置3个通道（Ia, Ib, Ic）时的总转换时间：

$$T_{total} = 3 \times (T_{samp} + T_{sar}) = 3 \times (SMP + 12.5) \times T_{ADCCLK}$$

以 `SMP = 6.5 cycles`，ADCCLK = 60MHz为例：

$$T_{total} = 3 \times (6.5 + 12.5) \times 16.67ns = 3 \times 19 \times 16.67ns \approx 950ns$$

这个时间必须在PWM触发时刻到下一个控制周期开始之间完成，否则会与下一个周期冲突。

---

## 5. 双ADC同步采样

### 5.1 为什么需要同步采样

FOC控制需要同一时刻的三相电流值。如果三相电流采样存在时间差 $\Delta t$，在电机旋转时会产生相位误差：

$$\Delta I_{phase} = I_{peak} \times \sin(\omega_e \times \Delta t)$$

其中：
- $\Delta I_{phase}$：采样时间差导致的电流相位误差 (A)
- $I_{peak}$：电流峰值 (A)
- $\omega_e$：电角速度 (rad/s)
- $\Delta t$：三相采样时间差 (s)

**数值示例**：电机运行在10000 RPM，7对极：

$$\omega_e = \frac{10000}{60} \times 7 \times 2\pi = 7330 \text{ rad/s}$$

如果采样时间差为 $1\mu s$：

$$\Delta I_{phase} = I_{peak} \times \sin(7330 \times 10^{-6}) = I_{peak} \times 0.00733 = 0.733\% \times I_{peak}$$

看起来很小，但在高速时（如30000 RPM），这个误差将达到2.2%，足以导致电流环性能下降。

### 5.2 ADC1和ADC2同步模式配置

STM32G474的ADC1和ADC2可以工作在同步模式，实现真正的同时采样：

```
          触发信号
              │
              ▼
         ┌─────────┐
         │  ADC1   │ ──→ CH1 (Ia)
         │ (Master)│ ──→ CH2 (Ib)
         └────┬────┘
              │ 同步触发
              ▼
         ┌─────────┐
         │  ADC2   │ ──→ CH11 (Ic 或 Vbus)
         │ (Slave) │
         └─────────┘
```

**同步模式类型**：

| 模式 | 描述 | 适用场景 |
|------|------|---------|
| ADC_MODE_INDEPENDENT | 独立模式，无同步 | 非同步采样 |
| ADC_DUALMODE_REGSIMULT | 规则组同步 | 多通道同步 |
| ADC_DUALMODE_INTERL | 规则组交错 | 提升采样率 |
| ADC_DUALMODE_INJECSIMULT | 注入组同步 | **电机控制首选** |
| ADC_DUALMODE_REGINJECTEC | 规则+注入同步 | 混合场景 |

**电机控制推荐**：`ADC_DUALMODE_INJECSIMULT`（注入组同步模式）

配置代码（HAL库）：

```c
ADC_MultiModeTypeDef multimode = {0};
multimode.Mode = ADC_DUALMODE_INJECSIMULT;  // 注入组同步
if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
}
```

### 5.3 采样两相电流 + KCL约束

三相电流满足基尔霍夫电流定律（KCL）：

$$I_a + I_b + I_c = 0$$

因此只需采样两相电流，第三相由计算得到：

$$I_c = -(I_a + I_b)$$

**优势**：
- 节省一个ADC通道
- 减少总转换时间
- 两个ADC各采样一相，天然同步

**劣势**：
- 计算值 $I_c$ 的噪声是 $I_a$ 和 $I_b$ 噪声的叠加
- 无法通过三相之和是否为零来校验采样正确性

**工程选择**：高性能伺服驱动器通常采样三相电流，利用 $I_a + I_b + I_c \approx 0$ 作为在线诊断条件。成本敏感方案采样两相。

### 5.4 同步误差分析

双ADC同步模式中，主从ADC之间存在几个ADCCLK周期的延迟：

$$\Delta t_{sync} \approx 1 \sim 2 \times T_{ADCCLK}$$

在60MHz ADCCLK下：

$$\Delta t_{sync} \approx 17 \sim 33 \text{ ns}$$

对应10000 RPM、7对极电机的相位误差：

$$\Delta I_{phase} = I_{peak} \times \sin(7330 \times 33 \times 10^{-9}) \approx 0.024\% \times I_{peak}$$

**结论**：双ADC同步误差在纳秒级，对电机控制完全可以忽略。

### 5.5 AxDr项目的实际配置分析

查看AxDr项目的ADC配置（[adc.c](file:///e:/gitee_CodeStorage/学习/MotorControl-main/AxDr/AxDr/Core/Src/adc.c)），发现：

```c
// ADC1: 注入组3通道 (Ia, Ib, Ic)
multimode.Mode = ADC_MODE_INDEPENDENT;  // 独立模式！
sConfigInjected.InjectedNbrOfConversion = 3;
sConfigInjected.InjectedChannel = ADC_CHANNEL_1;  // Rank1: Ic
sConfigInjected.InjectedChannel = ADC_CHANNEL_2;  // Rank2: Ib
sConfigInjected.InjectedChannel = ADC_CHANNEL_3;  // Rank3: Ia

// ADC2: 注入组1通道 (Vbus)
sConfigInjected.InjectedNbrOfConversion = 1;
sConfigInjected.InjectedChannel = ADC_CHANNEL_11; // Vbus
```

**注意**：AxDr项目使用的是 `ADC_MODE_INDEPENDENT` 而非同步模式。ADC1的3个注入通道是**顺序扫描**转换的，并非真正同时采样。这意味着Ia、Ib、Ic之间存在约 $19 \times T_{ADCCLK} \approx 317ns$ 的时间差。在低速应用中可接受，但高速应用建议改为双ADC同步模式。

---

## 6. 注入组与规则组

### 6.1 两组的本质区别

STM32的ADC有两组转换通道，它们的优先级和行为完全不同：

| 特性 | 规则组（Regular Group） | 注入组（Injected Group） |
|------|----------------------|------------------------|
| 通道数 | 最多16个 | 最多4个 |
| 触发方式 | 软件/外部触发 | 软件/外部触发 |
| 优先级 | 低 | **高（可打断规则组）** |
| 转换完成标志 | EOC | JEOC |
| 中断 | ADC_IRQn | ADC_IRQn（同一中断） |
| DMA支持 | 支持 | **不支持**（需手动读取JDRx） |
| 数据寄存器 | DR | JDR1~JDR4 |
| 典型用途 | 温度、母线电压等慢速信号 | **电流采样等实时信号** |

### 6.2 注入组打断规则组的机制

```
时间轴:
───┬─────────────────────────────────────────────┬───→
   │  规则组正在转换CH1 → CH2 → CH3              │
   │          ┌ 注入组触发 ──────────────────┐    │
   │          │ JCH1 → JCH2 → JCH3 → JCH4   │    │
   │          └──────────────────────────────┘    │
   │  规则组恢复 ← CH2继续 → CH3 →              │
   └─────────────────────────────────────────────┘
```

**关键机制**：
1. 注入组触发后，规则组当前转换**立即停止**
2. 注入组完成所有通道转换后，规则组从**被中断的通道继续**
3. 如果规则组配置了DMA，注入组打断期间DMA不会收到数据

### 6.3 电机控制中的正确用法

**注入组**：电流采样（由PWM定时器触发，保证时序确定性）

```c
// 注入组配置：3通道电流采样
sConfigInjected.InjectedNbrOfConversion = 3;
sConfigInjected.InjectedChannel = ADC_CHANNEL_1;  // Ia
sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;  // TIM1_CC4触发
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;
```

**规则组**：温度、母线电压等慢速信号（软件触发或定时器低频触发）

```c
// 规则组配置：母线电压、温度等
hadc1.Init.NbrOfConversion = 1;
hadc1.Init.ContinuousConvMode = DISABLE;
// 规则组可以配合DMA使用
hadc1.Init.DMAContinuousRequests = ENABLE;
```

### 6.4 注入组触发源

STM32G474 ADC注入组的触发源由JSQR寄存器的JEXTSEL[4:0]位选择：

| JEXTSEL[4:0] | 触发源 | 说明 |
|--------------|--------|------|
| 00000 | JADC1_INJC1 | 软件触发 |
| 00001 | TIM1_TRGO | 定时器1 TRGO |
| 00010 | TIM1_CC4 | **定时器1比较通道4（电机控制首选）** |
| 00011 | TIM2_TRGO | 定时器2 TRGO |
| 00100 | TIM3_CC4 | 定时器3比较通道4 |
| 00101 | TIM4_CC4 | 定时器4比较通道4 |
| ... | ... | 其他定时器 |
| 10000 | TIM8_CC4 | 定时器8比较通道4 |

**为什么选择TIM1_CC4？** 因为TIM1是PWM定时器，CC4通道可以精确设置比较值，在PWM周期的任意时刻触发ADC。这比TRGO更灵活。

### 6.5 注入组转换完成中断

注入组所有通道转换完成后，硬件置位JEOC（Injected End of Conversion）标志，触发ADC中断。

在ISR中读取注入组数据寄存器：

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 读取注入组转换结果
    // JDR1对应InjectedRank_1, JDR2对应InjectedRank_2, 以此类推
    uint16_t ia_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint16_t ib_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    uint16_t ic_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
}
```

**或者直接读寄存器（更快，适合中断中）**：

```c
// 直接读JDR寄存器，避免HAL库函数调用开销
uint16_t ia_raw = ADC1->JDR1;
uint16_t ib_raw = ADC1->JDR2;
uint16_t ic_raw = ADC1->JDR3;
```

> **重要提示**：注入组**不支持DMA传输**！这是STM32 ADC的硬件限制。注入组数据必须在中断中手动读取JDRx寄存器。规则组才支持DMA。这也是电机控制中电流采样用注入组+中断、慢速信号用规则组+DMA的原因之一。

---

## 7. PWM触发ADC中断

### 7.1 完整触发链

这是电机控制中最关键的时序链路：

```
TIM1计数器
    │
    ▼ 匹配CC4值
TIM1_CC4比较事件
    │
    ▼ 通过内部信号
ADC外部触发（JEXTSEL = TIM1_CC4）
    │
    ▼ ADC开始注入组转换
ADC注入组转换（3通道约950ns）
    │
    ▼ 转换完成
JEOC标志置位 → ADC中断
    │
    ▼ 进入ISR
读取JDRx → FOC计算 → 更新CCR
```

### 7.2 TRGO vs CC4触发

**TRGO（Trigger Output）方式**：

TRGO是定时器的主输出触发信号，可以配置为多种事件源：

```c
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  // 更新事件（计数器溢出）
// 或
sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;  // CC4比较事件
```

| TRGO源 | 描述 | 电机控制适用性 |
|--------|------|--------------|
| TIM_TRGO_RESET | 复位 | 不适用 |
| TIM_TRGO_ENABLE | 使能 | 不适用 |
| TIM_TRGO_UPDATE | 更新事件（计数器溢出/下溢） | 可用，但不够灵活 |
| TIM_TRGO_CC1~CC3 | 比较通道1~3 | 不适用（已用于PWM输出） |
| TIM_TRGO_OC4REF | **CC4比较事件** | **首选，精确控制采样时刻** |

**CC4直接触发方式**（推荐）：

ADC的JEXTSEL直接选择 `TIM1_CC4`，不经过TRGO。CC4通道不输出GPIO，仅用于内部触发。

```c
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
```

**为什么CC4比TRGO更好？**
- CC4的比较值可以独立设置，不影响TRGO的其他用途
- TRGO可能被其他外设使用（如触发另一个定时器）
- CC4可以精确控制采样时刻到任意位置

### 7.3 使用CC4精确控制采样时刻

在中心对齐PWM模式下，电流采样的最佳时刻是PWM三角波的**峰值附近**（上溢时刻），此时所有下桥臂MOSFET导通，电流流过采样电阻，采样值最稳定。

```
        ARR ────────────────────────────────────
           /\
          /  \
         /    \        ← 采样窗口：在峰值附近
        /      \
       /   ┌───── CC4匹配点（触发ADC）
      /    │      \
     /     │       \
   0 ──────┴──────────────────────────────────
           │←Δ→│
           ARR-Δ  ARR
```

CC4比较值设置：

$$CCR4 = ARR - \Delta$$

其中 $\Delta$ 需要满足：

$$\Delta > T_{ADC\_samp} + T_{ADC\_conv} + T_{margin}$$

即 $\Delta$ 必须大于ADC采样+转换时间加上安全裕量。

**计算示例**（AxDr项目参数）：
- TIM1 ARR = 4250
- ADCCLK = 60MHz（SYSCLK/4 = 170MHz/4 = 42.5MHz，注意实际配置）
- 注入组3通道，SMP = 2.5 cycles
- 总转换时间：$3 \times (2.5 + 12.5) \times (1/42.5MHz) \approx 1.06\mu s$

换算为定时器时钟周期（170MHz）：

$$\Delta_{tim} = 1.06\mu s \times 170MHz \approx 180 \text{ counts}$$

加上裕量（约50 counts）：

$$CCR4 = ARR - 230 = 4250 - 230 = 4020$$

**代码配置**：

```c
// 配置TIM1 CC4为ADC触发源
// CCR4值决定了ADC触发的精确时刻
#define ADC_TRIGGER_OFFSET  230  // 根据实际ADC转换时间计算

void ADC_TriggerConfig(void)
{
    // 设置CC4比较值
    TIM1->CCR4 = TIM1->ARR - ADC_TRIGGER_OFFSET;

    // 使能CC4比较中断（可选，用于调试时序）
    // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
}
```

### 7.4 触发边沿选择

ADC注入组触发边沿配置：

```c
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;
```

| 边沿选择 | 说明 |
|---------|------|
| RISING | 上升沿触发 |
| FALLING | 下降沿触发 |
| RISINGFALLING | 双边沿触发 |

**中心对齐模式下CC4的波形**：

```
中心对齐模式1（CMS=01）:
计数器:  0 → ARR → 0 → ARR → 0
CC4输出: __│‾‾│__│‾‾│__  （匹配时翻转）
         ↑         ↑
      上升沿    上升沿（向上计数时匹配）

中心对齐模式2（CMS=10）:
计数器:  0 → ARR → 0 → ARR → 0
CC4输出: __│‾‾│__│‾‾│__
               ↑         ↑
            下降沿    下降沿（向下计数时匹配）
```

**关键**：AxDr项目使用 `TIM_COUNTERMODE_CENTERALIGNED2`（中心对齐模式2），CC4匹配发生在**向下计数**时。选择FALLING边沿触发，意味着ADC在计数器向下计数经过CCR4值时触发——这正是PWM峰值附近的时刻。

### 7.5 AxDr项目的触发配置分析

从AxDr项目的代码可以看到：

```c
// tim.c: TIM1配置
htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;  // 中心对齐模式2
htim1.Init.Period = 4250;  // ARR = 4250

// adc.c: 注入组触发配置
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;  // TIM1_CC4触发
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;  // 下降沿
```

**但注意**：tim.c中TRGO配置为RESET：

```c
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
```

这没有问题，因为ADC使用的是CC4直接触发，不经过TRGO。TRGO可以用于其他目的。

---

## 8. ADC中断优先级

### 8.1 优先级安排原则

电机控制系统的中断优先级必须严格按实时性要求排列：

| 优先级 | 中断源 | 说明 | 典型优先级值 |
|--------|--------|------|-------------|
| **最高** | ADC注入组完成（JEOC） | 电流采样→FOC计算 | 0（最高） |
| 次高 | 定时器更新/比较 | PWM周期同步 | 1 |
| 中 | 通信接收（UART/SPI/CAN） | 参数配置 | 2~3 |
| 低 | DMA传输完成 | 数据搬运通知 | 4~5 |
| 最低 | 看门狗/系统 | 故障检测 | 6~7 |

**核心原则**：ADC中断（电流采样完成）必须是**最高或次高优先级**，因为FOC计算必须在下一个PWM周期触发前完成，否则控制周期会丢失。

### 8.2 中断延迟分析

从ADC转换完成到进入ISR的延迟：

$$T_{latency} = T_{flag} + T_{NVIC} + T_{stack} + T_{ISR\_entry}$$

其中：
- $T_{latency}$：中断总延迟 (s)
- $T_{flag}$：硬件标志位置位时间 (s)
- $T_{NVIC}$：NVIC中断向量获取时间 (s)
- $T_{stack}$：堆栈压入（保存现场）时间 (s)
- $T_{ISR\_entry}$：ISR入口跳转时间 (s)

| 延迟环节 | Cortex-M4 @170MHz | 说明 |
|---------|-------------------|------|
| JEOC标志置位 | 0 | 硬件自动 |
| NVIC响应 | 12 cycles | 中断向量获取 |
| 堆栈压入 | 8~12 cycles | 保存现场 |
| ISR入口 | 2~4 cycles | 跳转到ISR |

总延迟约 $12 + 12 = 24$ 个时钟周期：

$$T_{latency} \approx \frac{24}{170MHz} \approx 141ns$$

**如果ADC中断被其他中断抢占**，延迟将增加被抢占中断的执行时间。这就是为什么ADC中断必须是最高优先级。

### 8.3 FOC计算的时序约束

从PWM触发ADC到下一次PWM更新CCR的完整时序：

```
PWM峰值
    │
    ├── CC4匹配 → ADC触发
    │
    ├── ADC转换（约1~2μs）
    │
    ├── ADC中断延迟（约140ns）
    │
    ├── 进入ISR → 读取ADC值
    │
    ├── FOC计算（约3~8μs，取决于算法复杂度）
    │   ├── Clarke变换
    │   ├── Park变换
    │   ├── PID计算
    │   ├── 逆Park变换
    │   └── SVPWM计算
    │
    ├── 更新CCR1/CCR2/CCR3
    │
    └── 下一个PWM周期生效
```

**时序约束**：

$$T_{ADC} + T_{ISR\_latency} + T_{FOC} < T_{PWM\_half}$$

其中 $T_{PWM\_half}$ 是中心对齐PWM的半周期：

$$T_{PWM\_half} = \frac{ARR + 1}{f_{TIM\_CLK}}$$

AxDr项目：$T_{PWM\_half} = \frac{4250}{170MHz} = 25\mu s$（对应20kHz PWM频率）。

FOC计算必须在25μs内完成。实际FOC计算时间约5~10μs，裕量充足。

### 8.4 AxDr项目的中断优先级分析

```c
// adc.c: ADC中断优先级
HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);  // 抢占优先级0，子优先级0

// dma.c: DMA中断优先级
HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);  // 抢占优先级0
HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);  // 抢占优先级0
HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);  // 抢占优先级0
```

**问题**：ADC和DMA的抢占优先级都是0，这意味着它们不能互相抢占。如果DMA中断正在执行时ADC中断到来，ADC必须等待DMA中断执行完毕。这在电机控制中可能导致控制周期抖动。

**建议修改**：

```c
// ADC中断：最高优先级
HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);  // 抢占优先级0

// DMA中断：低于ADC
HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);  // 抢占优先级1
HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 1);
HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 2);
```

---

## 9. DMA配置

### 9.1 DMA模式选择

| 模式 | 描述 | 适用场景 |
|------|------|---------|
| Normal | 单次传输，传输完成后停止 | 一次性数据采集 |
| **Circular** | 循环传输，到达终点后自动回到起点继续 | **电机控制首选** |

**电机控制为什么用Circular模式？**

电流采样是周期性的（每个PWM周期采样一次），需要DMA持续搬运数据而无需CPU干预重新启动。Circular模式下，DMA到达缓冲区末尾后自动回到起始地址继续传输，形成"永不停歇"的数据流。

### 9.2 DMA通道映射

STM32G474使用DMAMUX（DMA请求复用器）将外设请求路由到DMA通道：

| DMA通道 | 默认请求 | 电机控制用途 |
|---------|---------|-------------|
| DMA1_Channel1 | ADC1 | 规则组ADC数据搬运 |
| DMA1_Channel2 | ADC2 | 双ADC模式从ADC数据 |
| DMA1_Channel3 | SPI1_RX | 编码器SPI通信 |
| DMA1_Channel4 | USART1_RX | 串口通信 |
| ... | ... | ... |

**注意**：注入组**不支持DMA**，只有规则组支持。这是STM32 ADC的硬件限制。

### 9.3 DMA数据宽度配置

```c
typedef struct {
    uint32_t PeriphDataAlignment;  // 外设数据宽度
    uint32_t MemDataAlignment;     // 内存数据宽度
    uint32_t Mode;                 // Normal/Circular
    uint32_t Direction;            // 外设→内存/内存→外设/内存→内存
} DMA_InitTypeDef;
```

**电机控制中的典型配置**：

| 参数 | 值 | 说明 |
|------|---|------|
| PeriphDataAlignment | DMA_PDATAALIGN_HALFWORD (16bit) | ADC数据寄存器为16位 |
| MemDataAlignment | DMA_MDATAALIGN_HALFWORD (16bit) | 内存中存储为16位 |
| Mode | DMA_CIRCULAR | 循环模式 |
| Direction | DMA_PERIPH_TO_MEMORY | 外设到内存 |

### 9.4 DMA寄存器级配置

```c
// DMA1 Channel1 配置：ADC1规则组 → 内存
void DMA_ADC_Config(void)
{
    // 使能DMA时钟
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    // 配置DMAMUX：ADC1请求路由到DMA1_Channel1
    DMAMUX1_Channel0->CCR = DMAMUX_C0CR_DMAREQ_ID;  // ADC1 request

    // 配置DMA通道
    DMA1_Channel1->CCR = 0;  // 先禁用

    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;      // 外设地址：ADC数据寄存器
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer;      // 内存地址
    DMA1_Channel1->CNDTR = BUFFER_SIZE;              // 传输数据量

    // 配置CCR寄存器
    DMA1_Channel1->CCR =
        DMA_CCR_PL_1              |  // 优先级：高
        DMA_CCR_MSIZE_0           |  // 内存数据宽度：16bit
        DMA_CCR_PSIZE_0           |  // 外设数据宽度：16bit
        DMA_CCR_MINC              |  // 内存地址自增
        DMA_CCR_CIRC              |  // 循环模式
        DMA_CCR_HTIE              |  // 半传输完成中断使能
        DMA_CCR_TCIE              |  // 全传输完成中断使能
        DMA_CCR_DIR_0;               // 方向：外设→内存

    // 使能DMA通道
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}
```

### 9.5 DMA中断

DMA通道有两个关键中断标志：

| 中断 | 标志 | 触发条件 |
|------|------|---------|
| 半传输完成（HT, Half Transfer） | HTIF | 传输完成一半数据时 |
| 全传输完成（TC, Transfer Complete） | TCIF | 传输完成全部数据时 |
| 传输错误（TE, Transfer Error） | TEIF | 传输发生错误时 |

---

## 10. DMA半传输/全传输中断

### 10.1 双缓冲原理

**这是DMA在电机控制中最精妙的应用！**

核心思想：DMA在Circular模式下持续搬运数据，CPU通过半传输中断和全传输中断交替处理数据，实现DMA搬运和CPU处理的**完全并行**。

```
缓冲区布局：
┌─────────────────────┬─────────────────────┐
│   前半部分 [0..N-1]  │   后半部分 [N..2N-1] │
└─────────────────────┴─────────────────────┘

时间线：
t0: DMA开始搬运到前半部分
t1: HT中断 ──→ DMA搬运后半部分，CPU处理前半部分
t2: TC中断 ──→ DMA回到前半部分搬运，CPU处理后半部分
t3: HT中断 ──→ DMA搬运后半部分，CPU处理前半部分
...（循环往复）
```

### 10.2 双缓冲工作机制详解

```
DMA传输指针:  ─────────────────────────────────→
              │ 前半部分 │ 后半部分 │ 前半部分 │
              ├──────────┼──────────┤──────────┤
CPU处理:      │          │ 处理前半 │ 处理后半 │
              │          │ (HT中断) │ (TC中断) │
              └──────────┴──────────┴──────────┘

关键：DMA写入的区域和CPU读取的区域永远不重叠！
```

**时序约束**：CPU处理半块数据的时间必须小于DMA搬运半块数据的时间，否则CPU来不及处理，数据会被DMA覆盖。

### 10.3 代码实现

```c
// 双缓冲区定义
// 假设规则组有4个通道（Vbus, Temp, Ia_offset, Ib_offset）
// 每个PWM周期采样一次，缓冲区大小 = 2 × 通道数
#define ADC_REGULAR_CHANNELS  4
#define ADC_DMA_BUFFER_SIZE   (ADC_REGULAR_CHANNELS * 2)  // 双缓冲

// 对齐到32字节（避免Cache问题，Cortex-M7需要，Cortex-M4可选）
ALIGN_32BYTES(volatile uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE]);

// 处理标志
typedef struct {
    uint16_t *ptr_first_half;   // 指向前半部分
    uint16_t *ptr_second_half;  // 指向后半部分
    volatile uint8_t data_ready; // 数据就绪标志
} dma_dual_buf_t;

dma_dual_buf_t dma_buf = {
    .ptr_first_half  = (uint16_t *)&adc_dma_buffer[0],
    .ptr_second_half = (uint16_t *)&adc_dma_buffer[ADC_REGULAR_CHANNELS],
    .data_ready = 0,
};

// DMA初始化
void ADC_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    // NVIC配置
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);  // 低于ADC中断
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // DMA通道配置
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_dma_buffer;
    DMA1_Channel1->CNDTR = ADC_DMA_BUFFER_SIZE;

    DMA1_Channel1->CCR =
        DMA_CCR_PL_1        |  // 高优先级
        DMA_CCR_MSIZE_0     |  // 内存16bit
        DMA_CCR_PSIZE_0     |  // 外设16bit
        DMA_CCR_MINC        |  // 内存自增
        DMA_CCR_CIRC        |  // 循环模式
        DMA_CCR_HTIE        |  // 半传输中断
        DMA_CCR_TCIE        |  // 全传输中断
        DMA_CCR_EN;           // 使能
}

// 半传输完成中断处理
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        // 半传输完成：前半部分数据已就绪
        DMA1->IFCR = DMA_IFCR_CHTIF1;  // 清除标志

        // 处理前半部分数据
        Process_ADC_Data(dma_buf.ptr_first_half);
    }

    if (DMA1->ISR & DMA_ISR_TCIF1) {
        // 全传输完成：后半部分数据已就绪
        DMA1->IFCR = DMA_IFCR_CTCIF1;  // 清除标志

        // 处理后半部分数据
        Process_ADC_Data(dma_buf.ptr_second_half);
    }
}

// 数据处理函数
void Process_ADC_Data(uint16_t *buf)
{
    // buf[0] = ADC通道1 (如Vbus)
    // buf[1] = ADC通道2 (如Temperature)
    // buf[2] = ADC通道3
    // buf[3] = ADC通道4

    float vbus = (float)buf[0] * vbus_ratio;
    float temp = (float)buf[1] * temp_ratio;
    // ... 其他处理
}
```

### 10.4 注入组+中断 vs 规则组+DMA双缓冲

在电机控制中，两种方式各有适用场景：

| 对比项 | 注入组+中断 | 规则组+DMA双缓冲 |
|--------|-----------|----------------|
| 实时性 | 最高（中断直接触发FOC） | 略低（需等DMA半传输完成） |
| CPU开销 | 每次中断都进入ISR | DMA搬运零CPU开销 |
| 通道数 | 最多4个 | 最多16个 |
| 适用信号 | 电流（高频、实时） | 温度/电压（低频、批量） |
| 数据搬运 | 手动读JDRx | DMA自动搬运 |

**最佳实践**：
- **电流采样**：注入组 + PWM触发 + 中断读取（保证时序确定性）
- **慢速信号**：规则组 + DMA双缓冲（减少CPU干预）

---

## 11. 数据对齐

### 11.1 左对齐 vs 右对齐

ADC数据寄存器的对齐方式由CFGR寄存器的ALIGN位控制：

```
12位ADC，右对齐（ALIGN=0）:
  31                              15                0
  ├───────────────────────────────┼─────────────────┤
  |         保留（0）              |  0  │ D11..D0   |
  └───────────────────────────────┴─────┴───────────┘
                                   ↑ 4位保留为0

12位ADC，左对齐（ALIGN=1）:
  31                15                              0
  ├─────────────────┼───────────────────────────────┤
  |  D11..D0  │ 0   |         保留（0）              |
  └───────────┴─────┴───────────────────────────────┘
               ↑ 4位保留为0
```

### 11.2 左对齐与Q15格式的关系

**左对齐是定点数运算的关键桥梁！**

左对齐后，12位ADC数据占据了16位的高12位：

$$D_{left} = D_{ADC} \times 16 = D_{ADC} \ll 4$$

这意味着ADC值被自动放大了16倍，可以直接作为Q15格式（范围-32768~32767）的有符号数使用（减去中点后）：

```c
// 右对齐方式（需要手动移位）
int16_t adc_val = (int16_t)(ADC1->JDR1);  // 0~4095
int16_t adc_signed = (adc_val - 2048);      // -2048~2047
int16_t adc_q15 = adc_signed << 4;          // 转为Q15: -32768~32751

// 左对齐方式（硬件自动完成移位）
int16_t adc_val = (int16_t)(ADC1->JDR1);  // 0~65520 (高12位有效)
int16_t adc_signed = adc_val - 0x8000;     // 减去中点，得到有符号Q15
```

**注意**：STM32G474的JDRx寄存器在注入组中总是**右对齐**存储的（无论CFGR中ALIGN位如何设置），这是因为注入组数据寄存器JDRx是32位宽，高16位用于双ADC模式存储从ADC数据。对于规则组DR寄存器，左对齐才有效。

### 11.3 电流采样偏置消除

电流采样电路通常将双极性电流偏置到ADC中点（1.65V for 3.3V参考），因此ADC原始值需要减去中点值：

$$I_{signed} = D_{ADC} - D_{midpoint}$$

对于12位ADC，中点值为2048：

$$I_{signed} = D_{ADC} - 2048$$

**偏置校准**：实际电路的中点值可能不是精确的2048，需要上电时校准：

```c
// AxDr项目的偏置校准方法（foc_drv.c）
void get_curr_off(void)
{
    float sum_a = 0, sum_b = 0, sum_c = 0;
    for (int i = 0; i < 1000; i++) {
        HAL_Delay(1);
        sum_a += (float)(ADC1->JDR3);  // Ia
        sum_b += (float)(ADC1->JDR2);  // Ib
        sum_c += (float)(ADC1->JDR1);  // Ic
    }
    mc_adc.ia_off = sum_a / 1000;
    mc_adc.ib_off = sum_b / 1000;
    mc_adc.ic_off = sum_c / 1000;
}
```

**注意**：校准必须在PWM关闭（电机无电流）时进行，否则校准值包含电流分量。

### 11.4 增益校准

从ADC原始值到实际电流值的转换：

$$I_{actual} = (D_{ADC} - D_{offset}) \times K_{current}$$

其中增益系数 $K_{current}$ 由硬件参数决定：

$$K_{current} = \frac{V_{REF}}{2^{12} \times R_s \times G_{amp}}$$

以AxDr项目为例（需要从实际硬件参数推算）：

```c
// 电流转换（foc_ctrl.c）
foc.i_a = ((float)mc_adc.ia - mc_adc.ia_off) * iratio;
foc.i_b = ((float)mc_adc.ib - mc_adc.ib_off) * iratio;
foc.i_c = ((float)mc_adc.ic - mc_adc.ic_off) * iratio;
```

其中 `iratio` 就是 $K_{current}$。

### 11.5 标幺值转换

在高级FOC算法中，电流值通常转换为标幺值（Per-Unit, PU）：

$$I_{pu} = \frac{I_{actual}}{I_{base}}$$

基值电流 $I_{base}$ 的选择：
- 通常取电机额定电流峰值：$I_{base} = \sqrt{2} \times I_{rated}$
- 或取ADC满量程对应电流：$I_{base} = I_{max\_measurable}$

标幺值的优势：
- 算法参数与具体硬件解耦
- 数值范围固定在 [-1, 1]，适合Q15/Q31定点数运算
- 不同功率等级的电机可以使用相同的控制参数

```c
// 标幺值转换示例
#define I_BASE  14.6f  // 基值电流（A），对应ADC满量程

float i_a_pu = foc.i_a / I_BASE;  // 范围 [-1.0, 1.0]
int16_t i_a_q15 = (int16_t)(i_a_pu * 32767);  // Q15格式
```

---

## 12. 完整的ADC+DMA+PWM联动配置流程

### 12.1 配置步骤总览

```
Step 1: 配置PWM定时器
    │  中心对齐模式、ARR、CCR1~3、死区
    ▼
Step 2: 配置ADC
    │  注入组通道、采样时间、触发源
    │  规则组通道（如需DMA）
    ▼
Step 3: 配置DMA
    │  Circular模式、双缓冲、HT/TC中断
    ▼
Step 4: 配置TRGO/CC4触发链
    │  定时器→ADC触发链
    ▼
Step 5: 启动顺序
    │  先DMA→再ADC→最后PWM
    ▼
Step 6: 验证时序
       示波器/逻辑分析仪确认
```

### 12.2 Step 1: 配置PWM定时器

```c
void PWM_Timer_Init(void)
{
    // 基本定时器配置
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;                                    // 无分频，170MHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;     // 中心对齐模式2
    htim1.Init.Period = 4250;                                    // ARR = 4250 → 20kHz PWM
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // PWM通道配置（CCR1~3用于三相PWM输出）
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                    // 初始占空比为0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

    // CC4通道配置（仅用于ADC触发，不输出GPIO）
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
    TIM1->CCR4 = 4250 - 230;  // 在峰值前230个计数周期触发ADC

    // 死区配置
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTime = {0};
    sBreakDeadTime.DeadTime = 50;  // 死区时间约300ns @170MHz
    sBreakDeadTime.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTime.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTime);
}
```

### 12.3 Step 2: 配置ADC

```c
void ADC_Init_For_FOC(void)
{
    // ========== ADC1 配置 ==========
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;  // 170/4 = 42.5MHz
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;             // 扫描模式（多通道）
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;               // 单次转换（由触发启动）
    hadc1.Init.NbrOfConversion = 1;                        // 规则组1通道
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;           // 保留旧数据
    HAL_ADC_Init(&hadc1);

    // 双ADC模式（如需同步采样）
    ADC_MultiModeTypeDef multimode = {0};
    multimode.Mode = ADC_DUALMODE_INJECSIMULT;  // 注入组同步
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

    // ========== 注入组配置（电流采样） ==========
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;  // 6.5个ADC时钟周期
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 3;         // 3通道：Ia, Ib, Ic
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.AutoInjectedConv = DISABLE;           // 禁止自动注入
    sConfigInjected.QueueInjectedContext = DISABLE;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;  // TIM1_CC4触发
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;
    sConfigInjected.InjecOversamplingMode = DISABLE;

    // 通道1: Ia (PA0, ADC1_IN1)
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    // 通道2: Ib (PA1, ADC1_IN2)
    sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    // 通道3: Ic (PA2, ADC1_IN3)
    sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    // ========== ADC2 配置（母线电压） ==========
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    HAL_ADC_Init(&hadc2);

    // ADC2注入组：母线电压
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedChannel = ADC_CHANNEL_11;     // PC5, ADC2_IN11
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;
    HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    // ========== ADC校准 ==========
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // ========== 中断优先级 ==========
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);  // 最高优先级
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}
```

### 12.4 Step 3: 配置DMA

```c
// DMA双缓冲区
#define ADC_REG_CHANNELS  4   // 规则组通道数
#define ADC_DMA_BUF_SIZE  (ADC_REG_CHANNELS * 2)

ALIGN_32BYTES(volatile uint16_t adc_reg_buffer[ADC_DMA_BUF_SIZE]);

void DMA_Init_For_ADC(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    // DMA1_Channel1 → ADC1规则组
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_reg_buffer;
    DMA1_Channel1->CNDTR = ADC_DMA_BUF_SIZE;

    DMA1_Channel1->CCR =
        DMA_CCR_PL_1        |  // 优先级：高
        DMA_CCR_MSIZE_0     |  // 内存：16bit
        DMA_CCR_PSIZE_0     |  // 外设：16bit
        DMA_CCR_MINC        |  // 内存地址自增
        DMA_CCR_CIRC        |  // 循环模式
        DMA_CCR_HTIE        |  // 半传输完成中断
        DMA_CCR_TCIE        |  // 全传输完成中断
        DMA_CCR_DIR_0;        // 外设→内存

    // DMA中断优先级（低于ADC中断）
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
```

### 12.5 Step 4: 配置触发链

```c
void Trigger_Chain_Config(void)
{
    // TIM1 CC4值设置（决定ADC触发时刻）
    // 在中心对齐模式2中，向下计数经过CCR4时产生比较事件
    // CCR4 = ARR - offset，offset根据ADC转换时间计算
    TIM1->CCR4 = TIM1->ARR - 230;

    // TRGO配置（如果使用TRGO触发ADC，此处用CC4直接触发，TRGO可配置为其他用途）
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;  // 不使用TRGO触发ADC
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}
```

### 12.6 Step 5: 启动顺序

**启动顺序至关重要！** 错误的启动顺序可能导致ADC丢失第一次触发或DMA数据错位。

```c
void FOC_Peripherals_Start(void)
{
    // ===== 1. 先启动DMA =====
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // ===== 2. 再启动ADC =====
    // 启动规则组+DMA（慢速信号）
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_reg_buffer, ADC_DMA_BUF_SIZE);

    // 启动注入组（电流采样，等待PWM触发）
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    // 如果使用双ADC同步模式
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    // ===== 3. 最后启动PWM =====
    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // 启动TIM1（PWM开始运行，触发ADC开始转换）
    HAL_TIM_Base_Start(&htim1);

    // ===== 4. 偏置校准 =====
    // PWM刚启动时先输出50%占空比，让电流稳定后校准偏置
    // 或者先不启动PWM，在零电流时校准
    get_curr_off();  // 偏置校准
}
```

### 12.7 Step 6: 验证时序

**用示波器验证的关键信号**：

```
通道1: PWM输出（上桥臂）
通道2: PWM输出（下桥臂）
通道3: ADC触发信号（CC4输出，需配置CC4为GPIO输出或用TRGO观测）
通道4: ADC中断响应（在ISR中翻转一个GPIO）

期望时序:
    PWM峰值
      │
      ├── CC4匹配（ADC触发）
      │
      ├── ~1μs ── ADC转换完成
      │
      ├── ~140ns ── 进入ISR
      │
      ├── GPIO翻转（标记ISR进入）
      │
      ├── ~5μs ── FOC计算
      │
      └── CCR更新（下一个PWM周期生效）
```

**验证代码**（在ISR中翻转GPIO用于时序测量）：

```c
// 在ADC注入组完成中断中
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 时序标记：进入ISR
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // 读取ADC值
    mc_adc.ia = ADC1->JDR3;
    mc_adc.ib = ADC1->JDR2;
    mc_adc.ic = ADC1->JDR1;
    mc_adc.vbus = ADC2->JDR1;

    // FOC计算
    // ... Clarke, Park, PID, InvPark, SVPWM ...

    // 时序标记：退出ISR
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
```

### 12.8 完整时序图

```
时间轴 (一个PWM周期 = 50μs @20kHz)
──────────────────────────────────────────────────────────────→

TIM1计数器:
    0 ──→ ARR ──→ 0
         ↑峰值

PWM输出:
    ┌──┐      ┌──────────────┐      ┌──┐
    │  │      │              │      │  │
────┘  └──────┘              └──────┘  └────
    ←dead→   ←  高侧导通  →   ←dead→

CC4触发:
                    ↓ (ARR-Δ处)
                ────┐
                    │ (触发脉冲)
                ────┘

ADC转换:
                    ├─ Ia ─┤─ Ib ─┤─ Ic ─┤
                    ←── 约1μs ──→

ADC中断:
                              ├─ ISR ──────────────┤
                              ← 读取+FOC计算 ~5μs →

CCR更新:
                                              ↓
                                    新的CCR值写入
                                    (下一个PWM周期生效)

DMA (规则组):
    ──────────────────────────────────────────────→
    持续搬运温度/电压等慢速信号，与FOC计算并行
```

---

## 13. 寄存器级快速参考

### 13.1 ADC关键寄存器

| 寄存器 | 功能 | 关键位 |
|--------|------|--------|
| ADC_CR | ADC控制 | ADEN(使能), ADCAL(校准), JADSTART(注入组启动) |
| ADC_CFGR | ADC配置 | CONT(连续), OVRMOD(溢出), ALIGN(对齐), EXTEN(触发边沿) |
| ADC_CFGR2 | ADC配置2 | ROVSE(过采样使能), OSR[9:0](过采样率) |
| ADC_SMPR1/2 | 采样时间 | SMPx[2:0] |
| ADC_JSQR | 注入组序列 | JL[1:0](通道数), JSQ1~4[4:0](通道), JEXTSEL[4:0](触发源), JEXTEN[1:0](触发边沿) |
| ADC_JDR1~4 | 注入组数据 | 32位，低16位为ADC1数据，高16位为ADC2数据（双ADC模式） |
| ADC_ISR | 中断状态 | JEOC(注入组完成), EOC(规则组完成), OVR(溢出) |
| ADC_IER | 中断使能 | JEOCIE, EOCIE, OVRIE |

### 13.2 TIM1关键寄存器

| 寄存器 | 功能 | 关键位 |
|--------|------|--------|
| TIM_CR1 | 定时器控制 | CEN(使能), CMS[1:0](中心对齐模式) |
| TIM_ARR | 自动重载值 | PWM周期 |
| TIM_CCR1~3 | 比较值1~3 | 三相PWM占空比 |
| TIM_CCR4 | 比较值4 | ADC触发时刻 |
| TIM_BDTR | 刹车和死区 | DTG[7:0](死区), MOE(主输出使能) |
| TIM_CR2 | 定时器控制2 | MMS[2:0](TRGO源) |
| TIM_CCER | 捕获/比较使能 | CC4E(CC4输出使能) |

### 13.3 DMA关键寄存器

| 寄存器 | 功能 | 关键位 |
|--------|------|--------|
| DMA_CCR | 通道配置 | EN, DIR[1:0], CIRC, HTIE, TCIE, MSIZE[1:0], PSIZE[1:0], MINC, PL[1:0] |
| DMA_CNDTR | 传输数据量 | NDT[15:0] |
| DMA_CPAR | 外设地址 | ADC1->DR的地址 |
| DMA_CMAR | 内存地址 | 缓冲区地址 |
| DMA_ISR | 中断状态 | HTIF1, TCIF1, TEIF1 |
| DMA_IFCR | 中断标志清除 | CHTIF1, CTCIF1, CTEIF1 |

---

## 14. 常见问题与调试指南

### 14.1 ADC采样值跳变

**现象**：ADC值偶尔出现大幅跳变

**排查步骤**：
1. 检查ADC触发时刻是否与PWM开关切换重叠
2. 用示波器测量ADC输入引脚的信号质量
3. 检查采样时间是否足够（增大SMP试试）
4. 检查电源纹波和地线质量

**常见原因**：
- 采样时刻太靠近PWM边沿，开关噪声耦合到采样电容
- 采样时间不足，内部采样电容未充分充电
- 电源纹波过大，影响ADC参考电压

### 14.2 三相电流之和不等于零

**现象**：$I_a + I_b + I_c \neq 0$

**排查步骤**：
1. 检查三相ADC是否同步采样（双ADC同步模式？）
2. 检查各相增益系数是否一致
3. 检查偏置校准是否正确
4. 检查ADC通道顺序是否与硬件对应

**常见原因**：
- 非同步采样导致时间差
- 各相运放增益不一致
- 偏置校准在非零电流时进行

### 14.3 控制周期抖动

**现象**：FOC计算完成时间不稳定

**排查步骤**：
1. 检查ADC中断优先级是否为最高
2. 检查是否有其他中断抢占ADC中断
3. 检查ISR中是否调用了阻塞函数（如HAL_Delay）
4. 检查Flash等待周期是否导致取指延迟

**常见原因**：
- ADC中断被低优先级中断延迟
- ISR中调用了HAL库函数（开销大）
- 编译器优化级别不够

### 14.4 DMA数据错位

**现象**：DMA搬运的数据通道顺序错乱

**排查步骤**：
1. 检查DMA CNDTR值是否正确
2. 检查CMAR地址是否对齐
3. 检查是否在DMA传输过程中修改了缓冲区
4. 检查HT/TC中断处理是否正确

**常见原因**：
- DMA启动时机不对（ADC已经开始转换后才启动DMA）
- 缓冲区地址未对齐
- 双缓冲处理逻辑错误

---

## 15. 从AxDr项目学到的实战经验

### 15.1 AxDr项目的ADC配置特点

从 [adc.c](file:///e:/gitee_CodeStorage/学习/MotorControl-main/AxDr/AxDr/Core/Src/adc.c) 和 [foc_ctrl.c](file:///e:/gitee_CodeStorage/学习/MotorControl-main/AxDr/AxDr/User/motor/foc_ctrl.c) 中可以总结：

1. **使用注入组+中断读取电流**：ADC1注入组3通道（Ia/Ib/Ic），ADC2注入组1通道（Vbus），都在 `HAL_ADCEx_InjectedConvCpltCallback` 中直接读取JDRx寄存器

2. **CC4触发**：两个ADC的注入组都配置为 `ADC_EXTERNALTRIGINJEC_T1_CC4` + `FALLING` 边沿

3. **中心对齐模式2**：TIM1配置为 `TIM_COUNTERMODE_CENTERALIGNED2`，CC4在向下计数时匹配

4. **独立模式而非同步模式**：ADC1和ADC2使用 `ADC_MODE_INDEPENDENT`，三相电流由ADC1顺序扫描，非真正同步

5. **偏置校准**：上电时在PWM关闭状态下采样1000次取平均作为偏置

6. **运行时间测量**：使用TIM3测量FOC计算时间，监控是否超时

### 15.2 改进建议

1. **双ADC同步模式**：将ADC1和ADC2配置为 `ADC_DUALMODE_INJECSIMULT`，ADC1采样Ia和Ib，ADC2采样Ic，实现真正同步采样

2. **增大采样时间**：`ADC_SAMPLETIME_2CYCLES_5` 偏小，建议至少 `ADC_SAMPLETIME_6CYCLES_5`

3. **中断优先级分离**：ADC中断优先级应高于DMA中断

4. **规则组+DMA**：将Vbus等慢速信号移到规则组，配合DMA搬运，减少注入组通道数

5. **使用CC4精确触发**：当前TRGO配置为RESET，需确认CC4值是否已正确设置

---

## 16. 总结

ADC深度配置与DMA数据搬运是电机控制中**硬件与算法的桥梁**。核心要点：

1. **注入组是电机控制的命脉**：电流采样必须用注入组，由PWM精确触发，保证时序确定性
2. **PWM触发是时序的灵魂**：CC4通道精确控制采样时刻，避免开关噪声
3. **DMA双缓冲是效率的保障**：规则组+DMA实现零CPU开销的数据搬运
4. **数据对齐是精度的基石**：偏置校准、增益校准、标幺值转换确保算法输入正确
5. **中断优先级是实时性的底线**：ADC中断必须最高优先级，否则控制周期抖动

**一句话总结**：电机控制中的ADC不是"读一个值"，而是"在正确的时刻、以正确的方式、读正确的值"。

---

> **交叉引用**：
> - ADV-HW-01 PWM深度配置与电流采样时序联动
> - HW-02 电流采样电路
> - HW-04 MCU外设与通信
> - ALG-05 有感FOC实现
> - ADV-ALG-09 标幺值系统与定点数运算

### 🔗 hpm_MC 工程关联

**ADC 采样** (`hpm_mcl_v2/core/sensor/hpm_mcl_analog.h`):
- `mcl_analog_t` 管理 10 通道采样，触发源为 PWM 定时器
- ADC 偏置自动校准（中点检测）
- IIR 数字滤波器替代模拟滤波器
- 从 ADC 到物理量的完整转换链
