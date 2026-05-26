# ALG-02 ADC 电流采样时序

**模块编号：** ALG-02
**模块名称：** 电流采样时序（Current Sampling Timing）
**文档版本：** v2.0
**适用对象：** 电机控制工程师、嵌入式开发者
**前置知识：** PWM 调制原理（中心对齐/边沿对齐）、SVPWM 空间矢量调制、ADC 采样基础、逆变器开关过程

---

## 1. 📌 核心摘要 ★★★☆☆ 🔰

**一句话总结：** 电流采样时刻的选择直接决定了 FOC 电流环的反馈质量——采样在 PWM 周期中心（零矢量中点）可获得无开关噪声的平均电流，是最佳工程实践。

**认知钩子：** 想象你在一个嘈杂的工厂里测量水位——如果你在水泵启动的瞬间读数，会得到一个被浪涌干扰的假值；如果你等水面平静后再读，就能得到真实水位。在 PWM 逆变器中，开关管的每一次切换都是一次"浪涌"，而零矢量区间就是短暂的水面平静期。

```
PWM载波（三角波）:     /\      /\      /\
                      /  \    /  \    /  \
开关噪声:          ▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁▁
                           ▔▔    ▔▔    ▔▔  ← 零矢量窗口（安全采样区）
ADC触发:               ↑      ↑      ↑
```

| 关键概念 | 说明 |
|---------|------|
| 零矢量窗口 | V0(000) 或 V7(111) 期间，无开关动作，无 di/dt 噪声 |
| 平均电流原理 | 三角波顶点处相电流等于该 PWM 周期的平均值 |
| 死区振铃 | 死区结束后 2~3μs 内存在 LC 谐振振铃，必须避开 |
| 单电阻 vs 多电阻 | 单电阻需在特定零矢量内分时采样，时序约束更严格 |

---

## 2. 📐 原理推导 ★★★☆☆ 📚

### 2.1 为什么要关注采样时序？

在 PWM 逆变器中，相电流含有大量开关频率分量（纹波）。如果 ADC 在开关管切换瞬间采样，会采到严重的开关噪声，导致电流反馈失真、控制环路振荡。

### 2.2 最佳采样时刻：PWM 周期中心

中心对齐 PWM 的三角波顶点对应零矢量 V0(000) 或 V7(111) 的中间时刻：

- **此时所有三相关断（V0）或全通（V7）**
- **无开关管切换 → 无 di/dt → 无噪声耦合**
- **电流值等于该 PWM 周期的平均值**

```
Ts周期示意：
|--V0--|--Vactive--|--V7--|--Vactive--|--V0--|
                    ↑ 采样           ↑ 采样
               (三角波顶点)    (三角波顶点)
```

### 2.3 采样策略对比

| 策略 | 时刻 | 电流值 | 噪声 | 推荐度 |
|------|------|--------|------|--------|
| 顶点采样 | PWM中心(V0/V7中) | 平均电流 | 极低 | ★★★★★ |
| 底点采样 | PWM边沿 | 瞬时电流 | 高 | ★★ |
| 双采样 | 顶+底 | 平均+瞬时 | 可滤波 | ★★★★ |

### 2.4 采样噪声的频谱特性

开关噪声集中在 PWM 频率的整数倍处（$f_{pwm}$, $2f_{pwm}$, …），远高于电流环带宽（~1kHz）。设计良好的电流环 PI 本身具有低通特性，但采到噪声峰值仍会导致控制量抖动。

---

## 3. 🔢 数学建模 ★★★☆☆ 📚

### 3.1 采样误差与 PWM 占空比的关系

在 SVPWM 调制下，最大零矢量持续时间决定了可用的 ADC 采样窗口：

$$T_{V0,max} = T_{PWM} \cdot \left(1 - m_{max}\right)$$

其中：
- $T_{PWM}$ —— PWM 周期（s），例如 10kHz → $100\mu s$
- $m_{max}$ —— 最大调制比，SVPWM 线性区可达 $2/\sqrt{3} \approx 1.15$

### 3.2 采样窗口的安全裕量

实际可用采样窗口需扣除死区振铃时间：

$$T_{safe} = T_{V0} - T_{dead} - T_{ringing}$$

其中：
- $T_{dead}$ —— 死区时间（s），典型值 $1 \sim 3\mu s$
- $T_{ringing}$ —— 开关振铃衰减时间（s），典型值 $2 \sim 3\mu s$

### 3.3 单电阻采样时序约束

单电阻采样需在一个 PWM 周期内完成两相电流的采样，最小有效矢量持续时间必须大于 ADC 转换时间：

$$T_{active,min} \geq T_{ADC\_conv} + T_{sample\_hold}$$

其中：
- $T_{ADC\_conv}$ —— ADC 转换时间（s），取决于 ADC 时钟和分辨率
- $T_{sample\_hold}$ —— 采样保持时间（s），典型值 $0.5 \sim 1\mu s$

---

## 4. 💻 代码实现 ★★★☆☆ 🔧

### 4.1 ADC 触发配置（PWM 中心触发）

```c
void adc_sync_with_pwm_center(void)
{
    pwm_cmp_config_t cmp_cfg;

    cmp_cfg.cmp_channel = PWM_CMP_CHANNEL_7;
    cmp_cfg.cmp_value = pwm_reload_value / 2;
    cmp_cfg.trigger = PWM_CMP_TRIG_ADC;
    pwm_config_cmp(PWM_BASE, &cmp_cfg);

    adc_set_trigger_source(ADC_BASE, ADC_TRIG_PWM);
    adc_set_trigger_polarity(ADC_BASE, ADC_TRIG_RISING_EDGE);
}
```

### 4.2 多通道 DMA 采样配置

```c
void adc_dma_sequence_config(void)
{
    adc_dma_config_t dma_cfg = {
        .channels = {
            ADC_CH_IA_PHASE,
            ADC_CH_IB_PHASE,
            ADC_CH_IC_PHASE,
            ADC_CH_VBUS,
        },
        .channel_count = 4,
        .burst_mode = true,
        .transfer_complete_isr = adc_dma_done_callback,
    };
    adc_init_dma_sequence(ADC_BASE, &dma_cfg);
}
```

### 4.3 过采样滤波（4 次采样均值）

```c
#define OVERSAMPLE_COUNT 4

uint16_t adc_oversample_read(uint8_t channel)
{
    uint32_t sum = 0;
    for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
        sum += adc_read_single(ADC_BASE, channel);
    }
    return (uint16_t)(sum / OVERSAMPLE_COUNT);
}
```

---

## 5. 🔧 参数整定 ★★★☆☆ 🔧

### 5.1 工程实践要点

- **单电阻采样**：必须在 V0 或 V7 内特定时刻采样，时序约束严格
- **双/三电阻采样**：在 PWM 中心触发 ADC，配合 DMA 自动完成
- **过采样**：在 V0/V7 区间内采样多次取平均，提升有效位数
- **死区影响**：采样时刻需避开死区刚结束的振铃期（~2-3μs）

### 5.2 典型参数参考值

| 参数 | 典型值 | 说明 |
|------|--------|------|
| PWM 频率 $f_{pwm}$ | 8~20 kHz | 功率器件和损耗权衡 |
| 采样时刻偏移 | PWM 中心 ±0 | 零矢量中点 |
| ADC 转换时间 | 1~3 μs | 12-bit SAR ADC |
| 过采样率 | 4~16 倍 | 每提高 4 倍获得 1 位 ENOB |
| 死区振铃等待 | 2~3 μs | 根据功率回路实测确定 |

### 5.3 验证方法

1. **示波器验证**：将 ADC 触发信号与 PWM 波形同时观测，确认触发位置在 V0/V7 中点
2. **电流波形 FFT**：观察无负载下的电流采样值频谱，不应出现明显 PWM 频率分量
3. **占空比极限测试**：调制比接近 1 时，确认零矢量窗口仍然满足采样需求

---

## 6. 🔗 硬件约束 ★★★★☆ ⚠️

### 6.1 ADC 触发与 PWM 同步

ADC 触发信号必须与 PWM 载波严格同步，抖动容限通常要求 $\leq 100ns$。触发路径延迟包括：
- PWM 比较器输出延迟：~50ns
- ADC 触发信号布线延迟：取决于 PCB 走线长度
- ADC 启动延迟：~2~3 个 ADC 时钟周期

**关键约束：** 若使用多片 ADC 交替采样（如双 ADC 交错模式），需保证各 ADC 的触发信号到达时间差异小于采样时钟周期的 1/4，否则等效于引入了通道间相位误差。

> 相关硬件模块：[HW-03 电流采样电路设计](file:///e:/gitee_CodeStorage/学习/MotorControl-main/motor-control-knowledge-base/hardware/HW-03-Current-Sampling-Circuit.md)

### 6.2 采样电阻布局

采样电阻（Shunt Resistor）在 PCB 上的布局直接影响采样精度：

- **Kelvin 接法（四线开尔文）**：电流路径与电压检测路径分离，消除 PCB 铜箔电阻引起的测量误差（典型误差可达 5~10%）
- **共模抑制**：差分 ADC 输入应紧靠采样电阻两端，减小环路面积以降低磁场耦合
- **热效应**：采样电阻温漂（TCR）需 $\leq 50ppm/°C$，否则满载温升 50°C 时将引入 0.25% 增益误差

### 6.3 运算放大器带宽

采样电阻信号需经运放调理后送入 ADC：

- **增益带宽积（GBWP）**：运放开环增益在 PWM 频率处必须足够抑制开关噪声。若运放 GBWP = 10MHz，闭环增益 G = 20，则闭环带宽仅 500kHz，对 PWM 边沿（$\sim 50ns$ 上升沿 = 等效 7MHz 带宽）的噪声抑制有限
- **建立时间**：运放输出需在 ADC 采样保持窗口内建立到目标精度（如 12-bit → 0.024% → 需约 9 倍时间常数）
- **摆率限制**：若运放摆率 $SR = 5V/\mu s$，信号跳变 3V 需 $600ns$，必须小于 ADC 采样保持时间

---

## 7. 🚀 前沿拓展 ★★★★★ 💡

### 7.1 过采样与抽取技术

利用零矢量窗口内的多次高速采样 + 数字低通滤波（CIC 滤波器），在不增加硬件成本的前提下提升 ADC 有效位数（ENOB）。每 4 倍过采样可换取约 1 位 ENOB 提升。现代 MCU（如 HPM6xxx 系列）内置 16-bit SAR ADC，配合片上过采样硬件可达到 14~15-bit 有效分辨率。

### 7.2 Σ-Δ ADC 电流采样

Σ-Δ 调制器以极高过采样率（$f_{mod} \gg f_{pwm}$）对电流连续采样，通过数字抽取滤波器输出高分辨率电流值。优势在于：
- 天然抗混叠，无需模拟低通滤波器
- 分辨率可达 16~18 bit
- 缺点：群延迟较大（$\sim 10 \sim 50\mu s$），对高速电流环不利

### 7.3 预测电流采样（Predictive Sampling）

当 PWM 占空比接近 0% 或 100% 时，零矢量窗口极窄甚至消失，此时可采用预测采样策略：
- 基于电流模型 $i(k+1) = f(i(k), v(k), \omega_e)$ 预测采样时刻的电流值
- 或使用两个相邻周期的采样值进行线性外推
- 已知在部分车规级电机控制器（低载波比工况）中有落地应用

### 7.4 自适应采样窗口

根据实时占空比动态调整 ADC 采样时刻，在零矢量窗口内选择最佳采样点。对于双电阻采样，可在两个连续 PWM 周期中交替采样不同相电流以放宽时序约束。

---

> **仿真体验**：拖动滑块观察不同采样时刻对应的噪声水平。绿色窗口=安全采样区。

:::sim sampling-timing

### 🔗 hpm_MC 代码实现参考

**v2 模拟采样** (`hpm_mcl_v2/core/sensor/hpm_mcl_analog.h`):
- `mcl_analog_t` 管理 10 个通道：Ia/Ib/Ic（相电流）+ Va/Vb/Vc（相电压）+ Vbus（母线电压）
- 每通道独立 IIR 低通滤波器配置（alpha 系数可调）
- ADC 结果到物理量的转换：`i = (adc - offset) / (opamp_gain × sample_res)`
- 支持偏置自动校准（中点检测）

**v1 采样配置** (`hpm_mcl/inc/hpm_bldc_define.h`):
- ADC采样通道数和电流偏置在 `BLDC_CONTRL_ADC_PARA` 中定义
- 支持单电阻/双电阻/三电阻采样模式

**示例**: `hpm_MC/samples/motor_ctrl/bldc_foc/src/bldc_foc.c` 中 ADC 中点校准流程