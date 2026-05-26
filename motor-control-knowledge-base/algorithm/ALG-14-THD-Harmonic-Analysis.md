# ALG-14 THD 谐波分析

**模块编号：** ALG-14
**模块名称：** THD 谐波分析（THD Harmonic Analysis）
**文档版本：** v2.0
**适用对象：** 电机控制工程师、嵌入式开发者、EMC/EMI 工程师
**前置知识：** ALG-09 SVPWM 空间矢量调制、傅里叶级数基础、ALG-10 过调制

---

## 1. 📌 核心摘要 ★★★☆☆ 🔰📚

总谐波失真（THD, Total Harmonic Distortion）是衡量逆变器输出电压/电流"纯净度"的核心指标。在电机驱动中，THD 的高低直接影响铁损、转矩脉动和 EMI 噪声。本模块的核心认知锚点：**电压 THD 和电流 THD 是两个不同概念**——电机绕组电感对高频谐波有自然衰减，因此评测调制策略时应优先使用 WTHD（Weighted THD）而非原始电压 THD。理解 SVPWM 相比 SPWM 在谐波性能上的优势，需要从"谐波能量频移"的角度切入。

```
    谐波能量分布对比（定性）：

    SPWM:  ┃           ▁▃▅███▅▃▁
           ┃          ┄┄┄┄┄┄┄┄┄┄┄
           ┗━━━━━━━━━━━━━━━━━━━━━→ f
           0  基波   f_sw → 边带密集

    SVPWM: ┃           ▁▃▅███▅▃▁
           ┃          ┄┄┄┄┄┄┄┄┄┄┄┄┄┄
           ┗━━━━━━━━━━━━━━━━━━━━━→ f
           0  基波   f_sw → 边带分散至更高频

    → 相同开关频率下，SVPWM 的低频谐波更少 → 电流 THD 更低
```

| 关键概念 | 说明 |
|----------|------|
| THD（电压） | $\text{THD}_V = \sqrt{\sum_{n=2}^{\infty} V_n^2} / V_1 \times 100\%$，衡量电压波形失真 |
| WTHD（加权） | $\text{WTHD} = \sqrt{\sum_{n=2}^{\infty} (V_n/n)^2} / V_1 \times 100\%$，模拟电感的一阶滤波效应 |
| 主导谐波 | SVPWM 中 5/7/11/13 次为低次特征谐波，PWM 开关频率附近为高次谐波簇 |
| 测量标准 | IEEE 519 要求电网 THD < 5%，电机驱动通常以电流 THD < 8% 为设计目标 |

---

## 2. 📐 原理推导 ★★★☆☆ 📚

### 2.1 什么是THD？

总谐波失真（Total Harmonic Distortion）量化了波形偏离理想正弦的程度：

$$\text{THD} = \frac{\sqrt{\sum_{n=2}^{\infty} V_n^2}}{V_1} \times 100\%$$

其中 $V_1$ 为基波幅值，$V_n$ 为n次谐波幅值。

### 2.2 WTHD — 电流加权THD

电机绕组是感性负载（$Z_L = R + j\omega L$），对高次谐波有自然衰减。WTHD考虑了这一滤波效应：

$$\text{WTHD} = \frac{\sqrt{\sum_{n=2}^{\infty} (V_n/n)^2}}{V_1} \times 100\%$$

WTHD与电流THD（而非电压THD）更直接相关，是评价调制策略对电机影响的更准确指标。

### 2.3 不同调制策略的谐波特征

| 调制方式 | 基波利用率 | THD | 主导谐波 | 特点 |
|----------|-----------|-----|---------|------|
| SPWM | 50% Vdc | ~70% | 开关频率附近 | 低利用率 |
| SVPWM | 57.7% Vdc | ~55% | 开关频率附近 | 利用率+15.5% |
| DPWM | 57.7% Vdc | ~60% | 开关频率±基波 | 开关损耗-33% |
| 方波/六步 | 63.7% Vdc | >200% | 5,7,11,13… | 最高利用率 |

### 2.4 SVPWM的谐波优势

SVPWM通过零序注入将基波附近（3次、9次等3的倍数次）的低频谐波转移到高频：

- **SPWM**：开关频率附近有大量边带谐波
- **SVPWM**：同样的谐波能量被推向更高频率 → 更容易被电机电感滤除 → **电流THD更小**

### 2.5 影响电机性能的谐波

| 谐波 | 影响 |
|------|------|
| 3次（零序） | 不产生转矩脉动（Y接法无通路），但共模噪声 |
| 5次、7次 | 产生6倍基频转矩脉动（6fe→6次纹波） |
| 11次、13次 | 12fe转矩脉动 |
| PWM频率± | 高频铁损增加，可能激发绕组谐振 |

### 2.6 测量与标准

- IEEE 519：电网谐波标准（THD < 5%）
- 电机驱动常用THD < 8%作为设计目标
- 实际电流THD远低于电压THD（因$Z_L \propto \omega$衰减）

> **仿真体验**：拖动调制比m，观察SPWM/SVPWM/六步的谐波频谱变化和THD对比。

:::sim thd

---

## 3. 🔢 数学建模 ★★★☆☆ 📚

### 3.1 FFT 频谱分析基础

对于离散采样的电流/电压信号 $x[k]$，$k = 0, 1, \dots, N-1$，其离散傅里叶变换（DFT）为：

$$X[m] = \sum_{k=0}^{N-1} x[k] \cdot e^{-j 2\pi mk/N}, \quad m = 0, 1, \dots, N-1$$

其中：
- $x[k]$：第 $k$ 个采样点，单位依信号类型（A 或 V）
- $N$：FFT 点数（通常取 2 的幂次，如 256、512、1024）
- $X[m]$：第 $m$ 个频率分量的复幅值
- 频率分辨率：$\Delta f = f_s / N$，$f_s$ 为采样频率（Hz）

第 $m$ 个频率分量对应的实际频率：
$$f_m = m \cdot \Delta f = m \cdot \frac{f_s}{N} \quad [\text{Hz}]$$

### 3.2 谐波分解与幅值提取

从 FFT 结果中提取各次谐波的幅值：

基波幅值（$m_1 = f_e / \Delta f$，其中 $f_e$ 为基波电频率）：
$$V_1 = \frac{2|X[m_1]|}{N} \quad [\text{V}]$$

第 $n$ 次谐波幅值（$m_n = n \cdot f_e / \Delta f$）：
$$V_n = \frac{2|X[m_n]|}{N} \quad [\text{V}], \quad n = 2, 3, 5, 7, 11, \dots$$

注意：幅值计算中乘 2 是因为 DFT 的对称性（单边谱），直流分量（$m=0$）不乘 2。

### 3.3 离散域 THD 计算公式

在离散域中，THD 计算截断到 $N/2$（奈奎斯特频率）：

$$\text{THD} = \frac{\sqrt{\sum_{n=2}^{N/2} V_n^2}}{V_1} \times 100\%$$

实际工程中通常只计算到第 40 ~ 50 次谐波（高次谐波幅值极小，对 THD 贡献可忽略）：

$$\text{THD}_{\text{practical}} = \frac{\sqrt{\sum_{n=2}^{H_{\max}} V_n^2}}{V_1} \times 100\%, \quad H_{\max} = \min\left(\frac{N}{2}, \frac{f_s}{2f_e}\right)$$

其中 $H_{\max}$ 为可分析的最高谐波次数。

### 3.4 RMS 值与 THD 的关系

信号的总 RMS 值 $V_{rms}$ 与基波 RMS 值 $V_{1,rms}$ 及 THD 的关系：

$$V_{rms} = V_{1,rms} \cdot \sqrt{1 + \left(\frac{\text{THD}}{100}\right)^2} \quad [\text{V}]$$

由此可得，THD = 10% 时，总 RMS 仅比基波 RMS 大 0.5%。THD 对发热的贡献来自谐波电流在绕组电阻上的额外损耗：$P_{harmonic} = I_{1,rms}^2 R \cdot (\text{THD}_I/100)^2$，单位 W。

---

## 4. 💻 代码实现 ★★★☆☆ 🔧

### 4.1 嵌入式 FFT 实现（实数输入，基-2 DIT）

以下为适用于嵌入式 MCU 的实数 FFT 实现，利用共轭对称性将 N 点实数 FFT 映射为 N/2 点复数 FFT，节省一半计算量：

```c
#include <math.h>
#include <stdint.h>

#define FFT_N       256
#define FFT_LOG2N   8

typedef struct {
    float real;
    float imag;
} complex_t;

static uint16_t bit_reverse(uint16_t idx, uint8_t log2n)
{
    uint16_t rev = 0;
    for (uint8_t i = 0; i < log2n; i++) {
        rev = (rev << 1) | (idx & 1);
        idx >>= 1;
    }
    return rev;
}

void fft_radix2(complex_t *data, uint16_t n, uint8_t log2n)
{
    uint16_t i, j, k;
    for (i = 0; i < n; i++) {
        j = bit_reverse(i, log2n);
        if (i < j) {
            complex_t tmp = data[i];
            data[i] = data[j];
            data[j] = tmp;
        }
    }

    for (uint16_t step = 1; step < n; step <<= 1) {
        float angle = -M_PI / (float)step;
        complex_t w = {cosf(angle), sinf(angle)};

        for (uint16_t group = 0; group < n; group += (step << 1)) {
            complex_t wk = {1.0f, 0.0f};

            for (uint16_t pair = 0; pair < step; pair++) {
                uint16_t idx1 = group + pair;
                uint16_t idx2 = group + pair + step;

                complex_t t = {
                    wk.real * data[idx2].real - wk.imag * data[idx2].imag,
                    wk.real * data[idx2].imag + wk.imag * data[idx2].real
                };

                data[idx2].real = data[idx1].real - t.real;
                data[idx2].imag = data[idx1].imag - t.imag;
                data[idx1].real = data[idx1].real + t.real;
                data[idx1].imag = data[idx1].imag + t.imag;

                complex_t w_next = {
                    wk.real * w.real - wk.imag * w.imag,
                    wk.real * w.imag + wk.imag * w.real
                };
                wk = w_next;
            }
        }
    }
}

void real_fft(float *input, complex_t *output, uint16_t n)
{
    uint16_t half_n = n / 2;
    complex_t *cdata = output;

    for (uint16_t i = 0; i < half_n; i++) {
        cdata[i].real = input[2 * i];
        cdata[i].imag = input[2 * i + 1];
    }

    fft_radix2(cdata, half_n, FFT_LOG2N - 1);

    output[0].real = cdata[0].real + cdata[0].imag;
    output[0].imag = 0.0f;

    for (uint16_t k = 1; k < half_n; k++) {
        float angle = -M_PI * (float)k / (float)half_n;
        complex_t wp = {0.5f, -0.5f};
        complex_t wm = {0.5f, 0.5f};
        complex_t w = {cosf(angle), sinf(angle)};

        complex_t Hk = {
            cdata[k].real + cdata[half_n - k].real,
            cdata[k].imag - cdata[half_n - k].imag
        };

        complex_t Gk = {
            (cdata[k].imag + cdata[half_n - k].imag) * w.real -
            (cdata[k].real - cdata[half_n - k].real) * w.imag,
            (cdata[k].imag + cdata[half_n - k].imag) * w.imag +
            (cdata[k].real - cdata[half_n - k].real) * w.real
        };

        output[k].real = wp.real * Hk.real - wp.imag * Gk.real +
                         wm.real * Hk.real - wm.imag * (-Gk.real);
        output[k].imag = wp.real * Hk.imag - wp.imag * Gk.imag +
                         wm.real * (-Hk.imag) - wm.imag * (-Gk.imag);
    }

    output[half_n].real = cdata[0].real - cdata[0].imag;
    output[half_n].imag = 0.0f;
}
```

### 4.2 THD 计算与谐波提取

```c
typedef struct {
    float thd_percent;
    float wthd_percent;
    float v1_magnitude;
    float harmonic_magnitudes[50];
    uint8_t harmonic_count;
} thd_result_t;

thd_result_t calculate_thd(complex_t *fft_out, uint16_t n,
                            float f_sample, float f_fundamental)
{
    thd_result_t result = {0};
    float freq_res = f_sample / (float)n;
    uint16_t bin_fund = (uint16_t)(f_fundamental / freq_res + 0.5f);
    uint16_t half_n = n / 2;

    float v1 = sqrtf(fft_out[bin_fund].real * fft_out[bin_fund].real +
                     fft_out[bin_fund].imag * fft_out[bin_fund].imag) * 2.0f / (float)n;
    result.v1_magnitude = v1;

    float harmonic_sum_sq = 0.0f;
    float weighted_sum_sq = 0.0f;
    uint8_t harmonic_idx = 0;

    for (uint16_t m = bin_fund * 2; m < half_n; m++) {
        if (m % bin_fund == 0 && m / bin_fund <= 50) {
            float vn = sqrtf(fft_out[m].real * fft_out[m].real +
                            fft_out[m].imag * fft_out[m].imag) * 2.0f / (float)n;
            uint16_t harmonic_order = (uint16_t)((float)m * freq_res / f_fundamental + 0.5f);

            harmonic_sum_sq += vn * vn;
            weighted_sum_sq += (vn / (float)harmonic_order) * (vn / (float)harmonic_order);

            if (harmonic_idx < 50) {
                result.harmonic_magnitudes[harmonic_idx++] = vn;
            }
        }
    }

    result.harmonic_count = harmonic_idx;

    if (v1 > 0.0f) {
        result.thd_percent = sqrtf(harmonic_sum_sq) / v1 * 100.0f;
        result.wthd_percent = sqrtf(weighted_sum_sq) / v1 * 100.0f;
    }

    return result;
}
```

---

### 🔗 hpm_MC 代码实现参考

**谐波抑制措施**:
- **死区补偿** (`hpm_mcl_v2/hpm_mcl_cfg.h`): 编译宏 `HPM_MCL_ENABLE_DEAD_AREA_COMPENSATION` 使能
- **d/q轴解耦** (`hpm_mcl_v2/hpm_mcl_cfg.h`): 编译宏 `HPM_MCL_ENABLE_DQ_AXIS_DECOUPLING` 使能
- **角度预测** (`hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h`): theta_forecast 减少角度延迟引起的谐波
- **SVPWM**: 标准七段式 SVPWM 固有低谐波特性，配合 PWM 中心对齐模式

**调试工具** (`hpm_mcl_v2/core/control/`):
- `hpm_mcl_debug.h` 支持 FIFO trace 输出，可导出电流波形做 FFT 谐波分析

---

## 5. 🔧 参数整定 ★★★☆☆ 🔧

### 5.1 FFT 测量参数设置

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| FFT 点数 $N$ | 256 / 512 / 1024 | 取 2 的幂次；点数越大频率分辨率越高，但计算量 $O(N\log N)$ |
| 采样频率 $f_s$ | $\geq 40 \times f_e$ | 确保至少捕捉到 20 次谐波（奈奎斯特 $f_s/2$） |
| 采样窗口长度 $T_w$ | $N / f_s$ | 应包含整数个基波周期以避免频谱泄漏 |
| 同步采样触发 | PWM 载波同步 | 避免 PWM 开关噪声混叠至采样信号 |

### 5.2 频率分辨率与可检测谐波

频率分辨率：
$$\Delta f = \frac{f_s}{N} \quad [\text{Hz}]$$

为保证能分辨相邻谐波（如 5 次和 7 次间隔为 $2f_e$），需满足：
$$\Delta f < 2f_e \quad \Rightarrow \quad N > \frac{f_s}{2f_e}$$

示例：$f_e = 50\text{ Hz}$，$f_s = 10\text{ kHz}$，则 $N > 100$。取 $N = 256$ 时，$\Delta f = 39.06\text{ Hz}$，可分辨基波及各次谐波。

最高可检测谐波次数受奈奎斯特频率限制：
$$H_{\max} = \left\lfloor \frac{f_s}{2f_e} \right\rfloor$$

### 5.3 窗函数选择

为避免频谱泄漏（非整数周期截断导致的能量扩散），应选用合适的窗函数：

| 窗函数 | 主瓣宽度 | 旁瓣衰减 | 适用场景 |
|--------|---------|---------|---------|
| 矩形窗 | $4\pi/N$ | -13 dB | 整数周期截断（理想同步采样） |
| Hanning窗 | $8\pi/N$ | -31 dB | 通用场景，频率分辨率与泄漏抑制的平衡 |
| Hamming窗 | $8\pi/N$ | -41 dB | 类似 Hanning，最小旁瓣更优 |
| Blackman窗 | $12\pi/N$ | -58 dB | 高动态范围测量，需精确分辨小幅值谐波 |
| Flat-top窗 | 宽主瓣 | -44 dB | **幅值精度优先**（如 THD 绝对幅值测量） |

**电机 THD 测量推荐**：Hanning 窗作为默认选择；若需精确幅值则用 Flat-top 窗，但需增加 FFT 点数以补偿主瓣展宽。

### 5.4 典型 THD 指标参考

| 应用场景 | 电流 THD 设计目标 | 备注 |
|----------|------------------|------|
| 伺服驱动 | < 3% | 低转矩脉动要求 |
| 工业变频器 | < 5% | 通用场合 |
| 家电变频 | < 8% | 成本敏感，可适度放宽 |
| 电动工具 | < 10% | 高速/过调制区 |

---

## 6. 🔗 硬件约束 ★★★☆☆ ⚠️

### 6.1 电流传感器带宽

电流传感器的 -3 dB 带宽 $f_{BW}$ 决定了可测量谐波的上限：

- 要准确测量第 $n$ 次谐波，需 $f_{BW} \geq n \cdot f_e$
- **霍尔传感器**（如 ACS712/ACS758）：$f_{BW}$ 约 80 ~ 120 kHz，通常满足要求
- **采样电阻 + 运放**：运放增益带宽积 $GBW$ 限制，需 $\text{Gain} \times (n \cdot f_e) < GBW$
- **相关模块**：[HW-05 电流采样电路]()

### 6.2 ADC 分辨率对 THD 测量的影响

ADC 的量化噪声本质上是宽带噪声，直接影响 THD 测量精度：

- **量化噪声 RMS**：$V_{q,rms} = V_{LSB} / \sqrt{12}$，其中 $V_{LSB} = V_{ref} / 2^{\text{bits}}$
- **信噪比 SNR**：$\text{SNR} = 6.02 \times \text{bits} + 1.76 \quad [\text{dB}]$（理想 ADC）
- 12-bit ADC 理论 SNR ≈ 74 dB，对应 THD 底噪约 0.02%
- 实际测量中，ADC 有效位数（ENOB）通常低于标称位数 1 ~ 2 bits，需留裕量
- **相关模块**：[HW-04 ADC 采样与调理电路]()

### 6.3 抗混叠滤波器设计

在 ADC 前端必须设置抗混叠低通滤波器，截止频率 $f_c$ 应满足：

$$f_c \leq \frac{f_s}{2}$$

推荐的二阶有源低通滤波器（Sallen-Key 拓扑）：

- **截止频率**：$f_c = \frac{1}{2\pi\sqrt{R_1 R_2 C_1 C_2}}$，通常取 $f_c \approx 0.3 \sim 0.4 \times f_s$
- **阻尼系数**：$\zeta = 0.707$（Butterworth 响应），平坦通带 + 适度滚降
- **阶数建议**：二阶通常足够，若 $f_s$ 较低且开关频率谐波丰富，考虑四阶
- **相关模块**：[HW-04 ADC 采样与调理电路]()

### 6.4 PWM 开关噪声耦合

PWM 的 $dv/dt$ 通过 PCB 走线间的寄生电容耦合到电流采样电路，在谐波频谱中表现为开关频率及其倍频处的尖峰：

- **对策 1**：电流采样时刻与 PWM 开关沿错开（中心对齐 PWM 在计数器下溢/上溢时采样）
- **对策 2**：模拟前端加入共模扼流圈或差模滤波
- **对策 3**：数字后处理中使用中值滤波或 notch filter 滤除已知的开关频率分量
- **相关模块**：[EMC-02 PCB 布局与电磁兼容]()

---

## 7. 🚀 前沿拓展 ★★★★★ 💡

### 7.1 在线 THD 实时监测

传统 THD 测量依赖离线 FFT 后处理。在线 THD 监测允许在电机运行时实时评估电流质量：

- **滑窗 FFT**：每 N 个采样点计算一次 FFT，滑动步长 < N 实现重叠分析
- **Goertzel 算法**：仅计算特定次谐波（如 5/7/11/13）的能量，计算量远小于完整 FFT，适合实时嵌入
- **自适应阈值**：根据转速/负载动态调整 THD 告警阈值（低速轻载允许更高 THD）
- 应用场景：设备自诊断、预防性维护（THD 趋势上升预示轴承磨损或绕组劣化）

### 7.2 有源谐波补偿（Active Harmonic Compensation）

通过对特定次谐波注入反相补偿电压，实现谐波电流的闭环抑制：

- 在同步旋转坐标系（$dq^e$）下，5 次谐波对应 $-6\omega_e$ 旋转，7 次对应 $+6\omega_e$ 旋转
- 通过多个谐波同步坐标系 PI 控制器并联，各补偿一个次数的谐波
- 补偿电压叠加到 FOC 输出电压指令中
- **代价**：需要额外的谐波电流提取滤波器（如多同步旋转积分器 MSRI），增加 MCU 计算负载
- 在精密伺服中可将特定次谐波（如 5/7 次）降低 80% 以上

### 7.3 扩频 PWM（Spread-Spectrum PWM）

通过随机化或周期性调制 PWM 开关频率，将集中在开关频率的谐波能量扩散到更宽频带：

- **随机 PWM（RPWM）**：每个 PWM 周期的频率在 $[f_{sw} - \Delta f, f_{sw} + \Delta f]$ 内随机选取
- **周期性频率调制**：开关频率按正弦/三角规律围绕中心频率摆动
- 效果：峰值 EMI 降低 10 ~ 15 dB，但总谐波能量不变
- **代价**：引入了低频（调制频率）的额外谐波分量，需在电流 THD 和 EMI 峰值之间权衡
- 在汽车牵引逆变器中应用广泛（满足 CISPR 25 传导发射限值）

### 7.4 载波移相并联技术

多个逆变器模块并联时，采用载波移相（Interleaving）技术可等效提高开关频率并降低总 THD：

- $k$ 路并联，载波相位依次偏移 $2\pi/k$
- 等效开关频率 $f_{eq} = k \cdot f_{sw}$，电流纹波幅值降低约 $1/k$
- 在大电流伺服（多模块并联）和新能源储能逆变器中有实际应用

### 7.5 基于 AI/ML 的谐波趋势预测

利用机器学习方法建立 THD 与工况（转速、转矩、温度、母线电压）的回归模型：

- 输入特征：$\omega_e, I_q, V_{dc}, T_{module}$
- 输出目标：$THD_I$ 预测值
- 在线学习：正常运行中持续采集数据更新模型，检测 THD 偏离预测值的异常
- 应用价值：区分"调制策略固有的正常谐波"和"硬件老化/故障引入的异常谐波"

:::sim thd