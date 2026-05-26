# 🔗 交叉引用映射表

> **用途：** 汇总硬件路径与算法路径之间的所有交叉引用，帮助学习者快速定位关联知识

---

## 硬件→算法映射

### HW-01 电机本体基础 → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 电感参数 $L_d, L_q$ | ALG-05 有感FOC | PI参数设计：$K_p = L_s \cdot \omega_{bw}$ | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#5-参数整定) |
| 电感参数 $L_d, L_q$ | ALG-09 高频注入 | 凸极率 $\xi = (L_q-L_d)/(L_d+L_q)$ 决定HFI可行性 | [ALG-09](../algorithm/ALG-09-High-Frequency-Injection.md#2-原理推导) |
| 磁链参数 $\psi_f$ | ALG-07 无感观测器 | 速度估算：$\omega_e = E/\psi_f$ | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#3-数学建模) |
| 电阻参数 $R_s$ | ALG-15 前沿研究 | 磁链积分漂移：$\psi = \int(u - R_s i)dt$ | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#6-硬件约束) |
| 极对数 $p$ | ALG-01 FOC理论 | Park变换角度：$\theta_e = p \cdot \theta_m$ | [ALG-01](../algorithm/ALG-01-FOC-Theory.md#2-原理推导) |

### HW-02 电流采样电路 → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 采样精度/量化误差 | ALG-07 无感观测器 | 观测器角度精度受限于电流采样精度 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#6-硬件约束) |
| 采样精度/量化误差 | ALG-15 前沿研究 | 磁链积分中 $R_s i$ 项误差累积 | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#6-硬件约束) |
| 采样噪声 | ALG-09 高频注入 | 高频电流信噪比受采样噪声限制 | [ALG-09](../algorithm/ALG-09-High-Frequency-Injection.md#6-硬件约束) |
| 采样延迟 | ALG-05 有感FOC | 电流环带宽受采样延迟限制 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#6-硬件约束) |

### HW-03 位置传感器接口 → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 角度精度 | ALG-05 有感FOC | Park变换精度直接影响Id/Iq解耦 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#6-硬件约束) |
| 角度延迟 | ALG-07 无感观测器 | 观测器PLL需要补偿传感器延迟 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#6-硬件约束) |
| 分辨率 | ALG-05 有感FOC | 低速性能受编码器分辨率限制 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#6-硬件约束) |

### HW-04 MCU外设与通信 → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| PWM死区 | ALG-13 保护与优化 | 死区补偿算法需匹配实际死区时间 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |
| ADC同步采样 | ALG-15 前沿研究 | ZOH离散化模型假设电压电流同步采样 | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#6-硬件约束) |
| 中断优先级 | ALG-05 有感FOC | 控制环路时序影响电流环性能 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#6-硬件约束) |
| PWM频率 | ALG-09 高频注入 | 注入频率受PWM频率限制 | [ALG-09](../algorithm/ALG-09-High-Frequency-Injection.md#6-硬件约束) |
| PWM频率 | ALG-15 前沿研究 | 载波比 $f_{ratio} = f_{PWM}/f_e$ | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#6-硬件约束) |

### HW-05 功率器件与栅极驱动 → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 死区时间 | ALG-09 高频注入 | 注入信号精度受死区影响 | [ALG-09](../algorithm/ALG-09-High-Frequency-Injection.md#6-硬件约束) |
| 死区时间 | ALG-13 保护与优化 | 死区补偿需匹配功率器件开关特性 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |
| 开关损耗 | ALG-07 无感观测器 | 反电动势观测精度受PWM频率限制 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#6-硬件约束) |
| 开关损耗 | ALG-15 前沿研究 | PWM频率受开关损耗限制→载波比受限 | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#6-硬件约束) |
| 管压降 | ALG-13 保护与优化 | 死区补偿需考虑管压降非线性 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |

### HW-06 电源管理与保护 → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 保护阈值 | ALG-13 保护与优化 | 软件保护策略需与硬件保护电路协同 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |
| 母线电压 | ALG-13 保护与优化 | 弱磁控制受母线电压限制 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |
| 过流保护电路 | ALG-13 保护与优化 | 比较器+刹车电路是软件保护的硬件后盾 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |

### HW-07 热设计与EMC → 算法关联

| 硬件知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 温升 | ALG-13 保护与优化 | 温度影响参数漂移→降额策略 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |
| EMI | ALG-07 无感观测器 | 采样噪声影响观测器精度 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#6-硬件约束) |
| NTC安装位置 | ALG-13 保护与优化 | 热延迟导致温度保护滞后 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#6-硬件约束) |

---

## 算法→硬件映射

### ALG-01 FOC理论基础 → 硬件约束

| 算法知识点 | 依赖的硬件模块 | 约束来源 | 引用链接 |
|-----------|-------------|---------|---------|
| Clarke变换 | HW-01 电机本体 | 三相绕组结构决定变换矩阵 | [HW-01](../hardware/HW-01-Motor-Basics.md#技术原理) |
| Park变换 | HW-01 电机本体 | 极对数决定电角度/机械角度关系 | [HW-01](../hardware/HW-01-Motor-Basics.md#关键参数) |
| 转矩方程 | HW-01 电机本体 | 磁链参数决定转矩常数 | [HW-01](../hardware/HW-01-Motor-Basics.md#数学模型) |

### ALG-05 有感FOC实现 → 硬件约束

| 算法知识点 | 依赖的硬件模块 | 约束来源 | 引用链接 |
|-----------|-------------|---------|---------|
| PI参数设计 | HW-01 电机本体 | $K_p = L_s \cdot \omega_{bw}$，需准确电感 | [HW-01](../hardware/HW-01-Motor-Basics.md#参数测量方法) |
| SVPWM调制 | HW-05 功率器件 | 逆变器拓扑决定扇区划分 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#逆变器拓扑) |
| 电流采样 | HW-02 电流采样 | 采样电路设计影响电流环性能 | [HW-02](../hardware/HW-02-Current-Sensing.md#滤波器设计) |
| 编码器接口 | HW-03 位置传感器 | 角度精度影响Park变换 | [HW-03](../hardware/HW-03-Position-Sensor.md#角度校准) |
| 控制周期 | HW-04 MCU外设 | 中断优先级和DMA配置 | [HW-04](../hardware/HW-04-MCU-Peripherals.md#中断优先级) |

### ALG-07 无感FOC观测器 → 硬件约束

| 算法知识点 | 依赖的硬件模块 | 约束来源 | 引用链接 |
|-----------|-------------|---------|---------|
| 观测器精度 | HW-02 电流采样 | 采样精度决定观测器性能上限 | [HW-02](../hardware/HW-02-Current-Sensing.md#ADC精度分析) |
| 反电动势 | HW-05 功率器件 | PWM频率和死区影响电压重构 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#死区时间设计) |
| 磁链估计 | HW-01 电机本体 | 电阻参数准确性影响积分精度 | [HW-01](../hardware/HW-01-Motor-Basics.md#参数测量方法) |
| SMO抖振 | HW-04 MCU外设 | 开关频率限制抖振频率上限 | [HW-04](../hardware/HW-04-MCU-Peripherals.md#PWM生成与死区) |

### ALG-09 高频注入法 → 硬件约束

| 算法知识点 | 依赖的硬件模块 | 约束来源 | 引用链接 |
|-----------|-------------|---------|---------|
| 注入信号 | HW-05 功率器件 | 死区和管压降影响注入精度 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#死区时间设计) |
| 凸极效应 | HW-01 电机本体 | 电机结构决定凸极率 | [HW-01](../hardware/HW-01-Motor-Basics.md#关键参数) |
| 信噪比 | HW-02 电流采样 | 采样噪声限制位置估算精度 | [HW-02](../hardware/HW-02-Current-Sensing.md#滤波器设计) |
| 注入频率 | HW-04 MCU外设 | PWM频率限制注入频率上限 | [HW-04](../hardware/HW-04-MCU-Peripherals.md#PWM生成与死区) |

### ALG-13 保护与优化 → 硬件约束

| 算法知识点 | 依赖的硬件模块 | 约束来源 | 引用链接 |
|-----------|-------------|---------|---------|
| 死区补偿 | HW-05 功率器件 | 功率器件开关特性决定补偿参数 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#死区时间设计) |
| 弱磁控制 | HW-06 电源管理 | 母线电压限制弱磁范围 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#母线电容选型) |
| 保护算法 | HW-06 电源管理 | 软硬件保护需协同设计 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#过流保护电路) |
| 温度保护 | HW-07 热设计 | NTC安装位置和热延迟 | [HW-07](../hardware/HW-07-Thermal-EMC-Design.md#热阻计算) |

### ALG-15 前沿研究 → 硬件约束

| 算法知识点 | 依赖的硬件模块 | 约束来源 | 引用链接 |
|-----------|-------------|---------|---------|
| 载波比 | HW-05 功率器件 | PWM频率受开关损耗限制 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#开关损耗) |
| ADC同步 | HW-04 MCU外设 | 采样时序影响ZOH模型精度 | [HW-04](../hardware/HW-04-MCU-Peripherals.md#ADC同步采样) |
| 电流采样精度 | HW-02 电流采样 | 磁链积分精度受采样精度影响 | [HW-02](../hardware/HW-02-Current-Sensing.md#ADC精度分析) |
| 电阻参数 | HW-01 电机本体 | 低速精度依赖电阻准确性 | [HW-01](../hardware/HW-01-Motor-Basics.md#参数测量方法) |

---
## 硬件基础→电控硬件/算法映射

### EE-04 MOSFET器件原理 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| MOS结构/沟道形成 | HW-05 功率器件 | 理解功率MOSFET的阈值电压、导通电阻物理来源 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#MOSFET选型) |
| 输出特性（截止/线性/饱和） | HW-05 功率器件 | 功率MOSFET工作在开关状态（截止↔线性），线性区导通电阻决定损耗 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#开关损耗) |
| 寄生电容Cgs/Cgd/Cds | HW-05 功率器件 | 米勒效应成因——Cgd充放电导致开关延迟 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#米勒效应) |
| 阈值电压Vth | HW-05 功率器件 | 栅极驱动电压必须远大于Vth才能完全导通 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#栅极驱动设计) |

### EE-06 IGBT器件原理 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| IGBT结构（MOSFET+BJT） | HW-05 功率器件 | MOSFET输入特性+BJT导通特性：电压控制、低饱和压降 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#IGBT选型) |
| 导通压降Vce(sat) | HW-05 功率器件 | IGBT导通压降(~1.5-2V)远大于MOSFET(~mV级)但电流密度高 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#导通损耗) |
| 关断拖尾电流 | HW-05 功率器件 | 少子存储导致关断拖尾→高频下损耗急剧增加 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#关断损耗) |
| MOSFET vs IGBT选型 | HW-05 功率器件 | 低压高频→MOSFET，高压大功率→IGBT | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#器件选型对比) |

### EE-08 三相逆变器原理 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 六开关三相桥拓扑 | HW-05 功率器件 | 逆变器拓扑是功率器件工作的直接场景 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#逆变器拓扑) |
| 八种开关状态 | ALG-05 有感FOC | SVPWM的八个基本矢量来源于八种开关组合 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#SVPWM) |
| 电压空间矢量 | ALG-01 FOC理论 | 六边形电压空间矢量→SVPWM调制策略的物理基础 | [ALG-01](../algorithm/ALG-01-FOC-Theory.md#SVPWM) |
| 死区插入 | ALG-13 保护与优化 | 上下管直通短路→死区时间必须匹配开关速度 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#死区补偿) |

### EE-09 H桥与电机驱动拓扑 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| H桥四象限运行 | HW-05 功率器件 | H桥→三相桥：从四个开关到六个开关的拓扑演化 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#逆变器拓扑) |
| PWM调速原理 | ALG-01 FOC理论 | 占空比→等效电压：PWM调压是SVPWM的基础概念 | [ALG-01](../algorithm/ALG-01-FOC-Theory.md#PWM调制) |
| 续流二极管 | HW-05 功率器件 | 体二极管续流：感性负载关断时电流不能突变 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#体二极管) |
| H桥死区插入 | ALG-13 保护与优化 | H桥直通→三相桥直通：死区保护的拓扑级来源 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#死区补偿) |

### EE-01 电阻电容电感基础 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 采样电阻选型(Rshunt) | HW-02 电流采样 | Rshunt决定电流分辨率与功耗权衡：大电阻=高精度+高发热 | [HW-02](../hardware/HW-02-Current-Sensing.md#采样电阻选型) |
| 直流母线电容(Cdc) | HW-06 电源管理 | 电容容量决定纹波抑制和瞬态响应能力 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#母线电容选型) |
| 电机绕组电感(Ld/Lq) | ALG-05 有感FOC | 电感直接决定电流环PI参数：Kp=L×ωc，Ki=R×ωc | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#PI参数整定) |
| LC谐振 | HW-07 热设计EMC | EMI滤波器LC谐振可能放大PWM谐波而非衰减 | [HW-07](../hardware/HW-07-Thermal-EMC-Design.md#EMI滤波) |
| 寄生电感(采样电阻引线) | HW-02 电流采样 | 引线电感×di/dt产生电压尖峰，叠加到采样信号上 | [HW-02](../hardware/HW-02-Current-Sensing.md#滤波器设计) |

### EE-02 二极管与整流 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 续流二极管 | HW-05 功率器件 | MOSFET体二极管+外部续流二极管：感性负载关断时电流不能突变 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#体二极管) |
| 三相整流桥 | HW-06 电源管理 | AC→DC变换：二极管整流是电机驱动直流母线的前级 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#整流电路) |
| 快恢复二极管(FRD) | HW-05 功率器件 | 高频PWM场景需要trr<100ns的快恢复管，普通整流管trr过大导致损耗 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#器件选型对比) |
| 肖特基二极管 | HW-06 电源管理 | 低压大电流场景：Vf≈0.3V vs 普通二极管0.7V，效率更高 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#辅助电源) |

### EE-03 BJT基础 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| BJT图腾柱 | EE-05 MOSFET栅极驱动 | 推挽输出级→图腾柱栅极驱动：NPN+PNP互补对提供大电流驱动 | [EE-05](../electronics-basics/EE-05-MOSFET-Gate-Drive.md#3-直观理解) |
| BJT vs MOSFET特性 | HW-05 功率器件 | 电流驱动vs电压驱动、饱和压降vs导通电阻→功率器件选型基础 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#器件选型对比) |
| BJT(输入)→MOSFET(开关) | EE-06 IGBT原理 | IGBT=BJT导通特性+MOSFET输入特性=电压控制低饱和压降 | [EE-06](../electronics-basics/EE-06-IGBT-Principles.md#2-问题引入) |
| 达林顿管 | HW-05 功率器件 | 高电流增益复合管：小基极电流控制大集电极电流 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#IGBT选型) |

### EE-05 MOSFET栅极驱动 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 栅极电阻Rg选择 | HW-05 功率器件 | Rg决定开关速度：小Rg→快开关→低损耗+高EMI；大Rg→慢开关→高损耗+低EMI | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#栅极驱动设计) |
| 死区时间 | ALG-13 保护与优化 | 死区>1μs→零电流钳位效应→转矩脉动；需软件死区补偿 | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#死区补偿) |
| 米勒钳位 | HW-05 功率器件 | 下管Cgd耦合导致栅极电压抬升→误触发导通→桥臂直通 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#米勒效应) |
| PWM信号路径 | HW-04 MCU外设 | MCU→栅极驱动IC：长线需RC滤波/终端匹配防反射误触发 | [HW-04](../hardware/HW-04-MCU-Peripherals.md#PWM生成与死区) |
| 隔离驱动 | HW-05 功率器件 | 高压(310V)场景需光耦/数字隔离器，低压(48V)可用非隔离驱动 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#栅极驱动设计) |

### EE-07 运算放大器 → 电控关联

| 硬件基础知识点 | 影响的电控模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 运放Vos(输入失调电压) | HW-02 电流采样 | Vos×增益=电流测量误差：LM358(Vos=2mV)→200mA误差；OPA333(Vos=10μV)→可忽略 | [HW-02](../hardware/HW-02-Current-Sensing.md#运放选型) |
| 差分放大器CMRR | HW-02 电流采样 | 共模抑制比决定共模电压(相电压)抑制能力→电流采样精度 | [HW-02](../hardware/HW-02-Current-Sensing.md#差分放大) |
| LPF截止频率设计 | ALG-05 有感FOC | fc太低→相位滞后→电流环带宽受限；fc太高→PWM噪声混叠 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#电流采样) |
| 仪表放大器 | HW-03 位置传感器 | 编码器差分信号(A+/A-、B+/B-)需仪表放大器抑制共模噪声 | [HW-03](../hardware/HW-03-Position-Sensor.md#ABZ接口) |
| 轨到轨运放 | HW-02 电流采样 | 低压(3.3V)ADC需轨到轨运放才能充分利用ADC量程 | [HW-02](../hardware/HW-02-Current-Sensing.md#运放选型) |

---
## 控制理论→算法映射

### CT-03 频域分析与Bode图 → 算法关联

| 控制理论知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 增益裕度/相位裕度 | ADV-ALG-01 带宽设计 | 带宽设计的稳定性判据——相位裕度≥45° | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#稳定性判据) |
| 穿越频率ωc | ADV-ALG-01 带宽设计 | 电流环带宽=ωc，穿越频率决定闭环响应速度 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#电流环带宽) |
| Bode图绘制与分析 | ADV-ALG-01 带宽设计 | PI+Bode图=带宽设计的核心工具链 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#Bode图分析) |
| 一阶/二阶系统频域特性 | ADV-ALG-01 带宽设计 | Ls/R电气时间常数从Bode图直接读出 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#系统辨识) |

### CT-04/05 PID控制 → 算法关联

| 控制理论知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| PI参数时域整定(Z-N法) | ADV-ALG-14 PID深度整定 | Z-N法提供PI参数初值，再在电机上微调 | [ADV-ALG-13](../advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md#Z-N整定) |
| PI参数频域整定 | ADV-ALG-14 PID深度整定 | 零极点对消法：PI零点对消电机极点，精确推导Kp/Ki | [ADV-ALG-13](../advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md#零极点对消) |
| 积分作用与稳态误差 | ALG-05 有感FOC | 电流环积分消除稳态静差→Id=Id_ref | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#PI参数整定) |
| 微分作用与噪声放大 | ADV-ALG-14 PID深度整定 | 电流环通常只用PI（D项放大噪声） | [ADV-ALG-13](../advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md#D项取舍) |
| 抗积分饱和 | ALG-05 有感FOC | 电压饱和时积分不累积→Back-calculation法 | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#电流环) |
| 并联vs串联PID结构 | ADV-ALG-14 PID深度整定 | 电机控制中并联型PI最常用，参数物理意义明确 | [ADV-ALG-13](../advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md#并联型PID) |

### CT-06 前馈控制 → 算法关联

| 控制理论知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| 前馈+反馈复合控制 | ADV-ALG-01 前馈解耦 | dq交叉耦合前馈：ωLd*iq前馈到Vd，ωLq*id前馈到Vq | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-07-Feedforward-Decoupling.md#前馈解耦) |
| 扰动前馈补偿 | ADV-ALG-01 前馈解耦 | 反电动势前馈：ωψf前馈到Vq，减小速度突变时的电流波动 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-07-Feedforward-Decoupling.md#反电动势前馈) |
| 2-DOF控制结构 | ADV-ALG-01 前馈解耦 | PI(反馈)+前馈(开环)=2自由度复合控制 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-07-Feedforward-Decoupling.md#2-DOF结构) |
| 前馈不改变稳定性 | ADV-ALG-01 前馈解耦 | 前馈在闭环之外→不影响特征方程→不改变稳定性 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-07-Feedforward-Decoupling.md#稳定性分析) |

### CT-11 状态观测器 → 算法关联

| 控制理论知识点 | 影响的算法模块 | 影响机制 | 引用链接 |
|-----------|-------------|---------|---------|
| Luenberger观测器 | ALG-07 无感观测器 | Luenberger观测器是SMO/EKF的理论基础 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#Luenberger观测器) |
| 极点配置 | ALG-07 无感观测器 | 观测器增益设计=极点配置→观测器带宽≥2×速度环带宽 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#增益设计) |
| Kalman滤波器 | ALG-07 无感观测器 | EKF处理电机非线性模型→在线估计角度、速度、参数 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#EKF观测器) |
| 可观测性 | ALG-07 无感观测器 | 零速/低速时反电动势不可观测→需要HFI辅助 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#零速问题) |
| 观测器收敛速度 | ALG-07 无感观测器 | 观测器动态必须快于控制环→但增益太高→噪声放大 | [ALG-07](../algorithm/ALG-07-Sensorless-Observers.md#收敛分析) |

---

## 🔋 功率变换→电控关联映射

### PP-04 PFC → HW-06 电源管理与保护

| 知识点 | 关联目标 | 关联机制 | 引用 |
|--------|---------|---------|------|
| PFC 升压拓扑 | HW-06 母线预充电 | PFC 升压电感=预充电限流电感的升级版 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#4-技术原理) |
| IEC 61000-3-2 谐波限值 | HW-06 输入EMI滤波 | >75W 必须PFC的法规依据 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#6-工程案例) |
| CCM 平均电流控制 | HW-06 母线电压控制 | PFC电压环与电机母线电压控制的相似性 | [HW-06](../hardware/HW-06-Power-Management-Protection.md#4-技术原理) |

### PP-07 功率变换→电控系统集成

| 知识点 | 关联目标 | 关联机制 | 引用 |
|--------|---------|---------|------|
| 辅助电源多路输出 | HW-06 电源管理 | Flyback多路输出→栅极驱动15V+MCU5V+传感器3.3V | [HW-06](../hardware/HW-06-Power-Management-Protection.md#3-直观理解) |
| 直流母线电容选型 | HW-05 功率器件 | 纹波电流额定值决定电容寿命 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#6-工程案例) |
| 制动斩波器 | HW-05 功率器件 | Rbrake=Vdc²/Pbrake_max 选型与IGBT匹配 | [HW-05](../hardware/HW-05-Power-Devices-Gate-Drivers.md#4-技术原理) |
| EMI 滤波器 | HW-07 热设计与EMC | 共模/差模噪声路径与抑制 | [HW-07](../hardware/HW-07-Thermal-EMC-Design.md#4-技术原理) |

## 交叉引用密度统计

| 模块 | 算法关联/硬件约束数量 | 达标(≥2) |
|------|---------------------|----------|
| HW-01 | 5个算法关联 | ✅ |
| HW-02 | 4个算法关联 | ✅ |
| HW-03 | 3个算法关联 | ✅ |
| HW-04 | 5个算法关联 | ✅ |
| HW-05 | 5个算法关联 | ✅ |
| HW-06 | 3个算法关联 | ✅ |
| HW-07 | 3个算法关联 | ✅ |
| ALG-01 | 3个硬件约束 | ✅ |
| ALG-05 | 5个硬件约束 | ✅ |
| ALG-07 | 4个硬件约束 | ✅ |
| ALG-09 | 4个硬件约束 | ✅ |
| ALG-13 | 4个硬件约束 | ✅ |
| ALG-15 | 4个硬件约束 | ✅ |
| EE-01 | 5个电控关联 | ✅ |
| EE-02 | 4个电控关联 | ✅ |
| EE-03 | 4个电控关联 | ✅ |
| EE-04 | 4个电控关联 | ✅ |
| EE-05 | 5个电控关联 | ✅ |
| EE-06 | 4个电控关联 | ✅ |
| EE-07 | 5个电控关联 | ✅ |
| EE-08 | 4个电控关联 | ✅ |
| EE-09 | 4个电控关联 | ✅ |
| SDK-01 | 1个代码映射(架构类) | ✅ |
| SDK-02 | 7个代码映射 | ✅ |
| SDK-03 | 4个代码映射 | ✅ |
| SDK-04 | 3个代码映射 | ✅ |
| SDK-05 | 2个代码映射 | ✅ |
| SDK-06 | 4个代码映射 | ✅ |

**所有基础篇模块均满足"至少2个交叉引用"的要求。SDK分析类模块以代码映射为主要关联方式。**

---

## 高级篇→基础篇映射

### ADV-HW-01 PWM深度配置与电流采样时序联动 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 中心对齐模式选择 | HW-04 MCU外设 | 从"怎么配"深化到"为什么必须这样配" | [HW-04](../hardware/HW-04-MCU-Peripherals.md#PWM生成与死区) |
| PWM模式0/1选择 | HW-04 MCU外设 | 从"寄存器配置"深化到"有效电平与极性的组合逻辑" | [HW-04](../hardware/HW-04-MCU-Peripherals.md#PWM生成与死区) |
| 单电阻采样移相 | ALG-05 有感FOC | 从"三相采样"深化到"单电阻移相时刻的精确控制" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#电流采样) |
| 死区补偿位置 | ALG-13 保护与优化 | 从"死区补偿原理"深化到"在控制链中的精确位置" | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#死区补偿) |
| PWM触发ADC时序 | HW-04 MCU外设 | 从"ADC基本配置"深化到"TRGO→ADC→中断的完整触发链" | [HW-04](../hardware/HW-04-MCU-Peripherals.md#ADC同步采样) |
| 载波比 | ALG-15 前沿研究 | 从"概念"深化到"载波比与控制性能的定量关系" | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#低载波比) |

### ADV-HW-02 ADC深度配置与DMA数据搬运 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 注入组vs规则组 | HW-04 MCU外设 | 从"ADC基本用法"深化到"注入组高优先级保证电流采样时序" | [HW-04](../hardware/HW-04-MCU-Peripherals.md#ADC同步采样) |
| 双ADC同步采样 | HW-02 电流采样 | 从"采样电路"深化到"双ADC同步消除相位误差" | [HW-02](../hardware/HW-02-Current-Sensing.md#采样方案) |
| DMA双缓冲 | HW-04 MCU外设 | 从"DMA基本配置"深化到"HT/TC中断实现零拷贝并行" | [HW-04](../hardware/HW-04-MCU-Peripherals.md#DMA配置) |
| PWM触发ADC | ADV-HW-01 PWM深度 | ADC触发时序与PWM配置的联动 | [ADV-HW-01](../advanced/hardware-algorithm-bridge/ADV-HW-01-PWM-Current-Sampling.md#PWM触发ADC) |
| 数据对齐与标幺值 | ADV-ALG-10 标幺值 | ADC原始值→有符号值→标幺值的完整转换链 | [ADV-ALG-10](../advanced/algorithm/ADV-ALG-09-Per-Unit-Fixed-Point.md#数据对齐) |

### ADV-HW-03 编码器深度处理与测速方法 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 多圈编码器处理 | HW-03 位置传感器 | 从"单圈接口"深化到"多圈计数与掉电保持" | [HW-03](../hardware/HW-03-Position-Sensor.md#编码器类型) |
| ABZ正交解码 | HW-03 位置传感器 | 从"接口协议"深化到"4倍频/滤波/最大频率限制" | [HW-03](../hardware/HW-03-Position-Sensor.md#ABZ接口) |
| 旋变解码 | HW-03 位置传感器 | 从"旋变原理"深化到"软件RDC(Type II PLL)实现" | [HW-03](../hardware/HW-03-Position-Sensor.md#旋变) |
| 霍尔角度平滑 | ALG-05 有感FOC | 从"霍尔换相"深化到"线性插值/PLL平滑算法" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#编码器接口) |
| 编码器校准对齐 | ALG-05 有感FOC | 从"角度偏移"深化到"直流对齐/高频注入对齐方法" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#角度校准) |
| 测速方法(M/T/PLL) | ALG-05 有感FOC | 从"简单差分"深化到"M/T法/PLL测速的精度对比" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#速度计算) |

### ADV-ALG-01 控制环带宽设计与滤波器工程 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 电流环带宽设计 | ALG-05 有感FOC | 从"PI参数公式"深化到"带宽驱动的系统设计方法" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#PI参数整定) |
| 速度环带宽设计 | ALG-05 有感FOC | 从"经验整定"深化到"对称最优法" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#速度环) |
| 滤波器设计 | ADV-HW-01 PWM深度 | 电流采样滤波与PWM时序的协调 | [ADV-HW-01](../advanced/hardware-algorithm-bridge/ADV-HW-01-PWM-Current-Sampling.md#PWM触发ADC) |
| Bode图分析 | ALG-01 FOC理论 | 从"开环/闭环概念"深化到"频域稳定性判据" | [ALG-01](../algorithm/ALG-01-FOC-Theory.md#控制结构) |

### ADV-ALG-05 弱磁控制与MTPA深度 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 电压椭圆与电流圆 | ALG-13 保护与优化 | 从"弱磁概念"深化到"电压/电流约束的几何分析" | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#弱磁控制) |
| MTPA轨迹推导 | ALG-13 保护与优化 | 从"MTPA概念"深化到"Lagrange乘子法完整推导" | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#MTPA) |
| 弱磁与MTPA协调 | ALG-13 保护与优化 | 从"独立介绍"深化到"全速域Id决策流程" | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#弱磁与MTPA) |
| 深度弱磁(过调制) | ADV-HW-01 PWM深度 | 过调制策略与PWM配置的关系 | [ADV-HW-01](../advanced/hardware-algorithm-bridge/ADV-HW-01-PWM-Current-Sampling.md#载波比) |
| 弱磁稳定性 | ADV-ALG-01 带宽设计 | 弱磁PI带宽与电流环带宽的协调 | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#电流环带宽) |

### ADV-ALG-07 前馈解耦与扰动补偿 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| dq交叉耦合 | ALG-05 有感FOC | 从"PI控制"深化到"前馈+反馈复合控制" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#电流环) |
| 反电动势前馈 | ALG-01 FOC理论 | 从"电压方程"深化到"可测扰动的前馈补偿" | [ALG-01](../algorithm/ALG-01-FOC-Theory.md#数学模型) |
| 前馈位置 | ADV-ALG-14 PID结构 | 前馈加在PI输出后而非输入的数学证明 | [ADV-ALG-13](../advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md#并联型PID) |
| 速度前馈 | ALG-05 有感FOC | 从"速度环PI"深化到"速度前馈+PI复合控制" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#速度环) |

### ADV-ALG-09 标幺值系统与定点数运算 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 标幺值基值选取 | ALG-15 前沿研究 | 从"浮点算法"深化到"标幺值系统设计" | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#离散化) |
| Q格式定点数 | ALG-15 前沿研究 | 从"连续域"深化到"资源受限平台的定点实现" | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#离散化) |
| CORDIC算法 | HW-04 MCU外设 | 从"软件计算"深化到"STM32G4硬件CORDIC加速" | [HW-04](../hardware/HW-04-MCU-Peripherals.md#计算加速) |
| 浮点转定点 | ADV-ALG-14 PID结构 | 定点PID实现与浮点PID的参数对应 | [ADV-ALG-13](../advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md#PI整定) |

### ADV-ALG-13 PID结构选择与深度整定 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 串联vs并联PID | ALG-05 有感FOC | 从"PI控制器"深化到"PID结构选择与参数转换" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#PI参数整定) |
| 抗积分饱和 | ALG-05 有感FOC | 从"基本PI"深化到"Back-calculation+积分限幅" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#电流环) |
| 零极点对消法 | ADV-ALG-01 带宽设计 | 从"带宽设计"深化到"PI参数的精确推导" | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#电流环带宽) |
| 对称最优法 | ADV-ALG-01 带宽设计 | 从"速度环带宽"深化到"对称最优法PI参数推导" | [ADV-ALG-01](../advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md#速度环带宽) |

### ADV-ALG-15 问题定位与调试方法论 → 基础篇关联

| 高级知识点 | 关联的基础模块 | 深化关系 | 引用链接 |
|-----------|-------------|---------|---------|
| 电流环问题定位 | ALG-05 有感FOC | 从"PI参数"深化到"振荡/稳态误差的系统性排查" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md#电流环) |
| 角度问题定位 | ADV-HW-03 编码器 | 从"编码器接口"深化到"角度偏移/跳变的诊断方法" | [ADV-HW-03](../advanced/hardware-algorithm-bridge/ADV-HW-03-Encoder-Speed.md#编码器校准) |
| 采样问题定位 | ADV-HW-02 ADC/DMA | 从"ADC配置"深化到"偏置/增益/时序错误的排查" | [ADV-HW-02](../advanced/hardware-algorithm-bridge/ADV-HW-02-ADC-DMA.md#数据对齐) |
| PWM问题定位 | ADV-HW-01 PWM深度 | 从"PWM配置"深化到"死区效应/抖动/互补异常的诊断" | [ADV-HW-01](../advanced/hardware-algorithm-bridge/ADV-HW-01-PWM-Current-Sampling.md#死区补偿) |
| 系统性调试流程 | ALG-13 保护与优化 | 从"保护算法"深化到"六步法系统化调试方法论" | [ALG-13](../algorithm/ALG-13-Protection-Optimization.md#保护策略) |

### SYS-01~04 系统与方法论 → 基础篇关联

| 高级模块 | 关联的基础模块 | 深化关系 | 引用链接 |
|---------|-------------|---------|---------|
| SYS-01 设计模式 | ALG-05 有感FOC | 从"控制代码"深化到"可维护/可扩展的软件架构" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md) |
| SYS-02 变频器vs电控 | ALG-01 FOC理论 | 从"FOC算法"深化到"不同应用场景的算法选择" | [ALG-01](../algorithm/ALG-01-FOC-Theory.md) |
| SYS-03 PFC vs 电控 | ALG-05 有感FOC | 从"FOC双环"深化到"PFC/FOC双环结构的类比与迁移" | [ALG-05](../algorithm/ALG-05-Sensored-FOC.md) |
| SYS-04 仿真到离散域 | ALG-15 前沿研究 | 从"离散化概念"深化到"完整的仿真→代码验证流程" | [ALG-15](../algorithm/ALG-15-Advanced-Research.md#离散化) |

---

## 高级篇交叉引用密度统计

| 模块 | 基础篇关联数 | 高级篇内部关联数 | 达标(≥2) |
|------|-----------|--------------|----------|
| ADV-HW-01 | 6 | 2 (→ADV-HW-02, →ADV-ALG-05) | ✅ |
| ADV-HW-02 | 5 | 2 (→ADV-HW-01, →ADV-ALG-09) | ✅ |
| ADV-HW-03 | 6 | 1 (→ADV-ALG-15) | ✅ |
| ADV-ALG-01 | 4 | 3 (→ADV-ALG-05, →ADV-ALG-07, →ADV-ALG-13) | ✅ |
| ADV-ALG-05 | 5 | 2 (→ADV-ALG-01, →ADV-HW-01) | ✅ |
| ADV-ALG-07 | 4 | 2 (→ADV-ALG-13, →ADV-ALG-01) | ✅ |
| ADV-ALG-09 | 4 | 2 (→ADV-ALG-13, →SYS-04) | ✅ |
| ADV-ALG-13 | 4 | 3 (→ADV-ALG-01, →ADV-ALG-07, →ADV-ALG-09) | ✅ |
| ADV-ALG-15 | 5 | 3 (→ADV-HW-01, →ADV-HW-02, →ADV-HW-03) | ✅ |
| SYS-01 | 1 | 0 | ✅(系统方法论类) |
| SYS-02 | 1 | 1 (→SYS-03) | ✅ |
| SYS-03 | 1 | 1 (→SYS-02) | ✅ |
| SYS-04 | 1 | 1 (→ADV-ALG-09) | ✅ |

**所有高级篇模块均满足交叉引用要求（系统方法论类模块因其跨领域性质，基础篇关联数要求放宽）。**

---

## 🔗 hpm_MC 代码 → 知识库文档 映射表

| hpm_MC 源码路径 | 关联知识库文档 |
|---------------|--------------|
| `hpm_mcl_v2/core/control/hpm_mcl_control.h` | ALG-01(FOC理论), ALG-07(PI电流调节器), ADV-ALG-01(带宽设计), ADV-ALG-14(PID整定), SDK-02-HPM-MC-v2-Core-Loop.md |
| `hpm_mcl_v2/core/loop/hpm_mcl_loop.h` | ALG-13(有感FOC), SDK-02-HPM-MC-v2-Core-Loop.md, ALG-02(初始位置检测) |
| `hpm_mcl/inc/hpm_smc.h` | ALG-01(无感观测器), CT-11(观测器设计), SDK-01-HPM-MC-Architecture.md |
| `hpm_mcl/inc/hpm_hfi.h` | ALG-10(高频注入), ALG-01(无感观测器), CT-11(观测器设计) |
| `hpm_mcl_v2/core/detect/hpm_mcl_detect.h` | ALG-14(保护优化), SDK-03-HPM-MC-v2-Detect.md, HW-06(电源保护) |
| `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h` | ALG-08(前沿研究), ADV-ALG-01(前馈解耦), SDK-04-HPM-MC-v2-Hybrid-Ctrl.md, CT-06(前馈控制) |
| `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h` | ALG-06(MTPA弱磁), ALG-11(速度环转矩), ADV-ALG-13(弱磁), SDK-05-HPM-MC-v2-Path-Plan.md |
| `hpm_mcl/inc/hpm_foc.h` | ALG-01(FOC理论), ALG-13(有感FOC), ALG-01(Clarke/Park变换), ALG-09(死区补偿) |
| `hpm_mcl/inc/hpm_block.h` | ALG-13(有感FOC), MC-LIB-Six-Step.md |
| `hpm_mcl/inc/hpm_over_zero.h` | ALG-01(无感观测器), MC-LIB-Six-Step.md |
| `hpm_mcl/inc/hpm_motor_math.h` | ALG-01(Clarke/Park变换), ADV-ALG-10(标幺值定点) |
| `hpm_mcl/inc/hpm_bldc_define.h` | ALG-14(保护优化), ALG-05(电流采样), HW-01(电机本体) |
| `hpm_mcl_v2/hpm_mcl_physical.h` | HW-01(电机本体), HW-06(电源保护), SDK-03-HPM-MC-v2-Detect.md |
| `hpm_mcl_v2/hpm_mcl_math.h` | ADV-ALG-10(标幺值定点) |
| `hpm_mcl_v2/hpm_mcl_cfg.h` | ALG-14(保护优化), ALG-04(THD谐波), ALG-09(死区补偿) |
| `hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h` | HW-03(位置传感器), ADV-HW-03(编码器测速), ALG-15(位置速度观测器) |
| `hpm_mcl_v2/core/sensor/hpm_mcl_analog.h` | HW-02(电流采样), ALG-05(电流采样时序), ADV-HW-02(ADC/DMA) |
| `hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h` | HW-04(MCU外设), HW-05(功率器件), ADV-HW-01(PWM采样) |
| `hpm_mcl_v2/encoder/hpm_mcl_abz.h` | HW-03(位置传感器), ADV-HW-03(编码器测速) |
| `hpm_mcl_v2/encoder/hpm_mcl_uvw.h` | HW-03(位置传感器) |
| `hpm_mcl_v2/core/control/hpm_mcl_filter.h` | ADV-ALG-01(带宽设计), SYS-04(仿真到离散) |
| `hpm_mcl_v2/core/control/hpm_mcl_debug.h` | ADV-ALG-08(调试方法论), ALG-04(THD谐波) |
| `hpm_MC/samples/motor_ctrl/bldc_foc/` | ALG-13(有感FOC), SDK-02-HPM-MC-v2-Core-Loop.md, SDK-06-HPM-MC-Sample-Apps.md |
| `hpm_MC/samples/motor_ctrl/bldc_hfi/` | ALG-10(高频注入), SDK-06-HPM-MC-Sample-Apps.md |
| `hpm_MC/samples/motor_ctrl/bldc_smc/` | ALG-01(无感观测器), SDK-06-HPM-MC-Sample-Apps.md |
| `hpm_MC/samples/motor_ctrl/bldc_offline_param_detection/` | SDK-03-HPM-MC-v2-Detect.md, SDK-06-HPM-MC-Sample-Apps.md |

---

## 🔗 知识库文档 → hpm_MC 代码 映射表

| 知识库文档 | 关联 hpm_MC 源码 |
|----------|-----------------|
| ALG-01 FOC理论 | `hpm_mcl_v2/core/control/hpm_mcl_control.h`, `hpm_mcl/inc/hpm_foc.h` |
| ALG-05 有感FOC | `hpm_mcl_v2/core/loop/hpm_mcl_loop.h`, `hpm_mcl/inc/hpm_foc.h`, `hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h` |
| ALG-07 无感观测器 | `hpm_mcl/inc/hpm_smc.h`, `hpm_mcl/inc/hpm_hfi.h`, `hpm_mcl/inc/hpm_over_zero.h` |
| ALG-09 高频注入 | `hpm_mcl/inc/hpm_hfi.h` |
| ALG-13 保护优化 | `hpm_mcl_v2/core/detect/hpm_mcl_detect.h`, `hpm_mcl_v2/hpm_mcl_cfg.h`, `hpm_mcl/inc/hpm_bldc_define.h` |
| ALG-15 前沿研究 | `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h`, `hpm_mcl_v2/core/detect/` |
| ALG-01 Clarke/Park | `hpm_mcl/inc/hpm_motor_math.h`, `hpm_mcl/inc/hpm_foc.h` |
| ALG-02 电流采样 | `hpm_mcl_v2/core/sensor/hpm_mcl_analog.h`, `hpm_mcl/inc/hpm_bldc_define.h` |
| ALG-10 过调制 | `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| ALG-03 PI调节器 | `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| ALG-06 位置速度观测器 | `hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h`, `hpm_mcl_v2/encoder/hpm_mcl_abz.h` |
| ALG-11 MTPA弱磁 | `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h` |
| ALG-14 THD谐波 | `hpm_mcl_v2/hpm_mcl_cfg.h`, `hpm_mcl_v2/core/control/hpm_mcl_debug.h` |
| ALG-04 死区补偿 | `hpm_mcl_v2/hpm_mcl_cfg.h`, `hpm_mcl/inc/hpm_foc.h` |
| ALG-08 初始位置检测 | `hpm_mcl_v2/core/loop/hpm_mcl_loop.h`, `hpm_mcl/inc/hpm_hfi.h` |
| ALG-12 速度环转矩 | `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h`, `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h` |
| HPM-MC-Architecture | `hpm_mcl_v2/*`, `hpm_mcl/*`（全量覆盖） |
| HPM-MC-v2-Core-Loop | `hpm_mcl_v2/core/loop/hpm_mcl_loop.h`, `hpm_mcl_v2/core/control/hpm_mcl_control.h`, `hpm_mcl_v2/core/sensor/*`, `hpm_mcl_v2/core/detect/*` |
| HPM-MC-v2-Detect | `hpm_mcl_v2/core/detect/hpm_mcl_detect.h`, `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| HPM-MC-v2-Hybrid-Ctrl | `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h`, `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h` |
| HPM-MC-v2-Path-Plan | `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h` |
| HPM-MC-Sample-Apps | `hpm_MC/samples/motor_ctrl/*/` |
| CT-04 PID控制 | `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| CT-05 PID整定 | `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| CT-06 前馈控制 | `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h` |
| CT-10 状态空间 | `hpm_mcl_v2/core/control/` |
| CT-11 观测器设计 | `hpm_mcl/inc/hpm_smc.h`, `hpm_mcl/inc/hpm_hfi.h` |
| CT-12 状态反馈 | `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h` |
| CT-13 LQR/LQG | `hpm_mcl_v2/core/control/hpm_mcl_control.h`, `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h` |
| HW-01 电机本体 | `hpm_mcl_v2/hpm_mcl_physical.h`, `hpm_mcl/inc/hpm_bldc_define.h` |
| HW-02 电流采样 | `hpm_mcl_v2/core/sensor/hpm_mcl_analog.h` |
| HW-03 位置传感器 | `hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h`, `hpm_mcl_v2/encoder/hpm_mcl_abz.h`, `hpm_mcl_v2/encoder/hpm_mcl_uvw.h` |
| HW-04 MCU外设 | `hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h` |
| HW-05 功率器件 | `hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h` |
| HW-06 电源保护 | `hpm_mcl_v2/core/detect/hpm_mcl_detect.h`, `hpm_mcl_v2/hpm_mcl_physical.h` |
| HW-07 热设计EMC | `hpm_mcl_v2/hpm_mcl_cfg.h` |
| ADV-ALG-01 带宽设计 | `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| ADV-ALG-13 MTPA弱磁 | `hpm_mcl_v2/core/control/hpm_mcl_path_plan.h` |
| ADV-ALG-01 前馈解耦 | `hpm_mcl_v2/core/control/hpm_mcl_hybrid_ctrl.h` |
| ADV-ALG-10 标幺值定点 | `hpm_mcl_v2/hpm_mcl_math.h`, `hpm_mcl/inc/hpm_motor_math.h` |
| ADV-ALG-14 PID整定 | `hpm_mcl_v2/core/control/hpm_mcl_control.h` |
| ADV-ALG-08 调试方法论 | `hpm_mcl_v2/core/control/hpm_mcl_debug.h` |
| ADV-HW-01 PWM采样 | `hpm_mcl_v2/core/drivers/hpm_mcl_drivers.h` |
| ADV-HW-02 ADC/DMA | `hpm_mcl_v2/core/sensor/hpm_mcl_analog.h` |
| ADV-HW-03 编码器测速 | `hpm_mcl_v2/core/sensor/hpm_mcl_encoder.h`, `hpm_mcl_v2/encoder/hpm_mcl_abz.h` |
| SYS-01~04 方法论 | `hpm_mcl_v2/core/*`（架构参考） |
| MC-LIB全系列 | 对比章节见各文档末尾「与hpm_MC对比」 |

---

## 🔗 HPM 官方知识库文章 → 知识库文档

| 官方文章 | URL | 关联文档 |
|---------|-----|---------|
| 先楫电机库简介 | kb.hpmicro.com/2025/01/03/ | SDK-01-HPM-MC-Architecture.md |
| 电机库使用入门 | kb.hpmicro.com/2024/06/25/ | ALG-01(FOC), ALG-13(有感FOC), SDK-06-HPM-MC-Sample-Apps.md |
| BLDC BLOCK 换相控制 | kb.hpmicro.com/2025/08/07/ | SDK-06-HPM-MC-Sample-Apps.md, ALG-13(有感FOC) |
| BLDC 过零控制技术 | kb.hpmicro.com/2025/09/23/ | ALG-01(无感观测器) |
| PID→3P-3Z 控制器 | kb.hpmicro.com/2024/10/16/ | SDK-02-HPM-MC-v2-Core-Loop.md, CT-04/05(PID) |
| 锁相环滤波器性能分析 | kb.hpmicro.com/2024/10/31/ | ALG-15(位置速度观测器) |
| 力位混合控制库使用指南 | kb.hpmicro.com/2025/12/29/ | SDK-04-HPM-MC-v2-Hybrid-Ctrl.md |
| 关节电机驱动电路图解 | kb.hpmicro.com/2026/03/20/ | HW-02(电流采样), HW-05(功率器件) |
| PLB实现滤波器 | kb.hpmicro.com/2024/12/27/ | HW-04(MCU外设) |
