# CT-05: PID 整定与工程实现

**副标题：从Ziegler-Nichols到频域法，从anti-windup到bumpless transfer——在真实电机上把PID调好调稳的完整工具箱**
**难度：** ★★★★☆ 专业级
**适用对象：** 伺服驱动调试工程师、电机控制系统开发者
**前置知识：** PID原理（CT-04）、伯德图（CT-03）、时域响应分析（CT-02）

---

## 1. 📌 核心摘要

**一句话讲清楚**：PID参数整定是「从数学公式到真实电机」的最后一步——Ziegler-Nichols提供粗调起点，频域法提供精确优化，而anti-windup、微分滤波、bumpless transfer等工程技巧确保PID在饱和、切换、噪声等真实工况下仍能正常工作。

**认知挂钩**：很多工程师看了教科书，Kp=Ls×ωc一套公式算出来，烧到DSP里却发现：「怎么还是震荡？」「启动时过流了！」「速度环和电流环切换时有冲击！」——**理论公式只给你起点，工程技巧（anti-windup、bumpless transfer、滤波）才是让PID在真实硬件上不翻车的保障。**

**与FOC算法的关联**：
- 🔗 **电流环PI整定完整流程**：从电机参数→ωc选择→Kp/Ki计算→离散化→anti-windup→示波器验证
- 🔗 **速度环自整定**：自动注入阶跃信号，分析响应，在线优化PI参数
- 🔗 **启动饱和保护**：anti-windup防止电流环积分在启动瞬间爆炸

---

## 2. 🤔 问题引入

### 工程师的真实困惑

**场景1：Z-N整定法得到的参数导致剧烈震荡**
```
工程师A:"按Ziegler-Nichols阶跃响应法算了Kp和Ki，
      烧进去后电机震荡得像地震..."
问题现象:
- Z-N法给出Kp=4.5, Ki=900
- 电流环严重震荡，频率约300Hz
- 远不如公式法(Ls×ωc)的整定结果
```

**场景2：PI参数在空载和满载下表现不同**
```
工程师B:"空载时调好的电流环，挂了负载后震荡..."
问题现象:
- 空载：阶跃响应完美，tr=1.5ms, Mp<1%
- 满载：出现2~3次震荡后稳定，Mp≈15%
- 不是电机参数的问题（参数已实测）
```

**场景3：多模式切换时的冲击**
```
工程师C:"从速度模式切换到位置模式时，电机猛地跳一下..."
问题现象:
- 切换瞬间Vq有阶跃跳变
- 积分器状态未处理导致切换冲击
- 对精密加工应用不可接受
```

### 核心问题

- Z-N法震荡 → Z-N是为过程控制（大时滞、慢系统）设计的，不适合快响应的电机电流环
- 负载变化 → 反电动势耦合在满载时更强，需要更大解耦前馈或不同的PI参数
- 模式切换冲击 → 缺少bumpless transfer（无扰切换）机制

### 学习目标

读完本模块，你将能够：
✅ **在多种PI整定方法中做出正确选择**——什么场景用什么方法
✅ **在DSP上实现完整的工程PI**——anti-windup、bumpless transfer、微分滤波
✅ **调试真实的电流环和速度环**——知道每一步看什么信号、怎么判断好坏

---

## 3. 💡 直观理解

### 类比1：Z-N法 = 用力关门，频域法 = 精确调节

**生活场景**：Ziegler-Nichols终极增益法：逐步加大关门力度，直到门撞上门框「哐」一声（等幅震荡），然后根据这个力度计算「最佳」关门力度。结果往往是：门确实关得快，但震得门框响。

**电机对应**：Z-N法让系统处于等幅震荡临界点，记录临界增益$K_u$和临界周期$T_u$，然后用经验公式计算PI参数。对于快速系统（电流环$\tau_e$仅几毫秒），Z-N得到的参数往往过于激进。

### 类比2：Anti-windup = 防止油门踩到底后松不回来

**生活场景**：定速巡航——上坡时（误差大）自动加油，如果坡太长油门一直在最大值（饱和），下坡时需要减油但刹车反应不过来（退出饱和需要时间）。

**电机对应**：电流环Vq输出达到Vdc极限后饱和，积分器继续累积→Vq想回落时积分器状态需先「消化」→电流不可控的时间窗口。Anti-windup在饱和发生时限制积分。

---

## 4. 🔬 技术原理

### 4.1 PI整定方法对比

#### 4.1.1 Ziegler-Nichols阶跃响应法

对被控对象施加阶跃输入，记录S形响应曲线的拐点斜率R和延迟时间L：

| 控制器 | Kp | Ti | Td |
|--------|-----|----|-----|
| P | 1/(RL) | — | — |
| PI | 0.9/(RL) | 3L | — |
| PID | 1.2/(RL) | 2L | 0.5L |

**FOC电流环适用性**：❌ 不推荐！电机RL模型响应太快（ms级），S形曲线不明显，且Z-N为振荡-衰减设计（ζ≈0.25），不适合要求Mp<5%的电流环。

#### 4.1.2 Ziegler-Nichols终极增益法

逐步增大Kp（纯P控制）直到系统等幅振荡，记录临界增益$K_u$和振荡周期$T_u$：

| 控制器 | Kp | Ti | Td |
|--------|-----|----|----|
| PI | 0.45Ku | Tu/1.2 | — |

**FOC电流环适用性**：⚠️ 慎用！找到Ku的过程本身就是危险的（让电流环震荡可能损坏硬件）。

#### 4.1.3 频域法——最适合FOC

基于电机参数和期望带宽直接计算（推导过程见 [CT-04 第4.2节](CT-04-PID-Control-Principles.md#42-pi控制器与零极点对消)）：

$$K_p = L_s \times \omega_c$$
$$K_i = R_s \times \omega_c$$

| 参数 | 对带宽的影响 | 对稳定性的影响 |
|------|------------|--------------|
| Kp ↑ | ωc ↑ → 响应↑ | PM ↓（延迟+滤波吃PM） |
| Ki ↑ | 对带宽影响小（已对消） | 超调↑（若对消不完全） |

**优势**：物理意义明确，直接映射到伯德图，对FOC电流环最为合适。

### 4.2 Anti-windup 实现

积分饱和（Integral Windup）是PI控制器在输出饱和时最致命的工程问题。

#### 方法1：积分限幅（Clamping）

```c
pi->integral += Ki * Ts * error;
if (pi->integral > I_MAX) pi->integral = I_MAX;
if (pi->integral < -I_MAX) pi->integral = -I_MAX;
```

简单但粗暴——本质是在无理地截断积分。

#### 方法2：条件积分（Conditional Integration）

```c
output = Kp * error + integral;
if (output > OUT_MAX || output < OUT_MIN) {
    // 输出已饱和，停止积分累积
    // integral 保持当前值
} else {
    integral += Ki * Ts * error;
}
```

比方法1更合理——饱和时不累积，不饱和时正常累积。

#### 方法3：反计算（Back-Calculation）——推荐

```c
float output_raw = Kp * error + integral;
float output = clamp(output_raw, OUT_MIN, OUT_MAX);
integral += Ki * Ts * error + Kb * (output - output_raw);
// Kb = 1/Tt (Tt: tracking time constant，通常Tt=Ti)
```

**原理**：当输出被限幅时，$(output - output\_raw)$不为零→反馈修正积分器→积分器被「拉回」合理范围。

### 4.3 Bumpless Transfer（无扰切换）

当PI参数在运行中改变，或控制器从手动切换到自动时，不应产生输出跳变。

**实现**：切换时重新计算积分状态，使切换瞬间的输出连续。

```c
void PI_BumplessParamChange(PI_Controller *pi, float new_Kp, float new_Ki) {
    // 保存当前输出
    float old_output = pi->output;
    // 更新参数
    pi->Kp = new_Kp;
    pi->Ki = new_Ki;
    // 反算积分状态使输出连续
    pi->integral = old_output - pi->Kp * pi->last_error;
}
```

### 4.4 FOC电流环整定的完整工程流程

```
步骤1：实测电机参数（Ls, Rs）
       - 方法：锁定转子，施加阶跃电压，记录电流响应→拟合Ls/Rs
       - 注意：多次测量取平均，记录温度

步骤2：确定ωc
       - 约束：ωc < 2π×fs/10, PM_eff ≥ 60°
       - 典型：fs=16kHz→ωc=1000~2000 rad/s

步骤3：计算PI参数
       - Kp = Ls × ωc, Ki = Rs × ωc
       - 零极点对消验证

步骤4：DSP离散化（Ts = 1/fs_current_loop）
       - 后向欧拉，带anti-windup
       - 积分限幅 = ±Vdc/√3

步骤5：小信号阶跃测试
       - Iq_ref: 0→0.5A（小信号，避免饱和）
       - 录波：观察tr, Mp, ts
       - 期望：Mp<5%, tr<2.2/ωc

步骤6：大信号阶跃测试
       - Iq_ref: 0→额定电流的80%
       - 观察anti-windup是否有效
       - Vq不应长时间保持饱和

步骤7：全转速范围验证
       - 转速从0到额定，观察电流波形
       - 关注反电动势解耦是否完全
```

---

## 5. 🔗 交叉视角

### 5.1 电流环整定→速度环的设计输入

电流环整定完成后，视其为一阶环节 $1/(1+s/\omega_c^{cur})$，用于速度环PI设计。

### 5.2 不同工况下的PI参数调度

| 工况 | PI参数调整 | 原因 |
|------|----------|------|
| 低速重载 | 略微降低Kp (90%) | 反电动势耦合增强 |
| 高速轻载 | 标准参数 | 耦合可控 |
| 弱磁区 | 大幅降低Kp (50~70%) | dq耦合增强、电压余量变小 |
| 过调制区 | 增强anti-windup | 六阶梯波模式电压饱和严重 |

### 5.3 速度环自整定原理

现代驱动器（如Copley、Elmo）的Auto-Tuning：
1. 注入小幅正弦扫频信号到Iq_ref
2. 测量速度响应→获取速度环频率响应
3. 从伯德图读取$\omega_c$和PM
4. 通过优化算法（如Nelder-Mead）搜索最优PI参数

### 5.4 hpm_MC 工程实践

**v2 PID 实现** (`hpm_mcl_v2/core/control/hpm_mcl_control.h`):
- 增量式 PID (`delta_pid()`) 用于电流环：抗积分饱和，适合快速响应
- 位置式 PID (`position_pid()`) 用于速度/位置环：适合级联外环
- PID 参数整定在 `mcl_control_pid_cfg_t` 中配置：kp/ki/kd + integral_limit + output_limit
- 双电流环 PID 独立整定（d轴/q轴分别调参）

**硬件加速**: HPM CLC（Current Loop Controller）硬件实现 PID，释放 CPU 用于其他任务
参考: [SDK-02-HPM-MC-v2-Core-Loop.md](../algorithm/HPM-MC/SDK-02-HPM-MC-v2-Core-Loop.md) 第3节「控制链核心」

---

## 6. 🎯 工程案例

### 案例1：完整电流环PI调优过程

**电机参数**：Ls=1.8mH, Rs=0.22Ω, Vdc=48V, In=15A
**PWM频率**：16kHz, 电流环10kHz

**调试记录**：
```
第1轮（公式法）：ωc=1500→Kp=2.7, Ki=330
  小信号Iq 0→1A：tr=1.6ms, Mp=0 ✅
  大信号Iq 0→12A：启动瞬间Vq冲到50V>48V→饱和→Iq过冲到16A→过流保护⚠️

第2轮（降低ωc）：ωc=1000→Kp=1.8, Ki=220
  大信号：Vq峰值38V<48V→无饱和→
  Iq过冲至13.5A(Mp≈12.5%)⚠️ 还是偏高

第3轮（增强anti-windup+减小启动给定斜率）：
  Iq_ref从0斜坡到12A（斜率5A/ms）+
  Kp=1.8, Ki=220 + anti-windup(back-calculation, Tt=Ti)
  → Iq平滑跟踪，Mp<3% ✅✅
```

**经验总结**：
1. 公式法给出的Kp在额定范围外可能太大
2. 大信号阶跃比小信号更容易暴露问题
3. 给定值斜率限制 + anti-windup = 工业级鲁棒性

### 案例2：速度环PI的「试凑→频域法」迭代

**背景**：AGV驱动，速度环需求Mp<5%, ts<200ms。

**初始试凑**：$K_p^{spd}=1.5, K_i^{spd}=30$ → Mp≈18%, ts≈350ms ❌

**频域法修正**：
- 电流环ωc=1200→速度环设计ωc_spd=150 rad/s
- $K_p^{spd} = J\omega_c^{spd}/K_t = 0.005\times150/0.6 = 1.25$
- $K_i^{spd} = K_p^{spd}\omega_c^{spd}/4 = 1.25\times150/4 = 46.9$
- 结果：Mp≈6%, ts≈180ms ✅

### 案例3：Bumpless Transfer 避免模式切换冲击

**背景**：伺服系统在「速度模式↔位置模式」间切换。未处理bumpless transfer→切换时积分器状态突变→Vq跳变→电机抖动。

**解决**：
```c
void SwitchToPositionMode() {
    // 速度环PI输出作为位置环输出的初始前馈
    float vq_initial = speed_pi.output;
    // 位置PI积分状态初始化
    pos_pi.integral = vq_initial;
    // 速度环积分清零
    speed_pi.integral = 0;
    // 模式切换
    control_mode = POSITION_MODE;
}
```

切换过程平滑，无冲击。

---

## 7. 📝 实践练习

### 练习1：Z-N vs 频域法对比

```
电机Ls=2mH, Rs=0.3Ω
分别用(a)Z-N终极增益法和(b)频域法(ωc=1200)整定PI，对比结果

Z-N法（实验测Ku≈5.2, Tu≈4ms）：
Kp=0.45×5.2=2.34, Ti=Tu/1.2=3.33ms→Ki=Kp/Ti=702
频域法：Kp=2mH×1200=2.4, Ki=0.3×1200=360
对比：Z-N的Ki(702)远大于频域法(360)→积分更激进→可能导致震荡
```

### 练习2：设计anti-windup方案

```
FOC电流环，Vdc=48V, Imax=20A, Ls=1.5mH, Rs=0.25Ω
设计anti-windup：积分限幅值？条件积分阈值？back-calculation增益Kb？
```

### 练习3：多模式切换的bumpless transfer设计

```
三种模式：力矩模式(Iq控制)、速度模式、位置模式
设计模式间切换的bumpless transfer策略（伪代码）
```

---

## 8. 🚀 前沿拓展

### 8.1 基于强化学习的PID自整定

使用RL代理在与电机仿真模型交互的过程中学习最优PI参数，然后在真实电机上fine-tune——已经在部分高端伺服中得到应用。

### 8.2 主动抗饱和（Active Anti-Windup）

相比被动限幅，主动抗饱和通过设计专门的补偿器$M(s)$从饱和误差到控制器输入构建额外的反馈回路——理论上可以保证大信号稳定性。

---

**文档信息**：
- 模块编号：CT-05
- 知识体系：控制理论基础
- 模块名称：PID 整定与工程实现
- 算法关联：电流环完整调优流程、anti-windup（back-calculation）、bumpless transfer


## 🧪 仿真验证
> 本模块的理论可在 [C 语言仿真](../simulation/SIM-00-C-Simulation-Overview.md) 中验证。
> 对应仿真模式：MODE_SELECT_VELOCITY_LOOP (4)，关键操作：改 FOC.delta（速度/电流带宽比 5~25），观察 Speed 子图响应速度变化
