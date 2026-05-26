# 电控超级知识库 - 项目交接文档

**最后更新：** 2026-05-25  
**项目代号：** motor-control-super-kb  
**当前状态：** ✅ v2.0 知识库构建完成（Batch 1~7 审核通过）

---

## 一、项目演进

### v1.0：三源合并（2026-05-01 完成）

将三个独立知识体系合并为统一的"电控超级知识库"：

| 知识源 | 产出路径 | 内容规模 |
|--------|---------|---------|
| `knowledge_20260428_MOSFET_IGBT_Gate_Driver/` | 硬件（HW） | 11个模块 → 7个HW模块 |
| `motor-learning-web/data/docs/` | 算法（ALG） | 5章理论 → 6个ALG模块 + 7个MC_LIB文档 |
| IEEE TPEL 2026 论文 | 前沿（ALG-15） | 离散时间非线性磁链观测器 |

### v2.0：全路径扩展（2026-05-17 完成设计，2026-05-25 完成审核）

在 v1.0 基础上扩展为 **10 条学习路径**：

| 路径 | 目录 | 模块数 | 说明 |
|------|------|--------|------|
| 硬件基础 | `electronics-basics/` | 9 (EE-01~09) | RLC→MOSFET→IGBT→H桥 |
| 控制理论 | `control-theory/` | 13 (CT-01~13) | 开闭环→PID→状态空间→LQR |
| 电控硬件 | `hardware/` | 7 (HW-01~07) | 电机→采样→传感→MCU→功率→电源→热 |
| 算法 | `algorithm/` | 16 (ALG-01~15 + MC-LIB 7) | FOC→观测器→HFI→保护→前沿 |
| hpm_MC 代码 | `algorithm/HPM-MC/` | 6 (SDK-01~06) | 先楫官方SDK代码分析 |
| 高级篇 | `advanced/` | 13 (ADV + SYS) | 带宽→弱磁→前馈→标幺→PID→调试 |
| 通信 | `communication/` | 7 (COM-01~07) | CAN→CAN-FD→Modbus→EtherCAT |
| 机械 | `mechanical/` | 3 (MEC-01~03) | 行星→摆线针轮→多圈编码器 |
| 功率变换 | `power-path/` | 7 (PP-01~07) | DCDC→隔离→LLC→PFC→磁件 |
| 交叉引用 | `cross-reference/` | 1 | 全量双向映射表 |

---

## 二、总体进度

```
v1.0 三源合并    ████████████ 100% ✅  2026-05-01
v2.0 全路径扩展   ████████████ 100% ✅  2026-05-25
  - EE硬件基础    ████████████ 100% ✅  9模块+9检验
  - CT控制理论    ████████████ 100% ✅  13模块+13检验
  - ALG扩展       ████████████ 100% ✅  6→16模块
  - ADV高级篇     ████████████ 100% ✅  13模块+检验
  - COM通信       ████████████ 100% ✅  7模块
  - MEC机械       ████████████ 100% ✅  3模块
  - PP功率变换     ████████████ 100% ✅  7模块+7检验
  - HPM-MC代码    ████████████ 100% ✅  6文档
  - 交叉引用映射   ████████████ 100% ✅  全量150+条引用
Batch 1~7 审核    ████████████ 100% ✅  2026-05-25
```

---

## 三、知识库完整文件清单

### 3.1 顶层文件

| 文件 | 说明 |
|------|------|
| `README.md` | 主索引，含10条路径导航、模块概览、目录结构 |
| `HANDOVER.md` | 本交接文档 |
| `TEMPLATE-SDK-Analysis.md` | SDK代码分析模板 |

### 3.2 硬件基础路径（9模块 + 9检验）

| 编号 | 文件 | 难度 |
|------|------|------|
| EE-01 | `electronics-basics/EE-01-Resistance-Capacitance-Inductance-Basics.md` | ★☆☆☆☆ |
| EE-02 | `electronics-basics/EE-02-Diodes-And-Rectification.md` | ★★☆☆☆ |
| EE-03 | `electronics-basics/EE-03-BJT-Basics.md` | ★★★☆☆ |
| EE-04 | `electronics-basics/EE-04-MOSFET-Principles.md` | ★★★☆☆ |
| EE-05 | `electronics-basics/EE-05-MOSFET-Gate-Drive.md` | ★★★★☆ |
| EE-06 | `electronics-basics/EE-06-IGBT-Principles.md` | ★★★☆☆ |
| EE-07 | `electronics-basics/EE-07-OpAmp.md` | ★★★☆☆ |
| EE-08 | `electronics-basics/EE-08-Rectifier-Inverter.md` | ★★★★☆ |
| EE-09 | `electronics-basics/EE-09-H-Bridge.md` | ★★★★☆ |

### 3.3 控制理论路径（13模块 + 13检验）

| 编号 | 文件 | 难度 |
|------|------|------|
| CT-01 | `control-theory/CT-01-Open-Loop-Closed-Loop.md` | ★☆☆☆☆ |
| CT-02 | `control-theory/CT-02-Time-Domain-Analysis.md` | ★★☆☆☆ |
| CT-03 | `control-theory/CT-03-Frequency-Response-Bode.md` | ★★★☆☆ |
| CT-04 | `control-theory/CT-04-PID-Control-Principles.md` | ★★☆☆☆ |
| CT-05 | `control-theory/CT-05-PID-Tuning-Implementation.md` | ★★★☆☆ |
| CT-06 | `control-theory/CT-06-Feedforward-Control.md` | ★★★☆☆ |
| CT-07 | `control-theory/CT-07-Nyquist-Stability.md` | ★★★★☆ |
| CT-08 | `control-theory/CT-08-Root-Locus.md` | ★★★★☆ |
| CT-09 | `control-theory/CT-09-Compensator-Design.md` | ★★★★☆ |
| CT-10 | `control-theory/CT-10-State-Space.md` | ★★★★☆ |
| CT-11 | `control-theory/CT-11-Observer-Design.md` | ★★★★★ |
| CT-12 | `control-theory/CT-12-State-Feedback.md` | ★★★★★ |
| CT-13 | `control-theory/CT-13-LQR-LQG.md` | ★★★★★ |

### 3.4 电控硬件路径（7模块 + 7检验）

| 编号 | 文件 | 难度 | 源文件 |
|------|------|------|--------|
| HW-01 | `hardware/HW-01-Motor-Basics.md` | ★★☆☆☆ | Motor_Basics.md |
| HW-02 | `hardware/HW-02-Current-Sensing.md` | ★★★☆☆ | Current_Sensing_Circuit.md |
| HW-03 | `hardware/HW-03-Position-Sensor.md` | ★★★☆☆ | Position_Sensor_Interface.md |
| HW-04 | `hardware/HW-04-MCU-Peripherals.md` | ★★★★☆ | MCU_Peripherals_Communication.md |
| HW-05 | `hardware/HW-05-Power-Devices-Gate-Drivers.md` | ★★★★★ | MOSFET_IGBT_Gate_Driver.md |
| HW-06 | `hardware/HW-06-Power-Management-Protection.md` | ★★★★☆ | Power_Management + Protection |
| HW-07 | `hardware/HW-07-Thermal-EMC-Design.md` | ★★★★★ | Thermal_EMC + PCB_Design |

### 3.5 算法路径（16模块 + MC_LIB 7文档 + 检验）

| 编号 | 文件 | 难度 |
|------|------|------|
| ALG-01 | `algorithm/ALG-01-FOC-Theory.md` | ★★☆☆☆ |
| ALG-02 | `algorithm/ALG-02-Current-Sampling-Timing.md` | ★★★★☆ |
| ALG-03 | `algorithm/ALG-03-PI-Current-Regulator.md` | ★★★☆☆ |
| ALG-04 | `algorithm/ALG-04-Deadtime-Compensation.md` | ★★★★☆ |
| ALG-05 | `algorithm/ALG-05-Sensored-FOC.md` | ★★★☆☆ |
| ALG-06 | `algorithm/ALG-06-Position-Speed-Observer.md` | ★★★★☆ |
| ALG-07 | `algorithm/ALG-07-Sensorless-Observers.md` | ★★★★☆ |
| ALG-08 | `algorithm/ALG-08-Initial-Position-Detection.md` | ★★★★★ |
| ALG-09 | `algorithm/ALG-09-High-Frequency-Injection.md` | ★★★★☆ |
| ALG-10 | `algorithm/ALG-10-Overmodulation.md` | ★★★★☆ |
| ALG-11 | `algorithm/ALG-11-MTPA-Flux-Weakening.md` | ★★★★☆ |
| ALG-12 | `algorithm/ALG-12-Speed-Loop-Torque-Observer.md` | ★★★★☆ |
| ALG-13 | `algorithm/ALG-13-Protection-Optimization.md` | ★★★★☆ |
| ALG-14 | `algorithm/ALG-14-THD-Harmonic-Analysis.md` | ★★★☆☆ |
| ALG-15 | `algorithm/ALG-15-Advanced-Research.md` | ★★★★★ |
| MC-LIB | `algorithm/MC-LIB/` (7文档) | ★★★★☆ |
| HPM-MC | `algorithm/HPM-MC/` (6文档) | ★★★★☆ |

### 3.6 高级篇（13模块 + 检验）

**高级算法（6个）：**
| 编号 | 文件 |
|------|------|
| ADV-ALG-01 | `advanced/algorithm/ADV-ALG-01-Bandwidth-Filter.md` |
| ADV-ALG-05 | `advanced/algorithm/ADV-ALG-05-Field-Weakening-MTPA.md` |
| ADV-ALG-07 | `advanced/algorithm/ADV-ALG-07-Feedforward-Decoupling.md` |
| ADV-ALG-09 | `advanced/algorithm/ADV-ALG-09-Per-Unit-Fixed-Point.md` |
| ADV-ALG-13 | `advanced/algorithm/ADV-ALG-13-PID-Structure-Tuning.md` |
| ADV-ALG-15 | `advanced/algorithm/ADV-ALG-15-Debug-Methodology.md` |

**硬件-算法桥接（3个）：**
| 编号 | 文件 |
|------|------|
| ADV-HW-01 | `advanced/hardware-algorithm-bridge/ADV-HW-01-PWM-Current-Sampling.md` |
| ADV-HW-02 | `advanced/hardware-algorithm-bridge/ADV-HW-02-ADC-DMA.md` |
| ADV-HW-03 | `advanced/hardware-algorithm-bridge/ADV-HW-03-Encoder-Speed.md` |

**系统与方法论（4个）：**
| 编号 | 文件 |
|------|------|
| SYS-01 | `advanced/system-methodology/SYS-01-Design-Patterns.md` |
| SYS-02 | `advanced/system-methodology/SYS-02-Inverter-vs-Motor-Control.md` |
| SYS-03 | `advanced/system-methodology/SYS-03-PFC-vs-Motor-Control.md` |
| SYS-04 | `advanced/system-methodology/SYS-04-Simulation-to-Discrete.md` |

### 3.7 通信路径（7模块）

| 编号 | 文件 |
|------|------|
| COM-01 | `communication/COM-01-CAN-Basics.md` |
| COM-02 | `communication/COM-02-CAN-FD.md` |
| COM-03 | `communication/COM-03-CAN-STM32.md` |
| COM-04 | `communication/COM-04-CAN-Simulation.md` |
| COM-05 | `communication/COM-05-Modbus.md` |
| COM-06 | `communication/COM-06-EtherCAT.md` |
| COM-07 | `communication/COM-07-Protocol-Compare.md` |

### 3.8 机械路径（3模块）

| 编号 | 文件 |
|------|------|
| MEC-01 | `mechanical/MEC-01-Planetary-Reducer.md` |
| MEC-02 | `mechanical/MEC-02-Cycloidal-Reducer.md` |
| MEC-03 | `mechanical/MEC-03-Multi-Turn-Encoder.md` |

### 3.9 功率变换路径（7模块 + 7检验）

| 编号 | 文件 |
|------|------|
| PP-01 | `power-path/PP-01-DCDC-Fundamental-Topologies.md` |
| PP-02 | `power-path/PP-02-Isolated-DCDC-Flyback-Forward-PushPull.md` |
| PP-03 | `power-path/PP-03-LLC-Resonant-Converter.md` |
| PP-04 | `power-path/PP-04-PFC-Power-Factor-Correction.md` |
| PP-05 | `power-path/PP-05-Magnetic-Design.md` |
| PP-06 | `power-path/PP-06-Loop-Compensation.md` |
| PP-07 | `power-path/PP-07-Power-Conversion-Motor-Drive.md` |

### 3.10 交叉引用系统

| 文件 | 内容 |
|------|------|
| `cross-reference/cross-reference-map.md` | 双向映射表：HW↔ALG(54条) + EE↔电控(36条) + CT→ADV/ALG(25条) + PP↔HW(7条) + ADV→基础(50条) + hpm_MC↔文档(52条) |

---

## 四、文档格式规范（接手必读）

### 硬件模块7段式结构

```markdown
# HW-XX 模块名称
## 📌 核心摘要
## 🤔 问题引入
## 💡 直观理解
## 🔬 技术原理
## 🔗 交叉视角
## 🎯 工程案例
## 📝 实践练习
```

### 算法模块7段式结构

```markdown
# ALG-XX 模块名称
## 📌 核心摘要
## 📐 原理推导
## 🔢 数学建模
## 💻 代码实现
## 🔧 参数整定
## 🔗 硬件约束
## 🚀 前沿拓展
```

### 命名规范

```
{PREFIX}-{NN}-{Topic}.md

前缀映射：
  EE  → electronics-basics
  CT  → control-theory
  HW  → hardware
  ALG → algorithm
  ADV-ALG → advanced/algorithm
  ADV-HW  → advanced/hardware-algorithm-bridge
  SYS → advanced/system-methodology
  COM → communication
  MEC → mechanical
  PP  → power-path
  SDK → algorithm/HPM-MC
  MC-LIB → algorithm/MC-LIB
```

### 难度标识

```
★☆☆☆☆ 入门级   ★★☆☆☆ 基础级   ★★★☆☆ 进阶级
★★★★☆ 专业级   ★★★★★ 专家级
```

---

## 五、交叉引用体系

交叉引用覆盖以下维度：

1. **HW → ALG**：硬件特性如何影响算法设计（29+条）
2. **ALG → HW**：算法决策受什么硬件约束（25+条）
3. **EE → HW/ALG**：硬件基础知识如何应用于电控实践（36条）
4. **CT → ALG/ADV**：控制理论如何支撑算法实现（25条）
5. **ADV → 基础**：高级篇如何深化基础篇内容（50条）
6. **PP → HW**：功率变换与电控硬件的关联（7条）
7. **hpm_MC ↔ 文档**：源码与知识库文档的双向映射（52条）

详见 `cross-reference/cross-reference-map.md`

---

## 六、已知问题与风险

| 问题 | 影响 | 应对措施 |
|------|------|---------|
| v2.0 扩展后 Web 前端未更新 | 侧边栏导航不完全反映10条路径 | 后续需更新 Sidebar.vue 支持多路径导航 |
| MC_LIB 文档仅复制未添加交叉引用 | MC_LIB 文档内缺少指向 ALG 模块的引用链接 | 后续在各 MC_LIB 文档头部添加关联模块链接 |
| 交叉引用使用相对路径 | 移动文件后链接可能失效 | 保持目录结构不变 |
| hpm_MC 源码路径为外部引用 | 知识库内无法直接点击访问 | 作为源码索引记录，需配合 AxDr 仓库使用 |
| 多数 assessment 文件与模块文件独立存放 | 检验题可通过 README 索引定位，但未在模块文档内直接链接 | 后续可添加模块→检验题的直连跳转 |

---

## 七、快速接手指引

### 场景A：维护/扩展知识库内容

1. 读本文件第四章了解7段式结构和命名规范
2. 读任一已完成的模块文件作为模板参考
3. 新增模块：按 `{PREFIX}-{NN}-{Topic}.md` 命名，遵循7段式结构
4. 更新交叉引用：在 `cross-reference/cross-reference-map.md` 中添加新条目
5. 更新 README.md 目录结构和学习路径表格
6. 新增检验题目：在对应模块目录下创建 `*-assessment.md`

### 场景B：从零了解项目

1. 读 `spec.md`（`docs/superpowers/specs/`）：了解完整需求定义
2. 读 `README.md`：了解知识库整体架构和10条学习路径
3. 选择学习路径：从任一 README 索引开始
4. 利用交叉引用：遇到 🔗 标记时跳转到关联模块

### 场景C：运行质量审核

1. 参考 `docs/superpowers/specs/plan.md` 中的四维检查清单（A/B/C/D）
2. 每批执行模板中的审查记录格式
3. 重点检查 D 维度（交叉引用完整性）

---

## 八、目录结构总览

```
motor-control-knowledge-base/
├── README.md
├── HANDOVER.md
├── TEMPLATE-SDK-Analysis.md
├── electronics-basics/          # 硬件基础（9模块+9检验）
├── control-theory/              # 控制理论（13模块+13检验）
├── hardware/                    # 电控硬件（7模块+7检验）
├── algorithm/                   # 算法（16模块+MC_LIB 7+HPM-MC 6+检验）
├── advanced/                    # 高级篇（13模块+检验）
│   ├── algorithm/               #   高级算法（6）
│   ├── hardware-algorithm-bridge/ # 硬件-算法桥接（3）
│   └── system-methodology/      #   系统方法论（4）
├── communication/               # 通信协议（7模块）
├── mechanical/                  # 机械传动（3模块）
├── power-path/                  # 功率变换（7模块+7检验）
└── cross-reference/             # 交叉引用映射表
```

**文件估算：** 约 130+ 文件（含模块文档、检验题目、索引 README）

---

## 九、审核批次记录

| 批次 | 范围 | 文件数 | 状态 | 审核日期 |
|------|------|--------|------|---------|
| Batch 1 | `algorithm/`（16模块） | 16 | ✅ | - |
| Batch 2 | `hardware/` + `MC-LIB/` | 14 | ✅ | - |
| Batch 3 | `control-theory/` | 13 | ✅ | - |
| Batch 4 | `HPM-MC/` + `electronics-basics/` | 15 | ✅ | - |
| Batch 5 | `advanced/` | 14 | ✅ | - |
| Batch 6 | `communication/` + `mechanical/` + `power-path/` | 17 | ✅ | - |
| Batch 7 | `cross-reference/` + `README.md` + `HANDOVER.md` | 3 | ✅ | 2026-05-25 |