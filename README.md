# MotorControl 项目总览 README

**最后更新：** 2026-05-27  
**项目性质：** 电机控制全栈学习平台（知识库 + Web应用 + 嵌入式代码 + 仿真工具）  
**适用对象：** 电机控制工程师、嵌入式开发者、电力电子工程师、自动控制学习者

---

## 一、项目全景地图

```
MotorControl-main/                          # 项目根目录
│
├── 📚 motor-control-knowledge-base/        # ★ 核心知识库（Markdown文档）
│   ├── electronics-basics/                 #   硬件基础路径（9模块）
│   ├── control-theory/                     #   控制理论路径（18模块，含ADRC）
│   ├── hardware/                           #   电控硬件路径（7模块）
│   ├── algorithm/                          #   算法路径（16+MC_LIB 7+HPM-MC 6）
│   ├── advanced/                           #   高级篇（13模块）
│   │   ├── algorithm/                      #     高级算法（6）
│   │   ├── hardware-algorithm-bridge/      #     硬件-算法桥接（3）
│   │   └── system-methodology/             #     系统方法论（4）
│   ├── communication/                      #   工业通信路径（7模块）
│   ├── mechanical/                         #   机械传动路径（3模块）
│   ├── power-path/                         #   功率变换路径（7模块）
│   ├── ODrive/                             #   ODrive开源固件代码分析（4）
│   ├── VESC/                               #   VESC开源固件代码分析（4）
│   ├── COMPARISON/                         #   ODrive vs VESC 对比（1）
│   └── cross-reference/                    #   交叉引用映射表（1）
│
├── 🌐 motor-learning-web/                  # ★ 主Web应用（知识库浏览/测验/仿真）
│   ├── backend/                            #   FastAPI 后端（Python）
│   │   └── app/api/
│   │       ├── docs.py                     #     知识库目录API + 文件服务 + 搜索
│   │       ├── quiz.py                     #     测验解析API
│   │       ├── simulation.py               #     仿真数据API
│   │       ├── annotations.py              #     标注API
│   │       └── serial_proxy.py             #     串口代理API
│   ├── frontend/                           #   Vue3 + TypeScript 前端
│   │   └── src/
│   │       ├── components/
│   │       │   ├── Sidebar.vue             #     侧边栏导航（多路径树）
│   │       │   ├── Content.vue             #     Markdown内容渲染
│   │       │   ├── QuizPanel.vue           #     测验面板
│   │       │   ├── SearchBox.vue           #     搜索框
│   │       │   ├── TocNav.vue              #     浮动目录导航
│   │       │   └── ThemeToggle.vue         #     亮/暗主题切换
│   │       ├── utils/markdown.ts           #     Markdown渲染管线
│   │       └── stores/progress.ts          #     学习进度管理
│   ├── sims/                               #   HTML仿真工具
│   │   ├── foc_sim.html                    #     FOC电机仿真
│   │   ├── oscillator_sim.html             #     弹簧振子仿真
│   │   ├── serial_assistant.html           #     串口助手
│   │   └── comm_sim.html                   #     CAN通信仿真
│   └── scripts/start.bat                   #   一键启动脚本
│
├── 📊 ExamDashboard/                       # 考试仪表盘（Next.js App Router）
│   ├── src/app/
│   │   ├── dashboard/page.tsx              #   仪表盘主页
│   │   ├── knowledge-graph/page.tsx        #   知识图谱可视化
│   │   ├── quiz/page.tsx                   #   测验页面
│   │   ├── notes/page.tsx                  #   笔记页面
│   │   ├── pdf-reader/page.tsx             #   PDF阅读器
│   │   └── api/                            #   API路由（标注/笔记/知识库）
│   └── src/data/                           #   章节数据/题目/知识图谱/学习记录
│
├── 📖 code-reader-web-app/                 # 代码阅读器Web应用（Vue3 + Monaco）
│   ├── backend/                            #   FastAPI 后端
│   │   └── app/api/
│   │       ├── files.py                    #     文件浏览API
│   │       ├── projects.py                 #     项目管理API
│   │       ├── analysis.py                 #     代码分析API
│   │       └── annotations.py              #     标注API
│   ├── frontend/src/                       #   Vue3前端
│   │   ├── views/HomeView.vue              #     项目列表页
│   │   ├── views/ProjectView.vue           #     代码浏览页
│   │   └── components/MonacoEditor.vue     #     Monaco编辑器组件
│   └── data/projects/                      #   已导入的项目数据
│
├── 🔌 AxDr/                                # STM32G474 FOC电机控制工程（Keil MDK）
│   └── AxDr/User/
│       ├── motor/                          #   FOC核心算法（观测器/坐标变换/控制环）
│       └── utils/                          #   工具库（PID/LPF/角度/串口协议）
│
├── 📦 MC_LIB/                              # 跨平台电机控制库
│   ├── 0_MCU/                              #   MCU适配层（STM32F4/RX32H6/Z20K14xM）
│   ├── 1_HD/                               #   硬件驱动层（BSP）
│   ├── 2_COM/                              #   通信层（RTT/数学库/UART/SPI驱动）
│   ├── 3_MC/                               #   电机控制层（方波/FOC浮点/FOC定点）
│   ├── 4_SYS/                              #   系统层（按键/系统任务/错误码）
│   └── 5_PRO/                              #   工程模板（各平台主程序）
│
├── 🎯 odriver_core/                        # ODrive开源固件源码（C++）
│   ├── MotorControl/                       #   电机控制核心（FOC/编码器/观测器）
│   └── communication/                      #   通信层（CAN/UART/USB/I2C/ASCII协议）
│
├── ⚡ vesc_core/                           # VESC开源固件源码（C）
│   ├── mc_interface/                       #   电机接口（SVM/FOC/观测器）
│   ├── hwconf/                             #   硬件配置（板卡/驱动/关机）
│   ├── encoder/                            #   编码器驱动
│   └── conf_general/                       #   配置管理（595行mc_configuration）
│
├── 🏭 hpm_MC/                              # 先楫半导体hpm_math DSP库
│
├── 🔧 lxfoc/                               # LxFOC相关代码（参数辨识/自整定）
│
├── 🎨 foc_motor_visualizer.html            # FOC电机可视化（独立HTML工具）
├── 🎨 lyapunov_spring_oscillator_v6_*.html # Lyapunov弹簧振子卷积过程演示
│
├── 📄 启动知识库.bat                        # 一键启动motor-learning-web
├── 📄 启动ExamDashboard.bat                # 一键启动ExamDashboard
├── 📄 checklist.md                         # 开发检查清单
├── 📄 ERROR_imformation..md               # 错误信息记录
└── 📄 *.pdf                                # 学术论文（离散时间非线性磁链观测器等）
```

---

## 二、核心子项目详解

### 2.1 知识库（motor-control-knowledge-base/）— 数据源

**角色：** 整个平台的内容数据源，所有Web应用从此目录读取Markdown文件。

| 子目录 | 路径ID | 模块数 | 文件前缀 | 说明 |
|--------|--------|--------|----------|------|
| `electronics-basics/` | electronicsBasics | 9 | EE-01~09 | RLC→MOSFET→IGBT→运放→H桥 |
| `control-theory/` | controlTheory | 18 | CT-01~18 | 含经典控制(CT-01~15)和ADRC(CT-16~18) |
| `hardware/` | hardware | 7 | HW-01~07 | 电机→采样→传感→MCU→功率→电源→热 |
| `algorithm/` | algorithm | 29+ | ALG/MC-LIB/SDK | 含ALG-00~15 + MC-LIB(7) + HPM-MC(6) |
| `advanced/algorithm/` | advAlgorithm | 6 | ADV-ALG-* | 带宽/弱磁/前馈/标幺/PID/调试 |
| `advanced/hardware-algorithm-bridge/` | advHwBridge | 3 | ADV-HW-* | PWM采样/ADC-DMA/编码器测速 |
| `advanced/system-methodology/` | sysMethodology | 4 | SYS-01~04 | 设计模式/变频器/PFC/仿真到离散 |
| `communication/` | communication | 7 | COM-01~07 | CAN/CAN-FD/Modbus/EtherCAT |
| `mechanical/` | mechanical | 3 | MEC-01~03 | 行星减速器/摆线/多圈编码器 |
| `power-path/` | powerPath | 7 | PP-01~07 | DCDC/隔离/LLC/PFC/磁件/环路 |
| `ODrive/` | odrive | 4 | OD-01~04 | FreeRTOS架构/FOC/三环/无感 |
| `VESC/` | vesc | 4 | VC-01~04 | ChibiOS架构/七种观测器/HFI/保护 |
| `COMPARISON/` | comparison | 1 | COMP-01 | ODrive vs VESC 11维度对比 |
| `cross-reference/` | crossReference | 1 | - | 全量双向交叉引用映射表 |

**特殊文件：**
- `README.md` — 知识库主索引（10条学习路径导航图）
- `HANDOVER.md` — 项目交接文档（含格式规范、审核批次、快速接手指引）
- `TEMPLATE-SDK-Analysis.md` — SDK代码分析文档模板
- `*/\*-assessment.md` — 各模块配套检验题目（quiz.py可解析为结构化测验）

### 2.2 主Web应用（motor-learning-web/）— 知识库浏览器

**技术栈：** Vue3 + TypeScript + Vite（前端） / FastAPI + Uvicorn（后端）

**关键能力：**
- **Markdown渲染**：支持 KaTeX 数学公式、highlight.js 代码高亮、答案折叠、交叉引用链接
- **侧边栏导航**：14条路径的树形结构 + 搜索 + 学习进度追踪
- **测验系统**：解析 assessment 文件为结构化测验题
- **交互式仿真**：30+ 个 Vue/HTML 仿真工具（Bode图/PID调节器/FOC/SVPWM 等）
- **串口通信**：通过 serial_proxy 实现浏览器与硬件串口的桥接
- **主题切换**：亮色/暗色主题

**API端点：**

| 方法 | 路径 | 功能 |
|------|------|------|
| GET | `/api/docs/list` | 返回完整知识库结构化目录（JSON） |
| GET | `/api/docs/file/{path}` | 返回指定Markdown文件内容 |
| GET | `/api/docs/search/{keyword}` | 全文搜索 |
| GET | `/api/quiz/parse/{path}` | 解析assessment为结构化测验 |
| GET | `/api/simulations` | 获取仿真工具列表 |

**启动方式：** 双击 `启动知识库.bat` 或手动：
```bash
# 后端（端口8001）
cd motor-learning-web/backend
venv\Scripts\python.exe -m uvicorn app.main:app --reload --port 8001

# 前端开发模式（端口3000，proxy到8001）
cd motor-learning-web/frontend
npm run dev
```

### 2.3 考试仪表盘（ExamDashboard/）

**技术栈：** Next.js 14 (App Router) + TypeScript + TailwindCSS

**功能页面：**
- 仪表盘主页 — 学习概览与快捷入口
- 知识图谱 — Cytoscape.js 可视化知识节点关系
- 测验页面 — 在线答题与评分
- 笔记页面 — 学习笔记管理
- PDF阅读器 — 在线阅读PDF论文

**API路由：** `/api/knowledge-base/doc/[...path]`（知识库文档代理）、`/api/notes`、`/api/annotations`

### 2.4 代码阅读器（code-reader-web-app/）

**技术栈：** Vue3 + TypeScript + Monaco Editor + FastAPI

**功能：**
- 项目文件树浏览
- Monaco Editor 代码高亮显示
- 代码标注（行级笔记）
- 多项目切换

### 2.5 嵌入式工程

| 工程 | MCU平台 | 用途 | IDE |
|------|---------|------|-----|
| `AxDr/` | STM32G474 | FOC电机控制（CORDIC加速） | Keil MDK |
| `MC_LIB/` | STM32F4/RX32H6/Z20K14xM | 跨平台电机控制库 | Keil/IAR/EWARM |
| `odriver_core/` | 通用C++ | ODrive伺服驱动器固件源码 | - |
| `vesc_core/` | 通用C | VESC电机控制器固件源码 | - |
| `hpm_MC/` | HPM6000系列 | 先楫DSP数学库 | - |
| `lxfoc/` | 通用C | 参数辨识与自整定参考代码 | - |

---

## 三、如何将自定义 Markdown 接入知识库 Web 系统

> 这是本项目的**核心扩展机制**。系统设计了**自动发现**能力，用户创建的 Markdown 文件可以无缝接入网页。

### 3.1 原理：后端自动发现机制

在 [docs.py](motor-learning-web/backend/app/api/docs.py) 中，存在一个关键的 **`_discover_unlisted_modules()`** 函数：

```python
def _discover_unlisted_modules(base_prefix: str, listed_ids: list) -> discovered:
    module_dir = DOCS_DIR / base_prefix
    for md_file in sorted(module_dir.glob("*.md")):
        stem = md_file.stem
        if stem in listed_ids:      # 跳过已注册的模块
            continue
        if stem.endswith("-assessment"):  # 跳过检验题
            continue
        if stem == "README":        # 跳过索引文件
            continue
        # 自动提取标题并构建模块条目
        discovered.append(_build_module_entry(base_prefix, stem, title))
```

**工作流程：**
1. 后端在 `parse_knowledge_base()` 中对每个子目录调用此函数
2. 该函数扫描目录下所有 `.md` 文件
3. 排除已在 `ORDER` 列表中注册的模块、assessment 文件、README 文件
4. 对剩余文件**自动提取标题**（优先取 `# ` 标题行，其次取 `**模块名称：**` 字段）
5. 将发现的模块追加到对应路径的模块列表末尾

### 3.2 操作步骤（三种方式）

#### 方式一：放入已有路径目录（推荐，最简单）

只需将你的 `.md` 文件放到 `motor-control-knowledge-base/` 下对应的子目录中即可。

**示例：** 你写了一篇关于"SiC MOSFET在电机驱动中的应用"的文章

```bash
# 放入硬件路径（会出现在🔧电控硬件路径末尾）
motor-control-knowledge-base/hardware/HW-08-SiC-MOSFET-Motor-Drive.md

# 或者放入功率变换路径（会出现在🔋功率变换路径末尾）
motor-control-knowledge-base/power-path/PP-08-SiC-MOSFET-Application.md
```

**命名建议：** 遵循 `{PREFIX}-{NN}-{Topic}.md` 格式，如 `HW-08-SiC-MOSFET-Motor-Drive.md`

**标题提取规则：** 系统会自动从文件第一行 `# 标题` 提取显示名称。如果需要中文标题，确保首行为：
```markdown
# HW-08 SiC MOSFET在电机驱动中的应用
```

#### 方式二：创建全新子目录（新路径）

如果你想创建一个全新的学习路径分类：

1. 在 `motor-control-knowledge-base/` 下新建目录，例如 `my-custom-path/`
2. 放入 `.md` 文件
3. **修改后端** [docs.py](motor-learning-web/backend/app/api/docs.py)，添加：
   ```python
   MY_ORDER = []  # 有序模块ID列表（可为空，全靠自动发现）
   
   def parse_knowledge_base():
       # ...existing code...
       my_modules = []
       my_modules.extend(_discover_unlisted_modules("my-custom-path", MY_ORDER))
       
       return {
           # ...existing sections...
           "myCustomPath": {
               "id": "myCustomPath",
               "title": "📝 我的自定义路径",
               "description": "自定义学习内容",
               "modules": my_modules,
           },
           "sectionsOrder": [
               # ...existing order...
               "myCustomPath",  # 添加到导航顺序
           ],
       }
   ```

4. **修改前端** [Sidebar.vue](motor-learning-web/frontend/src/components/Sidebar.vue)：在 `KnowledgeBase` 接口中添加新路径类型定义（如果前端有类型检查的话）

#### 方式三：仅放文件不注册（临时方案）

直接将 `.md` 文件放入任意已有子目录，**不做任何代码修改**。文件会被 `_discover_unlisted_modules()` 自动发现并出现在对应路径的**末尾位置**。

> ⚠️ 注意：这种方式文件会排在已注册模块之后，且没有 assessment 关联。

### 3.3 可选增强操作

如果你希望自定义文档更完整地集成到系统中：

| 功能 | 操作 | 文件位置 |
|------|------|----------|
| 配套检验题 | 创建 `{模块ID}-assessment.md` | 同目录下，遵循quiz.py解析格式 |
| HTML交互工具 | 创建 `{模块ID}-*.html` | 同目录下，会被自动识别为工具入口 |
| 交叉引用 | 在 `cross-reference-map.md` 中添加条目 | `cross-reference/cross-reference-map.md` |
| 导航顺序调整 | 将模块ID加入对应 `ORDER` 列表 | `docs.py` 中的 `*_ORDER` 变量 |
| 自定义显示标题 | 在 `docs.py` 的 `*_TITLES` 字典中添加 | `docs.py` 中的 `*_TITLES` 字典 |

### 3.4 Assessment 文件格式规范

如果要让你的自定义文档支持在线测验功能，需创建配套的 assessment 文件：

```markdown
# {模块名称} - 检验题目

## 第1题 [难度等级] - 题目标题 ★★

**问题:** 
请描述XXX的原理是什么？

**参考答案:**
XXX的核心原理是...

---
## 第2题 ...
```

**解析规则（[quiz.py](motor-learning-web/backend/app/api/quiz.py)）：**
- 题目标题匹配：`第X题` 或 `题目X`
- 问题行：`**问题:**`
- 参考答案：`**参考答案:**`
- 难度：`★` 数量（1~5个星）

### 3.5 验证接入是否成功

1. 启动后端：`cd motor-learning-web/backend && venv\Scripts\python.exe -m uvicorn app.main:app --port 8001`
2. 访问 `http://127.0.0.1:8001/api/docs/list` ，查看返回的 JSON 中是否包含你的文档
3. 启动前端：`cd motor-learning-web/frontend && npm run dev`
4. 打开浏览器访问 `http://localhost:3000`，检查侧边栏对应路径下是否出现你的文档

---

## 四、手把手实操：从新建 Markdown 到显示在网页中

> 本节以一个完整的示例，一步步演示如何创建一篇自定义文档并让它出现在知识库网页中。

### 4.1 场景设定

假设你要写一篇关于 **「GaN HEMT 在电机驱动中的应用」** 的技术文章，希望它出现在 **🔧 电控硬件路径** 下面。

### 4.2 第一步：创建 Markdown 文件

在 `motor-control-knowledge-base/hardware/` 目录下新建文件：

**文件路径：** `motor-control-knowledge-base/hardware/HW-08-GaN-HEMT-Motor-Drive.md`

**文件内容模板：**

```markdown
# HW-08 GaN HEMT器件及其在电机驱动中的应用

> **难度：** ★★★★☆ | **预计阅读时间：** 4-5小时 | **前置知识：** HW-05 MOSFET驱动与保护

## 📌 核心摘要

GaN HEMT（氮化镓高电子迁移率晶体管）相比传统Si/SiC器件具有更低的开关损耗...
（一句话讲清楚这篇文章解决什么问题）

## 正文开始...

### 1. GaN vs SiC vs Si 对比
...

### 2. GaN驱动电路设计要点
...

### 3. 在电机驱动中的实际应用案例
...

## 🔗 相关模块

- 上位关联：[HW-05 功率器件与栅极驱动](./HW-05-Power-Devices-Gate-Drivers.md)
- 算法关联：[ALG-04 死区补偿](../algorithm/ALG-04-Deadtime-Compensation.md)
```

**关键规则：**
- 文件首行 `# ` 后面的文字会自动成为侧边栏中的**显示标题**
- 文件名 = 模块ID（如 `HW-08-GaN-HEMT-Motor-Drive`），系统据此生成唯一标识
- 文件内可以用相对路径引用其他模块（如 `./HW-05-xxx.md` 或 `../algorithm/ALG-04-xxx.md`）

### 4.3 第二步：（可选）创建配套检验题

在同一目录下创建 assessment 文件：

**文件路径：** `motor-control-knowledge-base/hardware/HW-08-assessment.md`

```markdown
# HW-08 GaN HEMT - 检验题目

## 第1题 [入门级] - GaN与SiC的基本特性差异 ★★

**问题:**
请比较GaN HEMT与SiC MOSFET在以下三个维度的差异：
1) 击穿电压与导通电阻的权衡关系
2) 开关速度对驱动电路的要求
3) 成本趋势与适用功率范围

**参考答案:**
1) GaN在低压领域(<650V)的Rds(on)*Area远优于SiC，但高压领域SiC仍占优...
2) GaN开关边沿<2ns，要求驱动回路电感<1nH...
3) 目前GaN单管成本约为同规格SiC的60-80%...

---
## 第2题 [进阶级] - GaN驱动电路设计 ★★★

**问题:**
设计一个100V/10A的GaN半桥驱动电路时，需要重点考虑哪些因素？请列出至少4点并说明原因。

**参考答案:**
1) 栅极电压选择（通常6V而非Si的10-15V）...
2) 负压关断需求（防止米勒误导通）...
3) 驱动回路最小化（PCB布局）...
4) dv/dt抗扰能力...
```

### 4.4 第三步：验证后端能发现你的文件

启动后端（或确认已在运行），然后用浏览器/命令行测试：

```bash
# 方法1：浏览器直接访问
打开 http://127.0.0.1:8001/api/docs/list

# 方法2：curl 命令
curl http://127.0.0.1:8001/api/docs/list | findstr "GaN"
```

你应该能在返回的 JSON 中看到类似这样的条目：

```json
{
  "id": "hardware/HW-08-GaN-HEMT-Motor-Drive",
  "title": "HW-08 GaN HEMT器件及其在电机驱动中的应用",
  "path": "hardware/HW-08-GaN-HEMT-Motor-Drive.md",
  "assessment": "hardware/HW-08-assessment.md",
  "tools": null
}
```

它位于 `"hardware"` 路径的 `"modules"` 数组末尾（因为是通过自动发现的）。

### 4.5 第四步：在网页中查看效果

1. 打开浏览器访问 `http://127.0.0.1:8001`
2. 在左侧 **🔧 电控硬件路径** 下滚动到最底部
3. 你应该能看到 **「HW-08 GaN HEMT器件及其在电机驱动中的应用」**
4. 点击它 → 右侧内容区渲染你的 Markdown
5. 如果创建了 assessment，点击右上角 **「开始测验」** 按钮即可答题

### 4.6 进阶：控制显示顺序（可选）

如果你不想让自定义模块排在最末尾，而是插入到特定位置：

1. 编辑 [docs.py](motor-learning-web/backend/app/api/docs.py)
2. 找到第13行的 `HW_ORDER` 列表：
   ```python
   HW_ORDER = [
       "HW-01-Motor-Basics",
       # ... existing ...
       "HW-07-Thermal-EMC-Design",
       "HW-08-GaN-HEMT-Motor-Drive",  # ← 加这一行
   ]
   ```
3. 找到第52行的 `HW_TITLES` 字典：
   ```python
   HW_TITLES = {
       # ... existing ...
       "HW-08-GaN-HEMT-Motor-Drive": "HW-08 GaN HEMT器件及其在电机驱动中的应用",
   }
   ```
4. 保存后重启后端（或等 `--reload` 自动重载）
5. 刷新页面 → 你的模块现在排在 HW-07 后面而不是最末尾

### 4.7 进阶：创建全新的路径分类（可选）

如果你想建一个完全独立的新路径（比如「电力电子器件前沿」）：

**Step 1 — 建目录+放文件：**

```
motor-control-knowledge-base/
└── power-devices/              # 新建目录
    ├── PD-01-GaN-HEMT.md       # 你的文件
    └── PD-02-Super-Junction.md # 更多文件
```

**Step 2 — 注册到后端 docs.py：**

在 `docs.py` 中追加（约在第299行 `return {` 之前）：

```python
PD_ORDER = []  # 留空=全部靠自动发现

PD_TITLES = {}  # 留空=自动从文件提取标题
```

然后在 `parse_knowledge_base()` 函数体内追加：

```python
pd_modules = []
pd_modules.extend(_discover_unlisted_modules("power-devices", PD_ORDER))
```

并在返回字典中追加：

```python
"powerDevices": {
    "id": "powerDevices",
    "title": "💎 电力电子器件前沿",
    "description": "GaN/SiC/超结器件等新型功率半导体在电机驱动中的应用",
    "modules": pd_modules,
},
```

同时在 `sectionsOrder` 列表中加入 `"powerDevices"`。

**Step 3 — 重启后端，刷新页面。** 左侧导航栏底部会出现新的 **「💎 电力电子器件前沿」** 路径。

### 4.8 完整检查清单

| 步骤 | 操作 | 验证方法 |
|------|------|----------|
| ✅ 创建 `.md` 文件 | 放入目标子目录 | 文件存在于 `motor-control-knowledge-base/xxx/` |
| ✅ 首行写标题 | `# 我的标题` | 标题会显示在侧边栏 |
| ✅ (可选) 创建检验题 | `*-assessment.md` 同目录 | 测验按钮可点击 |
| ✅ 重启/等待后端热更新 | uvicorn `--reload` 模式自动检测 | 访问 `/api/docs/list` 能搜到 |
| ✅ 浏览器验证 | 打开 `http://127.0.0.1:8001` | 侧边栏对应路径下出现新条目 |
| ✅ (可选) 调整顺序 | 编辑 `docs.py` 的 `*_ORDER` | 条目位置符合预期 |

---

## 五、文件统计

| 类别 | 数量 | 说明 |
|------|------|------|
| 知识库 Markdown 文档 | ~130+ | 含模块文档 + 检验题目 + 索引文件 |
| 知识库路径数 | 14 | 含3个代码分析路径(ODrive/VESC/对比) |
| 交互式仿真工具 | 30+ | Vue组件 + HTML独立页面 |
| 嵌入式 C/C++ 源文件 | ~150+ | AxDr + MC_LIB + odriver_core + vesc_core |
| Web 前端组件 | ~15 | Vue3 组件 |
| Web API 端点 | ~8 | FastAPI 路由 |
| 学术论文 PDF | 2+ | 离散时间非线性磁链观测器等 |

---

## 六、快速启动指南

| 目标 | 操作 |
|------|------|
| 浏览知识库 | 双击 `启动知识库.bat` → 自动打开 http://127.0.0.1:8001 |
| 考试仪表盘 | 双击 `启动ExamDashboard.bat` → 打开 http://localhost:3000 |
| 代码阅读器 | `cd code-reader-web-app && scripts/start.bat` |
| 添加自定义文档 | 将 .md 放入 `motor-control-knowledge-base/` 对应子目录 → 重启后端 |

---

## 七、项目依赖关系

```
motor-control-knowledge-base/ (数据源)
    ↓ 读取Markdown文件
motor-learning-web/backend/ (FastAPI)
    ↓ 提供API + 静态文件
motor-learning-web/frontend/ (Vue3)
    ↓ 渲染用户界面
    浏览器 (用户)

motor-control-knowledge-base/ (数据源)
    ↓ 也可被代理读取
ExamDashboard/ (Next.js)

MC_LIB/ ← AxDr/ 引用MC_LIB作为控制库
odriver_core/ ← 知识库ODrive/ 分析此代码
vesc_core/ ← 知识库VESC/ 分析此代码
hpm_MC/ ← 知识库HPM-MC/ 分析此SDK
```
