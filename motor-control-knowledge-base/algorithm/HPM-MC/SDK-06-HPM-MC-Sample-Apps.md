# HPM-MC 示例应用指南 ★★★☆☆ 🔰📚🔧

**文档版本：** v1.0
**生成日期：** 2026-05-23
**适用对象：** 电机控制工程师、嵌入式开发者
**前置知识：** C语言编程、电机控制基础、HPM系列MCU

---

## 目录

1. [概述](#1-概述)
2. [示例应用详解](#2-示例应用详解)
   - [2.1 bldc_block — BLDC方波六步换相](#21-bldc_block--bldc方波六步换相)
   - [2.2 bldc_foc — BLDC/PMSM FOC控制](#22-bldc_foc--bldcpmsm-foc控制)
   - [2.3 bldc_hfi — 高频注入无感FOC](#23-bldc_hfi--高频注入无感foc)
   - [2.4 bldc_smc — 滑模观测器无感FOC](#24-bldc_smc--滑模观测器无感foc)
   - [2.5 bldc_over_zero — 过零检测BLDC控制](#25-bldc_over_zero--过零检测bldc控制)
   - [2.6 bldc_offline_param_detection — 电机参数离线辨识](#26-bldc_offline_param_detection--电机参数离线辨识)
   - [2.7 bldc_littlevgl_foc — LittleVGL图形界面FOC](#27-bldc_littlevgl_foc--littlevgl图形界面foc)
   - [2.8 step_motor_foc — 步进电机FOC控制](#28-step_motor_foc--步进电机foc控制)
   - [2.9 step_motor_closed_loop — 步进电机闭环控制](#29-step_motor_closed_loop--步进电机闭环控制)
3. [示例→hpm_mcl 模块映射表](#3-示例hpm_mcl-模块映射表)
4. [入门建议](#4-入门建议)

---

## 1. 概述

### 1.1 示例应用全景

HPM Motor Control SDK 在 `hpm_MC/samples/motor_ctrl/` 下提供了 **9个** 完整的电机控制示例应用，覆盖从基础方波控制到高级无感FOC的完整技术栈：

```
┌──────────────────────────────────────────────────────────────────────┐
│                   HPM MC 示例应用全景                                  │
├──────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  BLDC梯形波控制 (bldc_block)      Hall传感器 + 六步换相       │    │
│  │  BLDC无感过零 (bldc_over_zero)    反电动势过零 + 六步换相     │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  BLDC有感FOC (bldc_foc)         QEI编码器 + 硬件/软件电流环   │    │
│  │  GUI FOC演示 (bldc_littlevgl_foc)  LittleVGL触摸屏 + FOC      │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  无感HFI (bldc_hfi)             高频注入 + 零低速运行          │    │
│  │  无感SMO (bldc_smc)             滑模观测器 + 中高速运行        │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  离线参数辨识 (bldc_offline_param_detection)  Rs/Ld/Lq/Flux   │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  步进FOC (step_motor_foc)        梯形加减速 + 开环矢量控制     │    │
│  │  步进闭环 (step_motor_closed_loop) 位置/速度闭环 + 编码器反馈  │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                       │
└──────────────────────────────────────────────────────────────────────┘
```

### 1.2 示例组织方式

每个示例遵循统一目录结构：

```
bldc_xxx/
├── src/
│   ├── bldc_xxx.c          # 主程序（初始化、控制循环）
│   ├── bldc_xxx_callback.c # 回调函数（PWM/ADC/编码器中断）
│   ├── bldc_xxx_cfg.h      # 硬件配置宏
│   └── mcl_app_config.h    # hpm_mcl_v2 功能开关
├── CMakeLists.txt           # CMake构建配置
├── README_zh.rst            # 中文说明文档
├── README_en.rst            # 英文说明文档
└── app.yaml                 # 应用元信息
```

### 1.3 两套API共存说明

示例中使用了两套API体系：

| API版本 | 特征头文件 | 示例 |
|---------|-----------|------|
| **v1（旧版）** | `hpm_foc.h`, `hpm_block.h`, `hpm_smc.h`, `hpm_hfi.h`, `hpm_over_zero.h` | bldc_hfi, bldc_smc, bldc_over_zero, bldc_littlevgl_foc |
| **v2（新版）** | `hpm_mcl_loop.h`, `hpm_mcl_control.h`, `hpm_mcl_detect.h` 等 | bldc_block, bldc_foc, bldc_offline_param_detection, step_motor_* |

v2 采用 `mcl_*_t` 配置结构体 + 统一 `hpm_mcl_loop_init()` 初始化模式，更模块化。

---

## 2. 示例应用详解

### 2.1 bldc_block — BLDC方波六步换相

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：基于Hall传感器的BLDC梯形波六步换相速度闭环控制              │
│  核心模块：hpm_mcl_loop (mcl_mode_block), hpm_mcl_uvw           │
│  适用电机：BLDC（梯形波反电动势），极对数=2                        │
└──────────────────────────────────────────────────────────────────┘
```

**控制架构：**

```
Hall传感器 → UVW位置解码 → 六步换相表 → PWM输出 → BLDC电机
                  ↑                        ↓
                  └── 速度PI控制器 ←── IIR滤波 ←── 编码器速度
```

**关键配置（bldc_block.c）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQUENCY` | 20kHz | PWM载波频率 |
| `PWM_RELOAD` | `motor_clock_hz / 20k - 1` | PWM重载值 |
| `MOTOR0_SPD` | 20.0 r/s | 默认目标速度 |
| `SPEED_MAX / SPEED_MIN` | 40 / 5 r/s | 速度范围（含死区） |
| `TIMER_TS` | 1000 μs | 控制周期 |
| `MOTOR_POLE_NUM` | 2 | 极对数 |
| `BOARD_BLDC_BLOCK_SPEED_KP` | 0.0005 | 速度环比例增益 |
| `BOARD_BLDC_BLOCK_SPEED_KI` | 0.000009 | 速度环积分增益 |
| `speed_pid.integral_max/min` | ±0.95 | PID积分限幅 |
| `speed_pid.output_max/min` | ±1.0 | PID输出限幅（归一化占空比） |
| `i_max` | 9A | 最大电流 |
| `vbus` | 24V | 母线电压 |

**硬件支持：** Hall传感器 (HPMSOC_HAS_HPMSDK_HALL) 或 QEIV2 UVW模式 (HPMSOC_HAS_HPMSDK_QEIV2)

**运行方式：** 串口输入目标转速（r/s），支持正反转。

**mcl_app_config.h 特性开关：** 全部关闭（MCL_EN_* = 0），方波控制无需FOC特性。

**架构细节（HPM 官方资料补充）**:

bldc_block 示例实际使用 **v2 API**（`mcl_loop_t` 聚合体），而非 v1 的 `hpm_block.h`:

```
motor0_t {
    mcl_encoder_t encoder;         // Hall/QEI 编码器
    mcl_filter_iir_df1_t encoder_iir; // IIR 速度滤波器
    mcl_drivers_t drivers;         // PWM 驱动
    mcl_control_t control;         // 控制算法
    mcl_loop_t loop;               // 主控制环 (mcl_mode_block)
    mcl_analog_t analog;           // 模拟采样
    mcl_detect_t detect;           // 故障检测
}
```

**中断服务架构**:
- Hall 中断 → `hpm_mcl_uvw_get_theta()` → 获取角度 → `hpm_mcl_loop_refresh_block()` 刷新换相
- 定时器中断(1ms) → `hpm_mcl_detect_loop()` + `hpm_mcl_encoder_process()` + `hpm_mcl_loop()`

**六步换相序列**（基于 Hall 状态）:
| 扇区 | Hall | 导通相 | 角度 |
|------|------|--------|------|
| 1 | 101 | AH+BL | 0°-60° |
| 2 | 100 | AH+CL | 60°-120° |
| 3 | 110 | BH+CL | 120°-180° |
| 4 | 010 | BH+AL | 180°-240° |
| 5 | 011 | CH+AL | 240°-300° |
| 6 | 001 | CH+BL | 300°-360° |

**IIR 滤波器**: 使用二阶直接I型 IIR（`hpm_mcl_filter_iir_df1`），通过双线性变换从模拟域设计

参考: [HPM KB: BLDC BLOCK 换相控制技术](https://kb.hpmicro.com/2025/08/07/)

---

### 2.2 bldc_foc — BLDC/PMSM FOC控制

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：有感FOC速度/位置双闭环控制，支持软件/硬件/混合三种电流环     │
│  核心模块：hpm_mcl_loop, hpm_mcl_abz, hpm_mcl_detect             │
│  适用电机：表贴式PMSM（正弦波反电动势），极对数=2                  │
└──────────────────────────────────────────────────────────────────┘
```

**控制架构：**

```
QEI编码器 → ABZ位置解码 → Clarke/Park变换 → 电流PI(d/q) → 反Park → SVPWM
                              ↑                              ↓
                              └── 速度PI ←── 位置PI ←──┐    BLDC电机
                                                        │
                                              ADC电流采样 ← A相/B相
```

**三种电流环模式：**

| 模式 | `mcl_loop_mode_t` | 宏定义 | 电流环执行位置 | 特点 |
|------|-------------------|--------|---------------|------|
| 软件FOC | `mcl_mode_foc` | — | CPU (ADC ISR) | 通用，约1μs执行时间 |
| 硬件FOC | `mcl_mode_hardware_foc` | `HW_CURRENT_FOC_ENABLE` | VSC+CLC+QEO硬件链 | 零CPU开销电流环 |
| 混合FOC | `mcl_mode_hybrid_foc` | `MCL_HARDWARE_HYBRID_LOOP_ENABLE` | CLC硬件加速 | 部分硬件加速 |

**硬件FOC加速链（HW_CURRENT_FOC_ENABLE）：**

```
ADC → TRGM → VSC(Clarke+Park) → CLC(PI控制器) → TRGM → QEO(反Park+SVPWM) → PWM DAC
                                            ↑                ↑
                                      QEI位置信号        QEI位置信号
```

**关键配置（bldc_foc.c）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQUENCY` | 20kHz | PWM/电流环频率 |
| `PWM_DEAD_AREA_TICK` | 100 | 死区时间 |
| `CURRENT_LOOP_BANDWIDTH` | 500 Hz | 电流环带宽（自动计算kp/ki） |
| `speed_loop_ts` | 5 × PWM周期 | 速度环周期 |
| `position_loop_ts` | 20 × PWM周期 | 位置环周期 |
| `motor.res` | 0.0011Ω | 相电阻 |
| `motor.ls / ld / lq` | 0.00263H | 电感 |
| `motor.flux` | 0.0015 Wb | 永磁磁链 |
| `analog.opamp_gain` | 10 | 运放增益 |
| `analog.sample_res` | 0.01Ω | 采样电阻 |

**运行方式：** 先进行电机对中（三阶段对齐算法），然后选择速度模式或位置模式。

**mcl_app_config.h 特性开关：**

| 宏 | 值 | 说明 |
|----|-----|------|
| `MCL_EN_THETA_FORECAST` | 1 | 使能角度预测 |
| `MCL_EN_DQ_AXIS_DECOUPLING_FUNCTION` | 1 | 使能d/q轴解耦 |
| `MCL_EN_DEAD_AREA_COMPENSATION` | 1 | 使能死区补偿 |
| `MCL_EN_LOOP_TIME_COUNT` | 1 | 使能控制环时间统计 |

---

### 2.3 bldc_hfi — 高频注入无感FOC

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：基于方波高频注入的零速/低速无感FOC控制                      │
│  核心模块：hpm_foc (v1), hpm_hfi (高频注入算法)                    │
│  适用电机：凸极性明显的PMSM（IPM电机），极对数=2                    │
└──────────────────────────────────────────────────────────────────┘
```

**控制原理：**
在d轴注入高频方波电压信号，通过检测q轴高频电流响应来估计转子位置。利用电机的凸极效应（`Ld ≠ Lq`），在零速和极低速下仍可获得可靠的位置信息。

**关键配置（bldc_hfi.c + bldc_hfi_cfg.h）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `MOTOR0_HFI_SPD` | 1.1 r/s | 目标低速运行 |
| `inject_para.vh` | 0.625 × PWM_RELOAD | 注入电压幅值 |
| `inject_pll.kp` | `BOARD_BLDC_HFI_PLL_KP` (10.0) | PLL比例增益 |
| `inject_pll.ki` | `BOARD_BLDC_HFI_PLL_KI` (1.0) | PLL积分增益 |
| `inject_pll.filter` | 0.1 | 位置低通滤波系数 |
| `inject_pll.loop_s` | 0.00005s | PLL控制周期(50μs) |
| `speedloop_para.i_kp` | `MOTOR0_HFI_KP` (40) | 速度环Kp |
| `speedloop_para.i_ki` | 0.015 | 速度环Ki |
| `speedloop_para.i_max` | 300 | 速度环输出限幅 |
| `inject_pole_detect.current_d_init_val` | 80 | N/S极性辨识初始电流 |

**控制流程：**

```
1. 磁极辨识 (hpm_mcl_hfi_pole_detect) → 判断N/S极
2. 高频注入启动 → PLL锁相环跟踪转子位置
3. 速度闭环 → FOC电流环 → SVPWM输出
```

**注意事项：**
- 仅用于实验目的，demo级别实现
- 使用v1 API (`BLDC_CONTROL_FOC_PARA` 结构体)
- 运行速度为极低速1.1 r/s
- 支持SEGGER RTT数据可视化（J-Scope）

---

### 2.4 bldc_smc — 滑模观测器无感FOC

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：基于滑模观测器的中高速无感FOC速度控制                       │
│  核心模块：hpm_foc (v1), hpm_smc (滑模观测器)                     │
│  适用电机：表贴式PMSM，极对数=2                                    │
└──────────────────────────────────────────────────────────────────┘
```

**控制原理：**
通过电机数学模型构建滑模观测器，利用电流误差作为滑模面，估计反电动势，进而通过PLL提取转子位置和速度。在中高速段（>5%额定转速）具有良好性能。

**关键配置（bldc_smc.c + bldc_smc_cfg.h）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQUENCY` | 20kHz | PWM频率 |
| `MOTOR0_SPD` | 15 r/s | 起始速度 |
| `BLDC_ANGLE_SET_TIME_MS` | 2000ms | 对中时间 |
| 速度范围 | 15~40 r/s | 自动加减速循环 |

**控制流程：**

```
1. 电机对中 (2000ms) → 注入d轴电流将转子拉到已知位置
2. IF拖动启动 → 开环加速到SMO可观测速度
3. SMO闭环 → 切换为滑模观测器角度
4. 速度自动循环 → 15→40→15 r/s 持续运行
```

**注意事项：**
- 使用v1 API
- 存在最低速度限制（约5%额定转速），低速会失步
- 启动包含强制定位阶段，不可干预

---

### 2.5 bldc_over_zero — 过零检测BLDC控制

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：无传感器的BLDC方波六步换相速度控制                         │
│  核心模块：hpm_block (v1), hpm_over_zero (反电动势过零检测)       │
│  适用电机：BLDC（梯形波反电动势），极对数=2                        │
└──────────────────────────────────────────────────────────────────┘
```

**控制原理：**
在六步换相的悬空相上检测反电动势过零点（BEMF ZCP），过零后延时30°电角度执行换相。与Hall传感器方案相比，省去了位置传感器，降低了系统成本。

**关键配置（bldc_over_zero.c）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `MOTOR0_PWM_DUTY_INIT` | 5900 | 启动初始占空比 |
| `MOTOR0_SPD` | 15.0 r/s | 目标速度 |
| `TIMER_TIMES_1MS` | 10 | 定时器分频 |
| `QEI_WDOG_TIMEOUT` | 200000 | 看门狗超时 |

**控制流程：**

```
1. 强制定位 → 注入电流将转子拉到已知位置
2. 开环拖动 → 逐步加速到过零检测可检测速度
3. 过零检测切换 → 关闭拖动，切换到BEMF过零换相
4. 速度闭环 → 自动加减速循环 5→40→5 r/s
```

**硬件要求：**
- 需要电路板支持相电流/相电压采集
- 连接电机扩展板J6/J7/J12到UU/UV/UW

**注意事项：**
- 使用v1 API
- 低速过零信号弱，需要开环拖动加速到一定转速后切换
- 适用于风扇、水泵等对低速性能要求不高的场景

---

### 2.6 bldc_offline_param_detection — 电机参数离线辨识

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：自动检测电机相电阻、d/q轴电感、相电感、永磁磁链             │
│  核心模块：hpm_mcl_loop (mcl_mode_offline_param_detection)       │
│  适用电机：PMSM/BLDC                                              │
└──────────────────────────────────────────────────────────────────┘
```

**辨识参数：**

| 参数 | 符号 | 辨识方法 |
|------|------|---------|
| 相电阻 | Rs | 注入直流电压，测量稳态电流 |
| d轴电感 | Ld | 注入d轴高频电压，测量电流响应 |
| q轴电感 | Lq | 注入q轴高频电压，测量电流响应 |
| 相电感 | Ls | 注入交流电压，测量电流响应 |
| 永磁磁链 | Flux | 拖动电机旋转，测量反电动势 |

**关键配置（bldc_offline_param_detection.c）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQUENCY` | 20kHz | PWM频率 |
| `CURRENT_LOOP_BANDWIDTH` | 500 Hz | 电流环带宽 |
| `loop.mode` | `mcl_mode_offline_param_detection` | 离线辨识模式 |
| `PWM_DEAD_AREA_TICK` | 100 | 死区时间 |

**输出示例：**

```
motor parameter detection demo.
flux: 0.044300, ld: 0.000968, lq: 0.001808, ls: 0.002777, rs: 1.037876
flux: 0.044056, ld: 0.000992, lq: 0.001803, ls: 0.002795, rs: 1.022872
...
```

**注意事项：**
- 辨识前需先完成电机对中
- 辨识结果会循环输出，供观察数据稳定性
- 辨识得到的参数可直接填入 `bldc_foc.c` 的 `motor.cfg.mcl.physical.motor` 中使用

---

### 2.7 bldc_littlevgl_foc — LittleVGL图形界面FOC

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：带LCD触摸屏的图形界面FOC控制演示                            │
│  核心模块：hpm_foc (v1) + LittleVGL v8.3.5                       │
│  适用电机：表贴式PMSM，极对数=2                                    │
└──────────────────────────────────────────────────────────────────┘
```

**系统架构：**

```
触摸屏输入 → LittleVGL GUI → 速度/位置设定值 → FOC控制 → PWM输出
                ↑                                  ↓
                └── 实时显示 ←── 速度/位置反馈 ←── QEI编码器
```

**关键配置：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `LV_TICK` | 5ms | LVGL定时器周期 |
| `BLDC_ANGLE_SET_TIME_MS` | 2000ms | 对中时间 |
| `BLDC_CURRENT_SET_TIME_MS` | 200ms | 电流校准时间 |
| 显示屏型号 | LCD-800480W070TC | 7寸800×480触摸屏 |

**运行方式：**
1. 上电→初始化→电机对中（1~2秒）
2. 屏幕显示图形界面
3. 点击屏幕控件切换速度/位置模式
4. 实时图形化显示速度和位置曲线

**依赖：** LittleVGL 8.3.5、触摸屏驱动

---

### 2.8 step_motor_foc — 步进电机FOC控制

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：基于FOC的步进电机开环矢量控制，梯形加减速                    │
│  核心模块：hpm_mcl_loop (mcl_mode_step_foc)                      │
│  适用电机：42步进电机，极对数=50，两相                               │
└──────────────────────────────────────────────────────────────────┘
```

**控制原理：**
步进电机FOC将两相电流控制为相差90°的正弦波，通过控制正弦波的频率和幅值实现转速和转矩控制。与方波驱动相比，电流波形为正弦波，转矩脉动更小，噪音更低。

**关键配置（step_motor_foc.c）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQUENCY` | 40kHz | 步进电机需要更高PWM频率 |
| `motor.pole_num` | 50 | 步进电机高极对数 |
| `cfg.path` (路径规划) | 梯形加减速 | 通过mcl_path_plan实现 |
| `T_CURE_TIMES` | 1 | 电流环周期倍率 |

**重要差异（与bldc_foc对比）：**
- 无编码器 (`mcl_mode_step_foc` 不需要encoder)
- 使用 `mcl_path_plan` 进行梯形加减速路径规划
- 4通道PWM输出（步进电机两相四线）
- 极对数50（远大于普通BLDC的2~8极对）
- PWM频率40kHz（普通BLDC用20kHz）

**运行方式：** 自动循环切换速度和方向，串口打印速度信息，电流波形始终保持正弦波。

---

### 2.9 step_motor_closed_loop — 步进电机闭环控制

```
┌──────────────────────────────────────────────────────────────────┐
│  功能：带编码器反馈的步进电机位置/速度闭环FOC控制                   │
│  核心模块：hpm_mcl_loop, hpm_mcl_abz                             │
│  适用电机：42步进电机（带编码器），极对数=50                        │
└──────────────────────────────────────────────────────────────────┘
```

**与 step_motor_foc 的关键区别：**

| 特性 | step_motor_foc | step_motor_closed_loop |
|------|---------------|----------------------|
| 编码器 | 无 | QEI/ABZ编码器 |
| 控制模式 | 开环速度控制 | 闭环速度/位置控制 |
| 路径规划 | mcl_path_plan梯形加减速 | PID闭环调节 |
| loop.mode | `mcl_mode_step_foc` | `mcl_mode_foc` |
| 用户交互 | 自动循环 | 串口选择模式+输入参数 |

**关键配置（step_motor_closed_loop.c）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQUENCY` | 20kHz | PWM频率 |
| `SPEED_MAX` | 10 r/s | 最大速度 |
| `motor.pole_num` | 50 | 步进电机极对数 |
| `STEP_MOTOR_QEI_FOC_PHASE_COUNT_PER_REV` | 编码器线数 | 位置反馈分辨率 |

**运行方式：**
1. 串口选择模式：0=位置模式，1=速度模式
2. 速度模式：输入-10~+10 r/s
3. 位置模式：输入目标位置值，电机精确定位并保持

**应用场景：** 需要精确定位和防丢步的步进电机应用，如3D打印机、CNC、机器人关节等。

---

## 3. 示例→hpm_mcl 模块映射表

### 3.1 v2 API 示例模块使用矩阵

| 示例 | loop_mode | encoder | analog | drivers | control | detect | path_plan | abz | uvw |
|------|-----------|---------|--------|---------|---------|--------|-----------|-----|-----|
| **bldc_block** | block | ✓ | — | ✓ | ✓ | ✓ | — | — | ✓ |
| **bldc_foc** | foc/hw_foc/hybrid | ✓ | ✓ | ✓ | ✓ | ✓ | — | ✓ | — |
| **bldc_offline_param_detection** | offline_detect | ✓ | ✓ | ✓ | ✓ | ✓ | — | ✓ | — |
| **step_motor_foc** | step_foc | — | ✓ | ✓ | ✓ | ✓ | ✓ | — | — |
| **step_motor_closed_loop** | foc | ✓ | ✓ | ✓ | ✓ | ✓ | — | ✓ | — |

### 3.2 v1 API 示例模块使用矩阵

| 示例 | hpm_foc | hpm_smc | hpm_hfi | hpm_block | hpm_over_zero |
|------|---------|---------|---------|-----------|---------------|
| **bldc_hfi** | ✓ | — | ✓ | — | — |
| **bldc_smc** | ✓ | ✓ | — | — | — |
| **bldc_over_zero** | — | — | — | ✓ | ✓ |
| **bldc_littlevgl_foc** | ✓ | ✓ | — | — | — |

### 3.3 硬件加速模块使用矩阵

| 示例 | VSC | CLC | QEOv2 | TRGM | SYNT |
|------|-----|-----|-------|------|------|
| **bldc_foc (hw_foc)** | ✓ | ✓ | ✓ | ✓ | ✓ |
| **bldc_foc (hybrid)** | — | ✓ | — | — | ✓ |

### 3.4 传感器方案矩阵

| 示例 | Hall | QEI/ABZ | QEIV2-UVW | 无感 |
|------|------|---------|-----------|------|
| **bldc_block** | ✓ | — | ✓ | — |
| **bldc_foc** | — | ✓ | — | — |
| **bldc_hfi** | — | — | — | HFI |
| **bldc_smc** | — | — | — | SMO |
| **bldc_over_zero** | — | — | — | BEMF过零 |
| **bldc_offline_param_detection** | — | ✓ | — | — |
| **bldc_littlevgl_foc** | — | ✓ | — | — |
| **step_motor_foc** | — | — | — | 开环 |
| **step_motor_closed_loop** | — | ✓ | — | — |

---

## 4. 入门建议

### 4.1 推荐学习顺序

```
第一阶段：理解电机控制基础
  1. bldc_block          ← 最简单的方波控制，理解六步换相原理
  2. bldc_over_zero       ← 理解反电动势过零检测，无传感器入门

第二阶段：掌握FOC矢量控制
  3. bldc_foc             ← FOC核心示例，理解Clarke/Park/SVPWM
  4. bldc_littlevgl_foc   ← 可视化理解FOC运行过程

第三阶段：无感控制进阶
  5. bldc_smc             ← 滑模观测器，中高速无感方案
  6. bldc_hfi             ← 高频注入，零低速无感方案

第四阶段：工具与扩展
  7. bldc_offline_param_detection ← 参数辨识，FOC的前提
  8. step_motor_foc       ← 步进电机矢量控制
  9. step_motor_closed_loop ← 步进电机闭环控制
```

### 4.2 不同应用场景的示例选择指南

| 应用场景 | 推荐示例 | 原因 |
|---------|---------|------|
| 低成本风扇、水泵 | bldc_over_zero | 无传感器方波，成本最低 |
| 有Hall传感器的电动工具 | bldc_block | 带传感器方波，启动可靠 |
| 高性能伺服、机器人 | bldc_foc | 有感FOC，精度高、响应快 |
| 低速大转矩（无传感器） | bldc_hfi | HFI零低速运行 |
| 中高速风机、压缩机 | bldc_smc | SMO无感，简单可靠 |
| 未知电机参数调试 | bldc_offline_param_detection | 先辨识参数再配置FOC |
| 人机交互演示 | bldc_littlevgl_foc | GUI可视化 |
| 3D打印机、CNC | step_motor_closed_loop | 闭环防丢步 |
| 简单步进驱动 | step_motor_foc | 开环矢量，低噪音 |

### 4.3 快速上手检查清单

1. **硬件确认：** 核对电机参数（极对数、电压、电流）与示例配置是否匹配
2. **供电检查：** 给驱动板上电，观察电流无异常后再给核心板上电
3. **PWM频率：** 确认PWM频率设置正确
4. **死区时间：** 防止上下管直通，典型值50~100 tick
5. **ADC配置：** 确认参考电压、运放增益、采样电阻值
6. **编码器配置：** 确认分辨率、方向、AB相序
7. **安全第一：** 首次运行时时刻注意电流，异常立即断电
8. **从低速开始：** 先用低速验证基本功能，再逐步提高

---

## 5. 示例应用核心代码分析

本章节深入分析四个核心示例的源码实现，揭示从初始化到控制循环的完整代码路径，帮助开发者理解用户代码如何与hpm_mcl中间件交互。

### 5.1 FOC示例核心代码 (`bldc_foc.c`)

#### 5.1.1 初始化流程

**原理：** FOC控制的初始化需要完成硬件资源配置、电机参数设定、控制环参数计算、中间件模块初始化和对中校准五个阶段。电流环PI参数根据电机电感和电阻自动计算，公式为：

$$K_p = L_s \cdot (2\pi \cdot f_{bw})^2 \cdot T_s \cdot 1.5$$

$$K_i = R_s \cdot (2\pi \cdot f_{bw})^2 \cdot T_s \cdot 1.5$$

其中 $f_{bw}$ 为电流环带宽（500Hz），$T_s$ 为电流环周期（50μs）。

**代码：**

```c
int main(void)
{
    board_init();
    init_adc_bldc_pins();
    init_pwm_pins(MOTOR0_BLDCPWM);
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_QEI_CLOCK_SOURCE);
    motor_init();
    adc_isr_enable();
    timer_init();
    motor_adc_midpoint();
    hpm_mcl_loop_enable(&motor0.loop);
    motor_angle_align();
    speed = MOTOR0_SPD;
    user_speed.enable = true;
    user_speed.value = speed * MCL_PI * 2;
    hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
    // ... 速度/位置模式选择
}
```

**逐块解析：**

| 步骤 | 函数调用 | 作用 |
|------|---------|------|
| 1 | `board_init()` | HPM SDK板级初始化：时钟、GPIO、调试串口 |
| 2 | `init_adc_bldc_pins()` / `init_pwm_pins()` | 配置ADC和PWM引脚的TRGMUX路由 |
| 3 | `motor_init()` | **核心初始化**：填充motor0.cfg配置结构体，调用6个hpm_mcl_xxx_init()初始化中间件模块 |
| 4 | `adc_isr_enable()` | 使能ADC中断，注册ADC ISR |
| 5 | `timer_init()` | 配置GPTMR定时器（1ms周期），用于故障检测 |
| 6 | `motor_adc_midpoint()` | 采集200ms ADC零点偏移，存入`adc_u_midpoint`/`adc_v_midpoint` |
| 7 | `hpm_mcl_loop_enable()` | 使能控制环，`loop->enable = true` |
| 8 | `motor_angle_align()` | 三阶段对中算法，将转子拉到已知电角度位置 |

**motor_init()内部初始化链：**

```c
void motor_init(void)
{
    // 1. 物理参数配置
    motor0.cfg.mcl.physical.board.analog[analog_a_current].adc_reference_vol = 3.3;
    motor0.cfg.mcl.physical.board.analog[analog_a_current].opamp_gain = 10;
    motor0.cfg.mcl.physical.motor.i_max = 9;
    motor0.cfg.mcl.physical.motor.res = 0.0011;
    motor0.cfg.mcl.physical.motor.ls = 0.00263;
    // ...

    // 2. 电流环PI参数自动计算
    motor0.cfg.control.currentd_pid_cfg.cfg.kp = motor0.cfg.mcl.physical.motor.ls *
        (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
        motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;
    motor0.cfg.control.currentd_pid_cfg.cfg.ki = motor0.cfg.mcl.physical.motor.res *
        (powf(MOTOR0_CURRENT_LOOP_BANDWIDTH * 2 * MCL_PI, 2)) *
        motor0.cfg.mcl.physical.time.current_loop_ts * 1.5f;

    // 3. 回调函数注册
    motor0.cfg.drivers.callback.init = pwm_init;
    motor0.cfg.drivers.callback.update_duty_cycle = pwm_duty_set;
    motor0.cfg.analog.callback.get_value = adc_value_get;
    motor0.cfg.encoder.callback.get_theta = encoder_get_theta;

    // 4. 中间件模块初始化（顺序重要）
    hpm_mcl_analog_init(&motor0.analog, &motor0.cfg.analog, &motor0.cfg.mcl);
    hpm_mcl_filter_iir_df1_init(&motor0.encoder_iir, &motor0.cfg.encoder_iir, &motor0.encoder_iir_mem[0]);
    hpm_mcl_encoder_init(&motor0.encoder, &motor0.cfg.mcl, &motor0.cfg.encoder, &motor0.encoder_iir);
    hpm_mcl_drivers_init(&motor0.drivers, &motor0.cfg.drivers);
    hpm_mcl_control_init(&motor0.control, &motor0.cfg.control);
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL, NULL);
    hpm_mcl_detect_init(&motor0.detect, &motor0.cfg.detect, &motor0.loop, &motor0.encoder, &motor0.analog, &motor0.drivers);

    // 5. 高级特性使能
    hpm_mcl_enable_dq_axis_decoupling(&motor0.loop);
    hpm_mcl_enable_dead_area_compensation(&motor0.loop);
}
```

**工程要点：**
- 电流环PI参数由带宽自动推导，无需手动整定；速度环/位置环PI需根据负载惯量手动调整
- `motor0`结构体放置在Fast RAM（`ATTR_PLACE_AT_FAST_RAM_INIT`），确保ISR访问零等待
- 初始化顺序不可调换：analog → encoder → drivers → control → loop → detect，存在依赖关系
- 硬件FOC模式（`HW_CURRENT_FOC_ENABLE`）下，encoder.init设为NULL，编码器由硬件链直接读取

#### 5.1.2 ADC中断回调中的控制循环

**原理：** FOC电流环以PWM频率（20kHz）运行，由ADC采样完成中断触发。每次中断执行：编码器角度处理 → 完整FOC控制环（电流环+速度环+位置环）。

**代码：**

```c
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_ADC_IRQn, isr_adc)
void isr_adc(void)
{
    uint32_t status;
    status = hpm_adc_get_status_flags(&hpm_adc_u);
    if ((status & BOARD_BLDC_ADC_TRIG_FLAG) != 0) {
        hpm_adc_clear_status_flags(&hpm_adc_u, BOARD_BLDC_ADC_TRIG_FLAG);
        hpm_mcl_encoder_process(&motor0.encoder, motor0.cfg.mcl.physical.time.mcu_clock_tick / PWM_FREQUENCY);
        hpm_mcl_loop(&motor0.loop);
    }
}
```

**逐行解析：**

| 行 | 代码 | 作用 |
|----|------|------|
| 1 | `SDK_DECLARE_EXT_ISR_M(...)` | HPM SDK宏，声明ADC中断ISR并注册到向量表 |
| 2 | `hpm_adc_get_status_flags()` | 读取ADC中断状态，确认是PWM触发ADC完成 |
| 3 | `hpm_adc_clear_status_flags()` | 清除中断标志，避免重复进入 |
| 4 | `hpm_mcl_encoder_process()` | 编码器角度处理：读取QEI位置、计算速度、IIR滤波、角度预测 |
| 5 | `hpm_mcl_loop()` | **核心控制环入口**，根据loop->cfg->mode分发到具体控制函数 |

**v2 API内部调用链（`hpm_mcl_loop` → `hpm_mcl_current_foc_loop`）：**

```
hpm_mcl_loop()
  └─ hpm_mcl_current_foc_loop()
       ├─ hpm_mcl_encoder_get_theta()          // 获取电角度
       ├─ [位置环] position_pid()               // 位置PI → 速度参考
       ├─ [速度环] speed_pid()                  // 速度PI → q轴电流参考
       ├─ hpm_mcl_analog_get_value()            // 读取A/B相电流
       ├─ clarke(ia, ib, ic → α, β)             // Clarke变换
       ├─ park(α, β → Id, Iq)                   // Park变换
       ├─ currentd_pid() / currentq_pid()       // d/q轴电流PI
       ├─ [dq解耦] ud -= ω·Lq·Iq, uq += ω·(Ld·Id+ψ)  // 交叉解耦
       ├─ invpark(ud, uq → α, β)                // 反Park变换
       ├─ svpwm(α, β → duty_a, duty_b, duty_c)  // SVPWM调制
       ├─ [死区补偿] dead_area_compensation()    // 死区效应补偿
       └─ hpm_mcl_drivers_update_bldc_duty()    // 更新PWM占空比
```

**v1 API等效调用链（HFI/SMC示例中使用）：**

v1 API通过函数指针实现类似流程，核心函数为 `hpm_mcl_bldc_foc_ctrl_dq_to_pwm()`：

```c
void hpm_mcl_bldc_foc_ctrl_dq_to_pwm(BLDC_CONTROL_FOC_PARA *par)
{
    par->samplcurpar.func_sampl(&par->samplcurpar);      // 电流采样校准
    hpm_mcl_bldc_foc_clarke(..., &par->ialpha, &par->ibeta);  // Clarke
    bldc_foc_sin_cos(par->electric_angle, ..., &sin_angle, &cos_angle);  // sin/cos查表
    hpm_mcl_bldc_foc_park(..., &par->currentdpipar.cur, &par->currentqpipar.cur, ...);  // Park
    par->currentdpipar.func_pid(&par->currentdpipar);    // d轴PI
    par->currentqpipar.func_pid(&par->currentqpipar);    // q轴PI
    hpm_mcl_bldc_foc_inv_park(..., &par->ualpha, &par->ubeta, ...);  // 反Park
    par->pwmpar.target_alpha = par->ualpha;
    par->pwmpar.target_beta = par->ubeta;
    par->pwmpar.func_spwm(&par->pwmpar);                 // SVPWM
}
```

**v1 vs v2 API对比：**

| 维度 | v1 API | v2 API |
|------|--------|--------|
| 调用方式 | 函数指针链（`func_dqsvpwm`） | 统一入口（`hpm_mcl_loop`） |
| 角度获取 | 用户手动设置`electric_angle` | 自动从encoder模块获取 |
| 电流采样 | 用户手动调用`func_sampl` | 自动从analog模块获取 |
| dq解耦 | 无 | 可选使能 |
| 死区补偿 | 无 | 可选使能 |
| 硬件加速 | 不支持 | VSC+CLC+QEO硬件链 |

**工程要点：**
- ADC中断优先级设为1（最高），确保电流环实时性
- `hpm_mcl_encoder_process()`的第二个参数为MCU时钟周期数/PWM频率，用于角度预测的时间基准
- 硬件FOC模式下，`hpm_mcl_loop()`仅设置CLC期望值，Clarke/Park/PI/SVPWM由硬件完成

#### 5.1.3 三阶段对中算法

**原理：** 编码器只能提供相对位置，上电时需要确定转子的绝对电角度。三阶段对中通过逐步减小d轴电流，将转子稳定拉到d轴方向：

1. **阶段1：** 大电流粗对中（d=4A, q=0.6A扰动），快速将转子拉到d轴附近
2. **阶段2：** 中电流精对中（d=1.5A），减小振荡
3. **阶段3：** 小电流稳定（d=1.0A），消除残余振荡

**代码：**

```c
void motor_angle_align(void)
{
    mcl_motor_alignment_cfg_t alignment_cfg;
    alignment_cfg.algorithm = mcl_alignment_algorithm_three_stage;
    alignment_cfg.config.three_stage.stage1.d_current = 4.0f;
    alignment_cfg.config.three_stage.stage1.q_current = 0.6f;
    alignment_cfg.config.three_stage.stage1.delay_ms = 500;
    alignment_cfg.config.three_stage.stage2.d_current = 1.5f;
    alignment_cfg.config.three_stage.stage2.delay_ms = 800;
    alignment_cfg.config.three_stage.stage3.d_current = 1.0f;
    alignment_cfg.config.three_stage.stage3.delay_ms = 400;
    alignment_cfg.config.three_stage.final_delay_ms = 100;
    hpm_mcl_motor_angle_alignment(&motor0.loop, &alignment_cfg);
}
```

**工程要点：**
- q轴扰动电流（0.6A）用于打破对称性，避免转子恰好停在磁极中性线
- 硬件FOC模式下对中电流更大（d=8A），因为硬件链的电流环响应更快
- 对中完成后，编码器零点被校准，后续角度读取即为正确的电角度

---

### 5.2 方波控制示例核心代码 (`bldc_block.c`)

#### 5.2.1 初始化流程

**原理：** 方波控制无需FOC的Clarke/Park变换，初始化更简洁。核心是配置Hall/UVW编码器和六步换相表。

**代码：**

```c
int main(void)
{
    board_init();
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_QEI_CLOCK_SOURCE);
    init_pwm_pins(MOTOR0_BLDCPWM);
    motor_init();
    pwm_init();
    timer_init();
    hpm_mcl_loop_start_block(&motor0.loop);
    hall_irq_enable(BOARD_BLDC_HALL_BASE, HALL_EVENT_PHUPT_FLAG_MASK);
    hpm_mcl_loop_enable(&motor0.loop);
    speed = MOTOR0_SPD;
    user_speed.enable = true;
    user_speed.value = speed * MCL_2PI * MOTOR_POLE_NUM;
    hpm_mcl_loop_set_speed(&motor0.loop, user_speed);
    // ... 串口速度输入循环
}
```

**逐块解析：**

| 步骤 | 函数调用 | 与FOC的差异 |
|------|---------|------------|
| 1 | `board_init()` | 相同 |
| 2 | `motor_init()` | loop.mode = `mcl_mode_block`；无analog模块；encoder需`get_uvw_level`回调 |
| 3 | `pwm_init()` | **独立调用**（非通过drivers回调），6路独立PWM（非互补对） |
| 4 | `hpm_mcl_loop_start_block()` | **方波特有**：读取初始Hall状态，确定初始换相扇区 |
| 5 | `hall_irq_enable()` | 使能Hall传感器UVW跳变中断 |

**motor_init()关键配置：**

```c
void motor_init(void)
{
    motor0.cfg.mcl.physical.motor.hall = phase_120;     // 120° Hall排列
    motor0.cfg.encoder.precision = 6;                    // 6个扇区
    motor0.cfg.encoder.callback.get_uvw_level = encoder_get_uvw_level;  // Hall状态读取回调
    motor0.cfg.drivers.callback.disable_drivers = pwm_disable_channel;   // 单相禁用
    motor0.cfg.drivers.callback.enable_drivers = pwm_enable_channel;     // 单相使能
    motor0.cfg.loop.mode = mcl_mode_block;               // 方波模式
    motor0.cfg.loop.enable_speed_loop = true;

    hpm_mcl_encoder_init(&motor0.encoder, &motor0.cfg.mcl, &motor0.cfg.encoder, &motor0.encoder_iir);
    hpm_mcl_drivers_init(&motor0.drivers, &motor0.cfg.drivers);
    hpm_mcl_control_init(&motor0.control, &motor0.cfg.control);
    hpm_mcl_loop_init(&motor0.loop, &motor0.cfg.loop, &motor0.cfg.mcl,
                    &motor0.encoder, &motor0.analog, &motor0.control, &motor0.drivers, NULL);
    hpm_mcl_detect_init(&motor0.detect, &motor0.cfg.detect, &motor0.loop, &motor0.encoder, &motor0.analog, &motor0.drivers);
}
```

**工程要点：**
- 方波控制不需要`hpm_mcl_analog_init()`，因为无需电流采样
- PWM初始化在`motor_init()`外独立调用，因为方波PWM配置（6路独立）与FOC（3对互补）不同
- 速度设定值需乘以极对数和2π：`speed * MCL_2PI * MOTOR_POLE_NUM`，转换为机械角速度

#### 5.2.2 换相中断回调

**原理：** Hall传感器在每次UVW状态跳变时触发中断，中断服务函数完成：读取新Hall状态 → 计算电角度 → 刷新换相。

**代码（Hall传感器版本）：**

```c
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_HALL_IRQ, isr_hall)
void isr_hall(void)
{
    hall_clear_status(BOARD_BLDC_HALL_BASE, hall_get_status(BOARD_BLDC_HALL_BASE));
    hpm_mcl_uvw_get_theta(BOARD_BLDC_HALL_BASE, NULL, MOTOR_ANGLE_DETA, &encoder_theta);
    hpm_mcl_loop_refresh_block(&motor0.loop);
}
```

**代码（QEIV2-UVW版本）：**

```c
SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_QEIV2_IRQ, isr_qei)
{
    uint32_t status = qeiv2_get_status(BOARD_BLDC_QEIV2_BASE);
    qeiv2_clear_status(BOARD_BLDC_QEIV2_BASE, status);
    hpm_mcl_uvw_get_theta(BOARD_BLDC_QEIV2_BASE, &last_position, MOTOR_ANGLE_DETA, &encoder_theta);
    hpm_mcl_loop_refresh_block(&motor0.loop);
}
```

**逐行解析：**

| 行 | 代码 | 作用 |
|----|------|------|
| 1 | `hall_clear_status()` / `qeiv2_clear_status()` | 清除中断标志 |
| 2 | `hpm_mcl_uvw_get_theta()` | 核心换相函数：读取UVW状态 → 查换相表 → 计算电角度 |
| 3 | `hpm_mcl_loop_refresh_block()` | 刷新方波换相：根据新角度更新PWM导通相 |

**Hall信号处理与换相表的关系：**

`hpm_mcl_uvw_get_theta()` 内部执行以下映射：

$$\theta_{elec} = \text{sector} \times \frac{\pi}{3 \cdot p} + \Delta\theta$$

其中 $\text{sector}$ 由Hall UVW三位编码确定（1~6），$p$ 为极对数，$\Delta\theta$ 为换相超前角（`MOTOR_ANGLE_DETA`）。

`encoder_get_uvw_level()` 函数读取Hall状态：

```c
hpm_mcl_stat_t encoder_get_uvw_level(mcl_encoder_uvw_level_t *level)
{
    uint8_t hall_stat;
    hall_stat = hall_get_current_uvw_stat(BOARD_BLDC_HALL_BASE);
    level->w = ((hall_stat >> 0) & 0x01);
    level->v = ((hall_stat >> 1) & 0x01);
    level->u = ((hall_stat >> 2) & 0x01);
    return mcl_success;
}
```

**六步换相的软件实现流程（`hpm_mcl_block_loop`内部）：**

```
1. speed_pid(target_speed, actual_speed) → duty (0~1)
2. |duty| → PWM占空比
3. sign(duty) → 旋转方向 (motor_dir_forward / motor_dir_back)
4. hpm_mcl_drivers_enable_channel(AH) / disable_channel(AL)  // 根据换相表
5. hpm_mcl_drivers_update_bldc_duty(duty, duty, duty)        // 三相同占空比
```

**工程要点：**
- `MOTOR_ANGLE_DETA` = $\pm\frac{\pi}{3p}$，正负号由`BOARD_BLDC_HALL_DIR_INV`决定，用于匹配电机旋转方向与Hall信号序列
- Hall中断优先级设为1（最高），确保换相及时响应，延迟会导致转矩脉动
- QEIV2-UVW模式通过软件模拟Hall信号，`level_tbl[6] = {5,4,6,2,3,1}` 定义了位置到UVW编码的映射

---

### 5.3 HFI高频注入示例核心代码 (`bldc_hfi.c`)

#### 5.3.1 初始化流程

**原理：** HFI（High Frequency Injection）利用电机凸极效应（$L_d \neq L_q$），在d轴注入高频方波电压，通过检测q轴高频电流响应来估计转子位置。使用v1 API，通过函数指针链组织控制流程。

**代码：**

```c
int main(void)
{
    board_init();
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_QEI_CLOCK_SOURCE);
    init_adc_bldc_pins();
    init_pwm_pins(MOTOR0_BLDCPWM);
    bldc_init_par();
    adc_init();
    pwm_init();
    timer_init();
    set_adcval_middle();
    intc_m_enable_irq_with_priority(BOARD_BLDC_ADC_IRQn, 1);
    hpm_adc_enable_interrupts(&hpm_adc_u, BOARD_BLDC_ADC_TRIG_FLAG);
    motor0.adc_trig_event_callback = &motor0_highspeed_loop;
    board_delay_ms(100);
    motor0.inject_pole_detect.status = true;
    while (1) {
        ;
    }
}
```

**逐块解析：**

| 步骤 | 函数调用 | 作用 |
|------|---------|------|
| 1 | `bldc_init_par()` | **核心配置**：填充`MOTOR0_PARA`结构体，注册所有函数指针 |
| 2 | `set_adcval_middle()` | ADC零点校准（200ms采样取平均） |
| 3 | `motor0.adc_trig_event_callback = &motor0_highspeed_loop` | 注册ADC中断回调 |
| 4 | `motor0.inject_pole_detect.status = true` | 启动N/S极性辨识 |

**bldc_init_par()关键配置：**

```c
void bldc_init_par(void)
{
    BLDC_CONTROL_FOC_PARA *par = &motor0.foc_para;
    par->motorpar.i_lstator_h = 0.00263;
    par->motorpar.i_rstator_ohm = 0.0011;
    par->motorpar.i_samplingper_s = 0.00005;

    par->currentdpipar.i_kp = HPM_MOTOR_MATH_FL_MDF(10);
    par->currentdpipar.i_ki = HPM_MOTOR_MATH_FL_MDF(0.01);
    par->currentdpipar.i_max = HPM_MOTOR_MATH_FL_MDF(PWM_RELOAD * 0.9);
    par->currentdpipar.func_pid = (void(*)(void *))&hpm_mcl_bldc_foc_pi_contrl;

    par->pwmpar.func_spwm = (void(*)(void *))&hpm_mcl_bldc_foc_svpwm;
    par->pwmpar.pwmout.func_set_pwm = (void(*)(void *))&hpm_mcl_bldc_foc_pwmset;

    par->samplcurpar.func_sampl = (void(*)(void *))&hpm_mcl_bldc_foc_current_cal;
    par->func_dqsvpwm = (void (*)(void *, void *, void *, void *))&hpm_mcl_hfi_loop;

    motor0.inject_para.vh = 0.625 * PWM_RELOAD;
    motor0.inject_pll.kp = BOARD_BLDC_HFI_PLL_KP;
    motor0.inject_pll.ki = BOARD_BLDC_HFI_PLL_KI;
    motor0.inject_pll.filter = 0.1;
    motor0.inject_pll.loop_s = 0.00005;

    motor0.inject_pole_detect.current_d_init_val = 80;
    motor0.inject_pole_detect.func = (bool (*)(void *, void *, void *, void *))&hpm_mcl_hfi_pole_detect;
}
```

**v1 API函数指针链：**

```
ADC中断 → motor0_highspeed_loop()
             ├─ motor0_current_loop()
             │    ├─ func_sampl() → hpm_mcl_bldc_foc_current_cal()   // 电流校准
             │    ├─ func_dqsvpwm() → hpm_mcl_hfi_loop()             // HFI+FOC核心
             │    │    ├─ func_sampl()                                 // 电流校准（重复调用）
             │    │    ├─ hpm_mcl_bldc_foc_clarke()                   // Clarke
             │    │    ├─ hpm_mcl_hfi_core()                          // HFI信号提取
             │    │    ├─ hpm_mcl_hfi_pll()                           // PLL位置跟踪
             │    │    ├─ hpm_mcl_bldc_foc_park()                     // Park
             │    │    ├─ func_pid() × 2                              // d/q轴PI
             │    │    ├─ d轴输出 += inject->vh (注入电压)
             │    │    ├─ inject->vh = -inject->vh (方波翻转)
             │    │    ├─ hpm_mcl_bldc_foc_inv_park()                 // 反Park
             │    │    └─ func_spwm() → hpm_mcl_bldc_foc_svpwm()     // SVPWM
             │    └─ func_set_pwm() → hpm_mcl_bldc_foc_pwmset()      // PWM输出
             └─ func_getspd() → hpm_mcl_bldc_foc_al_speed()          // 速度计算
```

**工程要点：**
- `func_dqsvpwm`被替换为`hpm_mcl_hfi_loop`而非标准的`hpm_mcl_bldc_foc_ctrl_dq_to_pwm`，这是HFI注入的入口
- 注入电压`vh = 0.625 × PWM_RELOAD`，每个PWM周期翻转一次，形成方波注入
- PLL参数（kp=10, ki=1）决定了位置跟踪的动态响应，过大会振荡，过小跟踪延迟
- 电流环PI的i_max设为`PWM_RELOAD * 0.9`（非物理量纲），v1 API使用原始PWM计数值作为控制量

#### 5.3.2 HFI注入与位置跟踪

**原理：** HFI核心算法分为三步：

1. **高频信号提取：** 对相邻两个PWM周期的α/β电流做差分，提取高频分量
2. **PLL锁相环跟踪：** 以高频电流误差为输入，通过PI锁相环估计转子位置
3. **N/S极性辨识：** 注入d轴直流电流，比较0°和180°方向的电流响应差异

**HFI核心代码（`hpm_mcl_hfi_loop`）：**

```c
void hpm_mcl_hfi_loop(BLDC_CONTROL_FOC_PARA *par, hpm_hfi_para_t *inject,
        hpm_hfi_pll_para_t *pll, hpm_hfi_pole_detect_para_t *pole)
{
    static HPM_MOTOR_MATH_TYPE sin_angle = 0;
    static HPM_MOTOR_MATH_TYPE cos_angle = 0;

    par->samplcurpar.func_sampl(&par->samplcurpar);
    hpm_mcl_bldc_foc_clarke(par->samplcurpar.cal_u, par->samplcurpar.cal_v, par->samplcurpar.cal_w,
                    &par->ialpha, &par->ibeta);
    hpm_mcl_hfi_core(par->ialpha, par->ibeta, inject);
    hpm_mcl_hfi_pll(inject->alpha, inject->beta, sin_angle, cos_angle, pll);
    par->ialpha = inject->e_alpha;
    par->ibeta = inject->e_beta;
    hpm_mcl_hfi_sin_cos(par->electric_angle, &sin_angle, &cos_angle);
    hpm_mcl_bldc_foc_park(par->ialpha, par->ibeta,
                &par->currentdpipar.cur, &par->currentqpipar.cur,
                sin_angle, cos_angle);
    if (pole->currentd_force != 0) {
        par->currentdpipar.target = pole->currentd_force;
    }
    par->currentdpipar.func_pid(&par->currentdpipar);
    par->currentqpipar.func_pid(&par->currentqpipar);
    par->currentdpipar.outval += inject->vh;
    inject->vh = -inject->vh;
    hpm_mcl_bldc_foc_inv_park(par->currentdpipar.outval, par->currentqpipar.outval,
        &par->ualpha, &par->ubeta, sin_angle, cos_angle);
    par->pwmpar.target_alpha = par->ualpha;
    par->pwmpar.target_beta = par->ubeta;
    par->pwmpar.func_spwm(&par->pwmpar);
}
```

**逐块解析：**

| 代码块 | 作用 | 数学原理 |
|--------|------|---------|
| `hpm_mcl_hfi_core()` | 高频信号提取 | $e_\alpha = \frac{\alpha_k + \alpha_{k-1}}{2}$, $\Delta\alpha = \alpha_k - \alpha_{k-1}$ |
| `hpm_mcl_hfi_pll()` | PLL位置跟踪 | $\theta_{err} = \Delta\alpha \cdot \sin\hat\theta - \Delta\beta \cdot \cos\hat\theta$ → PI → $\hat\theta$ |
| `inject->vh = -inject->vh` | 方波翻转 | 每个PWM周期翻转注入电压极性 |
| `par->currentdpipar.outval += inject->vh` | 电压注入 | d轴PI输出叠加高频方波电压 |

#### 5.3.3 HFI与FOC的切换逻辑

**原理：** HFI仅在零低速有效（反电动势太弱时），中高速需切换到SMC/反电动势观测器。本demo为纯HFI实现，运行速度仅1.1 r/s，未实现切换逻辑。

**定时器中断中的速度环与极性辨识：**

```c
void isr_gptmr(void)
{
    // ... 每10ms执行一次
    motor0.speedloop_para.target = HPM_MOTOR_MATH_FL_MDF(user_setspeed);
    motor0.speedloop_para.cur = motor0.foc_para.speedcalpar.o_speedout_filter;
    if (motor0.inject_pole_detect.func(&motor0.foc_para, &motor0.inject_para,
                                        &motor0.inject_pll, &motor0.inject_pole_detect)) {
        if (motor0.inject_pole_detect.status) {
            motor0.speedloop_para.func_pid(&motor0.speedloop_para);
            motor0.foc_para.currentdpipar.target = 0;
            motor0.foc_para.currentqpipar.target = motor0.speedloop_para.outval;
        }
    } else {
        motor0.foc_para.currentdpipar.target = 0;
        motor0.foc_para.currentqpipar.target = 0;
    }
}
```

**切换逻辑说明：**
- `inject_pole_detect.func()` 返回`true`表示极性辨识完成，可以运行速度环
- 返回`false`表示正在辨识N/S极，此时d/q轴电流参考均设为0（仅靠极性辨识电流维持）
- 实际工程中，HFI→SMC切换需要在速度超过阈值时平滑过渡角度估计值

**工程要点：**
- HFI仅适用于凸极电机（IPM），表贴式电机（SPM）因$L_d \approx L_q$无法使用
- 注入电压幅值`vh`需根据电机电感凸极比调整，过小信噪比不足，过大产生转矩脉动
- PLL滤波系数`filter=0.1`用于平滑位置估计，过大会增加延迟
- 本demo为实验级实现，生产环境需增加HFI→SMC无缝切换逻辑

---

### 5.4 SMC滑模观测器示例核心代码 (`bldc_smc.c`)

#### 5.4.1 初始化流程

**原理：** SMC（Sliding Mode Controller/Observer）通过构建电机数学模型，利用电流误差驱动滑模面，估计反电动势，再通过PLL提取转子位置。适用于中高速（>5%额定转速）无感控制。

**SMC数学模型：**

电机α轴电压方程：

$$u_\alpha = R_s \cdot i_\alpha + L_s \cdot \frac{di_\alpha}{dt} + e_\alpha$$

滑模观测器构造：

$$\hat{u}_\alpha = R_s \cdot \hat{i}_\alpha + L_s \cdot \frac{d\hat{i}_\alpha}{dt} + K_{smc} \cdot \text{sgn}(\hat{i}_\alpha - i_\alpha)$$

当滑模到达条件满足时，开关项 $K_{smc} \cdot \text{sgn}(\hat{i}_\alpha - i_\alpha)$ 的平均值即为反电动势估计 $\hat{e}_\alpha$。

**代码：**

```c
int main(void)
{
    board_init();
    motor_clock_hz = clock_get_frequency(BOARD_BLDC_QEI_CLOCK_SOURCE);
    init_adc_bldc_pins();
    init_pwm_pins(MOTOR0_BLDCPWM);
    bldc_init_par();
    adc_init();
    pwm_init();
    timer_init();
    set_adval_middle();
    intc_m_enable_irq_with_priority(BOARD_BLDC_ADC_IRQn, 1);
    hpm_adc_enable_interrupts(&hpm_adc_u, BOARD_BLDC_ADC_TRIG_FLAG);
    motor0.adc_trig_event_callback = &motor0_highspeed_loop;
    board_delay_ms(1000);
    while (is_being_started) { ; }   // 等待IF拖动完成
    while (!is_being_started) {      // 速度自动循环
        board_delay_ms(200);
        if (is_add) { user_setspeed += 1; } else { user_setspeed -= 1; }
        if (user_setspeed >= 40) is_add = 0;
        if (user_setspeed <= 15) is_add = 1;
    }
}
```

**bldc_init_par()关键配置：**

```c
void bldc_init_par(void)
{
    BLDC_CONTROL_FOC_PARA *par = &motor0.foc_para;
    par->motorpar.func_smc_const = (void(*)(void *))&hpm_mcl_smc_const_cal;
    par->motorpar.func_smc_const(&par->motorpar);  // 预计算SMC常数

    par->func_dqsvpwm = (void (*)(void *, void *, void *, void *))hpm_mcl_smc_loop;

    motor0.smc_para.zero = 0.25;         // 滑模面系数
    motor0.smc_para.ksmc = 160;          // 滑模增益
    motor0.smc_para.filter_coeff = 0.025; // 低通滤波系数
    motor0.smc_para.func_smc = (void(*)(void *))hpm_mcl_smc_pos_cal;

    motor0.smc_pll.kp = 0.314;          // PLL比例增益
    motor0.smc_pll.ki = 0.007;          // PLL积分增益
    motor0.smc_pll.loop_in_sec = 0.00005; // PLL控制周期
    motor0.smc_pll.theta0 = 90;         // PLL初始角度
}
```

**工程要点：**
- `hpm_mcl_smc_const_cal()` 根据电机参数预计算SMC观测器常数，避免运行时浮点运算
- `func_dqsvpwm`被替换为`hpm_mcl_smc_loop`，与HFI类似通过函数指针注入观测器
- SMC参数（zero, ksmc, filter_coeff）需根据电机参数和速度范围仔细整定

#### 5.4.2 SMC位置估计代码

**代码：**

```c
void motor0_current_loop(float angle)
{
    uint8_t smc_en;
    smc_en = !is_being_started;
    motor0.foc_para.samplcurpar.adc_u = GET_ADC_12BIT_VALID_DATA(adc_buff[ADCU_INDEX][BOARD_BLDC_ADC_TRG*4]);
    motor0.foc_para.samplcurpar.adc_v = GET_ADC_12BIT_VALID_DATA(adc_buff[ADCV_INDEX][BOARD_BLDC_ADC_TRG*4]);
    motor0.foc_para.electric_angle = angle;
    motor0.foc_para.func_dqsvpwm(&motor0.foc_para, &motor0.smc_para, &motor0.smc_pll, &smc_en);
    motor0.foc_para.pwmpar.pwmout.func_set_pwm(&motor0.foc_para.pwmpar.pwmout);
}
```

**逐行解析：**

| 行 | 代码 | 作用 |
|----|------|------|
| 1 | `smc_en = !is_being_started` | 拖动阶段禁用SMC，闭环阶段使能 |
| 2 | ADC读取 | 直接从DMA缓冲区读取电流ADC值 |
| 3 | `electric_angle = angle` | 拖动阶段用`drag_angle`，闭环阶段由SMC内部更新 |
| 4 | `func_dqsvpwm()` → `hpm_mcl_smc_loop()` | SMC+FOC核心计算 |
| 5 | `func_set_pwm()` | 更新PWM输出 |

**`hpm_mcl_smc_loop`内部流程：**

```c
void hpm_mcl_smc_loop(BLDC_CONTROL_FOC_PARA *par, hpm_mcl_para_t *smc,
                       hpm_smc_pll_para_t *pll, uint8_t *is_smc_enable)
{
    par->samplcurpar.func_sampl(&par->samplcurpar);        // 电流校准
    hpm_mcl_bldc_foc_clarke(..., &par->ialpha, &par->ibeta); // Clarke
    if (par->pos_estimator_par.func != NULL) {
        par->pos_estimator_par.func(par->pos_estimator_par.par);  // SMC位置估计
        hpm_mcl_smc_pll(smc, pll);                          // PLL锁相环
        if (*is_smc_enable) {
            par->electric_angle = pll->theta;               // 闭环：使用SMC角度
        }
    }
    hpm_mcl_sin_cos(par->electric_angle, &sin_angle, &cos_angle);
    hpm_mcl_bldc_foc_park(...);                              // Park
    par->currentdpipar.func_pid(&par->currentdpipar);       // d轴PI
    par->currentqpipar.func_pid(&par->currentqpipar);       // q轴PI
    hpm_mcl_bldc_foc_inv_park(...);                          // 反Park
    par->pwmpar.func_spwm(&par->pwmpar);                     // SVPWM
}
```

**逐块解析：**

| 代码块 | 作用 | 关键参数 |
|--------|------|---------|
| `hpm_mcl_smc_pos_cal()` | 滑模观测器核心 | `ksmc=160`（滑模增益），`zero=0.25`（边界层厚度） |
| `hpm_mcl_smc_pll()` | PLL从反电动势提取位置 | `kp=0.314`, `ki=0.007` |
| `*is_smc_enable` | SMC使能开关 | 拖动阶段=0（开环），闭环阶段=1 |

#### 5.4.3 中高速无传感器控制的实现流程

**启动流程（定时器中断中实现）：**

```c
void isr_gptmr(void)
{
    // ... 每10ms执行
    // 1. 速度环
    motor0.speedloop_para.target = user_setspeed;
    motor0.speedloop_para.cur = pll_speed_control.o_speedout_filter;
    motor0.speedloop_para.func_pid(&motor0.speedloop_para);
    motor0.foc_para.currentqpipar.target = motor0.speedloop_para.outval;
    motor0.foc_para.currentdpipar.target = 0;

    // 2. 低速保护
    if (fabs(user_setspeed) < 5) {
        user_setspeed = 0;
        disable_all_pwm_output();   // 速度太低，SMC无法工作，停机
    }

    // 3. IF拖动启动
    if ((last_set_speed == 0) && (fabs(user_setspeed) >= 5)) {
        enable_all_pwm_output();
        if (start_times < 1000) {   // 约10秒拖动
            is_being_started = 1;
            motor0.foc_para.currentdpipar.target = 800;  // d轴大电流
            if (user_setspeed > 0) {
                drag_angle = (drag_angle + 10) % 360;    // 正转步进
            } else {
                drag_angle -= 10;                          // 反转步进
            }
            motor0.foc_para.currentqpipar.target = 0;
        } else {
            is_being_started = 0;    // 拖动完成，切换到SMC闭环
        }
    }
}
```

**启动状态机：**

```
┌─────────────┐    速度≥5r/s    ┌──────────────┐    10秒后     ┌──────────────┐
│  停机状态    │ ──────────────→ │  IF拖动启动   │ ────────────→ │  SMC闭环     │
│ (PWM关闭)   │                  │ (开环步进角)  │               │ (PLL角度)    │
└─────────────┘  ←────────────── └──────────────┘               └──────────────┘
     ↑               速度<5r/s
     └───────────────────────────────────────────────────────────────┘
```

**SMC参数整定关键点：**

| 参数 | 变量 | 整定方法 |
|------|------|---------|
| 滑模增益 | `ksmc = 160` | 需大于反电动势最大值，过大会增加抖振，过小无法到达滑模面 |
| 边界层 | `zero = 0.25` | 越小跟踪精度越高但抖振越大，越大越平滑但稳态误差增大 |
| 低通滤波 | `filter_coeff = 0.025` | 滤除滑模抖振的高频分量，过大会增加相位延迟 |
| PLL Kp | `0.314` | 决定位置跟踪带宽，过大会放大噪声 |
| PLL Ki | `0.007` | 消除稳态位置误差，过大会导致振荡 |

**工程要点：**
- SMC最低可靠工作速度约为5 r/s（约5%额定转速），低于此速度反电动势信噪比不足
- IF拖动阶段注入d轴大电流（800 PWM计数值≈25%占空比），强制将转子拉到d轴方向
- `drag_angle`每次增加10°电角度，对应开环加速斜率约10°/10ms = 1000°/s
- 速度自动循环（15→40→15 r/s）用于验证SMC在全速度范围内的跟踪性能

---

### 5.5 示例应用通用架构模式

#### 5.5.1 共同架构模式

所有四个示例共享以下架构模式：

```
┌──────────────────────────────────────────────────────────────────────┐
│                        应用层 (main / motor_init)                     │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐               │
│  │ 参数配置     │  │ 模式选择      │  │ 用户交互      │               │
│  │ (motor.cfg) │  │ (loop.mode)  │  │ (串口输入)    │               │
│  └──────┬──────┘  └──────┬───────┘  └──────┬───────┘               │
│         │                │                  │                        │
├─────────┼────────────────┼──────────────────┼────────────────────────┤
│         ▼                ▼                  ▼                        │
│                     中间件层 (hpm_mcl)                                  │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                    mcl_loop_t (控制环聚合体)                   │    │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │    │
│  │  │ encoder  │ │ analog   │ │ control  │ │ drivers  │      │    │
│  │  │ (角度/速度)│ │(电流采样)│ │(PI/SVPWM)│ │(PWM输出) │      │    │
│  │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘      │    │
│  │       │            │            │            │              │    │
│  ├───────┼────────────┼────────────┼────────────┼──────────────┤    │
│         ▼            ▼            ▼            ▼                    │
│                     板级层 (board / callback)                         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐              │
│  │ QEI/Hall │ │ ADC+DMA  │ │ PWM/PWMV2│ │ GPTMR    │              │
│  │ (硬件驱动)│ │(硬件驱动) │ │(硬件驱动) │ │(硬件驱动) │              │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘              │
└──────────────────────────────────────────────────────────────────────┘
```

#### 5.5.2 board层→middleware层→应用层的调用关系

**v2 API调用关系（bldc_foc / bldc_block）：**

```
应用层                    中间件层                          板级层(callback)
──────                    ──────────                        ────────────────
main()
 ├─ board_init()          → HPM SDK板级初始化
 ├─ motor_init()
 │   ├─ 填充cfg结构体
 │   ├─ hpm_mcl_analog_init()     → 注册 adc_init / adc_value_get
 │   ├─ hpm_mcl_encoder_init()    → 注册 qei_init / encoder_get_theta
 │   ├─ hpm_mcl_drivers_init()    → 注册 pwm_init / pwm_duty_set
 │   ├─ hpm_mcl_control_init()    → 注册 PI/SVPWM方法
 │   ├─ hpm_mcl_loop_init()       → 聚合所有模块
 │   └─ hpm_mcl_detect_init()     → 注册 disable_all_pwm_output
 ├─ hpm_mcl_loop_enable()
 └─ hpm_mcl_loop_set_speed()

ISR:
 ├─ isr_adc()
 │   ├─ hpm_mcl_encoder_process() → encoder.callback.get_theta() → qei_read
 │   └─ hpm_mcl_loop()            → analog.callback.get_value() → adc_read
 │       └─ hpm_mcl_current_foc_loop()
 │           └─ drivers.callback.update_duty_cycle() → pwm_duty_set
 └─ isr_gptmr()
     └─ hpm_mcl_detect_loop()     → detect.callback.disable_output()
```

**v1 API调用关系（bldc_hfi / bldc_smc）：**

```
应用层                    中间件层                          板级层(callback)
──────                    ──────────                        ────────────────
main()
 ├─ board_init()
 ├─ bldc_init_par()
 │   ├─ 填充MOTOR0_PARA结构体
 │   ├─ 注册 func_pid → hpm_mcl_bldc_foc_pi_contrl
 │   ├─ 注册 func_spwm → hpm_mcl_bldc_foc_svpwm
 │   ├─ 注册 func_set_pwm → hpm_mcl_bldc_foc_pwmset
 │   ├─ 注册 func_sampl → hpm_mcl_bldc_foc_current_cal
 │   ├─ 注册 func_dqsvpwm → hpm_mcl_hfi_loop / hpm_mcl_smc_loop
 │   └─ 注册 adc_trig_event_callback → motor0_highspeed_loop
 └─ set_adcval_middle()

ISR:
 ├─ isr_adc()
 │   └─ motor0.adc_trig_event_callback()
 │       └─ motor0_highspeed_loop()
 │           └─ motor0_current_loop()
 │               ├─ func_sampl()     → hpm_mcl_bldc_foc_current_cal
 │               ├─ func_dqsvpwm()   → hpm_mcl_hfi_loop / hpm_mcl_smc_loop
 │               └─ func_set_pwm()   → hpm_mcl_bldc_foc_pwmset
 └─ isr_gptmr()
     └─ 速度环 / 极性辨识 / 拖动逻辑
```

#### 5.5.3 中断回调机制

**双中断架构：**

所有示例均采用"ADC中断（电流环）+ 定时器中断（速度环/检测）"的双中断架构：

| 中断源 | 频率 | 优先级 | 执行内容 | 最大执行时间要求 |
|--------|------|--------|---------|----------------|
| ADC ISR | 20kHz (PWM频率) | 1（最高） | 电流环（Clarke/Park/PI/InvPark/SVPWM） | < 50μs (1个PWM周期) |
| GPTMR ISR | 100Hz~1kHz | 1 | 速度环/故障检测/极性辨识 | < 1ms |

**v2 API回调注册方式：**

```c
// 通过cfg结构体的callback字段注册
motor0.cfg.analog.callback.get_value = adc_value_get;      // 电流读取
motor0.cfg.encoder.callback.get_theta = encoder_get_theta;  // 角度读取
motor0.cfg.drivers.callback.update_duty_cycle = pwm_duty_set; // PWM更新
motor0.cfg.detect.callback.disable_output = disable_all_pwm_output; // 故障保护
```

**v1 API回调注册方式：**

```c
// 通过函数指针字段注册
par->currentdpipar.func_pid = &hpm_mcl_bldc_foc_pi_contrl;
par->pwmpar.func_spwm = &hpm_mcl_bldc_foc_svpwm;
par->pwmpar.pwmout.func_set_pwm = &hpm_mcl_bldc_foc_pwmset;
par->samplcurpar.func_sampl = &hpm_mcl_bldc_foc_current_cal;
motor0.adc_trig_event_callback = &motor0_highspeed_loop;
```

**v2 vs v1 回调机制对比：**

| 维度 | v2 API | v1 API |
|------|--------|--------|
| 注册方式 | cfg结构体callback字段 | 函数指针直接赋值 |
| 中断入口 | `hpm_mcl_loop()`统一分发 | 用户自定义回调函数 |
| 模块解耦 | 高（各模块独立初始化） | 低（全局结构体耦合） |
| 可移植性 | 高（板级代码通过callback隔离） | 中（需适配BLDC_CONTROL_FOC_PARA结构体） |
| 扩展性 | 高（新增loop_mode即可） | 中（需修改func_dqsvpwm指向） |

**工程要点：**
- ADC ISR必须在一个PWM周期内完成，否则会丢失采样点导致电流环不稳定
- v1 API中`motor0_highspeed_loop`是用户自定义的ADC回调入口，v2 API中由`hpm_mcl_loop`统一处理
- 故障检测（`hpm_mcl_detect_loop`）在定时器中断中运行，检测到异常后调用`disable_all_pwm_output`紧急关断
- 所有ISR共享`motor0`全局结构体，无需加锁（单核CPU，中断嵌套由优先级管理）