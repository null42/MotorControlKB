from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
import os
import re
from pathlib import Path
from typing import List, Dict, Optional

router = APIRouter()

BASE_DIR = Path(__file__).resolve().parent.parent.parent.parent.parent
DOCS_DIR = BASE_DIR / "motor-control-knowledge-base"

HW_ORDER = [
    "HW-01-Motor-Basics",
    "HW-02-Current-Sensing",
    "HW-03-Position-Sensor",
    "HW-04-MCU-Peripherals",
    "HW-05-Power-Devices-Gate-Drivers",
    "HW-06-Power-Management-Protection",
    "HW-07-Thermal-EMC-Design",
]

ALG_ORDER = [
    "ALG-01-FOC-Theory",
    "ALG-02-Current-Sampling-Timing",
    "ALG-03-PI-Current-Regulator",
    "ALG-04-Deadtime-Compensation",
    "ALG-05-Sensored-FOC",
    "ALG-06-Position-Speed-Observer",
    "ALG-07-Sensorless-Observers",
    "ALG-08-Initial-Position-Detection",
    "ALG-09-High-Frequency-Injection",
    "ALG-10-Overmodulation",
    "ALG-11-MTPA-Flux-Weakening",
    "ALG-12-Speed-Loop-Torque-Observer",
    "ALG-13-Protection-Optimization",
    "ALG-14-THD-Harmonic-Analysis",
    "ALG-15-Advanced-Research",
]

MCLIB_ORDER = [
    "MC-LIB-Architecture",
    "MC-LIB-FOC-Core",
    "MC-LIB-SVPWM",
    "MC-LIB-Control-Loop",
    "MC-LIB-Observer",
    "MC-LIB-Six-Step",
    "MC-LIB-Porting-Guide",
]

HW_TITLES = {
    "HW-01-Motor-Basics": "HW-01 电机本体基础",
    "HW-02-Current-Sensing": "HW-02 电流采样电路",
    "HW-03-Position-Sensor": "HW-03 位置传感器接口",
    "HW-04-MCU-Peripherals": "HW-04 MCU外设与通信",
    "HW-05-Power-Devices-Gate-Drivers": "HW-05 功率器件与栅极驱动",
    "HW-06-Power-Management-Protection": "HW-06 电源管理与保护",
    "HW-07-Thermal-EMC-Design": "HW-07 热设计与EMC",
}

ALG_TITLES = {
    "ALG-01-FOC-Theory": "ALG-01 FOC理论基础（含Clark-Park变换）",
    "ALG-02-Current-Sampling-Timing": "ALG-02 ADC 电流采样时序",
    "ALG-03-PI-Current-Regulator": "ALG-03 PI 电流调节器设计",
    "ALG-04-Deadtime-Compensation": "ALG-04 死区补偿策略",
    "ALG-05-Sensored-FOC": "ALG-05 有感FOC实现",
    "ALG-06-Position-Speed-Observer": "ALG-06 位置与速度观测器",
    "ALG-07-Sensorless-Observers": "ALG-07 无感FOC观测器",
    "ALG-08-Initial-Position-Detection": "ALG-08 启动定位与预定位",
    "ALG-09-High-Frequency-Injection": "ALG-09 高频注入法",
    "ALG-10-Overmodulation": "ALG-10 过调制与六阶梯波",
    "ALG-11-MTPA-Flux-Weakening": "ALG-11 MTPA 与弱磁控制",
    "ALG-12-Speed-Loop-Torque-Observer": "ALG-12 速度环与负载转矩观测器",
    "ALG-13-Protection-Optimization": "ALG-13 保护与优化",
    "ALG-14-THD-Harmonic-Analysis": "ALG-14 THD 谐波分析",
    "ALG-15-Advanced-Research": "ALG-15 前沿研究",
}

MCLIB_TITLES = {
    "MC-LIB-Architecture": "架构总览",
    "MC-LIB-FOC-Core": "FOC核心模块",
    "MC-LIB-SVPWM": "SVPWM模块",
    "MC-LIB-Control-Loop": "控制环模块",
    "MC-LIB-Observer": "观测器模块",
    "MC-LIB-Six-Step": "六步换相模块",
    "MC-LIB-Porting-Guide": "移植使用指南",
}

HPMC_ORDER = [
    "SDK-01-HPM-MC-Architecture",
    "SDK-02-HPM-MC-v2-Core-Loop",
    "SDK-03-HPM-MC-v2-Detect",
    "SDK-04-HPM-MC-v2-Hybrid-Ctrl",
    "SDK-05-HPM-MC-v2-Path-Plan",
    "SDK-06-HPM-MC-Sample-Apps",
]

HPMC_TITLES = {
    "SDK-01-HPM-MC-Architecture": "架构总览 (v1/v2对比)",
    "SDK-02-HPM-MC-v2-Core-Loop": "v2 主循环与控制调度",
    "SDK-03-HPM-MC-v2-Detect": "离线参数检测",
    "SDK-04-HPM-MC-v2-Hybrid-Ctrl": "力位混合控制",
    "SDK-05-HPM-MC-v2-Path-Plan": "路径规划",
    "SDK-06-HPM-MC-Sample-Apps": "9个示例应用分析",
}

ADV_HW_ORDER = [
    "ADV-HW-01-PWM-Current-Sampling",
    "ADV-HW-02-ADC-DMA",
    "ADV-HW-03-Encoder-Speed",
]

ADV_ALG_ORDER = [
    "ADV-ALG-01-Bandwidth-Filter",
    "ADV-ALG-05-Field-Weakening-MTPA",
    "ADV-ALG-07-Feedforward-Decoupling",
    "ADV-ALG-09-Per-Unit-Fixed-Point",
    "ADV-ALG-13-PID-Structure-Tuning",
    "ADV-ALG-15-Debug-Methodology",
]

SYS_ORDER = [
    "SYS-01-Design-Patterns",
    "SYS-02-Inverter-vs-Motor-Control",
    "SYS-03-PFC-vs-Motor-Control",
    "SYS-04-Simulation-to-Discrete",
]

ADV_HW_TITLES = {
    "ADV-HW-01-PWM-Current-Sampling": "ADV-HW-01 PWM深度配置与电流采样时序联动",
    "ADV-HW-02-ADC-DMA": "ADV-HW-02 ADC深度配置与DMA数据搬运",
    "ADV-HW-03-Encoder-Speed": "ADV-HW-03 编码器深度处理与测速方法",
}

ADV_ALG_TITLES = {
    "ADV-ALG-01-Bandwidth-Filter": "ADV-ALG-01 控制环带宽设计与滤波器工程",
    "ADV-ALG-05-Field-Weakening-MTPA": "ADV-ALG-05 弱磁控制与MTPA深度",
    "ADV-ALG-07-Feedforward-Decoupling": "ADV-ALG-07 前馈解耦与扰动补偿",
    "ADV-ALG-09-Per-Unit-Fixed-Point": "ADV-ALG-09 标幺值系统与定点数运算",
    "ADV-ALG-13-PID-Structure-Tuning": "ADV-ALG-13 PID结构选择与深度整定",
    "ADV-ALG-15-Debug-Methodology": "ADV-ALG-15 问题定位与调试方法论",
}

SYS_TITLES = {
    "SYS-01-Design-Patterns": "SYS-01 设计模式在电机控制中的应用",
    "SYS-02-Inverter-vs-Motor-Control": "SYS-02 变频器开发与电控开发异同",
    "SYS-03-PFC-vs-Motor-Control": "SYS-03 PFC开发与电控开发异同",
    "SYS-04-Simulation-to-Discrete": "SYS-04 仿真到实现——连续域到离散域",
}

EE_ORDER = [
    "EE-01-Resistance-Capacitance-Inductance-Basics",
    "EE-02-Diodes-And-Rectification",
    "EE-03-BJT-Basics",
    "EE-04-MOSFET-Principles",
    "EE-05-MOSFET-Gate-Drive",
    "EE-06-IGBT-Principles",
    "EE-07-OpAmp",
    "EE-08-Rectifier-Inverter",
    "EE-09-H-Bridge",
]

EE_TITLES = {
    "EE-01-Resistance-Capacitance-Inductance-Basics": "EE-01 电阻、电容、电感基础",
    "EE-02-Diodes-And-Rectification": "EE-02 二极管与整流电路",
    "EE-03-BJT-Basics": "EE-03 三极管（BJT）基础",
    "EE-04-MOSFET-Principles": "EE-04 MOSFET 原理与特性",
    "EE-05-MOSFET-Gate-Drive": "EE-05 MOSFET 驱动与保护",
    "EE-06-IGBT-Principles": "EE-06 IGBT 原理与选型",
    "EE-07-OpAmp": "EE-07 运算放大器",
    "EE-08-Rectifier-Inverter": "EE-08 整流与逆变拓扑",
    "EE-09-H-Bridge": "EE-09 H桥与电机驱动拓扑",
}

CT_ORDER = [
    "CT-01-Open-Loop-Closed-Loop",
    "CT-02-Time-Domain-Analysis",
    "CT-03-Frequency-Response-Bode",
    "CT-04-PID-Control-Principles",
    "CT-05-PID-Tuning-Implementation",
    "CT-06-Feedforward-Control",
    "CT-07-Nyquist-Stability",
    "CT-08-Root-Locus",
    "CT-09-Compensator-Design",
    "CT-10-State-Space",
    "CT-11-Observer-Design",
    "CT-12-State-Feedback",
    "CT-13-LQR-LQG",
]

CT_TITLES = {
    "CT-01-Open-Loop-Closed-Loop": "CT-01 开环与闭环控制系统",
    "CT-02-Time-Domain-Analysis": "CT-02 时域响应分析",
    "CT-03-Frequency-Response-Bode": "CT-03 频率响应与伯德图",
    "CT-04-PID-Control-Principles": "CT-04 PID 控制原理",
    "CT-05-PID-Tuning-Implementation": "CT-05 PID 整定与工程实现",
    "CT-06-Feedforward-Control": "CT-06 前馈控制",
    "CT-07-Nyquist-Stability": "CT-07 奈奎斯特稳定判据",
    "CT-08-Root-Locus": "CT-08 根轨迹分析",
    "CT-09-Compensator-Design": "CT-09 系统校正设计",
    "CT-10-State-Space": "CT-10 状态空间方程",
    "CT-11-Observer-Design": "CT-11 观测器设计",
    "CT-12-State-Feedback": "CT-12 状态反馈与极点配置",
    "CT-13-LQR-LQG": "CT-13 最优控制（LQR/LQG）",
}


def get_simulations() -> List[Dict]:
    return [
        {"id": "sim-rlc", "name": "RLC 充放电", "simName": "rlc", "type": "vue", "docId": "electronics-basics/EE-01-Resistance-Capacitance-Inductance-Basics"},
        {"id": "sim-mosfet-switching", "name": "MOSFET 开关波形", "simName": "mosfet-switching", "type": "vue", "docId": "electronics-basics/EE-05-MOSFET-Gate-Drive"},
        {"id": "sim-svpwm", "name": "SVPWM 矢量合成", "simName": "svpwm", "type": "vue", "docId": "electronics-basics/EE-08-Rectifier-Inverter"},

        {"id": "sim-bode-plot", "name": "Bode 图交互仿真", "simName": "bode-plot", "type": "vue", "docId": "control-theory/CT-03-Frequency-Response-Bode"},
        {"id": "sim-pid-tuner", "name": "PID 参数调节", "simName": "pid-tuner", "type": "vue", "docId": "control-theory/CT-04-PID-Control-Principles"},
        {"id": "sim-oscillator", "name": "弹簧振子仿真", "simName": "oscillator-sim", "type": "html", "url": "/sims/oscillator_sim.html", "docId": "control-theory/CT-05-PID-Tuning-Implementation"},

        {"id": "sim-park-transform", "name": "Park 变换可视化", "simName": "park-transform", "type": "vue", "docId": "algorithm/ALG-01-FOC-Theory"},
        {"id": "sim-clarke-park", "name": "Clarke & Park 坐标变换", "simName": "clarke-park", "type": "vue", "docId": "algorithm/ALG-01-FOC-Theory"},
        {"id": "sim-foc-sim", "name": "FOC 电机仿真", "simName": "foc-sim", "type": "html", "url": "/sims/foc_sim.html", "docId": "algorithm/ALG-01-FOC-Theory"},
        {"id": "sim-sampling-timing", "name": "ADC 电流采样时序", "simName": "sampling-timing", "type": "vue", "docId": "algorithm/ALG-02-Current-Sampling-Timing"},
        {"id": "sim-current-loop", "name": "dq 电流环 PI 调节器", "simName": "current-loop", "type": "vue", "docId": "algorithm/ALG-03-PI-Current-Regulator"},
        {"id": "sim-deadtime-comp", "name": "死区补偿策略", "simName": "deadtime-comp", "type": "vue", "docId": "algorithm/ALG-04-Deadtime-Compensation"},
        {"id": "sim-observer", "name": "位置/速度观测器", "simName": "observer", "type": "vue", "docId": "algorithm/ALG-06-Position-Speed-Observer"},
        {"id": "sim-initial-position", "name": "启动定位与预定位", "simName": "initial-position", "type": "vue", "docId": "algorithm/ALG-08-Initial-Position-Detection"},
        {"id": "sim-overmodulation", "name": "过调制与六阶梯波", "simName": "overmodulation", "type": "vue", "docId": "algorithm/ALG-10-Overmodulation"},
        {"id": "sim-mtpa", "name": "MTPA 与弱磁控制", "simName": "mtpa", "type": "vue", "docId": "algorithm/ALG-11-MTPA-Flux-Weakening"},
        {"id": "sim-speed-loop", "name": "速度环与转矩观测器", "simName": "speed-loop", "type": "vue", "docId": "algorithm/ALG-12-Speed-Loop-Torque-Observer"},
        {"id": "sim-thd", "name": "THD 谐波分析", "simName": "thd", "type": "vue", "docId": "algorithm/ALG-14-THD-Harmonic-Analysis"},

        {"id": "sim-emacheny", "name": "Emacheny SPMSM 辨识", "simName": "emacheny", "type": "vue", "docId": "advanced/system-methodology/SYS-04-Simulation-to-Discrete"},

        {"id": "sim-serial-assistant", "name": "🔌 串口助手", "simName": "serial-assistant", "type": "html", "url": "/sims/serial_assistant.html", "docId": "algorithm/ALG-01-FOC-Theory"},
        {"id": "sim-comm-sim", "name": "📡 CAN通信协议仿真", "simName": "comm-sim", "type": "html", "url": "/sims/comm_sim.html", "docId": "communication/COM-01-CAN-Basics"},
    ]

PP_ORDER = [
    "PP-01-DCDC-Fundamental-Topologies",
    "PP-02-Isolated-DCDC-Flyback-Forward-PushPull",
    "PP-03-LLC-Resonant-Converter",
    "PP-04-PFC-Power-Factor-Correction",
    "PP-05-Magnetic-Design",
    "PP-06-Loop-Compensation",
    "PP-07-Power-Conversion-Motor-Drive",
]

PP_TITLES = {
    "PP-01-DCDC-Fundamental-Topologies": "PP-01 DC-DC 基础拓扑（Buck/Boost/Buck-Boost）",
    "PP-02-Isolated-DCDC-Flyback-Forward-PushPull": "PP-02 隔离型 DC-DC（Flyback/Forward/Push-Pull）",
    "PP-03-LLC-Resonant-Converter": "PP-03 半桥与全桥 LLC 谐振变换器",
    "PP-04-PFC-Power-Factor-Correction": "PP-04 功率因数校正（PFC）",
    "PP-05-Magnetic-Design": "PP-05 磁元件设计（电感/变压器）",
    "PP-06-Loop-Compensation": "PP-06 开关电源环路补偿",
    "PP-07-Power-Conversion-Motor-Drive": "PP-07 功率变换在电机驱动中的应用",
}

COM_ORDER = [
    "COM-01-CAN-Basics",
    "COM-02-CAN-FD",
    "COM-03-CAN-STM32",
    "COM-04-CAN-Simulation",
    "COM-05-Modbus",
    "COM-06-EtherCAT",
    "COM-07-Protocol-Compare",
]

COM_TITLES = {
    "COM-01-CAN-Basics": "COM-01 CAN总线基础",
    "COM-02-CAN-FD": "COM-02 CAN FD扩展",
    "COM-03-CAN-STM32": "COM-03 STM32 FDCAN实现",
    "COM-04-CAN-Simulation": "COM-04 CAN通信仿真",
    "COM-05-Modbus": "COM-05 Modbus协议",
    "COM-06-EtherCAT": "COM-06 EtherCAT协议",
    "COM-07-Protocol-Compare": "COM-07 协议选型对比",
}

MEC_ORDER = [
    "MEC-01-Planetary-Reducer",
    "MEC-02-Cycloidal-Reducer",
    "MEC-03-Multi-Turn-Encoder",
]

MEC_TITLES = {
    "MEC-01-Planetary-Reducer": "MEC-01 行星减速器",
    "MEC-02-Cycloidal-Reducer": "MEC-02 摆线减速器",
    "MEC-03-Multi-Turn-Encoder": "MEC-03 多圈编码器",
}

ODRIVE_ORDER = [
    "OD-01-Architecture",
    "OD-02-FOC-Core",
    "OD-03-Control-Strategy",
    "OD-04-Encoder-Sensorless",
]

ODRIVE_TITLES = {
    "OD-01-Architecture": "OD-01 系统架构与FreeRTOS线程模型",
    "OD-02-FOC-Core": "OD-02 FOC核心算法链（Clarke/Park/SVM/PI）",
    "OD-03-Control-Strategy": "OD-03 三环级联控制与输入模式",
    "OD-04-Encoder-Sensorless": "OD-04 编码器接口与无感观测器",
}

VESC_ORDER = [
    "VC-01-Architecture",
    "VC-02-FOC-Core",
    "VC-03-Observer-Sensorless",
    "VC-04-System-Protection",
]

VESC_TITLES = {
    "VC-01-Architecture": "VC-01 系统架构与ChibiOS线程模型",
    "VC-02-FOC-Core": "VC-02 FOC完整算法链与电流控制",
    "VC-03-Observer-Sensorless": "VC-03 七种观测器与HFI高频注入",
    "VC-04-System-Protection": "VC-04 多维度限流与故障保护系统",
}


def _build_module_entry(base_prefix: str, module_id: str, title: str, has_assessment: bool = True) -> Dict:
    short_id = re.match(r'(HW-\d+|ALG-\d+|ADV-HW-\d+|ADV-ALG-\d+|SYS-\d+|EE-\d+|CT-\d+|PP-\d+|COM-\d+|MEC-\d+)', module_id)
    assessment_id = short_id.group(1) if short_id else module_id
    assessment_rel = f"{base_prefix}/{assessment_id}-assessment.md"
    assessment_path = DOCS_DIR / assessment_rel
    if has_assessment and not assessment_path.exists():
        has_assessment = False
    entry = {
        "id": f"{base_prefix}/{module_id}",
        "title": title,
        "path": f"{base_prefix}/{module_id}.md",
        "assessment": assessment_rel if has_assessment else None,
    }
    return entry


def parse_knowledge_base() -> Dict:
    hardware_modules = []
    for mod_id in HW_ORDER:
        title = HW_TITLES.get(mod_id, mod_id)
        hardware_modules.append(_build_module_entry("hardware", mod_id, title))

    algorithm_modules = []
    for mod_id in ALG_ORDER:
        title = ALG_TITLES.get(mod_id, mod_id)
        algorithm_modules.append(_build_module_entry("algorithm", mod_id, title))

    mclib_modules = []
    for mod_id in MCLIB_ORDER:
        title = MCLIB_TITLES.get(mod_id, mod_id)
        mclib_modules.append({
            "id": f"algorithm/MC-LIB/{mod_id}",
            "title": title,
            "path": f"algorithm/MC-LIB/{mod_id}.md",
            "assessment": None,
        })

    hpmc_modules = []
    for mod_id in HPMC_ORDER:
        title = HPMC_TITLES.get(mod_id, mod_id)
        hpmc_modules.append({
            "id": f"algorithm/HPM-MC/{mod_id}",
            "title": title,
            "path": f"algorithm/HPM-MC/{mod_id}.md",
            "assessment": None,
        })

    adv_hw_modules = []
    for mod_id in ADV_HW_ORDER:
        title = ADV_HW_TITLES.get(mod_id, mod_id)
        adv_hw_modules.append(_build_module_entry("advanced/hardware-algorithm-bridge", mod_id, title))

    adv_alg_modules = []
    for mod_id in ADV_ALG_ORDER:
        title = ADV_ALG_TITLES.get(mod_id, mod_id)
        adv_alg_modules.append(_build_module_entry("advanced/algorithm", mod_id, title))

    sys_modules = []
    for mod_id in SYS_ORDER:
        title = SYS_TITLES.get(mod_id, mod_id)
        sys_modules.append(_build_module_entry("advanced/system-methodology", mod_id, title))

    ee_modules = []
    for mod_id in EE_ORDER:
        title = EE_TITLES.get(mod_id, mod_id)
        ee_modules.append(_build_module_entry("electronics-basics", mod_id, title))

    ct_modules = []
    for mod_id in CT_ORDER:
        title = CT_TITLES.get(mod_id, mod_id)
        ct_modules.append(_build_module_entry("control-theory", mod_id, title))

    pp_modules = []
    for mod_id in PP_ORDER:
        title = PP_TITLES.get(mod_id, mod_id)
        pp_modules.append(_build_module_entry("power-path", mod_id, title))

    com_modules = []
    for mod_id in COM_ORDER:
        title = COM_TITLES.get(mod_id, mod_id)
        com_modules.append(_build_module_entry("communication", mod_id, title))

    mec_modules = []
    for mod_id in MEC_ORDER:
        title = MEC_TITLES.get(mod_id, mod_id)
        mec_modules.append(_build_module_entry("mechanical", mod_id, title))

    odrive_modules = []
    for mod_id in ODRIVE_ORDER:
        title = ODRIVE_TITLES.get(mod_id, mod_id)
        odrive_modules.append({
            "id": f"ODrive/{mod_id}",
            "title": title,
            "path": f"ODrive/{mod_id}.md",
            "assessment": None,
        })

    vesc_modules = []
    for mod_id in VESC_ORDER:
        title = VESC_TITLES.get(mod_id, mod_id)
        vesc_modules.append({
            "id": f"VESC/{mod_id}",
            "title": title,
            "path": f"VESC/{mod_id}.md",
            "assessment": None,
        })

    comparison_entry = {
        "id": "COMPARISON/COMP-01-ODrive-vs-VESC",
        "title": "🔄 ODrive vs VESC 全面对比分析",
        "path": "COMPARISON/COMP-01-ODrive-vs-VESC.md",
        "assessment": None,
    }

    cross_ref = {
        "id": "cross-reference/cross-reference-map",
        "title": "🔗 交叉引用映射表",
        "path": "cross-reference/cross-reference-map.md",
        "assessment": None,
    }

    return {
        "simulations": get_simulations(),
        "hardware": {
            "id": "hardware",
            "title": "🔧 电控硬件路径",
            "description": "从硬件底层理解电控系统，建立「硬件决定算法边界」的认知",
            "modules": hardware_modules,
        },
        "algorithm": {
            "id": "algorithm",
            "title": "🧮 算法路径",
            "description": "从控制算法理解电控系统，建立「算法受硬件约束」的认知",
            "modules": algorithm_modules,
        },
        "mclib": {
            "id": "mclib",
            "title": "💻 MC_LIB代码实践",
            "description": "算法路径配套MC_LIB电机控制库的代码实践文档",
            "modules": mclib_modules,
        },
        "hpmc": {
            "id": "hpmc",
            "title": "🏭 hpm_MC 代码分析",
            "description": "HPMicro 先楫半导体官方电机控制SDK深度代码分析（hpm_mcl v1 + v2）",
            "modules": hpmc_modules,
        },
        "advHwBridge": {
            "id": "advHwBridge",
            "title": "⚡ 硬件-算法桥梁（高级）",
            "description": "外设配置与算法实现的联动关系，从「怎么配」到「为什么这样配」",
            "modules": adv_hw_modules,
        },
        "advAlgorithm": {
            "id": "advAlgorithm",
            "title": "🔬 高级算法（高级）",
            "description": "控制性能的极限优化：带宽、弱磁、前馈、标幺值、PID深度、调试",
            "modules": adv_alg_modules,
        },
        "sysMethodology": {
            "id": "sysMethodology",
            "title": "🏗️ 系统与方法论（高级）",
            "description": "软件架构、跨领域迁移、仿真验证的系统级方法论",
            "modules": sys_modules,
        },
        "electronicsBasics": {
            "id": "electronicsBasics",
            "title": "🔌 硬件基础路径",
            "description": "从RLC基础到H桥驱动——建立电机控制工程师必备的电力电子硬件基础",
            "modules": ee_modules,
        },
        "controlTheory": {
            "id": "controlTheory",
            "title": "📐 控制理论路径",
            "description": "从经典控制到现代控制——建立电机控制工程师的自动控制理论基础",
            "modules": ct_modules,
        },
        "powerPath": {
            "id": "powerPath",
            "title": "🔋 功率变换路径",
            "description": "从DC-DC到LLC到磁元件设计——建立电机驱动工程师的功率变换基础",
            "modules": pp_modules,
        },
        "communication": {
            "id": "communication",
            "title": "📡 工业通信路径",
            "description": "CAN/CAN FD/EtherCAT/Modbus等工业通信协议原理、实现与选型",
            "modules": com_modules,
        },
        "mechanical": {
            "id": "mechanical",
            "title": "⚙️ 传动与编码器路径",
            "description": "行星/摆线减速器原理与选型、多圈绝对值编码器原理与接口",
            "modules": mec_modules,
        },
        "odrive": {
            "id": "odrive",
            "title": "🎯 ODrive 代码分析",
            "description": "ODrive 开源伺服驱动器固件深度分析：FreeRTOS架构、FOC算法链、三环级联控制、编码器与无感观测器",
            "modules": odrive_modules,
        },
        "vesc": {
            "id": "vesc",
            "title": "⚡ VESC 代码分析",
            "description": "VESC 开源电机控制器固件深度分析：ChibiOS架构、七种无感观测器、HFI高频注入、多维度限流保护系统",
            "modules": vesc_modules,
        },
        "comparison": {
            "id": "comparison",
            "title": "🔄 ODrive vs VESC 对比分析",
            "description": "两大开源电机控制固件的全面对比：架构差异、FOC实现、无感控制、保护机制、编码器支持、通信能力",
            "modules": [comparison_entry],
        },
        "crossReference": cross_ref,
        "referenceAppendix": {
            "id": "referenceAppendix",
            "title": "📚 参考附录",
            "description": "控制理论、电子基础、电源路径等交叉参考知识",
            "subSections": [
                {
                    "id": "electronicsBasics",
                    "title": "🔌 硬件基础路径",
                    "description": "从RLC基础到H桥驱动——建立电机控制工程师必备的电力电子硬件基础",
                    "modules": ee_modules,
                },
                {
                    "id": "controlTheory",
                    "title": "📐 控制理论路径",
                    "description": "从经典控制到现代控制——建立电机控制工程师的自动控制理论基础",
                    "modules": ct_modules,
                },
                {
                    "id": "powerPath",
                    "title": "🔋 功率变换路径",
                    "description": "从DC-DC到LLC到磁元件设计——建立电机驱动工程师的功率变换基础",
                    "modules": pp_modules,
                },
            ],
        },
        "sectionsOrder": [
            "hardware",
            "algorithm",
            "mclib",
            "hpmc",
            "advHwBridge",
            "advAlgorithm",
            "sysMethodology",
            "referenceAppendix",
            "communication",
            "mechanical",
            "odrive",
            "vesc",
            "comparison",
        ],
    }


@router.get("/list")
async def list_docs():
    return parse_knowledge_base()


@router.get("/file/{file_path:path}")
async def get_doc(file_path: str):
    full_path = DOCS_DIR / file_path
    if not full_path.exists():
        raise HTTPException(status_code=404, detail="Document not found")
    resolved = full_path.resolve()
    base_resolved = DOCS_DIR.resolve()
    relative = os.path.relpath(str(resolved), str(base_resolved))
    if relative.startswith("..") or os.path.isabs(relative):
        raise HTTPException(status_code=403, detail="Access denied")
    return FileResponse(full_path, media_type="text/markdown")


@router.get("/search/{keyword}")
async def search_docs(keyword: str):
    results = []
    if not DOCS_DIR.exists():
        return {"results": []}

    for file in DOCS_DIR.rglob("*.md"):
        try:
            content = file.read_text(encoding="utf-8")
            if keyword.lower() in content.lower():
                rel_path = file.relative_to(DOCS_DIR)
                lines = content.split("\n")
                matches = []
                for i, line in enumerate(lines):
                    if keyword.lower() in line.lower():
                        start = max(0, i - 2)
                        end = min(len(lines), i + 3)
                        context = "\n".join(lines[start:end])
                        matches.append({
                            "line": i + 1,
                            "context": context[:200]
                        })
                        if len(matches) >= 3:
                            break
                results.append({
                    "filename": str(rel_path).replace("\\", "/"),
                    "matches": matches
                })
        except Exception:
            continue

    return {"results": results}
