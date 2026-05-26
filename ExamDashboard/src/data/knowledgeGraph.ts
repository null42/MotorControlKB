import { KnowledgeNode, KnowledgeEdge } from "@/lib/types";

// ============================================================================
// 节点定义：算法路径 (18) + 硬件路径 (14) + 通信路径 (6) + 传动路径 (3) = 共 41 个知识点节点
// ============================================================================

export const knowledgeNodes: KnowledgeNode[] = [
  // ────────────────── 算法路径：FOC理论基础 ──────────────────
  {
    id: "alg-park",
    label: "Clarke/Park变换",
    path: "algorithm",
    subject: "foc-theory",
    relatedQuestions: [1, 2],
  },
  {
    id: "alg-svpwm",
    label: "SVPWM调制",
    path: "algorithm",
    subject: "foc-theory",
    relatedQuestions: [3, 4, 10],
  },
  {
    id: "alg-dq",
    label: "dq轴解耦控制",
    path: "algorithm",
    subject: "foc-theory",
    relatedQuestions: [5, 6],
  },
  {
    id: "alg-mtpa",
    label: "MTPA控制",
    path: "algorithm",
    subject: "foc-theory",
    relatedQuestions: [7, 8, 9],
  },

  // ────────────────── 算法路径：有感FOC实现 ──────────────────
  {
    id: "alg-encoder",
    label: "编码器接口",
    path: "algorithm",
    subject: "sensored-foc",
    relatedQuestions: [11, 12],
  },
  {
    id: "alg-cur-loop",
    label: "电流环设计",
    path: "algorithm",
    subject: "sensored-foc",
    relatedQuestions: [13, 14, 20],
  },
  {
    id: "alg-spd-loop",
    label: "速度环设计",
    path: "algorithm",
    subject: "sensored-foc",
    relatedQuestions: [15, 16],
  },
  {
    id: "alg-field-weak",
    label: "弱磁控制",
    path: "algorithm",
    subject: "sensored-foc",
    relatedQuestions: [17, 18, 19],
  },

  // ────────────────── 算法路径：无感FOC观测器 ──────────────────
  {
    id: "alg-smo",
    label: "滑模观测器",
    path: "algorithm",
    subject: "sensorless-foc",
    relatedQuestions: [21, 22, 30],
  },
  {
    id: "alg-bemf",
    label: "反电动势法",
    path: "algorithm",
    subject: "sensorless-foc",
    relatedQuestions: [23, 24],
  },
  {
    id: "alg-flux-obs",
    label: "磁链观测器",
    path: "algorithm",
    subject: "sensorless-foc",
    relatedQuestions: [25, 26],
  },
  {
    id: "alg-startup",
    label: "无感启动",
    path: "algorithm",
    subject: "sensorless-foc",
    relatedQuestions: [27, 28, 29],
  },

  // ────────────────── 算法路径：高频注入法 ──────────────────
  {
    id: "alg-pulsating",
    label: "脉振高频注入",
    path: "algorithm",
    subject: "hfi",
    relatedQuestions: [31, 32, 40],
  },
  {
    id: "alg-square",
    label: "方波注入",
    path: "algorithm",
    subject: "hfi",
    relatedQuestions: [33, 34],
  },
  {
    id: "alg-ns-detect",
    label: "N/S极判断",
    path: "algorithm",
    subject: "hfi",
    relatedQuestions: [35, 36],
  },
  {
    id: "alg-transition",
    label: "全速域切换",
    path: "algorithm",
    subject: "hfi",
    relatedQuestions: [37, 38, 39],
  },

  // ────────────────── 算法路径：保护与优化 ──────────────────
  {
    id: "alg-ocp",
    label: "过流保护算法",
    path: "algorithm",
    subject: "protection",
    relatedQuestions: [41, 42, 45],
  },
  {
    id: "alg-deadtime",
    label: "死区补偿",
    path: "algorithm",
    subject: "protection",
    relatedQuestions: [43, 44, 46],
  },

  // ────────────────── 硬件路径：电机与采样 ──────────────────
  {
    id: "hw-motor",
    label: "电机本体基础",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [51, 52],
  },
  {
    id: "hw-cur-samp",
    label: "电流采样电路",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [53, 55],
  },
  {
    id: "hw-encoder-hw",
    label: "位置传感器",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [54],
  },
  {
    id: "hw-mcu-periph",
    label: "MCU外设配置",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [55, 56],
  },

  // ────────────────── 硬件路径：功率器件与驱动 ──────────────────
  {
    id: "hw-power",
    label: "功率器件与驱动",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [56, 57, 58],
  },
  {
    id: "hw-gate-drv",
    label: "栅极驱动器",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [56, 57],
  },
  {
    id: "hw-bootstrap",
    label: "自举电路",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [57],
  },
  {
    id: "hw-dc-link",
    label: "母线电容设计",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [53, 58],
  },

  // ────────────────── 硬件路径：保护与可靠性 ──────────────────
  {
    id: "hw-protection-hw",
    label: "硬件保护电路",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [58, 59],
  },
  {
    id: "hw-thermal",
    label: "热设计",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [59],
  },
  {
    id: "hw-emc",
    label: "EMC设计",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [60],
  },
  {
    id: "hw-pcb",
    label: "PCB布局",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [60],
  },

  // ────────────────── 硬件路径：信号链 ──────────────────
  {
    id: "hw-adc",
    label: "ADC采样设计",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [53, 55],
  },
  {
    id: "hw-pwm-dead",
    label: "PWM死区设计",
    path: "hardware",
    subject: "hardware",
    relatedQuestions: [56, 44],
  },

  // ────────────────── 通信路径：工业通信协议 ──────────────────
  {
    id: "com-can-basics",
    label: "CAN总线基础",
    path: "communication",
    subject: "communication",
    relatedQuestions: [],
  },
  {
    id: "com-can-fd",
    label: "CAN FD扩展",
    path: "communication",
    subject: "communication",
    relatedQuestions: [],
  },
  {
    id: "com-can-stm32",
    label: "STM32 FDCAN实现",
    path: "communication",
    subject: "communication",
    relatedQuestions: [],
  },
  {
    id: "com-modbus",
    label: "Modbus协议",
    path: "communication",
    subject: "communication",
    relatedQuestions: [],
  },
  {
    id: "com-ethercat",
    label: "EtherCAT协议",
    path: "communication",
    subject: "communication",
    relatedQuestions: [],
  },
  {
    id: "com-protocol-compare",
    label: "协议选型对比",
    path: "communication",
    subject: "communication",
    relatedQuestions: [],
  },

  // ────────────────── 传动路径：减速器与编码器 ──────────────────
  {
    id: "mec-planetary",
    label: "行星减速器",
    path: "mechanical",
    subject: "mechanical",
    relatedQuestions: [],
  },
  {
    id: "mec-cycloidal",
    label: "摆线减速器",
    path: "mechanical",
    subject: "mechanical",
    relatedQuestions: [],
  },
  {
    id: "mec-multi-turn-encoder",
    label: "多圈编码器",
    path: "mechanical",
    subject: "mechanical",
    relatedQuestions: [],
  },
];

// ============================================================================
// 边定义：先修关系 + 相关关系 + 跨域引用，共 48 + 17 = 65 条边
// ============================================================================

export const knowledgeEdges: KnowledgeEdge[] = [
  // ═══════════════════ 算法路径内部：先修关系 ═══════════════════
  // FOC理论基础链
  { source: "alg-park", target: "alg-svpwm", relation: "prerequisite" },
  { source: "alg-park", target: "alg-dq", relation: "prerequisite" },
  { source: "alg-dq", target: "alg-mtpa", relation: "prerequisite" },
  { source: "alg-svpwm", target: "alg-dq", relation: "prerequisite" },

  // 有感FOC链：Park变换 → 电流环(内环) → 速度环(外环) → 弱磁
  { source: "alg-park", target: "alg-cur-loop", relation: "prerequisite" },
  { source: "alg-cur-loop", target: "alg-spd-loop", relation: "prerequisite" },
  { source: "alg-dq", target: "alg-field-weak", relation: "prerequisite" },
  { source: "alg-spd-loop", target: "alg-field-weak", relation: "prerequisite" },

  // 无感FOC链
  { source: "alg-park", target: "alg-smo", relation: "prerequisite" },
  { source: "alg-park", target: "alg-bemf", relation: "prerequisite" },
  { source: "alg-bemf", target: "alg-smo", relation: "prerequisite" },
  { source: "alg-bemf", target: "alg-startup", relation: "prerequisite" },
  { source: "alg-smo", target: "alg-startup", relation: "prerequisite" },

  // 高频注入链
  { source: "alg-park", target: "alg-pulsating", relation: "prerequisite" },
  { source: "alg-pulsating", target: "alg-ns-detect", relation: "prerequisite" },
  { source: "alg-ns-detect", target: "alg-transition", relation: "prerequisite" },
  { source: "alg-smo", target: "alg-transition", relation: "prerequisite" },

  // 保护链
  { source: "alg-cur-loop", target: "alg-ocp", relation: "prerequisite" },
  { source: "alg-svpwm", target: "alg-deadtime", relation: "prerequisite" },

  // ═══════════════════ 算法路径内部：相关关系 ═══════════════════
  { source: "alg-svpwm", target: "alg-dq", relation: "related" },
  { source: "alg-cur-loop", target: "alg-spd-loop", relation: "related" },
  { source: "alg-smo", target: "alg-bemf", relation: "related" },
  { source: "alg-smo", target: "alg-flux-obs", relation: "related" },
  { source: "alg-flux-obs", target: "alg-startup", relation: "related" },
  { source: "alg-pulsating", target: "alg-square", relation: "related" },
  { source: "alg-square", target: "alg-transition", relation: "related" },
  { source: "alg-cur-loop", target: "alg-ocp", relation: "related" },

  // ═══════════════════ 硬件路径内部：先修关系 ═══════════════════
  { source: "hw-motor", target: "hw-cur-samp", relation: "prerequisite" },
  { source: "hw-motor", target: "hw-encoder-hw", relation: "prerequisite" },
  { source: "hw-mcu-periph", target: "hw-adc", relation: "prerequisite" },
  { source: "hw-mcu-periph", target: "hw-pwm-dead", relation: "prerequisite" },
  { source: "hw-power", target: "hw-gate-drv", relation: "prerequisite" },
  { source: "hw-gate-drv", target: "hw-bootstrap", relation: "prerequisite" },
  { source: "hw-gate-drv", target: "hw-dc-link", relation: "prerequisite" },
  { source: "hw-gate-drv", target: "hw-protection-hw", relation: "prerequisite" },
  { source: "hw-pcb", target: "hw-emc", relation: "prerequisite" },
  { source: "hw-adc", target: "hw-cur-samp", relation: "prerequisite" },

  // ═══════════════════ 硬件路径内部：相关关系 ═══════════════════
  { source: "hw-protection-hw", target: "hw-thermal", relation: "related" },
  { source: "hw-pwm-dead", target: "hw-gate-drv", relation: "related" },
  { source: "hw-dc-link", target: "hw-emc", relation: "related" },
  { source: "hw-power", target: "hw-protection-hw", relation: "related" },
  { source: "hw-thermal", target: "hw-power", relation: "related" },

  // ═══════════════════ 跨域引用：硬件 ↔ 算法 ═══════════════════
  { source: "hw-motor", target: "alg-park", relation: "cross-reference" },
  { source: "hw-cur-samp", target: "alg-cur-loop", relation: "cross-reference" },
  { source: "hw-encoder-hw", target: "alg-encoder", relation: "cross-reference" },
  { source: "hw-gate-drv", target: "alg-deadtime", relation: "cross-reference" },
  { source: "hw-pwm-dead", target: "alg-deadtime", relation: "cross-reference" },
  { source: "hw-adc", target: "alg-cur-loop", relation: "cross-reference" },
  { source: "hw-mcu-periph", target: "alg-svpwm", relation: "cross-reference" },
  { source: "hw-dc-link", target: "alg-svpwm", relation: "cross-reference" },
  { source: "hw-cur-samp", target: "alg-bemf", relation: "cross-reference" },
  { source: "hw-thermal", target: "alg-ocp", relation: "cross-reference" },
  { source: "hw-bootstrap", target: "alg-svpwm", relation: "cross-reference" },
  { source: "hw-power", target: "alg-svpwm", relation: "cross-reference" },

  // ═══════════════════ 通信路径内部：先修关系 ═══════════════════
  { source: "com-can-basics", target: "com-can-fd", relation: "prerequisite" },
  { source: "com-can-basics", target: "com-can-stm32", relation: "prerequisite" },
  { source: "com-can-fd", target: "com-can-stm32", relation: "prerequisite" },
  { source: "com-can-basics", target: "com-modbus", relation: "prerequisite" },
  { source: "com-can-basics", target: "com-ethercat", relation: "prerequisite" },
  { source: "com-modbus", target: "com-protocol-compare", relation: "prerequisite" },
  { source: "com-ethercat", target: "com-protocol-compare", relation: "prerequisite" },
  { source: "com-can-fd", target: "com-protocol-compare", relation: "prerequisite" },

  // ═══════════════════ 通信路径内部：相关关系 ═══════════════════
  { source: "com-can-basics", target: "com-modbus", relation: "related" },
  { source: "com-can-fd", target: "com-ethercat", relation: "related" },
  { source: "com-modbus", target: "com-ethercat", relation: "related" },

  // ═══════════════════ 传动路径内部：先修关系 ═══════════════════
  { source: "mec-planetary", target: "mec-cycloidal", relation: "prerequisite" },
  { source: "mec-planetary", target: "mec-multi-turn-encoder", relation: "prerequisite" },

  // ═══════════════════ 传动路径内部：相关关系 ═══════════════════
  { source: "mec-planetary", target: "mec-cycloidal", relation: "related" },

  // ═══════════════════ 跨域引用：通信 ↔ 硬件/算法 ═══════════════════
  { source: "hw-mcu-periph", target: "com-can-stm32", relation: "cross-reference" },
  { source: "com-can-basics", target: "alg-ocp", relation: "cross-reference" },

  // ═══════════════════ 跨域引用：传动 ↔ 硬件/算法 ═══════════════════
  { source: "hw-motor", target: "mec-planetary", relation: "cross-reference" },
  { source: "hw-encoder-hw", target: "mec-multi-turn-encoder", relation: "cross-reference" },
  { source: "mec-planetary", target: "alg-spd-loop", relation: "cross-reference" },
];
