import { MCKnowledgeNode, MCKnowledgeEdge } from "@/lib/types";

export const mcKnowledgeNodes: MCKnowledgeNode[] = [
  // ══════════════════════════════════════════════════════════════════════
  // 硬件路径 (7 nodes)
  // ══════════════════════════════════════════════════════════════════════

  {
    id: "hw-motor",
    label: "电机本体基础",
    path: "hardware",
    chapter: "hardware",
    description:
      "PMSM与BLDC电机结构、dq轴电感Ld/Lq、永磁磁链ψf等关键参数的定义与测量方法，以及电机在静止/旋转坐标系下的数学模型推导。",
    relatedQuestionIds: [51, 52],
    subTopics: [
      "PMSM/BLDC定转子结构",
      "Ld/Lq/ψf参数定义与测量",
      "dq轴数学模型建立",
      "反电动势与转矩常数关系",
    ],
  },

  {
    id: "hw-cur-samp",
    label: "电流采样电路",
    path: "hardware",
    chapter: "hardware",
    description:
      "基于采样电阻或霍尔传感器的相电流检测方案，涵盖差分放大器选型、偏置电压设计、ADC采样时序同步等关键技术细节。",
    relatedQuestionIds: [53, 55],
    subTopics: [
      "采样电阻方案（低端/相端/母线）",
      "霍尔电流传感器方案",
      "差分放大器设计与噪声抑制",
      "ADC触发与PWM同步采样",
    ],
  },

  {
    id: "hw-pos-sensor",
    label: "位置传感器接口",
    path: "hardware",
    chapter: "hardware",
    description:
      "增量式/绝对式编码器、磁编码器、旋转变压器等位置传感器的工作原理、接口协议（ABI/SPI/旋变励磁解调）及角度校准方法。",
    relatedQuestionIds: [54],
    subTopics: [
      "增量编码器ABI协议与倍频",
      "绝对式编码器SPI/BiSS协议",
      "磁编码器与霍尔角度传感器",
      "旋变励磁与解调电路",
      "编码器零位校准与偏移补偿",
    ],
  },

  {
    id: "hw-mcu-periph",
    label: "MCU外设与通信",
    path: "hardware",
    chapter: "hardware",
    description:
      "电机控制MCU的PWM高级定时器配置、ADC同步注入采样、DMA数据搬运机制，以及UART/CAN/SPI通信接口在电机控制系统中的应用。",
    relatedQuestionIds: [55, 56],
    subTopics: [
      "高级定时器PWM中心对称模式",
      "ADC注入组与规则组同步采样",
      "DMA双缓冲乒乓传输",
      "CAN总线与UART调试通信",
      "SPI高速编码器数据读取",
    ],
  },

  {
    id: "hw-power-drv",
    label: "功率器件与栅极驱动",
    path: "hardware",
    chapter: "hardware",
    description:
      "MOSFET与IGBT的开关特性、导通损耗与开关损耗分析，米勒效应引起寄生导通的风险与抑制方法，以及自举电容充电与维持机制。",
    relatedQuestionIds: [56, 57, 58],
    subTopics: [
      "MOSFET/IGBT开关特性与损耗",
      "米勒效应与dv/dt寄生导通",
      "自举电路原理与电容选型",
      "栅极电阻选择与驱动电流",
      "负压关断与有源米勒箝位",
    ],
  },

  {
    id: "hw-pwr-protect",
    label: "电源管理与保护",
    path: "hardware",
    chapter: "hardware",
    description:
      "电机控制板的DC-DC/LDO电源拓扑设计，总线过流/过压/欠压保护电路，功率级温度保护与逐波限流保护的硬件实现方案。",
    relatedQuestionIds: [58, 59],
    subTopics: [
      "DC-DC Buck/LDO电源拓扑选型",
      "母线电容容量与纹波电流计算",
      "过流保护（逐波限流/打嗝保护）",
      "过压/欠压保护电路设计",
      "NTC/PTC温度检测与保护阈值",
    ],
  },

  {
    id: "hw-thermal-emc",
    label: "热设计与EMC",
    path: "hardware",
    chapter: "hardware",
    description:
      "功率器件热阻计算与散热方案设计，开关电源EMI传导/辐射噪声的滤波策略，VHF高速PCB布局中的关键走线与接地方案。",
    relatedQuestionIds: [59, 60],
    subTopics: [
      "结-壳-环境热阻模型与散热器",
      "功率PCB铜箔载流与温升计算",
      "共模/差模EMI滤波器设计",
      "高频电流环面积最小化布局",
      "功率地与信号地单点接地",
    ],
  },

  // ══════════════════════════════════════════════════════════════════════
  // 算法路径 (6 nodes)
  // ══════════════════════════════════════════════════════════════════════

  {
    id: "alg-foc-theory",
    label: "FOC理论基础",
    path: "algorithm",
    chapter: "foc-theory",
    description:
      "Clarke/Park坐标变换将三相电流映射到旋转dq坐标系实现励磁与转矩分量的解耦控制，SVPWM空间矢量调制实现电压矢量的精确合成。",
    relatedQuestionIds: Array.from({ length: 20 }, (_, i) => i + 1),
    subTopics: [
      "Clarke变换(abc→αβ)",
      "Park变换(αβ→dq)与角度依赖",
      "dq轴电压方程与耦合项",
      "转矩方程(id=0/最大转矩电流比)",
      "SVPWM七段式/五段式调制",
    ],
  },

  {
    id: "alg-sensored",
    label: "有感FOC实现",
    path: "algorithm",
    chapter: "sensored-foc",
    description:
      "基于编码器实时角度反馈的有传感器FOC闭环控制，含电流环PI参数整定、速度环级联控制架构、V/F启动平滑过渡至闭环的全流程。",
    relatedQuestionIds: Array.from({ length: 20 }, (_, i) => i + 21),
    subTopics: [
      "编码器角度与转速实时解析",
      "电流内环PI参数整定方法",
      "速度外环级联控制与Anti-Windup",
      "V/F开环→FOC闭环平滑切换",
      "制动能量回馈与母线电压控制",
    ],
  },

  {
    id: "alg-sensorless",
    label: "无感FOC观测器",
    path: "algorithm",
    chapter: "sensorless-foc",
    description:
      "无位置传感器条件下的转子角度与转速估计，涵盖滑模观测器(SMO)、磁链观测器、扩展卡尔曼滤波(EKF)、模型参考自适应(MRAS)及PLL角度跟踪。",
    relatedQuestionIds: Array.from({ length: 20 }, (_, i) => i + 41),
    subTopics: [
      "滑模观测器SMO与低通滤波",
      "磁链观测器（电压/电流模型）",
      "扩展卡尔曼滤波EKF递推估计",
      "模型参考自适应MRAS设计",
      "PLL锁相环角度跟踪与归一化",
    ],
  },

  {
    id: "alg-hfi",
    label: "高频注入法",
    path: "algorithm",
    chapter: "hfi",
    description:
      "利用IPMSM转子凸极性，在静止/估计坐标系注入高频电压/电流信号并提取角度信息，实现零速和极低速下的无传感器稳定运行与N/S极辨识。",
    relatedQuestionIds: Array.from({ length: 20 }, (_, i) => i + 61),
    subTopics: [
      "脉振高频正弦注入法",
      "方波电压注入与信号提取",
      "转子凸极性建模与响应解调",
      "N/S磁极极性二次判别",
      "零低速→高速全速域平滑切换",
    ],
  },

  {
    id: "alg-protection",
    label: "保护与优化",
    path: "algorithm",
    chapter: "protection",
    description:
      "软件层面的过流/过压/欠压/缺相实时检测与保护策略，死区效应分析与补偿算法，以及弱磁控制与MTPA在宽转速范围的效率优化。",
    relatedQuestionIds: Array.from({ length: 20 }, (_, i) => i + 81),
    subTopics: [
      "软件过流逐周检测与快速关断",
      "过压/欠压/缺相故障诊断",
      "PWM死区效应与电压畸变补偿",
      "弱磁控制（电压闭环/前馈）",
      "MTPA查表法与多项式拟合",
    ],
  },

  {
    id: "alg-research",
    label: "前沿研究",
    path: "algorithm",
    chapter: "foc-theory",
    description:
      "学术前沿方向：离散时间域磁链观测器避免连续域近似的误差、低载波比fpwm/f0下传统PWM策略的改进、零阶保持器(ZOH)对数字控制系统稳定性的影响分析。",
    relatedQuestionIds: [3, 4, 10],
    subTopics: [
      "离散时间磁链观测器设计",
      "低载波比fpwm/f0工况分析",
      "ZOH零阶保持器时延效应",
      "逆变器非线性补偿与辨识",
      "无速度传感器全速域统一方案",
    ],
  },
];

export const mcKnowledgeEdges: MCKnowledgeEdge[] = [
  // ══════════════════════════════════════════════════════════════════════
  // 硬件路径内部：先修关系
  // ══════════════════════════════════════════════════════════════════════
  { source: "hw-motor", target: "hw-cur-samp", relation: "prerequisite" },
  { source: "hw-motor", target: "hw-pos-sensor", relation: "prerequisite" },
  { source: "hw-motor", target: "hw-mcu-periph", relation: "prerequisite" },
  { source: "hw-cur-samp", target: "hw-mcu-periph", relation: "prerequisite" },
  { source: "hw-mcu-periph", target: "hw-power-drv", relation: "prerequisite" },
  { source: "hw-power-drv", target: "hw-pwr-protect", relation: "prerequisite" },
  { source: "hw-power-drv", target: "hw-thermal-emc", relation: "prerequisite" },
  { source: "hw-pwr-protect", target: "hw-thermal-emc", relation: "prerequisite" },

  // ══════════════════════════════════════════════════════════════════════
  // 硬件路径内部：相关关系
  // ══════════════════════════════════════════════════════════════════════
  { source: "hw-cur-samp", target: "hw-pos-sensor", relation: "related" },
  { source: "hw-power-drv", target: "hw-pwr-protect", relation: "related" },
  { source: "hw-pwr-protect", target: "hw-thermal-emc", relation: "related" },
  { source: "hw-mcu-periph", target: "hw-cur-samp", relation: "related" },
  { source: "hw-pos-sensor", target: "hw-mcu-periph", relation: "related" },

  // ══════════════════════════════════════════════════════════════════════
  // 算法路径内部：先修关系
  // ══════════════════════════════════════════════════════════════════════
  { source: "alg-foc-theory", target: "alg-sensored", relation: "prerequisite" },
  { source: "alg-foc-theory", target: "alg-sensorless", relation: "prerequisite" },
  { source: "alg-sensored", target: "alg-sensorless", relation: "prerequisite" },
  { source: "alg-sensorless", target: "alg-hfi", relation: "prerequisite" },
  { source: "alg-foc-theory", target: "alg-protection", relation: "prerequisite" },
  { source: "alg-sensored", target: "alg-protection", relation: "prerequisite" },
  { source: "alg-sensorless", target: "alg-protection", relation: "prerequisite" },
  { source: "alg-sensorless", target: "alg-research", relation: "prerequisite" },

  // ══════════════════════════════════════════════════════════════════════
  // 算法路径内部：相关关系
  // ══════════════════════════════════════════════════════════════════════
  { source: "alg-sensored", target: "alg-protection", relation: "related" },
  { source: "alg-hfi", target: "alg-sensored", relation: "related" },
  { source: "alg-sensorless", target: "alg-hfi", relation: "related" },
  { source: "alg-hfi", target: "alg-protection", relation: "related" },
  { source: "alg-research", target: "alg-sensorless", relation: "related" },
  { source: "alg-research", target: "alg-protection", relation: "related" },

  // ══════════════════════════════════════════════════════════════════════
  // 跨域引用：硬件 ↔ 算法（每节点至少2条双向/多向边）
  // ══════════════════════════════════════════════════════════════════════

  // ── hw-motor ↔ 算法 ──
  {
    source: "hw-motor",
    target: "alg-foc-theory",
    relation: "cross-reference",
    label: "电机Ld/Lq/ψf参数是FOC模型的基础输入",
  },
  {
    source: "alg-foc-theory",
    target: "hw-motor",
    relation: "cross-reference",
    label: "FOC控制理论依赖电机的精确数学模型",
  },
  {
    source: "hw-motor",
    target: "alg-sensorless",
    relation: "cross-reference",
    label: "凸极比Lq/Ld影响无感观测器的收敛性",
  },

  // ── hw-cur-samp ↔ 算法 ──
  {
    source: "hw-cur-samp",
    target: "alg-sensored",
    relation: "cross-reference",
    label: "采样精度与噪声直接决定电流环控制性能",
  },
  {
    source: "hw-cur-samp",
    target: "alg-sensorless",
    relation: "cross-reference",
    label: "电流采样扰动经观测器放大产生角度估计误差",
  },
  {
    source: "alg-sensorless",
    target: "hw-cur-samp",
    relation: "cross-reference",
    label: "无感观测器精度高度依赖低噪声电流采样",
  },

  // ── hw-pos-sensor ↔ 算法 ──
  {
    source: "hw-pos-sensor",
    target: "alg-sensored",
    relation: "cross-reference",
    label: "编码器精度与延迟直接影响Park变换角度准确性",
  },
  {
    source: "alg-sensored",
    target: "hw-pos-sensor",
    relation: "cross-reference",
    label: "有感FOC性能受限于位置传感器的分辨率与带宽",
  },
  {
    source: "hw-pos-sensor",
    target: "alg-sensorless",
    relation: "cross-reference",
    label: "编码器数据可作为无感观测器的标定基准",
  },

  // ── hw-mcu-periph ↔ 算法 ──
  {
    source: "hw-mcu-periph",
    target: "alg-sensored",
    relation: "cross-reference",
    label: "PWM死区时间与中心对称模式影响输出电压波形质量",
  },
  {
    source: "hw-mcu-periph",
    target: "alg-protection",
    relation: "cross-reference",
    label: "MCU硬件故障检测与软件保护策略协同实现快速保护",
  },
  {
    source: "alg-research",
    target: "hw-mcu-periph",
    relation: "cross-reference",
    label: "离散时间控制算法需要精确的MCU中断时序与PWM更新时刻",
  },
  {
    source: "alg-sensorless",
    target: "hw-mcu-periph",
    relation: "cross-reference",
    label: "复杂观测器算法的实时计算依赖于MCU浮点算力与DMA效率",
  },

  // ── hw-power-drv ↔ 算法 ──
  {
    source: "hw-power-drv",
    target: "alg-sensorless",
    relation: "cross-reference",
    label: "MOSFET开关非线性与死区效应引入电压畸变，恶化观测器估计精度",
  },
  {
    source: "hw-power-drv",
    target: "alg-protection",
    relation: "cross-reference",
    label: "功率器件开关特性是死区补偿算法设计的物理基础",
  },
  {
    source: "alg-hfi",
    target: "hw-power-drv",
    relation: "cross-reference",
    label: "高频注入信号在通过逆变器时受开关非线性的调制与衰减",
  },
  {
    source: "alg-protection",
    target: "hw-power-drv",
    relation: "cross-reference",
    label: "保护算法需要功率器件的热阻模型与安全工作区SOA参数",
  },

  // ── hw-pwr-protect ↔ 算法 ──
  {
    source: "hw-pwr-protect",
    target: "alg-protection",
    relation: "cross-reference",
    label: "硬件过流/过压检测信号输入作为软件保护的状态机触发源",
  },
  {
    source: "alg-protection",
    target: "hw-pwr-protect",
    relation: "cross-reference",
    label: "软件保护策略依赖于硬件保护电路的传感器精度与响应速度",
  },
  {
    source: "hw-pwr-protect",
    target: "alg-sensored",
    relation: "cross-reference",
    label: "电源纹波与电压跌落影响编码器供电稳定性和信号质量",
  },

  // ── hw-thermal-emc ↔ 算法 ──
  {
    source: "hw-thermal-emc",
    target: "alg-protection",
    relation: "cross-reference",
    label: "功率器件温升影响Rds(on)等参数，需要在降额策略中动态补偿",
  },
  {
    source: "hw-thermal-emc",
    target: "alg-hfi",
    relation: "cross-reference",
    label: "开关电源EMI噪声频段与高频注入信号频段重叠，降低信噪比",
  },
  {
    source: "alg-hfi",
    target: "hw-thermal-emc",
    relation: "cross-reference",
    label: "高频注入信号可能通过PCB寄生耦合加剧近场辐射干扰",
  },
  {
    source: "alg-protection",
    target: "hw-thermal-emc",
    relation: "cross-reference",
    label: "降额保护策略需要结合热阻模型与PCB散热能力综合设计",
  },

  // ── alg-foc-theory ↔ 硬件 ──
  {
    source: "alg-foc-theory",
    target: "hw-cur-samp",
    relation: "cross-reference",
    label: "Clarke变换要求三相电流严格同步采样，对ADC时序提出要求",
  },
  {
    source: "alg-foc-theory",
    target: "hw-power-drv",
    relation: "cross-reference",
    label: "SVPWM过调制策略需考虑功率器件的电压输出能力极限",
  },

  // ── alg-sensorless ↔ 硬件 ──
  {
    source: "alg-sensorless",
    target: "hw-power-drv",
    relation: "cross-reference",
    label: "逆变器非线性效应对低转速无感观测器精度影响显著",
  },
  {
    source: "alg-sensorless",
    target: "hw-thermal-emc",
    relation: "cross-reference",
    label: "电机参数温漂导致观测器模型失配，需考虑在线参数辨识",
  },

  // ── alg-research ↔ 算法/硬件 ──
  {
    source: "alg-research",
    target: "alg-sensorless",
    relation: "cross-reference",
    label: "离散时间磁链观测器是对传统连续域观测器在数字实现中的改进",
  },
  {
    source: "alg-research",
    target: "hw-power-drv",
    relation: "cross-reference",
    label: "低载波比工况下逆变器非线性效应加剧，需针对性的补偿策略",
  },
];
