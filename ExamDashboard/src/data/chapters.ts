export interface ChapterMeta {
  id: string;
  name: string;
  shortName: string;
  color: string;
  icon: string;
  order: number;
  description: string;
}

export const CHAPTERS: ChapterMeta[] = [
  {
    id: 'foc-theory',
    name: 'FOC理论基础',
    shortName: 'FOC理论',
    color: '#4fc3f7',
    icon: '📐',
    order: 1,
    description: '涵盖Clarke/Park坐标变换、dq轴电机模型、转矩方程、SVPWM空间矢量调制、PI控制器设计、MTPA控制策略等FOC核心理论基础',
  },
  {
    id: 'sensored-foc',
    name: '有感FOC实现',
    shortName: '有感FOC',
    color: '#ff8a65',
    icon: '🔍',
    order: 2,
    description: '增量式/绝对式编码器原理与校准、电流采样方案、电流环与速度环PI整定、死区补偿、前馈解耦、弱磁控制等有感FOC工程实现',
  },
  {
    id: 'sensorless-foc',
    name: '无感FOC观测器',
    shortName: '无感FOC',
    color: '#81c784',
    icon: '🔮',
    order: 3,
    description: '滑模观测器(SMO)、磁链观测器、Luenberger观测器、扩展卡尔曼滤波(EKF)、PLL位置跟踪、I/F启动及开闭环切换等无传感器控制技术',
  },
  {
    id: 'hfi',
    name: '高频注入法',
    shortName: '高频注入',
    color: '#ba68c8',
    icon: '⚡',
    order: 4,
    description: '基于凸极性的零速/极低速位置估计：脉振注入与旋转注入、方波注入、同步解调、N/S极性判别、全速域切换策略、逆变器非线性影响',
  },
  {
    id: 'protection',
    name: '保护与故障诊断',
    shortName: '保护诊断',
    color: '#ef5350',
    icon: '🛡️',
    order: 5,
    description: '硬件过流保护(Trip-Zone)、过压泵升保护、堵转检测、DESAT短路保护、温度监控与降额策略、故障分级恢复、直通防护、自举电容可靠性',
  },
  {
    id: 'hardware',
    name: '硬件知识',
    shortName: '硬件知识',
    color: '#ffd54f',
    icon: '🔧',
    order: 6,
    description: 'MOSFET/IGBT选型与驱动、栅极驱动电路设计、电流采样方案、母线电容计算、散热设计与热仿真、PCB布局布线、EMC与安规等硬件工程实践',
  },
  {
    id: 'communication',
    name: '工业通信协议',
    shortName: '通信协议',
    color: '#4dd0e1',
    icon: '📡',
    order: 7,
    description: 'CAN/CAN FD/EtherCAT/Modbus等工业通信协议原理、实现与选型',
  },
  {
    id: 'mechanical',
    name: '传动与编码器',
    shortName: '传动编码',
    color: '#a1887f',
    icon: '⚙️',
    order: 8,
    description: '行星/摆线减速器原理与选型、多圈绝对值编码器原理与接口',
  },
];

export const CHAPTER_MAP: Record<string, ChapterMeta> = {};
CHAPTERS.forEach((ch) => {
  CHAPTER_MAP[ch.id] = ch;
});
