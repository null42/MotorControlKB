import * as fs from "fs";
import * as path from "path";

const KNOWLEDGE_BASE = path.resolve(__dirname, "..", "..", "motor-control-knowledge-base");
const DATA_DIR = path.resolve(__dirname, "..", "src", "data");
const OUT_QUESTIONS = path.join(DATA_DIR, "mc-questions.generated.ts");
const OUT_KNOWLEDGE_GRAPH = path.join(DATA_DIR, "mc-knowledgeGraph.generated.ts");
const OUT_OUTLINE = path.join(DATA_DIR, "mc-outline.generated.ts");

const CHAPTER_MAP: Record<string, string> = {
  "ALG-01": "foc-theory",
  "ALG-02": "sensored-foc",
  "ALG-03": "sensorless-foc",
  "ALG-04": "hfi",
  "ALG-05": "protection",
  "ALG-06": "foc-theory",
  "HW-01": "hardware",
  "HW-02": "hardware",
  "HW-03": "hardware",
  "HW-04": "hardware",
  "HW-05": "hardware",
  "HW-06": "hardware",
  "HW-07": "hardware",
  "ADV-ALG-01": "sensored-foc",
  "ADV-ALG-02": "protection",
  "ADV-ALG-03": "sensored-foc",
  "ADV-ALG-04": "sensorless-foc",
  "ADV-ALG-05": "sensorless-foc",
  "ADV-ALG-06": "foc-theory",
  "ADV-HW-01": "hardware",
  "ADV-HW-02": "hardware",
  "ADV-HW-03": "hardware",
  "SYS-01": "protection",
  "SYS-02": "sensorless-foc",
  "SYS-03": "hardware",
  "SYS-04": "protection",
};

const KNOWN_MC_LIB_FILES = [
  "MC-LIB-Architecture.md",
  "MC-LIB-FOC-Core.md",
  "MC-LIB-SVPWM.md",
  "MC-LIB-Control-Loop.md",
  "MC-LIB-Observer.md",
  "MC-LIB-Six-Step.md",
  "MC-LIB-Porting-Guide.md",
];

interface ParsedQuestion {
  stem: string;
  referenceAnswer: string;
  explanation: string;
  difficulty: number;
  tags: string[];
  codeRefs: string[];
  hint1: string;
  hint2: string;
  chapter: string;
}

function scanAssessmentFiles(): string[] {
  const results: string[] = [];
  const dirs = ["algorithm", "hardware", "advanced"];

  for (const dir of dirs) {
    const fullDir = path.join(KNOWLEDGE_BASE, dir);
    if (!fs.existsSync(fullDir)) continue;
    walkDir(fullDir, results);
  }

  return results.filter((f) => f.endsWith("-assessment.md"));
}

function walkDir(dir: string, results: string[]) {
  const entries = fs.readdirSync(dir, { withFileTypes: true });
  for (const entry of entries) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      walkDir(full, results);
    } else if (entry.isFile()) {
      results.push(full);
    }
  }
}

function extractModulePrefix(filename: string): string {
  const basename = path.basename(filename, ".md");
  const match = basename.match(/^(ALG-\d+|HW-\d+|ADV-ALG-\d+|ADV-HW-\d+|SYS-\d+)/);
  return match ? match[1] : "";
}

function countStars(s: string): number {
  const full = s.match(/★/g);
  return full ? full.length : 0;
}

function extractDifficultyFromBlock(block: string, sectionHeader: string): number {
  const titleMatch = block.match(/^###\s+题目\d+[：:][^(（]*[(（]([★☆]+)[)）]/m);
  if (titleMatch) return countStars(titleMatch[1]);

  const sectionMatch = sectionHeader.match(/[（(]([★☆]+)[)）]/);
  if (sectionMatch) return countStars(sectionMatch[1]);

  const metaMatch = block.match(/难度[：:]\s*([★☆]+)/);
  if (metaMatch) return countStars(metaMatch[1]);

  const distMatch = block.match(/难度分布[：:][^]*?([★☆]+)/);
  if (distMatch) return countStars(distMatch[1]);

  return 3;
}

function extractCodeRefs(text: string): string[] {
  const refs = new Set<string>();
  const patterns = [
    /MC-LIB[/\\][^\s,)"]+\.(?:c|h|md)/g,
    /`([^`]+\.(?:c|h))`/g,
    /([A-Za-z0-9_/\\-]+\.(?:c|h))\b/g,
  ];
  for (const pattern of patterns) {
    let m: RegExpExecArray | null;
    while ((m = pattern.exec(text)) !== null) {
      const ref = m[1] || m[0];
      if (ref.length > 4 && !ref.startsWith("http")) {
        refs.add(ref.replace(/`/g, "").trim());
      }
    }
  }
  return Array.from(refs).slice(0, 5);
}

function extractTags(stem: string, modulePrefix: string): string[] {
  const tags: string[] = [];
  const tagKeywords: Record<string, string[]> = {
    "Clarke": ["Clarke变换", "坐标变换"],
    "Park": ["Park变换", "坐标变换"],
    "SVPWM": ["SVPWM", "空间矢量调制"],
    "PI": ["PI控制", "参数整定"],
    "MTPA": ["MTPA", "效率优化"],
    "弱磁": ["弱磁控制", "高速运行"],
    "观测器": ["观测器", "无感控制"],
    "SMO": ["滑模观测器", "无感控制"],
    "EKF": ["卡尔曼滤波", "无感控制"],
    "高频注入": ["高频注入", "无感控制"],
    "死区": ["死区补偿", "PWM"],
    "过流": ["过流保护", "故障保护"],
    "过压": ["过压保护", "故障保护"],
    "MOSFET": ["MOSFET", "功率器件"],
    "IGBT": ["IGBT", "功率器件"],
    "编码器": ["编码器", "位置传感器"],
    "电流采样": ["电流采样", "采样电路"],
    "EMC": ["EMC", "电磁兼容"],
    "带宽": ["带宽设计", "控制环"],
    "前馈": ["前馈解耦", "控制策略"],
    "电感": ["电机参数", "Ld/Lq"],
    "磁链": ["电机参数", "磁链"],
    "温度": ["温度补偿", "热设计"],
    "PWM": ["PWM", "调制策略"],
    "dq": ["dq轴", "坐标变换"],
    "反电动势": ["反电动势", "无感控制"],
    "锁相环": ["PLL", "角度跟踪"],
    "PLL": ["PLL", "角度跟踪"],
    "凸极": ["凸极性", "高频注入"],
    "载波比": ["载波比", "离散控制"],
    "ZOH": ["ZOH", "离散控制"],
  };

  for (const [keyword, t] of Object.entries(tagKeywords)) {
    if (stem.includes(keyword)) {
      for (const tag of t) {
        if (!tags.includes(tag)) tags.push(tag);
      }
    }
  }

  if (modulePrefix.startsWith("HW")) {
    if (!tags.includes("硬件")) tags.unshift("硬件");
  }
  if (modulePrefix.startsWith("ALG")) {
    if (!tags.includes("算法")) tags.unshift("算法");
  }

  return tags.slice(0, 8);
}

function generateHints(stem: string, referenceAnswer: string, tags: string[], chapter: string): { hint1: string; hint2: string } {
  const chapterHints: Record<string, { h1: string; h2: string }> = {
    "foc-theory": { h1: "回顾FOC理论基础中的坐标变换、dq轴模型和SVPWM等核心概念。", h2: "从Clarke/Park变换矩阵出发，结合dq电压方程进行分析推导。" },
    "sensored-foc": { h1: "结合有感FOC的控制架构，考虑电流环、速度环和前馈解耦的作用。", h2: "关注电流采样、角度反馈和PI参数整定对控制性能的影响。" },
    "sensorless-foc": { h1: "无感FOC的核心是观测器设计，需要从电机反电动势模型中提取位置信息。", h2: "分析观测器的收敛性、参数鲁棒性以及低速/零速条件下的局限性。" },
    "hfi": { h1: "高频注入法利用电机的凸极性（Ld ≠ Lq）来在零速和低速下提取转子位置。", h2: "关注高频信号注入方式、信号解调策略以及多凸极性/交叉饱和的影响。" },
    "protection": { h1: "保护与优化涉及过流、过压、温度、堵转等多层安全机制。", h2: "从故障检测阈值、保护响应时间和系统安全状态转换角度进行分析。" },
    "hardware": { h1: "从硬件电路原理出发，理解电流采样、栅极驱动和信号调理的物理约束。", h2: "结合实际芯片手册、PCB布局和电磁兼容要求进行分析。" },
  };

  const defaultHints = chapterHints[chapter] || { h1: "结合本章节的核心概念和方法进行分析思考。", h2: "从基本原理出发，梳理关键公式和推导步骤。" };

  let hint1 = defaultHints.h1;
  let hint2 = defaultHints.h2;

  if (tags.length >= 3) {
    const keyTags = tags.slice(0, 3).join("、");
    hint2 = `本题涉及 ${keyTags} 等领域，请结合这些概念的核心要点进行深入分析。`;
  }

  const stemLower = stem.toLowerCase();
  if (stemLower.includes("计算") || stemLower.includes("推导")) {
    hint1 = "这是一个计算/推导题，先写出核心公式，再代入数值进行逐步计算。";
  }
  if (stemLower.includes("比较") || stemLower.includes("对比") || stemLower.includes("区别")) {
    hint1 = "这是一个对比分析题，先分别阐述各方案的核心原理，再对比优缺点。";
  }
  if (stemLower.includes("为什么") || stemLower.includes("原因")) {
    hint1 = "先回答核心结论，再从物理本质/数学模型层面解释\"为什么\"。";
  }
  if (stemLower.includes("实现") || stemLower.includes("代码") || stemLower.includes("如何")) {
    hint1 = "从工程实现角度出发，结合代码逻辑和硬件约束进行分析。";
  }

  return { hint1, hint2 };
}

function parseOpenEndedQuestion(block: string, sectionHeader: string): ParsedQuestion | null {
  const stemMatch = block.match(/\*\*(?:问题|题目)[：:]\*\*\s*([\s\S]*?)(?=\*\*参考|\*\*正确|$)/);
  if (!stemMatch) return null;

  const stem = stemMatch[1].trim();

  const answerMatch = block.match(/\*\*参考答案[：:]\*\*\s*([\s\S]*?)(?=\n###\s|\n##\s|\n---\s|\n\*\*|$)/);
  const referenceAnswer = answerMatch ? answerMatch[1].trim() : "";

  const explanationMatch = block.match(/\*\*(?:解析|分析)[：:]\*\*\s*([\s\S]*?)(?=\n###\s|\n##\s|\n---\s|$)/);
  const explanation = explanationMatch ? explanationMatch[1].trim() : "";

  const difficulty = extractDifficultyFromBlock(block, sectionHeader);

  return {
    stem,
    referenceAnswer,
    explanation,
    difficulty,
    tags: [],
    codeRefs: [],
    hint1: "",
    hint2: "",
    chapter: "",
  };
}

function parseMultipleChoiceQuestion(block: string): ParsedQuestion | null {
  const stemMatch = block.match(/\*\*(?:问题|题目)[：:]\*\*\s*([\s\S]*?)(?=\nA\.\s|$)/);
  if (!stemMatch) return null;

  const stem = stemMatch[1].trim();

  const options: string[] = [];
  for (const letter of ["A", "B", "C", "D"]) {
    const optMatch = block.match(new RegExp(`^${letter}\\.\\s+([\\s\\S]*?)(?=\\n[B-Z]\\.\\s|\\n\\*\\*正确|\\n\\*\\*题目|$)`));
    if (optMatch) options.push(optMatch[1].trim());
  }

  const correctMatch = block.match(/\*\*正确答案[：:]\*\*\s*([A-D])/);
  const correct = correctMatch ? correctMatch[1] : "";

  const explanationMatch = block.match(/\*\*详细解析[：:]\*\*\s*([\s\S]*?)(?=\n###\s|\n##\s|\n---\s|$)/);
  const explanation = explanationMatch ? explanationMatch[1].trim() : "";

  let referenceAnswer = "";
  if (correct && options.length > 0) {
    const idx = correct.charCodeAt(0) - 65;
    const optionText = options[idx] || "";
    referenceAnswer = `正确答案：${correct}\n\n${optionText}`;
  }

  if (explanation) {
    referenceAnswer += `\n\n解析：\n${explanation}`;
  }

  const starsMatch = block.match(/\(([★☆]+)\)/);
  const difficulty = starsMatch ? countStars(starsMatch[1]) : 3;

  const fullStem = stem + (options.length > 0 ? "\n\n" + options.map((o, i) => `${String.fromCharCode(65 + i)}. ${o}`).join("\n") : "");

  return {
    stem: fullStem,
    referenceAnswer,
    explanation,
    difficulty,
    tags: [],
    codeRefs: [],
    hint1: "",
    hint2: "",
    chapter: "",
  };
}

function parseAssessmentFile(filepath: string): { questions: ParsedQuestion[]; modulePrefix: string; errors: number } {
  const content = fs.readFileSync(filepath, "utf-8");
  const modulePrefix = extractModulePrefix(filepath);
  const chapter = CHAPTER_MAP[modulePrefix] || "foc-theory";
  const questions: ParsedQuestion[] = [];
  let errors = 0;

  const isAdvancedFormat = content.match(/^##\s+第\d+题/m);

  if (isAdvancedFormat) {
    const blocks = content.split(/\n(?=##\s+第\d+题)/);
    for (const block of blocks) {
      if (!block.match(/^##\s+第\d+题/)) continue;
      const q = parseMultipleChoiceQuestion(block);
      if (q && q.stem.length > 10) {
        q.tags = extractTags(q.stem, modulePrefix);
        q.codeRefs = extractCodeRefs(block);
        q.chapter = chapter;
        const hints = generateHints(q.stem, q.referenceAnswer, q.tags, chapter);
        q.hint1 = hints.hint1;
        q.hint2 = hints.hint2;
        questions.push(q);
      } else {
        errors++;
      }
    }
    return { questions, modulePrefix, errors };
  }

  const sections = content.split(/\n(?=##\s)/);
  for (const section of sections) {
    const sectionHeaderLine = section.split("\n")[0] || "";

    if (section.includes("A.\n") || section.includes("A. ")) {
      const blocks = section.split(/\n(?=###\s+题目)/);
      for (const block of blocks) {
        if (!block.match(/^###\s+题目/)) continue;
        const q = parseMultipleChoiceQuestion(block);
        if (q && q.stem.length > 10) {
          q.tags = extractTags(q.stem, modulePrefix);
          q.codeRefs = extractCodeRefs(block);
          q.chapter = chapter;
          const hints = generateHints(q.stem, q.referenceAnswer, q.tags, chapter);
          q.hint1 = hints.hint1;
          q.hint2 = hints.hint2;
          questions.push(q);
        } else {
          errors++;
        }
      }
    } else {
      const blocks = section.split(/\n(?=###\s+题目)/);
      for (const block of blocks) {
        if (!block.match(/^###\s+题目/)) continue;
        if (block.includes("正确答案")) {
          const q = parseMultipleChoiceQuestion(block);
          if (q && q.stem.length > 10) {
            q.tags = extractTags(q.stem, modulePrefix);
            q.codeRefs = extractCodeRefs(block);
            q.chapter = chapter;
            const hints = generateHints(q.stem, q.referenceAnswer, q.tags, chapter);
            q.hint1 = hints.hint1;
            q.hint2 = hints.hint2;
            questions.push(q);
          } else {
            errors++;
          }
        } else {
          const q = parseOpenEndedQuestion(block, sectionHeaderLine);
          if (q && q.stem.length > 10) {
            q.tags = extractTags(q.stem + "\n" + q.referenceAnswer, modulePrefix);
            q.codeRefs = extractCodeRefs(block + "\n" + q.referenceAnswer);
            q.chapter = chapter;
            const hints = generateHints(q.stem, q.referenceAnswer, q.tags, chapter);
            q.hint1 = hints.hint1;
            q.hint2 = hints.hint2;
            questions.push(q);
          } else {
            errors++;
          }
        }
      }
    }
  }

  return { questions, modulePrefix, errors };
}

function parseReadmeStationTable(filepath: string): { stations: Array<{ stationNum: number; name: string; description: string; difficulty: string }>; mcLibDocs: Array<{ name: string; description: string; relatedModule: string }> } {
  const stations: Array<{ stationNum: number; name: string; description: string; difficulty: string }> = [];
  const mcLibDocs: Array<{ name: string; description: string; relatedModule: string }> = [];

  if (!fs.existsSync(filepath)) return { stations, mcLibDocs };

  const content = fs.readFileSync(filepath, "utf-8");
  const lines = content.split("\n");

  let inStationTable = false;
  let inMcLibTable = false;
  let headerSkipped = false;

  for (const line of lines) {
    const trimmed = line.trim();

    if (trimmed.startsWith("## 模块列表") || trimmed.startsWith("## 模块列表")) {
      inStationTable = true;
      inMcLibTable = false;
      headerSkipped = false;
      continue;
    }

    if (trimmed.includes("MC_LIB") || trimmed.startsWith("## MC_LIB")) {
      inStationTable = false;
      inMcLibTable = true;
      headerSkipped = false;
      continue;
    }

    if (trimmed.startsWith("## ") && inStationTable) {
      inStationTable = false;
      continue;
    }

    if (trimmed.startsWith("## ") && inMcLibTable) {
      inMcLibTable = false;
      continue;
    }

    if ((inStationTable || inMcLibTable) && trimmed.startsWith("|---")) {
      headerSkipped = true;
      continue;
    }

    if (!headerSkipped && (inStationTable || inMcLibTable)) continue;

    if (inStationTable && trimmed.startsWith("|")) {
      const cells = trimmed.split("|").map((c) => c.trim()).filter((c) => c.length > 0);
      if (cells.length >= 4) {
        const stationNum = parseInt(cells[0]);
        const nameMatch = cells[1].match(/\[([^\]]+)\]/);
        const name = nameMatch ? nameMatch[1] : cells[1];
        const description = cells[2].replace(/^FOC为什么.*/, "FOC理论基础与坐标变换").replace(/^如何.*/, cells[2]);
        const difficulty = cells[3];

        if (!isNaN(stationNum) && name) {
          stations.push({ stationNum, name, description, difficulty });
        }
      }
    }

    if (inMcLibTable && trimmed.startsWith("|")) {
      const cells = trimmed.split("|").map((c) => c.trim()).filter((c) => c.length > 0);
      if (cells.length >= 3 && cells[1].includes("MC-LIB")) {
        const nameMatch = cells[0].match(/\[([^\]]+)\]/);
        const name = nameMatch ? nameMatch[1] : cells[0];
        const description = cells[1];
        const relatedModule = cells[2];
        mcLibDocs.push({ name, description, relatedModule });
      }
    }

    if (inStationTable && trimmed === "" && stations.length > 0) {
      inStationTable = false;
    }
    if (inMcLibTable && trimmed === "" && mcLibDocs.length > 0) {
      inMcLibTable = false;
    }
  }

  return { stations, mcLibDocs };
}

function generateQuestionsFile(allQuestions: ParsedQuestion[]): string {
  const lines: string[] = [];
  lines.push("// @generated - DO NOT EDIT MANUALLY");
  lines.push("// Generated by: npm run generate-mc-data");
  lines.push("// Source: motor-control-knowledge-base/**/*-assessment.md");
  lines.push("");
  lines.push('import { MCQuestion } from "@/lib/types";');
  lines.push("");
  lines.push("export const mcQuestions: MCQuestion[] = [");

  for (let i = 0; i < allQuestions.length; i++) {
    const q = allQuestions[i];
    lines.push(`  {`);
    lines.push(`    id: ${i + 1},`);
    lines.push(`    chapter: "${q.chapter}",`);
    lines.push(`    stem: ${JSON.stringify(q.stem)},`);
    lines.push(`    referenceAnswer: ${JSON.stringify(q.referenceAnswer)},`);
    lines.push(`    explanation: ${JSON.stringify(q.explanation)},`);
    lines.push(`    difficulty: ${q.difficulty},`);
    lines.push(`    tags: ${JSON.stringify(q.tags)},`);
    lines.push(`    codeRefs: ${JSON.stringify(q.codeRefs)},`);
    lines.push(`    hint1: ${JSON.stringify(q.hint1)},`);
    lines.push(`    hint2: ${JSON.stringify(q.hint2)},`);
    lines.push(`  },`);
  }

  lines.push("];");
  lines.push("");

  return lines.join("\n");
}

function generateKnowledgeGraphFile(
  questions: ParsedQuestion[],
  algoStations: Array<{ stationNum: number; name: string; description: string; difficulty: string }>,
  hwStations: Array<{ stationNum: number; name: string; description: string; difficulty: string }>,
  algoMcLib: Array<{ name: string; description: string; relatedModule: string }>,
): string {
  const lines: string[] = [];
  lines.push("// @generated - DO NOT EDIT MANUALLY");
  lines.push("// Generated by: npm run generate-mc-data");
  lines.push("// Source: motor-control-knowledge-base README & cross-reference-map.md");
  lines.push("");
  lines.push('import { MCKnowledgeNode, MCKnowledgeEdge } from "@/lib/types";');
  lines.push("");

  const nodes: Array<{
    id: string;
    label: string;
    path: "hardware" | "algorithm";
    chapter: string;
    description: string;
    relatedQuestionIds: number[];
    subTopics: string[];
  }> = [];

  const hwQuestionIndices = questions
    .map((q, i) => ({ q, idx: i + 1 }))
    .filter(({ q }) => q.chapter === "hardware");

  const hwPerStation = hwStations.length > 0 ? Math.ceil(hwQuestionIndices.length / hwStations.length) : 0;

  for (let sIdx = 0; sIdx < hwStations.length; sIdx++) {
    const hw = hwStations[sIdx];
    const nodeId = `hw-station-${hw.stationNum}`;
    const start = sIdx * hwPerStation;
    const end = Math.min(start + hwPerStation, hwQuestionIndices.length);
    const relatedIds = hwQuestionIndices.slice(start, end).map(({ idx }) => idx);

    nodes.push({
      id: nodeId,
      label: hw.name,
      path: "hardware",
      chapter: "hardware",
      description: hw.description,
      relatedQuestionIds: relatedIds,
      subTopics: [],
    });
  }

  const chapterToStation: Record<string, number> = {
    "foc-theory": 1,
    "sensored-foc": 2,
    "sensorless-foc": 3,
    "hfi": 4,
    "protection": 5,
  };

  for (const algo of algoStations) {
    const algoChapterMap: Record<number, string> = {
      1: "foc-theory",
      2: "sensored-foc",
      3: "sensorless-foc",
      4: "hfi",
      5: "protection",
      6: "foc-theory",
    };
    const chapter = algoChapterMap[algo.stationNum] || "foc-theory";

    const relatedIds = questions
      .map((q, i) => ({ q, idx: i + 1 }))
      .filter(({ q }) => q.chapter === chapter)
      .map(({ idx }) => idx);

    const subTopics = algoMcLib
      .filter((d) => d.relatedModule.includes(`ALG-${algo.stationNum}`) || d.relatedModule === "全部")
      .map((d) => d.name);

    nodes.push({
      id: `alg-station-${algo.stationNum}`,
      label: algo.name,
      path: "algorithm",
      chapter,
      description: algo.description,
      relatedQuestionIds: relatedIds,
      subTopics,
    });
  }

  lines.push("export const mcKnowledgeNodes: MCKnowledgeNode[] = [");
  for (const node of nodes) {
    lines.push(`  {`);
    lines.push(`    id: "${node.id}",`);
    lines.push(`    label: "${node.label}",`);
    lines.push(`    path: "${node.path}",`);
    lines.push(`    chapter: "${node.chapter}",`);
    lines.push(`    description: ${JSON.stringify(node.description)},`);
    lines.push(`    relatedQuestionIds: ${JSON.stringify(node.relatedQuestionIds)},`);
    lines.push(`    subTopics: ${JSON.stringify(node.subTopics)},`);
    lines.push(`  },`);
  }
  lines.push("];");
  lines.push("");

  const edges: Array<{ source: string; target: string; relation: string; label?: string }> = [];

  for (let i = 0; i < hwStations.length - 1; i++) {
    edges.push({
      source: `hw-station-${hwStations[i].stationNum}`,
      target: `hw-station-${hwStations[i + 1].stationNum}`,
      relation: "prerequisite",
    });
  }

  for (let i = 0; i < algoStations.length - 1; i++) {
    edges.push({
      source: `alg-station-${algoStations[i].stationNum}`,
      target: `alg-station-${algoStations[i + 1].stationNum}`,
      relation: "prerequisite",
    });
  }

  for (const hw of hwStations) {
    for (const algo of algoStations) {
      edges.push({
        source: `hw-station-${hw.stationNum}`,
        target: `alg-station-${algo.stationNum}`,
        relation: "cross-reference",
        label: `${hw.name} -> ${algo.name}`,
      });
    }
  }

  lines.push("export const mcKnowledgeEdges: MCKnowledgeEdge[] = [");
  for (const edge of edges) {
    if (edge.label) {
      lines.push(`  { source: "${edge.source}", target: "${edge.target}", relation: "${edge.relation}", label: "${edge.label}" },`);
    } else {
      lines.push(`  { source: "${edge.source}", target: "${edge.target}", relation: "${edge.relation}" },`);
    }
  }
  lines.push("];");
  lines.push("");

  return lines.join("\n");
}

function generateOutlineFile(
  algoStations: Array<{ stationNum: number; name: string; description: string; difficulty: string }>,
  hwStations: Array<{ stationNum: number; name: string; description: string; difficulty: string }>,
): string {
  const lines: string[] = [];
  lines.push("// @generated - DO NOT EDIT MANUALLY");
  lines.push("// Generated by: npm run generate-mc-data");
  lines.push("// Source: motor-control-knowledge-base README.md station tables");
  lines.push("");
  lines.push("export interface PdfOutlineItem {");
  lines.push("  label: string;");
  lines.push("  page: number;");
  lines.push("  chapter?: string;");
  lines.push("  description?: string;");
  lines.push("  children?: PdfOutlineItem[];");
  lines.push("}");
  lines.push("");

  const algoStationToChapter: Record<number, string> = {
    1: "foc-theory",
    2: "sensored-foc",
    3: "sensorless-foc",
    4: "hfi",
    5: "protection",
    6: "foc-theory",
  };

  const chapterOutlineNames: Record<string, string> = {
    "foc-theory": "FOC理论基础",
    "sensored-foc": "有感FOC实现",
    "sensorless-foc": "无感FOC观测器",
    "hfi": "高频注入法",
    "protection": "保护与优化",
  };

  const chapterPageStarts: Record<string, number> = {
    "foc-theory": 1,
    "sensored-foc": 41,
    "sensorless-foc": 101,
    "hfi": 161,
    "protection": 221,
  };

  const chapterNextPage: Record<string, number> = {
    "foc-theory": 41,
    "sensored-foc": 101,
    "sensorless-foc": 161,
    "hfi": 221,
    "protection": 281,
  };

  const groupedAlgo: Record<string, typeof algoStations> = {};
  for (const s of algoStations) {
    const chapter = algoStationToChapter[s.stationNum] || "foc-theory";
    if (!groupedAlgo[chapter]) groupedAlgo[chapter] = [];
    groupedAlgo[chapter].push(s);
  }

  const chapterOrder = ["foc-theory", "sensored-foc", "sensorless-foc", "hfi", "protection"];
  let nextHwPage = 281;

  const outlineItems: Array<{
    label: string;
    page: number;
    chapter?: string;
    description?: string;
    children?: Array<{ label: string; page: number; description?: string }>;
  }> = [];

  for (const chapter of chapterOrder) {
    const stations = groupedAlgo[chapter] || [];
    if (stations.length === 0) continue;

    const startPage = chapterPageStarts[chapter] || 1;
    const chapterName = chapterOutlineNames[chapter] || chapter;

    const children = stations.map((s, idx) => {
      const childPage = startPage + idx * 8;
      return {
        label: `${chapterOrder.indexOf(chapter) + 1}.${idx + 1} ${s.name}`,
        page: childPage,
        description: s.description,
      };
    });

    outlineItems.push({
      label: `第${chapterOrder.indexOf(chapter) + 1}章 ${chapterName}`,
      page: startPage,
      chapter,
      children,
    });
  }

  const hwChildren = hwStations.map((s, idx) => ({
    label: `附.${idx + 1} ${s.name}`,
    page: nextHwPage + idx * 17,
    description: s.description,
  }));

  outlineItems.push({
    label: "附录: 硬件基础",
    page: nextHwPage,
    chapter: "hardware",
    children: hwChildren,
  });

  lines.push("export const mcOutline: PdfOutlineItem[] = [");
  for (const item of outlineItems) {
    if (item.children && item.children.length > 0) {
      lines.push(`  {`);
      lines.push(`    label: ${JSON.stringify(item.label)},`);
      lines.push(`    page: ${item.page},`);
      lines.push(`    chapter: ${JSON.stringify(item.chapter)},`);
      lines.push(`    children: [`);
      for (const child of item.children) {
        lines.push(`      {`);
        lines.push(`        label: ${JSON.stringify(child.label)},`);
        lines.push(`        page: ${child.page},`);
        if (child.description) lines.push(`        description: ${JSON.stringify(child.description)},`);
        lines.push(`      },`);
      }
      lines.push(`    ],`);
      lines.push(`  },`);
    } else {
      lines.push(`  {`);
      lines.push(`    label: ${JSON.stringify(item.label)},`);
      lines.push(`    page: ${item.page},`);
      lines.push(`    chapter: ${JSON.stringify(item.chapter)},`);
      lines.push(`  },`);
    }
  }
  lines.push("];");
  lines.push("");

  return lines.join("\n");
}

function main() {
  console.log("MC Data Pipeline: Scanning assessment files...\n");

  const files = scanAssessmentFiles();
  console.log(`Found ${files.length} assessment files:\n`);

  const allQuestions: ParsedQuestion[] = [];
  const chapterCounts: Record<string, number> = {};
  let totalErrors = 0;
  const fileResults: Array<{ file: string; module: string; count: number; errors: number }> = [];

  for (const file of files) {
    const result = parseAssessmentFile(file);
    const relativePath = path.relative(KNOWLEDGE_BASE, file);
    console.log(`  ${relativePath}: ${result.questions.length} questions${result.errors > 0 ? ` (${result.errors} errors)` : ""}`);

    fileResults.push({
      file: relativePath,
      module: result.modulePrefix,
      count: result.questions.length,
      errors: result.errors,
    });

    for (const q of result.questions) {
      allQuestions.push(q);
      chapterCounts[q.chapter] = (chapterCounts[q.chapter] || 0) + 1;
    }
    totalErrors += result.errors;
  }

  console.log(`\nTotal: ${allQuestions.length} questions from ${files.length} files`);
  console.log(`Errors: ${totalErrors}`);

  const errorRate = allQuestions.length > 0 ? (totalErrors / (allQuestions.length + totalErrors)) * 100 : 0;
  console.log(`Error rate: ${errorRate.toFixed(1)}%`);

  console.log("\nQuestions by chapter:");
  for (const [ch, count] of Object.entries(chapterCounts)) {
    console.log(`  ${ch}: ${count}`);
  }

  console.log("\nGenerating output files...");

  const qContent = generateQuestionsFile(allQuestions);
  fs.mkdirSync(DATA_DIR, { recursive: true });
  fs.writeFileSync(OUT_QUESTIONS, qContent, "utf-8");
  console.log(`  -> ${path.relative(process.cwd(), OUT_QUESTIONS)}`);

  const algoReadme = path.join(KNOWLEDGE_BASE, "algorithm", "README.md");
  const hwReadme = path.join(KNOWLEDGE_BASE, "hardware", "README.md");
  const { stations: algoStations, mcLibDocs: algoMcLib } = parseReadmeStationTable(algoReadme);
  const { stations: hwStations } = parseReadmeStationTable(hwReadme);

  const kgContent = generateKnowledgeGraphFile(allQuestions, algoStations, hwStations, algoMcLib);
  fs.writeFileSync(OUT_KNOWLEDGE_GRAPH, kgContent, "utf-8");
  console.log(`  -> ${path.relative(process.cwd(), OUT_KNOWLEDGE_GRAPH)}`);

  const outlineContent = generateOutlineFile(algoStations, hwStations);
  fs.writeFileSync(OUT_OUTLINE, outlineContent, "utf-8");
  console.log(`  -> ${path.relative(process.cwd(), OUT_OUTLINE)}`);

  if (errorRate > 10) {
    console.error("\nERROR: Error rate exceeds 10% threshold. Build failed.");
    console.error("Please check the following files:");
    for (const r of fileResults.filter((r) => r.errors > 0)) {
      console.error(`  - ${r.file}: ${r.errors} parse errors`);
    }
    process.exit(1);
  }

  console.log("\nDone.");
}

main();
