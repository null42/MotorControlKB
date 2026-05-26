export type Subject =
  | "foc-theory"
  | "sensored-foc"
  | "sensorless-foc"
  | "hfi"
  | "protection"
  | "hardware";

export const SUBJECT_LABELS: Record<Subject, string> = {
  "foc-theory": "FOC理论基础",
  "sensored-foc": "有感FOC实现",
  "sensorless-foc": "无感FOC观测器",
  "hfi": "高频注入法",
  "protection": "保护与优化",
  "hardware": "硬件基础",
};

export const SUBJECT_COLORS: Record<Subject, string> = {
  "foc-theory": "#4fc3f7",
  "sensored-foc": "#ff8a65",
  "sensorless-foc": "#81c784",
  "hfi": "#ba68c8",
  "protection": "#ef5350",
  "hardware": "#ffb74d",
};

export interface Question {
  id: number;
  subject: Subject;
  stem: string;
  options: string[];
  answer: number;
  explanation: string;
  tags: string[];
  hint1: string;
  hint2: string;
  docPage: number;
}

export interface StudyRecord {
  date: string;
  questionId: number;
  correct: boolean;
  chapter: string;
}

export interface KnowledgeNode {
  id: string;
  label: string;
  path: "hardware" | "algorithm";
  subject: Subject;
  relatedQuestions: number[];
}

export interface KnowledgeEdge {
  source: string;
  target: string;
  relation: "prerequisite" | "related" | "cross-reference";
}

export interface MCQuestion {
  id: number;
  chapter: string;
  stem: string;
  referenceAnswer: string;
  explanation: string;
  difficulty: number;
  tags: string[];
  codeRefs: string[];
  hint1: string;
  hint2: string;
}

export interface ChapterMeta {
  id: string;
  name: string;
  shortName: string;
  color: string;
  icon: string;
  order: number;
  description: string;
}

export interface MCKnowledgeNode {
  id: string;
  label: string;
  path: "hardware" | "algorithm";
  chapter: string;
  description: string;
  relatedQuestionIds: number[];
  subTopics: string[];
}

export interface MCKnowledgeEdge {
  source: string;
  target: string;
  relation: "prerequisite" | "related" | "cross-reference";
  label?: string;
}
