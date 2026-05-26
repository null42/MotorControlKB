"use client";

import { useEffect, useMemo, useState } from "react";
import dynamic from "next/dynamic";
import Link from "next/link";
import AppLayout from "@/components/AppLayout";
import { mcKnowledgeNodes, mcKnowledgeEdges } from "@/data/mc-knowledgeGraph.generated";
import { CHAPTERS } from "@/data/chapters";
import { getChapterStats } from "@/data/studyRecords";
import type { MCKnowledgeNode, MCKnowledgeEdge, StudyRecord } from "@/lib/types";
import { X, Network, BookOpen } from "lucide-react";

const CytoscapeGraph = dynamic(() => import("./CytoscapeGraph"), {
  ssr: false,
  loading: () => (
    <div className="flex-1 flex items-center justify-center text-muted-foreground text-sm">
      加载图谱引擎...
    </div>
  ),
});

const NODE_DOC_MAP: Record<string, string> = {
  "hw-station-1": "hardware/HW-01-Motor-Basics.md",
  "hw-station-2": "hardware/HW-02-Current-Sensing.md",
  "hw-station-3": "hardware/HW-03-Position-Sensor.md",
  "hw-station-4": "hardware/HW-04-MCU-Peripherals.md",
  "hw-station-5": "hardware/HW-05-Power-Devices-Gate-Drivers.md",
  "hw-station-6": "hardware/HW-06-Power-Management-Protection.md",
  "hw-station-7": "hardware/HW-07-Thermal-EMC-Design.md",
  "alg-station-1": "algorithm/ALG-01-FOC-Theory.md",
  "alg-station-2": "algorithm/ALG-02-Sensored-FOC.md",
  "alg-station-3": "algorithm/ALG-03-Sensorless-Observers.md",
  "alg-station-4": "algorithm/ALG-04-High-Frequency-Injection.md",
  "alg-station-5": "algorithm/ALG-05-Protection-Optimization.md",
  "alg-station-6": "algorithm/ALG-06-Advanced-Research.md",
};

const CHAPTER_COLORS: Record<string, string> = {};
for (const ch of CHAPTERS) {
  CHAPTER_COLORS[ch.id] = ch.color;
}
CHAPTER_COLORS["hardware"] = "#ffb74d";

const CHAPTER_SHORT_NAMES: Record<string, string> = {};
for (const ch of CHAPTERS) {
  CHAPTER_SHORT_NAMES[ch.id] = ch.shortName;
}
CHAPTER_SHORT_NAMES["hardware"] = "硬件";

const PATH_LABELS: Record<string, string> = {
  algorithm: "算法路径",
  hardware: "硬件路径",
};

type FilterValue = "all" | "algorithm" | "hardware";

function getAccuracyColor(acc: number): string {
  if (acc >= 0.8) return "#4ade80";
  if (acc >= 0.5) return "#facc15";
  if (acc > 0) return "#f87171";
  return "#6b7280";
}

function getReviewAdvice(acc: number): string {
  if (acc >= 0.8) return "该知识点掌握良好，保持定期复习即可。";
  if (acc >= 0.5) return "部分掌握，建议针对关联题目进行专项练习。";
  if (acc > 0) return "掌握不足，请回顾教材对应章节并完成所有关联题目。";
  return "尚未开始学习，建议从基础入手逐步攻克。";
}

export default function KnowledgeGraphPage() {
  const [filter, setFilter] = useState<FilterValue>("all");
  const [selectedNode, setSelectedNode] = useState<MCKnowledgeNode | null>(null);
  const [chapterStats, setChapterStats] = useState<ReturnType<typeof getChapterStats>>({});

  useEffect(() => {
    try {
      const raw = localStorage.getItem("mc-answer-records");
      if (!raw) { setChapterStats({}); return; }
      const answerMap: Record<number, string> = JSON.parse(raw);
      const today = new Date().toISOString().split("T")[0];
      const records: StudyRecord[] = [];
      for (const [qIdStr, assessment] of Object.entries(answerMap)) {
        const questionId = parseInt(qIdStr);
        records.push({ date: today, questionId, correct: assessment === "correct", chapter: "unknown" });
      }
      for (const node of mcKnowledgeNodes) {
        for (const rec of records) {
          if (node.relatedQuestionIds.includes(rec.questionId)) rec.chapter = node.chapter;
        }
      }
      setChapterStats(getChapterStats(records));
    } catch {
      setChapterStats({});
    }
  }, []);

  const nodeAccuracy = useMemo(() => {
    const map: Record<string, number> = {};
    for (const node of mcKnowledgeNodes) {
      const stats = chapterStats[node.chapter];
      map[node.id] = stats ? stats.rate : 0;
    }
    return map;
  }, [chapterStats]);

  const filteredElements = useMemo(() => {
    let nodes = mcKnowledgeNodes;
    let edges = mcKnowledgeEdges;

    if (filter === "algorithm") {
      nodes = nodes.filter((n) => n.path === "algorithm");
    } else if (filter === "hardware") {
      nodes = nodes.filter((n) => n.path === "hardware");
    }

    edges = edges.filter(
      (e) => nodes.some((n) => n.id === e.source) && nodes.some((n) => n.id === e.target)
    );

    const cyNodes = nodes.map((n) => {
      const acc = nodeAccuracy[n.id] ?? 0;
      const color = getAccuracyColor(acc);
      return {
        data: {
          id: n.id, label: n.label, path: n.path, chapter: n.chapter,
          accuracy: Math.round(acc * 100), relatedQuestions: n.relatedQuestionIds.length,
        },
        style: {
          "background-color": color, "border-color": CHAPTER_COLORS[n.chapter] || "#6b7280",
          "border-width": 2, color: "#1a1a2e", label: n.label,
          "text-valign": "center", "text-halign": "center",
          "font-size": 11, "text-wrap": "wrap", "text-max-width": 80,
          width: 55, height: 55, shape: "ellipse",
        },
      };
    });

    const cyEdges = edges.map((e) => {
      const hasLabel = e.relation === "cross-reference" && !!e.label;
      return {
        data: { source: e.source, target: e.target, relation: e.relation, label: e.label || "" },
        style: {
          "line-color":
            e.relation === "prerequisite" ? "hsl(220, 15%, 40%)" :
            e.relation === "cross-reference" ? "hsl(35, 60%, 50%)" : "hsl(220, 15%, 25%)",
          "line-style": e.relation === "prerequisite" || e.relation === "cross-reference" ? "solid" : "dashed",
          "line-dash-pattern": e.relation === "related" ? [6, 3] : undefined,
          "target-arrow-shape": "triangle",
          "target-arrow-color": e.relation === "cross-reference" ? "hsl(35, 60%, 50%)" : "hsl(220, 15%, 40%)",
          "arrow-scale": 0.8, width: e.relation === "cross-reference" ? 2 : 1.5,
          ...(hasLabel ? {
            label: e.label || "", "font-size": 9, "text-background-color": "hsl(220, 15%, 6%)",
            "text-background-opacity": 0.85, "text-background-padding": 2,
            color: "hsl(35, 40%, 70%)", "text-wrap": "wrap", "text-max-width": 120,
            "edge-text-rotation": "autorotate",
          } : {}),
        },
        classes: e.relation,
      };
    });

    return { nodes: cyNodes, edges: cyEdges };
  }, [filter, nodeAccuracy]);

  const handleSelectNode = (node: MCKnowledgeNode | null) => {
    setSelectedNode(node);
  };

  return (
    <AppLayout>
      <div className="h-full flex">
        <div className="flex-1 flex flex-col min-w-0">
          <div className="px-4 py-3 border-b border-border flex items-center gap-2 bg-card/50 flex-wrap">
            <Network className="h-5 w-5 text-primary shrink-0" />
            <h1 className="text-lg font-bold shrink-0">电机控制知识图谱</h1>
            <span className="text-muted-foreground text-xs shrink-0">硬件·算法 双路径知识网络</span>

            <div className="ml-auto flex gap-1 flex-wrap">
              {(["all", "algorithm", "hardware"] as FilterValue[]).map((fv) => (
                <button key={fv} onClick={() => setFilter(fv)}
                  className={`px-3 py-1 rounded-lg text-xs font-medium border transition-colors ${
                    filter === fv
                      ? "bg-accent border-primary/30 text-foreground"
                      : "border-transparent text-muted-foreground hover:text-foreground"
                  }`}
                >
                  {fv === "all" ? "全部" : fv === "algorithm" ? "算法路径" : "硬件路径"}
                </button>
              ))}
            </div>

            <div className="flex items-center gap-2 ml-4 text-[10px] text-muted-foreground shrink-0">
              <span className="inline-block w-2.5 h-2.5 rounded-sm bg-green-400" /> 掌握
              <span className="inline-block w-2.5 h-2.5 rounded-sm bg-yellow-400" /> 一般
              <span className="inline-block w-2.5 h-2.5 rounded-sm bg-red-400" /> 薄弱
              <span className="inline-block w-2.5 h-2.5 rounded-sm bg-gray-500" /> 未学
              <span className="mx-1 text-border">|</span>
              <span className="font-medium">边:</span>
              <span className="inline-block w-3 h-0.5 bg-[hsl(220,15%,40%)]" /> 实线(先修/跨域)
              <span className="inline-block w-3 h-0 border-t border-dashed border-[hsl(220,15%,25%)]" /> 虚线(相关)
            </div>
          </div>

          <CytoscapeGraph
            elements={filteredElements}
            knowledgeNodes={mcKnowledgeNodes}
            onSelectNode={handleSelectNode}
          />
        </div>

        {selectedNode && (
          <aside className="w-80 border-l border-border bg-card p-4 flex flex-col gap-3 overflow-auto shrink-0">
            <div className="flex items-center justify-between">
              <h3 className="font-semibold text-sm">{selectedNode.label}</h3>
              <button onClick={() => setSelectedNode(null)} className="text-muted-foreground hover:text-foreground">
                <X className="h-4 w-4" />
              </button>
            </div>

            <div className="text-xs text-muted-foreground space-y-2">
              <div className="flex gap-1.5 flex-wrap">
                <span className="inline-block px-1.5 py-0.5 rounded text-[10px] font-medium"
                  style={{
                    backgroundColor: (CHAPTER_COLORS[selectedNode.chapter] || "#6b7280") + "20",
                    color: CHAPTER_COLORS[selectedNode.chapter] || "#6b7280",
                  }}
                >
                  {CHAPTER_SHORT_NAMES[selectedNode.chapter] || selectedNode.chapter}
                </span>
                <span className="inline-block px-1.5 py-0.5 rounded text-[10px] font-medium bg-accent text-muted-foreground">
                  {PATH_LABELS[selectedNode.path] || selectedNode.path}
                </span>
              </div>

              {selectedNode.description && (
                <p className="text-[11px] leading-relaxed">{selectedNode.description}</p>
              )}

              <p>
                当前正确率：
                <span className="font-bold text-foreground">{Math.round((nodeAccuracy[selectedNode.id] ?? 0) * 100)}%</span>
              </p>

              <p>
                关联题目：
                <Link href={`/exam?ids=${selectedNode.relatedQuestionIds.join(",")}`}
                  className="font-semibold text-blue-400 hover:underline"
                >
                  {selectedNode.relatedQuestionIds.length} 道 → 前往刷题
                </Link>
              </p>

              {selectedNode.subTopics.length > 0 && (
                <div>
                  <p className="font-medium text-foreground mb-1">子知识点：</p>
                  <div className="flex flex-wrap gap-1">
                    {selectedNode.subTopics.map((st) => (
                      <span key={st} className="inline-block px-1.5 py-0.5 rounded bg-accent text-[10px]">{st}</span>
                    ))}
                  </div>
                </div>
              )}

              <div className="mt-2 p-2 rounded bg-accent/50 border border-border">
                <p className="font-medium text-foreground text-[11px]">复习建议</p>
                <p className="text-[11px] mt-1">{getReviewAdvice(nodeAccuracy[selectedNode.id] ?? 0)}</p>
              </div>

              {NODE_DOC_MAP[selectedNode.id] && (
                <Link href={`/pdf-reader?doc=${encodeURIComponent(NODE_DOC_MAP[selectedNode.id])}`}
                  className="flex items-center justify-center gap-1.5 mt-1 px-3 py-2 rounded-lg bg-primary/10 border border-primary/20 text-xs font-medium text-primary hover:bg-primary/20 transition-colors"
                >
                  <BookOpen className="h-3.5 w-3.5" /> 查看知识库文档
                </Link>
              )}
            </div>
          </aside>
        )}
      </div>
    </AppLayout>
  );
}
