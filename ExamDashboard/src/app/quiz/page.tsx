"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import AppLayout from "@/components/AppLayout";
import { mcQuestions } from "@/data/mc-questions.generated";
import { CHAPTERS, CHAPTER_MAP } from "@/data/chapters";
import type { MCQuestion } from "@/lib/types";
import { difficultyStars, difficultyLabel } from "@/lib/utils";
import {
  Lightbulb,
  ArrowRight,
  ArrowLeft,
  Code,
  X,
  Tag,
  Eye,
  Terminal,
  FileCode,
  CheckCircle2,
  AlertTriangle,
  XCircle,
  ChevronRight,
  RotateCcw,
} from "lucide-react";
import Link from "next/link";
import { renderMarkdown } from "@/lib/markdown";
import "katex/dist/katex.min.css";

type QuizMode = "sequential" | "random" | "wrong" | "weak";
type SelfAssessment = "correct" | "partial" | "incorrect";

const MODE_LABELS: Record<QuizMode, string> = {
  sequential: "顺序",
  random: "随机",
  wrong: "错题",
  weak: "薄弱",
};

const MODE_LIST: QuizMode[] = ["sequential", "random", "wrong", "weak"];

const ASSESSMENT_OPTIONS: { key: SelfAssessment; label: string; icon: typeof CheckCircle2; colorClass: string }[] = [
  { key: "correct", label: "完全正确", icon: CheckCircle2, colorClass: "text-green-400 border-green-500/40 hover:bg-green-500/10" },
  { key: "partial", label: "部分正确", icon: AlertTriangle, colorClass: "text-yellow-400 border-yellow-500/40 hover:bg-yellow-500/10" },
  { key: "incorrect", label: "不正确", icon: XCircle, colorClass: "text-red-400 border-red-500/40 hover:bg-red-500/10" },
];

function getChapterAccuracy(
  answerRecords: Record<number, SelfAssessment>,
  chapterQuestions: MCQuestion[]
): number {
  const chapterIds = new Set(chapterQuestions.map((q) => q.id));
  let total = 0;
  let score = 0;
  for (const [idStr, assessment] of Object.entries(answerRecords)) {
    const id = Number(idStr);
    if (chapterIds.has(id)) {
      total++;
      if (assessment === "correct") score += 1;
      else if (assessment === "partial") score += 0.5;
    }
  }
  return total > 0 ? score / total : 0;
}

export default function QuizPage() {
  const [selectedChapters, setSelectedChapters] = useState<Set<string>>(new Set());
  const [mode, setMode] = useState<QuizMode>("sequential");
  const [currentIdx, setCurrentIdx] = useState(0);
  const [revealed, setRevealed] = useState(false);
  const [assessed, setAssessed] = useState<SelfAssessment | null>(null);
  const [hintLevel, setHintLevel] = useState(0);
  const [wrongSet, setWrongSet] = useState<Set<number>>(new Set());
  const [answerRecords, setAnswerRecords] = useState<Record<number, SelfAssessment>>({});
  const [codePreviewOpen, setCodePreviewOpen] = useState(false);
  const [hydrated, setHydrated] = useState(false);

  useEffect(() => {
    try {
      const ws = localStorage.getItem("mc-wrongset");
      if (ws) setWrongSet(new Set(JSON.parse(ws)));
    } catch { /* ignore */ }
    try {
      const ar = localStorage.getItem("mc-answer-records");
      if (ar) setAnswerRecords(JSON.parse(ar));
    } catch { /* ignore */ }
    setHydrated(true);
  }, []);

  useEffect(() => {
    if (hydrated) {
      localStorage.setItem("mc-wrongset", JSON.stringify(Array.from(wrongSet)));
    }
  }, [wrongSet, hydrated]);

  useEffect(() => {
    if (hydrated) {
      localStorage.setItem("mc-answer-records", JSON.stringify(answerRecords));
    }
  }, [answerRecords, hydrated]);

  const allChaptersSelected = selectedChapters.size === 0;

  const filteredQuestions = useMemo(() => {
    let qs: MCQuestion[] = allChaptersSelected
      ? [...mcQuestions]
      : mcQuestions.filter((q) => selectedChapters.has(q.chapter));

    switch (mode) {
      case "random":
        return [...qs].sort(() => Math.random() - 0.5);
      case "wrong":
        return qs.filter((q) => wrongSet.has(q.id));
      case "weak": {
        const chapterAccuracies: Record<string, number> = {};
        CHAPTERS.forEach((ch) => {
          const chQs = qs.filter((q) => q.chapter === ch.id);
          chapterAccuracies[ch.id] = getChapterAccuracy(answerRecords, chQs);
        });
        return [...qs].sort((a, b) => {
          const accA = chapterAccuracies[a.chapter] ?? 0;
          const accB = chapterAccuracies[b.chapter] ?? 0;
          if (accA !== accB) return accA - accB;
          return a.id - b.id;
        });
      }
      default:
        return qs;
    }
  }, [selectedChapters, mode, wrongSet, answerRecords, allChaptersSelected]);

  const currentQuestion = filteredQuestions[currentIdx] ?? null;
  const hasPrev = currentIdx > 0;
  const hasNext = currentIdx < filteredQuestions.length - 1;

  const currentChapterMeta = currentQuestion
    ? CHAPTER_MAP[currentQuestion.chapter] ?? null
    : null;

  const stats = useMemo(() => {
    const answeredIds = Object.keys(answerRecords).map(Number);
    const totalAnswered = answeredIds.length;
    let totalScore = 0;
    for (const [_, assessment] of Object.entries(answerRecords)) {
      if (assessment === "correct") totalScore += 1;
      else if (assessment === "partial") totalScore += 0.5;
    }
    const accuracy = totalAnswered > 0 ? Math.round((totalScore / totalAnswered) * 100) : 0;
    const correctCount = Object.values(answerRecords).filter((v) => v === "correct").length;
    const partialCount = Object.values(answerRecords).filter((v) => v === "partial").length;
    return { totalAnswered, accuracy, wrongSetSize: wrongSet.size, correctCount, partialCount };
  }, [answerRecords, wrongSet]);

  const renderedStem = useMemo(() => {
    if (!currentQuestion) return "";
    return renderMarkdown(currentQuestion.stem);
  }, [currentQuestion]);

  const renderedAnswer = useMemo(() => {
    if (!currentQuestion) return "";
    return renderMarkdown(currentQuestion.referenceAnswer);
  }, [currentQuestion]);

  const renderedExplanation = useMemo(() => {
    if (!currentQuestion) return "";
    return renderMarkdown(currentQuestion.explanation);
  }, [currentQuestion]);

  const renderedHint1 = useMemo(() => {
    if (!currentQuestion) return "";
    return renderMarkdown(currentQuestion.hint1);
  }, [currentQuestion]);

  const renderedHint2 = useMemo(() => {
    if (!currentQuestion) return "";
    return renderMarkdown(currentQuestion.hint2);
  }, [currentQuestion]);

  const resetQuestionState = useCallback(() => {
    setRevealed(false);
    setAssessed(null);
    setHintLevel(0);
    setCodePreviewOpen(false);
  }, []);

  const navigateTo = useCallback(
    (idx: number) => {
      setCurrentIdx(idx);
      resetQuestionState();
    },
    [resetQuestionState]
  );

  const handlePrev = useCallback(() => {
    if (hasPrev) navigateTo(currentIdx - 1);
  }, [hasPrev, currentIdx, navigateTo]);

  const handleNext = useCallback(() => {
    if (hasNext) navigateTo(currentIdx + 1);
  }, [hasNext, currentIdx, navigateTo]);

  const handleReveal = useCallback(() => {
    setRevealed(true);
  }, []);

  const handleSelfAssess = useCallback(
    (assessment: SelfAssessment) => {
      if (!currentQuestion || assessed) return;
      setAssessed(assessment);
      setAnswerRecords((prev) => ({ ...prev, [currentQuestion.id]: assessment }));
      if (assessment !== "correct") {
        setWrongSet((prev) => new Set(prev).add(currentQuestion.id));
      }
    },
    [currentQuestion, assessed]
  );

  const handleResetAssessment = useCallback(() => {
    if (!currentQuestion) return;
    setAssessed(null);
    setAnswerRecords((prev) => {
      const next = { ...prev };
      delete next[currentQuestion.id];
      return next;
    });
    setWrongSet((prev) => {
      const next = new Set(prev);
      next.delete(currentQuestion.id);
      return next;
    });
  }, [currentQuestion]);

  const toggleChapter = useCallback((chapterId: string) => {
    setSelectedChapters((prev) => {
      const next = new Set(prev);
      if (next.has(chapterId)) {
        next.delete(chapterId);
      } else {
        next.add(chapterId);
      }
      return next;
    });
    setCurrentIdx(0);
    resetQuestionState();
  }, [resetQuestionState]);

  const selectAllChapters = useCallback(() => {
    setSelectedChapters(new Set());
    setCurrentIdx(0);
    resetQuestionState();
  }, [resetQuestionState]);

  const handleModeChange = useCallback(
    (m: QuizMode) => {
      setMode(m);
      setCurrentIdx(0);
      resetQuestionState();
    },
    [resetQuestionState]
  );

  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      if (!currentQuestion) return;
      if (e.key === "Enter" && !revealed) {
        e.preventDefault();
        handleReveal();
      }
      if (e.key === "ArrowRight" && assessed && hasNext) {
        e.preventDefault();
        handleNext();
      }
      if (e.key === "ArrowLeft" && hasPrev) {
        e.preventDefault();
        handlePrev();
      }
      if (e.key === "h" && hintLevel < 2 && !assessed) {
        e.preventDefault();
        setHintLevel((h) => Math.min(h + 1, 2));
      }
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, [currentQuestion, revealed, assessed, hasNext, hasPrev, hintLevel, handleReveal, handleNext, handlePrev]);

  if (!hydrated) {
    return (
      <AppLayout>
        <div className="p-6 max-w-3xl mx-auto">
          <h1 className="text-2xl font-bold">灵活刷题</h1>
          <p className="text-muted-foreground mt-1 text-sm">五大章节领域 · 开放式自评 · 二步提示引导</p>
          <div className="mt-6 p-8 bg-card border border-border rounded-xl animate-pulse">
            <div className="h-4 bg-accent rounded w-3/4 mb-3" />
            <div className="h-4 bg-accent rounded w-1/2" />
          </div>
        </div>
      </AppLayout>
    );
  }

  if (filteredQuestions.length === 0) {
    return (
      <AppLayout>
        <div className="p-6 max-w-3xl mx-auto">
          <h1 className="text-2xl font-bold">灵活刷题</h1>
          <p className="text-muted-foreground mt-1 text-sm">五大章节领域 · 开放式自评 · 二步提示引导</p>

          <div className="mt-3 flex gap-1.5 flex-wrap">
            <button
              onClick={selectAllChapters}
              className={`px-2.5 py-0.5 rounded text-[11px] font-medium border transition-colors ${
                allChaptersSelected
                  ? "bg-accent border-primary/30 text-foreground"
                  : "border-transparent text-muted-foreground hover:text-foreground"
              }`}
            >
              全部
            </button>
            {CHAPTERS.map((ch) => {
              const selected = selectedChapters.has(ch.id);
              return (
                <button
                  key={ch.id}
                  onClick={() => toggleChapter(ch.id)}
                  className={`px-2.5 py-0.5 rounded text-[11px] font-medium border transition-colors ${
                    selected
                      ? "bg-accent border-primary/30 text-foreground"
                      : "border-transparent text-muted-foreground hover:text-foreground"
                  }`}
                  style={{
                    borderColor: selected ? ch.color + "60" : undefined,
                  }}
                >
                  <span className="mr-1">{ch.icon}</span>
                  {ch.shortName}
                </button>
              );
            })}
          </div>

          <div className="mt-4 p-8 bg-card border border-border rounded-xl text-center">
            <p className="text-muted-foreground text-lg">
              {mode === "wrong" ? "错题本为空，继续加油！" : "当前筛选条件下无题目"}
            </p>
            {mode === "wrong" && (
              <button
                onClick={() => handleModeChange("sequential")}
                className="mt-3 px-4 py-1.5 rounded-lg text-xs font-medium bg-primary/10 border border-primary/30 text-primary hover:bg-primary/20 transition-colors"
              >
                切换到顺序刷题
              </button>
            )}
          </div>
        </div>
      </AppLayout>
    );
  }

  const questionNumber = currentIdx + 1;
  const totalQuestions = filteredQuestions.length;
  const stars = currentQuestion ? difficultyStars(currentQuestion.difficulty) : "";
  const dLabel = currentQuestion ? difficultyLabel(currentQuestion.difficulty) : "";

  return (
    <AppLayout>
      <div className="p-6 max-w-3xl mx-auto">
        {/* 标题行 */}
        <div className="flex items-center gap-3 flex-wrap">
          <h1 className="text-2xl font-bold">灵活刷题</h1>
          <span className="text-xs text-muted-foreground">开放式自评 · 二步提示 · 代码引用</span>
        </div>

        {/* 统计栏 */}
        <div className="mt-2 flex items-center gap-4 text-xs text-muted-foreground flex-wrap bg-card/60 border border-border/60 rounded-lg px-3 py-1.5">
          <span>
            已答 <strong className="text-foreground">{stats.totalAnswered}</strong> 题
          </span>
          <span>
            正确率{" "}
            <strong
              className={
                stats.accuracy >= 80
                  ? "text-green-400"
                  : stats.accuracy >= 50
                  ? "text-yellow-400"
                  : "text-red-400"
              }
            >
              {stats.accuracy}%
            </strong>
          </span>
          <span>
            错题{" "}
            <strong className="text-red-400">{stats.wrongSetSize}</strong> 题
          </span>
          {currentChapterMeta && (
            <span className="flex items-center gap-1">
              当前章节:
              <span
                className="px-1.5 py-0.5 rounded text-[10px] font-medium"
                style={{
                  backgroundColor: currentChapterMeta.color + "20",
                  color: currentChapterMeta.color,
                }}
              >
                {currentChapterMeta.icon} {currentChapterMeta.shortName}
              </span>
            </span>
          )}
        </div>

        {/* 章节筛选 chips */}
        <div className="mt-3 flex gap-1.5 flex-wrap">
          <button
            onClick={selectAllChapters}
            className={`px-2.5 py-0.5 rounded text-[11px] font-medium border transition-colors ${
              allChaptersSelected
                ? "bg-accent border-primary/30 text-foreground"
                : "border-transparent text-muted-foreground hover:text-foreground"
            }`}
          >
            全部
          </button>
          {CHAPTERS.map((ch) => {
            const selected = selectedChapters.has(ch.id);
            return (
              <button
                key={ch.id}
                onClick={() => toggleChapter(ch.id)}
                className={`px-2.5 py-0.5 rounded text-[11px] font-medium border transition-colors ${
                  selected
                    ? "bg-accent border-primary/40 text-foreground"
                    : "border-transparent text-muted-foreground hover:text-foreground"
                }`}
                style={{
                  borderColor: selected ? ch.color + "60" : undefined,
                }}
              >
                <span className="mr-1">{ch.icon}</span>
                {ch.shortName}
              </button>
            );
          })}
        </div>

        {/* 模式切换 */}
        <div className="mt-2 flex items-center gap-1">
          <span className="text-[10px] text-muted-foreground mr-1">模式:</span>
          {MODE_LIST.map((m) => (
            <button
              key={m}
              onClick={() => handleModeChange(m)}
              className={`px-2 py-0.5 rounded text-[10px] font-medium border transition-colors ${
                mode === m
                  ? "bg-accent border-primary/30 text-foreground"
                  : "border-transparent text-muted-foreground hover:text-foreground"
              }`}
            >
              {MODE_LABELS[m]}
            </button>
          ))}
        </div>

        {/* 题目卡片 */}
        <div className="mt-4 bg-card border border-border rounded-xl overflow-hidden">
          {/* 题目头部 */}
          <div className="px-5 py-3 border-b border-border flex items-center gap-2 flex-wrap bg-card/70">
            <span className="text-xs text-muted-foreground font-mono">
              第 {questionNumber} / {totalQuestions} 题
            </span>
            {currentChapterMeta && (
              <span
                className="inline-flex items-center gap-1 px-1.5 py-0.5 rounded text-[10px] font-medium"
                style={{
                  backgroundColor: currentChapterMeta.color + "20",
                  color: currentChapterMeta.color,
                }}
              >
                {currentChapterMeta.icon} {currentChapterMeta.shortName}
              </span>
            )}
            <span className="text-[10px] text-muted-foreground ml-auto">
              <span className="text-yellow-400">{stars}</span>{" "}
              <span className="ml-0.5">{dLabel}</span>
            </span>
            <span className="text-[10px] text-muted-foreground bg-accent/50 px-1.5 py-0.5 rounded">
              ID: {currentQuestion!.id}
            </span>
          </div>

          {/* 题干 */}
          <div className="px-5 py-4">
            <div
              className="text-sm leading-relaxed prose-sm dark:prose-invert max-w-none katex-content"
              dangerouslySetInnerHTML={{ __html: renderedStem }}
            />
          </div>

          {/* 知识标签 */}
          {currentQuestion!.tags.length > 0 && (
            <div className="px-5 pb-2 flex items-center gap-1.5 flex-wrap">
              <Tag className="h-3 w-3 text-muted-foreground shrink-0" />
              {currentQuestion!.tags.map((tag) => (
                <Link
                  key={tag}
                  href={`/knowledge-graph?tag=${encodeURIComponent(tag)}`}
                  className="px-1.5 py-0.5 rounded text-[10px] border border-border/60 bg-accent/40 text-muted-foreground hover:text-foreground hover:border-primary/30 transition-colors no-underline"
                >
                  {tag}
                </Link>
              ))}
            </div>
          )}

          {/* 代码引用按钮 */}
          {currentQuestion!.codeRefs.length > 0 && (
            <div className="px-5 pb-2">
              <button
                onClick={() => setCodePreviewOpen(!codePreviewOpen)}
                className="inline-flex items-center gap-1 px-2.5 py-1 rounded text-[11px] font-medium bg-slate-100 border border-slate-200 text-slate-600 hover:text-slate-800 hover:border-slate-300 transition-colors"
              >
                <Code className="h-3 w-3" />
                查看代码 ({currentQuestion!.codeRefs.length})
                <ChevronRight
                  className={`h-3 w-3 transition-transform ${codePreviewOpen ? "rotate-90" : ""}`}
                />
              </button>
            </div>
          )}

          {/* 代码预览面板 */}
          {codePreviewOpen && currentQuestion!.codeRefs.length > 0 && (
            <div className="mx-5 mb-3 rounded-lg overflow-hidden border border-slate-200">
              <div className="flex items-center gap-2 px-3 py-1.5 bg-slate-50 border-b border-slate-200">
                <Terminal className="h-3.5 w-3.5 text-emerald-600" />
                <span className="text-[10px] text-slate-500 font-mono tracking-wide">
                  代码引用
                </span>
                <button
                  onClick={() => setCodePreviewOpen(false)}
                  className="ml-auto text-slate-400 hover:text-slate-600 transition-colors"
                >
                  <X className="h-3 w-3" />
                </button>
              </div>
              <div className="px-4 py-3 bg-slate-50/80 font-mono text-xs space-y-2">
                {currentQuestion!.codeRefs.map((ref, i) => (
                  <div key={i} className="flex items-start gap-2">
                    <FileCode className="h-3.5 w-3.5 text-green-400 mt-0.5 shrink-0" />
                    <div>
                      <span className="text-green-400">{ref}</span>
                      <p className="text-gray-600 mt-0.5 leading-relaxed">
                        {"/* "}C 源码文件 — 请参见项目仓库{" */"}
                      </p>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* 提示区 */}
          <div className="px-5 pb-3 space-y-2">
            {hintLevel >= 1 && currentQuestion!.hint1 && (
              <div className="p-3 rounded-lg bg-yellow-500/10 border border-yellow-500/20 text-xs animate-in fade-in slide-in-from-top-1">
                <Lightbulb className="h-3.5 w-3.5 inline mr-1 text-yellow-400" />
                <span className="text-yellow-500/70 font-medium mr-1">提示一：</span>
                <span
                  className="text-muted-foreground"
                  dangerouslySetInnerHTML={{ __html: renderedHint1 }}
                />
              </div>
            )}
            {hintLevel >= 2 && currentQuestion!.hint2 && (
              <div className="p-3 rounded-lg bg-yellow-500/10 border border-yellow-500/20 text-xs animate-in fade-in slide-in-from-top-1">
                <Lightbulb className="h-3.5 w-3.5 inline mr-1 text-yellow-400" />
                <span className="text-yellow-500/70 font-medium mr-1">提示二：</span>
                <span
                  className="text-muted-foreground"
                  dangerouslySetInnerHTML={{ __html: renderedHint2 }}
                />
              </div>
            )}
          </div>

          {/* 参考答案区域 */}
          {revealed && (
            <div className="px-5 pb-3">
              <div className="p-4 rounded-lg bg-accent/50 border border-border animate-in fade-in slide-in-from-top-2">
                <div className="flex items-center gap-1.5 mb-2">
                  <Eye className="h-3.5 w-3.5 text-primary" />
                  <span className="text-xs font-medium text-foreground">参考答案</span>
                </div>
                <div
                  className="text-xs leading-relaxed text-muted-foreground katex-content"
                  dangerouslySetInnerHTML={{ __html: renderedAnswer }}
                />
              </div>
            </div>
          )}

          {/* 答案解析（自评后显示） */}
          {assessed && (
            <div className="px-5 pb-3">
              <div
                className={`p-4 rounded-lg border animate-in fade-in slide-in-from-top-2 ${
                  assessed === "correct"
                    ? "bg-green-500/10 border-green-500/20"
                    : assessed === "partial"
                    ? "bg-yellow-500/10 border-yellow-500/20"
                    : "bg-red-500/10 border-red-500/20"
                }`}
              >
                <p className="text-xs font-medium mb-1.5">
                  {assessed === "correct" && (
                    <span className="text-green-400">自评：完全正确 — 干得漂亮！</span>
                  )}
                  {assessed === "partial" && (
                    <span className="text-yellow-400">自评：部分正确 — 继续加油！</span>
                  )}
                  {assessed === "incorrect" && (
                    <span className="text-red-400">自评：不正确 — 仔细阅读解析！</span>
                  )}
                </p>
                <div
                  className="text-xs leading-relaxed text-muted-foreground katex-content"
                  dangerouslySetInnerHTML={{ __html: renderedExplanation }}
                />
              </div>
            </div>
          )}

          {/* 操作栏 */}
          <div className="px-5 py-3 border-t border-border flex items-center gap-2 flex-wrap bg-card/40">
            {/* 上一题 */}
            <button
              onClick={handlePrev}
              disabled={!hasPrev}
              className="inline-flex items-center gap-1 px-3 py-1.5 rounded-lg text-xs font-medium border border-border text-muted-foreground hover:text-foreground hover:border-border/80 disabled:opacity-30 transition-colors"
            >
              <ArrowLeft className="h-3.5 w-3.5" />
              上一题
            </button>

            {/* 提示按钮 */}
            {!assessed && hintLevel < 2 && (
              <button
                onClick={() => setHintLevel((h) => Math.min(h + 1, 2))}
                className="inline-flex items-center gap-1 px-3 py-1.5 rounded-lg text-xs font-medium bg-yellow-500/10 border border-yellow-500/20 text-yellow-600 hover:bg-yellow-500/20 transition-colors"
              >
                <Lightbulb className="h-3.5 w-3.5" />
                提示 ({2 - hintLevel})
              </button>
            )}

            <div className="ml-auto flex items-center gap-2">
              {/* 未揭示答案：显示"查看答案"按钮 */}
              {!revealed && (
                <button
                  onClick={handleReveal}
                  className="inline-flex items-center gap-1 px-4 py-1.5 rounded-lg text-xs font-medium bg-primary text-primary-foreground hover:bg-primary/90 transition-colors"
                >
                  <Eye className="h-3.5 w-3.5" />
                  查看答案
                </button>
              )}

              {/* 已揭示但未自评：显示自评按钮 */}
              {revealed && !assessed && (
                <div className="flex items-center gap-1.5">
                  <span className="text-[10px] text-muted-foreground mr-1">自评:</span>
                  {ASSESSMENT_OPTIONS.map((opt) => {
                    const Icon = opt.icon;
                    return (
                      <button
                        key={opt.key}
                        onClick={() => handleSelfAssess(opt.key)}
                        className={`inline-flex items-center gap-1 px-2.5 py-1 rounded-lg text-[11px] font-medium border transition-colors ${opt.colorClass}`}
                      >
                        <Icon className="h-3.5 w-3.5" />
                        {opt.label}
                      </button>
                    );
                  })}
                </div>
              )}

              {/* 已自评：显示重新评定 + 下一题 */}
              {assessed && (
                <div className="flex items-center gap-2">
                  <button
                    onClick={handleResetAssessment}
                    className="inline-flex items-center gap-1 px-2.5 py-1.5 rounded-lg text-[11px] font-medium border border-border text-muted-foreground hover:text-foreground transition-colors"
                  >
                    <RotateCcw className="h-3 w-3" />
                    重新评定
                  </button>
                  <button
                    onClick={handleNext}
                    disabled={!hasNext}
                    className="inline-flex items-center gap-1 px-4 py-1.5 rounded-lg text-xs font-medium bg-primary text-primary-foreground hover:bg-primary/90 disabled:opacity-30 transition-colors"
                  >
                    下一题
                    <ArrowRight className="h-3.5 w-3.5" />
                  </button>
                </div>
              )}
            </div>
          </div>

          {/* 键盘快捷键提示 */}
          <div className="px-5 py-2 border-t border-border/50 text-[10px] text-muted-foreground flex items-center gap-3 flex-wrap bg-card/20">
            <span>
              <kbd className="px-1 py-0.5 rounded bg-accent border border-border text-[10px] font-mono">
                Enter
              </kbd>{" "}
              查看答案
            </span>
            <span>
              <kbd className="px-1 py-0.5 rounded bg-accent border border-border text-[10px] font-mono">
                &rarr;
              </kbd>{" "}
              下一题
            </span>
            <span>
              <kbd className="px-1 py-0.5 rounded bg-accent border border-border text-[10px] font-mono">
                &larr;
              </kbd>{" "}
              上一题
            </span>
            <span>
              <kbd className="px-1 py-0.5 rounded bg-accent border border-border text-[10px] font-mono">
                H
              </kbd>{" "}
              提示
            </span>
          </div>
        </div>
      </div>
    </AppLayout>
  );
}
