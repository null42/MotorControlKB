"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import dynamic from "next/dynamic";
import Link from "next/link";
import AppLayout from "@/components/AppLayout";
import { CHAPTERS, CHAPTER_MAP } from "@/data/chapters";
import {
  getHeatmapData,
  getChapterStats,
  getWeakChapters,
} from "@/data/studyRecords";
import { mcQuestions } from "@/data/mc-questions.generated";
import type { StudyRecord } from "@/lib/types";
import {
  ChevronLeft,
  ChevronRight,
  Flame,
  AlertTriangle,
  ArrowRight,
} from "lucide-react";

const ReactECharts = dynamic(() => import("echarts-for-react"), { ssr: false });

function getChapterForId(questionId: number): string {
  if (questionId <= 20) return "foc-theory";
  if (questionId <= 40) return "sensored-foc";
  if (questionId <= 60) return "sensorless-foc";
  if (questionId <= 80) return "hfi";
  return "protection";
}

export default function DashboardPage() {
  const [records, setRecords] = useState<StudyRecord[]>([]);
  const [selectedYear, setSelectedYear] = useState(new Date().getFullYear());
  const [selectedDate, setSelectedDate] = useState<string | null>(null);

  useEffect(() => {
    try {
      const raw = localStorage.getItem("mc-answer-records");
      if (!raw) { setRecords([]); return; }
      const answerMap: Record<number, string> = JSON.parse(raw);
      const today = new Date().toISOString().split("T")[0];
      const result: StudyRecord[] = [];
      for (const [qIdStr, assessment] of Object.entries(answerMap)) {
        const questionId = parseInt(qIdStr);
        const question = mcQuestions.find((q) => q.id === questionId);
        result.push({
          date: today,
          questionId,
          correct: assessment === "correct",
          chapter: question?.chapter || "",
        });
      }
      setRecords(result);
    } catch {
      setRecords([]);
    }
  }, []);

  const heatmapData = useMemo(() => getHeatmapData(records), [records]);
  const totalSolved = useMemo(
    () => heatmapData.reduce((s, [, c]) => s + c, 0),
    [heatmapData],
  );

  const subjectAccuracy = useMemo(() => {
    const map: Record<string, { correct: number; total: number }> = {};
    for (const ch of CHAPTERS) {
      map[ch.id] = { correct: 0, total: 0 };
    }
    for (const r of records) {
      const subj = getChapterForId(r.questionId);
      map[subj].total++;
      if (r.correct) map[subj].correct++;
    }
    return map;
  }, [records]);

  const chapterStats = useMemo(
    () => getChapterStats(records),
    [records],
  );

  const weakChapterIds = useMemo(
    () => getWeakChapters(records, 3),
    [records],
  );

  const dailyDetail = useMemo(() => {
    if (!selectedDate) return [];
    return records.filter((r) => r.date === selectedDate);
  }, [records, selectedDate]);

  const yearHeatmapData = useMemo(() => {
    return heatmapData.filter(([d]) => d.startsWith(String(selectedYear)));
  }, [heatmapData, selectedYear]);

  const heatmapOption = useMemo(() => {
    const maxVal = Math.max(...yearHeatmapData.map(([, v]) => v), 1);

    return {
      tooltip: {
        backgroundColor: "hsl(220 15% 10%)",
        borderColor: "hsl(220 15% 20%)",
        textStyle: { color: "#e2e8f0", fontSize: 13 },
        formatter: (p: { data: [string, number] }) => {
          const [date, count] = p.data;
          return `${date}<br/>刷题数: <b>${count}</b>`;
        },
      },
      calendar: {
        top: 20,
        left: 30,
        right: 30,
        range: String(selectedYear),
        cellSize: [18, 18],
        splitLine: { lineStyle: { color: "hsl(220 15% 12%)" } },
        itemStyle: {
          borderColor: "hsl(220 15% 8%)",
          borderWidth: 2,
          borderRadius: 2,
        },
        dayLabel: { color: "hsl(220 15% 50%)", fontSize: 10 },
        monthLabel: { color: "hsl(220 15% 50%)", fontSize: 11 },
        yearLabel: { show: false },
      },
      visualMap: {
        min: 0,
        max: maxVal,
        orient: "horizontal",
        left: "center",
        bottom: 0,
        itemWidth: 12,
        itemHeight: 12,
        textStyle: { color: "hsl(220 15% 60%)" },
        inRange: {
          color: [
            "hsl(220 15% 12%)",
            "#1a472a",
            "#006d32",
            "#26a641",
            "#39d353",
          ],
        },
      },
      series: [
        {
          type: "heatmap",
          coordinateSystem: "calendar",
          data: yearHeatmapData,
          emphasis: {
            itemStyle: { shadowBlur: 10, shadowColor: "rgba(0,0,0,0.5)" },
          },
        },
      ],
    };
  }, [yearHeatmapData, selectedYear]);

  const gaugeOption = useMemo(() => {
    const series = CHAPTERS.map((ch, i) => {
      const { correct, total } = subjectAccuracy[ch.id] ?? {
        correct: 0,
        total: 0,
      };
      const rate = total > 0 ? Math.round((correct / total) * 100) : 0;
      return {
        type: "gauge" as const,
        center: [`${10 + i * 18}%`, "55%"],
        radius: "70%",
        startAngle: 200,
        endAngle: -20,
        min: 0,
        max: 100,
        splitNumber: 5,
        axisLine: {
          lineStyle: {
            width: 10,
            color: [
              [rate / 100, ch.color],
              [1, "hsl(220 15% 15%)"],
            ],
          },
        },
        pointer: { show: false },
        axisTick: { show: false },
        splitLine: { show: false },
        axisLabel: { show: false },
        detail: {
          formatter: "{value}%",
          fontSize: 14,
          fontWeight: "bold",
          color: ch.color,
          offsetCenter: [0, "60%"],
        },
        title: {
          offsetCenter: [0, "90%"],
          fontSize: 12,
          color: "hsl(220 15% 60%)",
        },
        data: [{ value: rate, name: ch.shortName }],
      };
    });

    return {
      backgroundColor: "transparent",
      series,
    };
  }, [subjectAccuracy]);

  const onChartClick = useCallback(
    (params: { data?: [string, number] }) => {
      if (params.data?.[0]) {
        setSelectedDate(params.data[0]);
      }
    },
    [],
  );

  return (
    <AppLayout>
      <div className="p-6 space-y-6 max-w-7xl mx-auto">
        {/* 头部 */}
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-2xl font-bold flex items-center gap-2">
              <Flame className="h-6 w-6 text-orange-400" />
              电控知识仪表盘
            </h1>
            <p className="text-muted-foreground mt-1">
              五大章节学习进度追踪，薄弱环节一目了然
            </p>
          </div>
        </div>

        {/* 统计卡片 */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
          {[
            { label: "累计答题", value: totalSolved, unit: "道" },
            { label: "刷题天数", value: heatmapData.length, unit: "天" },
            { label: "当前年份", value: selectedYear, unit: "年" },
            { label: "题库总量", value: 100, unit: "道" },
          ].map((s) => (
            <div
              key={s.label}
              className="bg-card border border-border rounded-xl p-4"
            >
              <p className="text-muted-foreground text-xs">{s.label}</p>
              <p className="text-2xl font-bold mt-1">
                {s.value.toLocaleString()}
                <span className="text-sm text-muted-foreground ml-1">
                  {s.unit}
                </span>
              </p>
            </div>
          ))}
        </div>

        {/* 热力图 + 年份切换 */}
        <div className="bg-card border border-border rounded-xl p-4">
          <div className="flex items-center justify-between mb-2">
            <h2 className="text-lg font-semibold">学习进度热力图</h2>
            <div className="flex items-center gap-1">
              <button
                onClick={() => setSelectedYear((y) => y - 1)}
                className="p-1 rounded hover:bg-accent text-muted-foreground"
              >
                <ChevronLeft className="h-4 w-4" />
              </button>
              <span className="text-sm font-medium w-14 text-center">
                {selectedYear}
              </span>
              <button
                onClick={() => setSelectedYear((y) => y + 1)}
                className="p-1 rounded hover:bg-accent text-muted-foreground"
              >
                <ChevronRight className="h-4 w-4" />
              </button>
            </div>
          </div>
          <ReactECharts
            option={heatmapOption}
            style={{ height: 220 }}
            onEvents={{ click: onChartClick }}
          />
        </div>

        {/* 章节掌握度 + 薄弱环节 + 当日详情 */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
          {/* 左列：仪表盘 + 薄弱环节 */}
          <div className="space-y-4">
            <div className="bg-card border border-border rounded-xl p-4">
              <h2 className="text-lg font-semibold mb-2">章节掌握度</h2>
              <ReactECharts option={gaugeOption} style={{ height: 220 }} />
            </div>

            {/* 薄弱环节提示 */}
            <div className="bg-card border border-border rounded-xl p-4">
              <h2 className="text-lg font-semibold flex items-center gap-2 mb-3">
                <AlertTriangle className="h-5 w-5 text-yellow-400" />
                薄弱环节提示
              </h2>
              {weakChapterIds.length === 0 ? (
                <p className="text-muted-foreground text-sm text-center py-4">
                  暂无数据，开始刷题后会显示薄弱章节
                </p>
              ) : (
                <div className="space-y-3">
                  {weakChapterIds.map((chId) => {
                    const ch = CHAPTER_MAP[chId];
                    const stat =
                      chapterStats[chId] ?? {
                        total: 0,
                        correct: 0,
                        rate: 0,
                      };
                    const percent = Math.round(stat.rate * 100);
                    const accuracyColor =
                      percent < 40
                        ? "text-red-400"
                        : percent < 60
                          ? "text-yellow-400"
                          : "text-orange-400";
                    return (
                      <div
                        key={chId}
                        className="flex items-center gap-3 p-3 rounded-lg border border-border"
                        style={{
                          backgroundColor: `${ch.color}10`,
                          borderLeft: `3px solid ${ch.color}`,
                        }}
                      >
                        <span className="text-xl">{ch.icon}</span>
                        <div className="flex-1 min-w-0">
                          <p className="text-sm font-medium truncate">
                            {ch.shortName}
                          </p>
                          <p className={`text-xs font-bold ${accuracyColor}`}>
                            正确率 {percent}%
                          </p>
                          <p className="text-xs text-muted-foreground">
                            建议优先复习
                          </p>
                        </div>
                        <Link
                          href={`/quiz?chapter=${chId}`}
                          className="flex items-center gap-1 text-xs text-muted-foreground hover:text-foreground transition-colors shrink-0"
                          style={{ color: ch.color }}
                        >
                          去刷题
                          <ArrowRight className="h-3 w-3" />
                        </Link>
                      </div>
                    );
                  })}
                </div>
              )}
            </div>
          </div>

          {/* 右列：当日详情 */}
          <div className="bg-card border border-border rounded-xl p-4 h-fit">
            <h2 className="text-lg font-semibold mb-2">
              {selectedDate
                ? `${selectedDate} 刷题详情`
                : "点击热力图查看详情"}
            </h2>
            {dailyDetail.length === 0 ? (
              <p className="text-muted-foreground text-sm mt-8 text-center">
                点击上方热力图的某个日期，这里将显示当天刷过的题目
              </p>
            ) : (
              <div className="space-y-2 max-h-[520px] overflow-auto">
                {dailyDetail.map((r, i) => {
                  const chMeta = CHAPTER_MAP[r.chapter];
                  return (
                    <div
                      key={i}
                      className="flex items-center gap-3 p-2 rounded-lg border border-border text-sm"
                    >
                      <span className="text-muted-foreground shrink-0 w-8">
                        #{r.questionId}
                      </span>
                      {chMeta && (
                        <span
                          className="px-1.5 py-0.5 rounded text-xs font-medium shrink-0"
                          style={{
                            backgroundColor: `${chMeta.color}20`,
                            color: chMeta.color,
                          }}
                        >
                          {chMeta.shortName}
                        </span>
                      )}
                      <span
                        className={`px-1.5 py-0.5 rounded text-xs font-medium shrink-0 ml-auto ${
                          r.correct
                            ? "bg-green-500/20 text-green-400"
                            : "bg-red-500/20 text-red-400"
                        }`}
                      >
                        {r.correct ? "正确" : "错误"}
                      </span>
                    </div>
                  );
                })}
              </div>
            )}
          </div>
        </div>
      </div>
    </AppLayout>
  );
}
