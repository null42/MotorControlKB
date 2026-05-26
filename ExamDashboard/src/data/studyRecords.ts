import { StudyRecord } from "@/lib/types";

/**
 * 简单确定性伪随机数生成器（Mulberry32），
 * 基于日期序号生成种子，保证同一天内多次调用结果一致。
 */
function seededRandom(seed: number): () => number {
  return () => {
    seed |= 0;
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * 判断指定日期是否为工作日（周一到周五）。
 */
function isWeekday(date: Date): boolean {
  const day = date.getDay();
  return day !== 0 && day !== 6;
}

/**
 * 格式化 Date 为 YYYY-MM-DD 字符串。
 */
function formatDate(date: Date): string {
  const y = date.getFullYear();
  const m = String(date.getMonth() + 1).padStart(2, "0");
  const d = String(date.getDate()).padStart(2, "0");
  return `${y}-${m}-${d}`;
}

/**
 * 根据题目 ID 映射到所属章节。
 */
const CHAPTER_MAP = [
  { id: "foc-theory",     range: [1, 17] },
  { id: "sensored-foc",   range: [18, 34] },
  { id: "sensorless-foc", range: [35, 51] },
  { id: "hfi",            range: [52, 68] },
  { id: "protection",     range: [69, 85] },
  { id: "hardware",       range: [86, 100] },
] as const;

function getChapterForQuestion(questionId: number): string {
  for (const c of CHAPTER_MAP) {
    if (questionId >= c.range[0] && questionId <= c.range[1]) {
      return c.id;
    }
  }
  return "foc-theory";
}

/**
 * 获取当年第 N 天的序号（从 1 月 1 日为 0 开始计数）。
 */
function dayOfYear(date: Date): number {
  const year = date.getFullYear();
  const start = new Date(year, 0, 0);
  return Math.floor((date.getTime() - start.getTime()) / (1000 * 60 * 60 * 24));
}

/**
 * 生成当前年份截止到今天的所有学习记录。
 *
 * - 跳过周末（周六、周日）
 * - 约 60% 的工作日有学习活动
 * - 每个活跃日随机 1~8 条答题记录
 * - questionId 范围 1~100
 * - 整体正确率约 65%
 * - 使用基于日期的确定性随机，保证同一天生成的数据稳定
 */
export function generateStudyRecords(): StudyRecord[] {
  const records: StudyRecord[] = [];
  const now = new Date();
  const year = now.getFullYear();
  const today = new Date(year, now.getMonth(), now.getDate());

  // 从 1 月 1 日开始遍历到 today
  const cursor = new Date(year, 0, 1);
  while (cursor <= today) {
    if (isWeekday(cursor)) {
      // 基于该日在年内的序号生成确定性种子
      const rand = seededRandom(dayOfYear(cursor));

      // 约 60% 的工作日有活动
      if (rand() < 0.6) {
        // 1~8 道题
        const count = Math.floor(rand() * 8) + 1;
        const dateStr = formatDate(cursor);

        for (let i = 0; i < count; i++) {
          // questionId: 1 ~ 100
          const questionId = Math.floor(rand() * 100) + 1;
          // 约 65% 正确率
          const correct = rand() < 0.65;

          records.push({
            date: dateStr,
            questionId,
            correct,
            chapter: getChapterForQuestion(questionId),
          });
        }
      }
    }

    // 推进到下一天
    cursor.setDate(cursor.getDate() + 1);
  }

  return records;
}

/**
 * 将学习记录按日期聚合为热力图数据。
 *
 * @param records - 学习记录数组
 * @returns 元组数组，每项为 [日期字符串, 当日答题数量]
 */
export function getHeatmapData(records: StudyRecord[]): Array<[string, number]> {
  const countMap: Record<string, number> = {};
  for (const r of records) {
    countMap[r.date] = (countMap[r.date] || 0) + 1;
  }
  return Object.entries(countMap) as Array<[string, number]>;
}

/**
 * 按章节汇总答题统计。
 *
 * @returns 以章节 ID 为键的对象，包含总题数、正确数、正确率。
 */
export function getChapterStats(
  records: StudyRecord[],
): Record<string, { total: number; correct: number; rate: number }> {
  const stats: Record<string, { total: number; correct: number }> = {};
  for (const r of records) {
    if (!stats[r.chapter]) {
      stats[r.chapter] = { total: 0, correct: 0 };
    }
    stats[r.chapter].total += 1;
    if (r.correct) {
      stats[r.chapter].correct += 1;
    }
  }
  const result: Record<string, { total: number; correct: number; rate: number }> = {};
  for (const ch of Object.keys(stats)) {
    const { total, correct } = stats[ch];
    result[ch] = { total, correct, rate: total > 0 ? correct / total : 0 };
  }
  return result;
}

/**
 * 返回正确率最低的 N 个章节 ID。
 * 若 records 为空则返回空数组；N 不足时返回实际数量。
 */
export function getWeakChapters(records: StudyRecord[], topN: number): string[] {
  const stats = getChapterStats(records);
  return Object.entries(stats)
    .sort(([, a], [, b]) => a.rate - b.rate)
    .slice(0, topN)
    .map(([id]) => id);
}
