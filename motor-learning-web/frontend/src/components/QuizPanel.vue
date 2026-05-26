<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import axios from 'axios'
import { useProgress } from '../stores/progress'
import { renderMarkdown } from '../utils/markdown'
import type { AssessmentData } from '../types/quiz'

const props = defineProps<{
  assessmentPath: string
}>()

const emit = defineEmits<{
  close: []
}>()

const { saveQuizAnswer, submitQuiz, getQuizProgress } = useProgress()

const assessment = ref<AssessmentData | null>(null)
const loading = ref(false)
const userAnswers = ref<Record<string, string>>({})
const revealedAnswers = ref<Set<string>>(new Set())
const submitted = ref(false)
const score = ref(0)

const existingProgress = computed(() => {
  return getQuizProgress(props.assessmentPath)
})

const totalQuestions = computed(() => assessment.value?.questions.length ?? 0)

const loadAssessment = async () => {
  if (!props.assessmentPath) return
  loading.value = true
  try {
    const response = await axios.get<AssessmentData>(`/api/quiz/parse/${props.assessmentPath}`)
    assessment.value = response.data
    if (existingProgress.value) {
      userAnswers.value = { ...existingProgress.value.answers }
      submitted.value = existingProgress.value.submitted
      score.value = existingProgress.value.score
    }
  } catch (error) {
    console.error('Failed to load assessment:', error)
  } finally {
    loading.value = false
  }
}

const toggleReveal = (questionId: string) => {
  if (revealedAnswers.value.has(questionId)) {
    revealedAnswers.value.delete(questionId)
  } else {
    revealedAnswers.value.add(questionId)
  }
}

const handleSubmit = () => {
  submitted.value = true
  let correct = 0
  for (const q of assessment.value?.questions ?? []) {
    if (userAnswers.value[q.id] && userAnswers.value[q.id].trim().length > 0) {
      correct++
    }
  }
  score.value = correct
  submitQuiz(props.assessmentPath, correct, totalQuestions.value)
}

const handleAnswerChange = (questionId: string, answer: string) => {
  userAnswers.value[questionId] = answer
  saveQuizAnswer(props.assessmentPath, questionId, answer)
}

const getDifficultyLabel = (difficulty: string): string => {
  if (!difficulty) return ''
  const count = difficulty.split('★').length - 1
  if (count <= 2) return '基础'
  if (count <= 3) return '进阶'
  return '专业'
}

const getDifficultyClass = (difficulty: string): string => {
  if (!difficulty) return ''
  const count = difficulty.split('★').length - 1
  if (count <= 2) return 'difficulty-basic'
  if (count <= 3) return 'difficulty-intermediate'
  return 'difficulty-advanced'
}

onMounted(() => {
  loadAssessment()
})
</script>

<template>
  <div class="quiz-backdrop" @click="emit('close')"></div>
  <div class="quiz-panel">
    <div class="quiz-header">
      <div class="quiz-title-row">
        <h2 class="quiz-title">{{ assessment?.title || '知识检验' }}</h2>
        <button class="close-btn" @click="emit('close')">✕</button>
      </div>
      <div v-if="assessment" class="quiz-meta">
        <span class="meta-item">📝 {{ assessment.totalQuestions }}道题</span>
        <span class="meta-item">{{ assessment.difficultyInfo }}</span>
        <span v-if="submitted" class="meta-item score" :class="{ 'score-good': score >= totalQuestions * 0.7 }">
          得分: {{ score }}/{{ totalQuestions }}
        </span>
      </div>
    </div>

    <div v-if="loading" class="quiz-loading">加载中...</div>

    <div v-else-if="assessment" class="quiz-body">
      <div v-for="(section, sIdx) in assessment.sections" :key="sIdx" class="quiz-section">
        <h3 class="section-title" :class="getDifficultyClass(section.difficulty)">
          <span class="difficulty-badge" :class="getDifficultyClass(section.difficulty)">
            {{ getDifficultyLabel(section.difficulty) }}
          </span>
          {{ section.title }}
        </h3>

        <div v-for="(question, qIdx) in section.questions" :key="question.id" class="question-card">
          <div class="question-header">
            <span class="question-number">{{ qIdx + 1 }}</span>
            <span class="question-title">{{ question.title }}</span>
          </div>

          <div v-if="question.problem" class="question-problem" v-html="renderMarkdown(question.problem)"></div>

          <div class="answer-area">
            <textarea
              v-model="userAnswers[question.id]"
              class="answer-input"
              placeholder="在此输入你的答案..."
              :disabled="submitted"
              @input="handleAnswerChange(question.id, ($event.target as HTMLTextAreaElement).value)"
              rows="3"
            ></textarea>

            <button
              class="reveal-btn"
              @click="toggleReveal(question.id)"
            >
              {{ revealedAnswers.has(question.id) ? '隐藏参考答案' : '查看参考答案' }}
            </button>

            <div v-if="revealedAnswers.has(question.id)" class="reference-answer">
              <div class="answer-label">参考答案：</div>
              <div class="answer-content" v-html="renderMarkdown(question.referenceAnswer)"></div>
            </div>
          </div>
        </div>
      </div>

      <div class="quiz-actions">
        <button v-if="!submitted" class="submit-btn" @click="handleSubmit">
          提交答案
        </button>
        <div v-else class="submit-result">
          <div class="result-icon">{{ score >= totalQuestions * 0.7 ? '🎉' : '💪' }}</div>
          <div class="result-text">
            {{ score >= totalQuestions * 0.7 ? '表现优秀！' : '继续加油！' }}
            正确作答 {{ score }}/{{ totalQuestions }} 题
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.quiz-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  background: rgba(0, 0, 0, 0.3);
  z-index: 199;
}

.quiz-panel {
  position: fixed;
  top: 0;
  right: 0;
  width: 50%;
  min-width: 400px;
  max-width: 700px;
  height: 100vh;
  background: var(--content-bg);
  box-shadow: -4px 0 20px rgba(0, 0, 0, 0.15);
  z-index: 200;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.quiz-header {
  padding: 1.25rem 1.5rem;
  border-bottom: 2px solid var(--primary-color);
  flex-shrink: 0;
}

.quiz-title-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.quiz-title {
  font-size: 1.2rem;
  color: var(--text-color);
  margin: 0;
}

.close-btn {
  background: none;
  border: 1px solid var(--border-color);
  width: 32px;
  height: 32px;
  border-radius: 6px;
  cursor: pointer;
  font-size: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.close-btn:hover {
  background-color: var(--code-bg);
}

.quiz-meta {
  display: flex;
  gap: 1rem;
  margin-top: 0.5rem;
  font-size: 0.8rem;
  color: var(--text-secondary);
  flex-wrap: wrap;
}

.meta-item {
  padding: 0.2rem 0.5rem;
  background: var(--code-bg);
  border-radius: 4px;
}

.score-good {
  color: #16a34a;
  background: #f0fdf4;
}

.quiz-body {
  flex: 1;
  overflow-y: auto;
  padding: 1.5rem;
}

.quiz-section {
  margin-bottom: 2rem;
}

.section-title {
  font-size: 1rem;
  margin-bottom: 1rem;
  padding-bottom: 0.5rem;
  border-bottom: 1px solid var(--border-color);
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.difficulty-badge {
  font-size: 0.7rem;
  padding: 0.15rem 0.5rem;
  border-radius: 10px;
  font-weight: 600;
}

.difficulty-basic {
  background: #dbeafe;
  color: #1d4ed8;
}

.difficulty-intermediate {
  background: #fef3c7;
  color: #92400e;
}

.difficulty-advanced {
  background: #fee2e2;
  color: #991b1b;
}

.question-card {
  margin-bottom: 1.5rem;
  border: 1px solid var(--border-color);
  border-radius: 8px;
  padding: 1rem;
  background: var(--bg-color);
}

.question-header {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 0.75rem;
}

.question-number {
  width: 24px;
  height: 24px;
  border-radius: 50%;
  background: var(--primary-color);
  color: white;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.75rem;
  font-weight: 600;
  flex-shrink: 0;
}

.question-title {
  font-weight: 600;
  font-size: 0.9rem;
}

.question-problem {
  font-size: 0.85rem;
  line-height: 1.7;
  margin-bottom: 0.75rem;
  color: var(--text-color);
}

.question-problem :deep(p) {
  margin-bottom: 0.5rem;
}

.answer-area {
  margin-top: 0.75rem;
}

.answer-input {
  width: 100%;
  padding: 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  font-size: 0.85rem;
  font-family: inherit;
  resize: vertical;
  min-height: 60px;
  outline: none;
  transition: border-color 0.2s;
  box-sizing: border-box;
}

.answer-input:focus {
  border-color: var(--primary-color);
}

.answer-input:disabled {
  background: var(--code-bg);
  color: var(--text-secondary);
}

.reveal-btn {
  margin-top: 0.5rem;
  background: none;
  border: 1px dashed var(--primary-color);
  color: var(--primary-color);
  padding: 0.35rem 0.75rem;
  border-radius: 4px;
  cursor: pointer;
  font-size: 0.8rem;
}

.reveal-btn:hover {
  background: var(--primary-color);
  color: white;
  border-style: solid;
}

.reference-answer {
  margin-top: 0.75rem;
  padding: 0.75rem;
  background: var(--content-bg);
  border-left: 3px solid #16a34a;
  border-radius: 0 6px 6px 0;
  font-size: 0.85rem;
  line-height: 1.7;
}

.answer-label {
  font-weight: 600;
  color: #16a34a;
  margin-bottom: 0.5rem;
  font-size: 0.8rem;
}

.answer-content :deep(p) {
  margin-bottom: 0.5rem;
}

.quiz-actions {
  margin-top: 2rem;
  text-align: center;
}

.submit-btn {
  padding: 0.75rem 2rem;
  background: var(--primary-color);
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 1rem;
  font-weight: 600;
  cursor: pointer;
  transition: background 0.2s;
}

.submit-btn:hover {
  background: var(--primary-hover);
}

.submit-result {
  padding: 1.5rem;
  background: var(--bg-color);
  border-radius: 8px;
  text-align: center;
}

.result-icon {
  font-size: 2rem;
  margin-bottom: 0.5rem;
}

.result-text {
  font-size: 0.95rem;
  color: var(--text-color);
}

.quiz-loading {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-secondary);
}

@media (max-width: 768px) {
  .quiz-panel {
    width: 100%;
    min-width: unset;
  }
}
</style>
