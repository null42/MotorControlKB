import { ref, watch } from 'vue'
import type { LearningProgress, ModuleProgress, QuizProgress } from '../types/progress'
import { STORAGE_KEY } from '../types/progress'

function loadProgress(): LearningProgress {
  try {
    const raw = localStorage.getItem(STORAGE_KEY)
    if (raw) return JSON.parse(raw)
  } catch (e) {
    console.error('Failed to load progress:', e)
  }
  return { modules: {}, quizzes: {}, totalStudyTime: 0, lastStudyDate: null, streakDays: 0 }
}

function saveProgress(data: LearningProgress) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(data))
  } catch (e) {
    console.error('Failed to save progress:', e)
  }
}

const progress = ref<LearningProgress>(loadProgress())

watch(progress, (val) => saveProgress(val), { deep: true })

export function useProgress() {
  const markVisited = (moduleId: string) => {
    if (!progress.value.modules[moduleId]) {
      progress.value.modules[moduleId] = {
        moduleId,
        visited: true,
        completed: false,
        lastVisitTime: Date.now(),
        visitCount: 1,
        readingProgress: 0,
      }
    } else {
      const mod = progress.value.modules[moduleId]
      mod.visited = true
      mod.lastVisitTime = Date.now()
      mod.visitCount++
    }
    updateStreak()
  }

  const markCompleted = (moduleId: string) => {
    if (!progress.value.modules[moduleId]) {
      progress.value.modules[moduleId] = {
        moduleId,
        visited: true,
        completed: true,
        lastVisitTime: Date.now(),
        visitCount: 1,
        readingProgress: 100,
      }
    } else {
      progress.value.modules[moduleId].completed = true
      progress.value.modules[moduleId].readingProgress = 100
    }
  }

  const updateReadingProgress = (moduleId: string, percent: number) => {
    if (!progress.value.modules[moduleId]) {
      markVisited(moduleId)
    }
    progress.value.modules[moduleId].readingProgress = Math.max(
      progress.value.modules[moduleId].readingProgress,
      percent
    )
  }

  const saveQuizAnswer = (assessmentId: string, questionId: string, answer: string) => {
    if (!progress.value.quizzes[assessmentId]) {
      progress.value.quizzes[assessmentId] = {
        assessmentId,
        answers: {},
        submitted: false,
        score: 0,
        totalQuestions: 0,
        submittedAt: null,
      }
    }
    progress.value.quizzes[assessmentId].answers[questionId] = answer
  }

  const submitQuiz = (assessmentId: string, score: number, total: number) => {
    if (!progress.value.quizzes[assessmentId]) return
    const quiz = progress.value.quizzes[assessmentId]
    quiz.submitted = true
    quiz.score = score
    quiz.totalQuestions = total
    quiz.submittedAt = Date.now()
  }

  const isModuleVisited = (moduleId: string): boolean => {
    return progress.value.modules[moduleId]?.visited ?? false
  }

  const isModuleCompleted = (moduleId: string): boolean => {
    return progress.value.modules[moduleId]?.completed ?? false
  }

  const getModuleProgress = (moduleId: string): ModuleProgress | undefined => {
    return progress.value.modules[moduleId]
  }

  const getQuizProgress = (assessmentId: string): QuizProgress | undefined => {
    return progress.value.quizzes[assessmentId]
  }

  const getOverallProgress = (): { visited: number; total: number; completed: number } => {
    const modules = Object.values(progress.value.modules)
    return {
      visited: modules.filter(m => m.visited).length,
      completed: modules.filter(m => m.completed).length,
      total: 30,
    }
  }

  const updateStreak = () => {
    const today = new Date().toISOString().slice(0, 10)
    if (progress.value.lastStudyDate === today) return
    const yesterday = new Date(Date.now() - 86400000).toISOString().slice(0, 10)
    if (progress.value.lastStudyDate === yesterday) {
      progress.value.streakDays++
    } else {
      progress.value.streakDays = 1
    }
    progress.value.lastStudyDate = today
  }

  const resetProgress = () => {
    progress.value = {
      modules: {},
      quizzes: {},
      totalStudyTime: 0,
      lastStudyDate: null,
      streakDays: 0,
    }
  }

  return {
    progress,
    markVisited,
    markCompleted,
    updateReadingProgress,
    saveQuizAnswer,
    submitQuiz,
    isModuleVisited,
    isModuleCompleted,
    getModuleProgress,
    getQuizProgress,
    getOverallProgress,
    resetProgress,
  }
}
