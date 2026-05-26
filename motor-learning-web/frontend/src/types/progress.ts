export interface ModuleProgress {
  moduleId: string
  visited: boolean
  completed: boolean
  lastVisitTime: number
  visitCount: number
  readingProgress: number
}

export interface QuizProgress {
  assessmentId: string
  answers: Record<string, string>
  submitted: boolean
  score: number
  totalQuestions: number
  submittedAt: number | null
}

export interface LearningProgress {
  modules: Record<string, ModuleProgress>
  quizzes: Record<string, QuizProgress>
  totalStudyTime: number
  lastStudyDate: string | null
  streakDays: number
}

export const STORAGE_KEY = 'motor-learning-progress'
