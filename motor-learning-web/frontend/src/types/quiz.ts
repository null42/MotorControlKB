export interface QuizQuestion {
  id: string
  title: string
  problem: string
  referenceAnswer: string
  difficulty: string
}

export interface QuizSection {
  title: string
  difficulty: string
  questions: QuizQuestion[]
}

export interface AssessmentData {
  title: string
  module: string
  totalQuestions: number
  difficultyInfo: string
  sections: QuizSection[]
  questions: QuizQuestion[]
}
