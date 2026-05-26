export interface Project {
  id: string
  name: string
  path: string
  file_count: number
  created_at: string
}

export interface FileNode {
  name: string
  path: string
  type: 'file' | 'directory'
  children?: FileNode[]
}

export interface FileContent {
  path: string
  content: string
  language: string
}

export interface Highlight {
  id: string
  project_id: string
  file_path: string
  start_line: number
  start_column: number
  end_line: number
  end_column: number
  color: string
  created_at: string
}

export interface Annotation {
  id: string
  project_id: string
  file_path: string
  line_number: number
  content: string
  created_at: string
  updated_at: string
}

export interface Location {
  file_path: string
  line: number
  character: number
}

export interface Symbol {
  name: string
  kind: string
  location: Location
  preview?: string
}

export interface Tab {
  id: string
  path: string
  name: string
  language: string
  content: string
  isDirty: boolean
}

export interface SplitPane {
  id: string
  tabs: Tab[]
  activeTabId: string | null
  size: number
}
