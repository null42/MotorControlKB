import axios from 'axios'
import type { Project, FileNode, FileContent, Highlight, Annotation, Symbol, Location } from '@/types'

const api = axios.create({
  baseURL: '/api',
  timeout: 30000,
})

export const projectApi = {
  uploadZip: async (file: File, name?: string) => {
    const formData = new FormData()
    formData.append('file', file)
    if (name) formData.append('name', name)
    const response = await api.post<{ success: boolean; project: Project }>('/projects/upload/zip', formData, {
      headers: { 'Content-Type': 'multipart/form-data' },
    })
    return response.data
  },

  uploadFolder: async (files: FileList, name: string) => {
    const formData = new FormData()
    formData.append('name', name)
    for (let i = 0; i < files.length; i++) {
      formData.append('files', files[i])
    }
    const response = await api.post<{ success: boolean; project: Project }>('/projects/upload/folder', formData, {
      headers: { 'Content-Type': 'multipart/form-data' },
    })
    return response.data
  },

  list: async () => {
    const response = await api.get<{ projects: Project[] }>('/projects/list')
    return response.data.projects
  },

  get: async (id: string) => {
    const response = await api.get<Project>(`/projects/${id}`)
    return response.data
  },

  delete: async (id: string) => {
    const response = await api.delete(`/projects/${id}`)
    return response.data
  },
}

export const fileApi = {
  getTree: async (projectId: string) => {
    const response = await api.get<{ tree: FileNode[] }>(`/files/${projectId}/tree`)
    return response.data.tree
  },

  getContent: async (projectId: string, path: string) => {
    const response = await api.get<FileContent>(`/files/${projectId}/file`, {
      params: { path },
    })
    return response.data
  },

  search: async (projectId: string, query: string) => {
    const response = await api.get<{ results: { name: string; path: string; type: string }[] }>(
      `/files/${projectId}/search`,
      { params: { query } }
    )
    return response.data.results
  },
}

export const annotationApi = {
  createHighlight: async (highlight: Omit<Highlight, 'id' | 'created_at'>) => {
    const response = await api.post<{ success: boolean; highlight: Highlight }>('/annotations/highlights', highlight)
    return response.data.highlight
  },

  getHighlights: async (projectId: string, filePath?: string) => {
    const response = await api.get<{ highlights: Highlight[] }>(`/annotations/highlights/${projectId}`, {
      params: filePath ? { file_path: filePath } : {},
    })
    return response.data.highlights
  },

  deleteHighlight: async (highlightId: string) => {
    await api.delete(`/annotations/highlights/${highlightId}`)
  },

  createAnnotation: async (annotation: Omit<Annotation, 'id' | 'created_at' | 'updated_at'>) => {
    const response = await api.post<{ success: boolean; annotation: Annotation }>('/annotations/annotations', annotation)
    return response.data.annotation
  },

  getAnnotations: async (projectId: string, filePath?: string) => {
    const response = await api.get<{ annotations: Annotation[] }>(`/annotations/annotations/${projectId}`, {
      params: filePath ? { file_path: filePath } : {},
    })
    return response.data.annotations
  },

  updateAnnotation: async (annotationId: string, content: string) => {
    const response = await api.put<{ success: boolean; updated_at: string }>(`/annotations/annotations/${annotationId}`, {
      content,
    })
    return response.data
  },

  deleteAnnotation: async (annotationId: string) => {
    await api.delete(`/annotations/annotations/${annotationId}`)
  },
}

export const analysisApi = {
  getDefinition: async (projectId: string, filePath: string, line: number, character: number) => {
    const response = await api.post<{ found: boolean; location: Location | null }>('/analysis/definition', {
      project_id: projectId,
      file_path: filePath,
      line,
      character,
    })
    return response.data
  },

  getReferences: async (projectId: string, filePath: string, line: number, character: number) => {
    const response = await api.post<{ locations: Location[] }>('/analysis/references', {
      project_id: projectId,
      file_path: filePath,
      line,
      character,
    })
    return response.data.locations
  },

  searchSymbols: async (projectId: string, query: string) => {
    const response = await api.post<{ symbols: Symbol[] }>('/analysis/symbols', {
      project_id: projectId,
      query,
    })
    return response.data.symbols
  },

  getCallGraph: async (projectId: string, functionName?: string) => {
    const response = await api.get<{ nodes: any[]; edges: any[] }>(`/analysis/call-graph/${projectId}`, {
      params: functionName ? { function_name: functionName } : {},
    })
    return response.data
  },
}

export default api
