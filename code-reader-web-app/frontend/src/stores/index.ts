import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { Project, FileNode, Tab, SplitPane, Highlight, Annotation } from '@/types'
import { projectApi, fileApi, annotationApi } from '@/api'

export const useProjectStore = defineStore('project', () => {
  const projects = ref<Project[]>([])
  const currentProject = ref<Project | null>(null)
  const fileTree = ref<FileNode[]>([])
  const loading = ref(false)

  const loadProjects = async () => {
    loading.value = true
    try {
      projects.value = await projectApi.list()
    } finally {
      loading.value = false
    }
  }

  const loadProject = async (id: string) => {
    loading.value = true
    try {
      currentProject.value = await projectApi.get(id)
      fileTree.value = await fileApi.getTree(id)
    } finally {
      loading.value = false
    }
  }

  const uploadZip = async (file: File, name?: string) => {
    const result = await projectApi.uploadZip(file, name)
    await loadProjects()
    return result.project
  }

  const uploadFolder = async (files: FileList, name: string) => {
    const result = await projectApi.uploadFolder(files, name)
    await loadProjects()
    return result.project
  }

  const deleteProject = async (id: string) => {
    await projectApi.delete(id)
    projects.value = projects.value.filter((p) => p.id !== id)
  }

  return {
    projects,
    currentProject,
    fileTree,
    loading,
    loadProjects,
    loadProject,
    uploadZip,
    uploadFolder,
    deleteProject,
  }
})

export const useEditorStore = defineStore('editor', () => {
  const panes = ref<SplitPane[]>([
    {
      id: 'pane-0',
      tabs: [],
      activeTabId: null,
      size: 100,
    },
  ])
  const activePaneId = ref('pane-0')
  const highlights = ref<Highlight[]>([])
  const annotations = ref<Annotation[]>([])

  const activePane = computed(() => panes.value.find((p) => p.id === activePaneId.value))
  const activeTab = computed(() => {
    if (!activePane.value?.activeTabId) return null
    return activePane.value.tabs.find((t) => t.id === activePane.value!.activeTabId)
  })

  const generateTabId = () => `tab-${Date.now()}-${Math.random().toString(36).slice(2, 9)}`

  const openFile = async (projectId: string, path: string) => {
    const existingTab = panes.value
      .flatMap((p) => p.tabs)
      .find((t) => t.path === path)

    if (existingTab) {
      const pane = panes.value.find((p) => p.tabs.some((t) => t.id === existingTab.id))
      if (pane) {
        pane.activeTabId = existingTab.id
        activePaneId.value = pane.id
      }
      return
    }

    try {
      const fileContent = await fileApi.getContent(projectId, path)
      
      const tab: Tab = {
        id: generateTabId(),
        path,
        name: path.split(/[/\\]/).pop() || path,
        language: fileContent.language,
        content: fileContent.content,
        isDirty: false,
      }

      const paneIndex = panes.value.findIndex((p) => p.id === activePaneId.value)
      
      if (paneIndex !== -1) {
        const currentPane = panes.value[paneIndex]
        panes.value[paneIndex] = {
          ...currentPane,
          tabs: [...currentPane.tabs, tab],
          activeTabId: tab.id
        }
      }

      await loadHighlights(projectId, path)
      await loadAnnotations(projectId, path)
    } catch (error) {
      console.error('Failed to open file:', error)
      throw error
    }
  }

  const closeTab = (tabId: string) => {
    for (let paneIndex = 0; paneIndex < panes.value.length; paneIndex++) {
      const pane = panes.value[paneIndex]
      const tabIndex = pane.tabs.findIndex((t) => t.id === tabId)
      if (tabIndex !== -1) {
        pane.tabs.splice(tabIndex, 1)
        if (pane.activeTabId === tabId) {
          pane.activeTabId = pane.tabs[Math.max(0, tabIndex - 1)]?.id || null
        }
        
        if (pane.tabs.length === 0 && panes.value.length > 1) {
          panes.value.splice(paneIndex, 1)
          const remainingPane = panes.value[0]
          if (remainingPane) {
            remainingPane.size = 100
            activePaneId.value = remainingPane.id
          }
        }
        break
      }
    }
  }

  const closePane = (paneId: string) => {
    const paneIndex = panes.value.findIndex((p) => p.id === paneId)
    if (paneIndex !== -1 && panes.value.length > 1) {
      panes.value.splice(paneIndex, 1)
      const remainingPane = panes.value[0]
      if (remainingPane) {
        remainingPane.size = 100
        activePaneId.value = remainingPane.id
      }
    }
  }

  const setActiveTab = (paneId: string, tabId: string) => {
    const pane = panes.value.find((p) => p.id === paneId)
    if (pane) {
      pane.activeTabId = tabId
      activePaneId.value = paneId
    }
  }

  const splitPane = (direction: 'horizontal' | 'vertical') => {
    const newPaneId = `pane-${Date.now()}`
    const currentPane = panes.value.find((p) => p.id === activePaneId.value)

    if (currentPane && currentPane.tabs.length > 0 && currentPane.activeTabId) {
      const activeTab = currentPane.tabs.find((t) => t.id === currentPane.activeTabId)
      if (activeTab) {
        const newTab: Tab = {
          ...activeTab,
          id: generateTabId(),
        }
        const newPane: SplitPane = {
          id: newPaneId,
          tabs: [newTab],
          activeTabId: newTab.id,
          size: 50,
        }
        currentPane.size = 50
        panes.value.push(newPane)
        activePaneId.value = newPaneId
      }
    }
  }

  const loadHighlights = async (projectId: string, filePath?: string) => {
    highlights.value = await annotationApi.getHighlights(projectId, filePath)
  }

  const addHighlight = async (highlight: Omit<Highlight, 'id' | 'created_at'>) => {
    const newHighlight = await annotationApi.createHighlight(highlight)
    highlights.value.push(newHighlight)
    return newHighlight
  }

  const removeHighlight = async (highlightId: string) => {
    await annotationApi.deleteHighlight(highlightId)
    highlights.value = highlights.value.filter((h) => h.id !== highlightId)
  }

  const loadAnnotations = async (projectId: string, filePath?: string) => {
    annotations.value = await annotationApi.getAnnotations(projectId, filePath)
  }

  const addAnnotation = async (annotation: Omit<Annotation, 'id' | 'created_at' | 'updated_at'>) => {
    const newAnnotation = await annotationApi.createAnnotation(annotation)
    annotations.value.push(newAnnotation)
    return newAnnotation
  }

  const updateAnnotation = async (annotationId: string, content: string) => {
    await annotationApi.updateAnnotation(annotationId, content)
    const annotation = annotations.value.find((a) => a.id === annotationId)
    if (annotation) {
      annotation.content = content
    }
  }

  const removeAnnotation = async (annotationId: string) => {
    await annotationApi.deleteAnnotation(annotationId)
    annotations.value = annotations.value.filter((a) => a.id !== annotationId)
  }

  return {
    panes,
    activePaneId,
    highlights,
    annotations,
    activePane,
    activeTab,
    openFile,
    closeTab,
    closePane,
    setActiveTab,
    splitPane,
    loadHighlights,
    addHighlight,
    removeHighlight,
    loadAnnotations,
    addAnnotation,
    updateAnnotation,
    removeAnnotation,
  }
})
