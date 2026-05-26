﻿<template>
  <div class="project-view">
    <el-container>
      <el-header class="header">
        <div class="header-left">
          <el-button link @click="goHome">
            <el-icon><ArrowLeft /></el-icon>
            返回
          </el-button>
          <span class="project-name">{{ projectStore.currentProject?.name }}</span>
        </div>
        <div class="header-center">
          <el-input
            v-model="searchQuery"
            placeholder="搜索符号..."
            :prefix-icon="Search"
            clearable
            @keyup.enter="handleSearch"
            style="width: 300px"
          />
        </div>
        <div class="header-right">
          <el-button @click="toggleMobileSidebar" class="mobile-only">
            <el-icon><Menu /></el-icon>
          </el-button>
        </div>
      </el-header>

      <el-container class="main-container">
        <el-aside
          :width="sidebarWidth"
          class="sidebar"
          :class="{ 'mobile-hidden': !mobileSidebarVisible }"
        >
          <div class="sidebar-header">
            <span>文件</span>
            <el-input
              v-model="fileFilter"
              placeholder="过滤文件..."
              size="small"
              clearable
            />
          </div>
          <FileTree
            :nodes="filteredFileTree"
            :loading="projectStore.loading"
            @select="handleFileSelect"
          />
        </el-aside>

        <el-main class="main-content">
          <div class="panes-container" :style="panesStyle">
            <EditorPane
              v-for="pane in editorStore.panes"
              :key="pane.id"
              :pane="pane"
              :active="pane.id === editorStore.activePaneId"
              :can-close="editorStore.panes.length > 1"
              :highlights="editorStore.highlights"
              :annotations="editorStore.annotations"
              @split="handleSplit"
              @close-tab="handleCloseTab"
              @close-pane="handleClosePane"
              @activate="handleActivateTab"
              @add-highlight="handleAddHighlight"
              @remove-highlight="handleRemoveHighlight"
              @add-annotation="handleAddAnnotation"
              @remove-annotation="handleRemoveAnnotation"
            />
          </div>
          
          <div v-if="!editorStore.activeTab" class="empty-state">
            <el-icon size="64"><Document /></el-icon>
            <p>选择文件开始浏览代码</p>
          </div>
        </el-main>

        <el-drawer
          v-model="searchDrawerVisible"
          title="搜索结果"
          direction="rtl"
          size="400px"
        >
          <SearchResults
            :results="searchResults"
            @select="handleSearchResultSelect"
          />
        </el-drawer>
      </el-container>
    </el-container>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { Search } from '@element-plus/icons-vue'
import { useProjectStore, useEditorStore } from '@/stores'
import { analysisApi } from '@/api'
import type { FileNode, Symbol, Highlight, Annotation } from '@/types'
import FileTree from '@/components/FileTree.vue'
import EditorPane from '@/components/EditorPane.vue'
import SearchResults from '@/components/SearchResults.vue'

const route = useRoute()
const router = useRouter()
const projectStore = useProjectStore()
const editorStore = useEditorStore()

const projectId = computed(() => route.params.id as string)

const sidebarWidth = ref('280px')
const fileFilter = ref('')
const searchQuery = ref('')
const searchDrawerVisible = ref(false)
const searchResults = ref<Symbol[]>([])
const mobileSidebarVisible = ref(false)

const filteredFileTree = computed(() => {
  if (!fileFilter.value) return projectStore.fileTree
  return filterTree(projectStore.fileTree, fileFilter.value.toLowerCase())
})

const panesStyle = computed(() => {
  return {
    display: 'flex',
    flexDirection: 'row',
    width: '100%',
    height: '100%',
  }
})

const filterTree = (nodes: FileNode[], query: string): FileNode[] => {
  return nodes.reduce((acc: FileNode[], node) => {
    if (node.type === 'file') {
      if (node.name.toLowerCase().includes(query)) {
        acc.push(node)
      }
    } else if (node.children) {
      const filteredChildren = filterTree(node.children, query)
      if (filteredChildren.length > 0) {
        acc.push({ ...node, children: filteredChildren })
      }
    }
    return acc
  }, [])
}

onMounted(async () => {
  await projectStore.loadProject(projectId.value)
})

onUnmounted(() => {
  editorStore.panes = [
    {
      id: 'pane-0',
      tabs: [],
      activeTabId: null,
      size: 100,
    },
  ]
})

const goHome = () => {
  router.push('/')
}

const handleFileSelect = async (path: string) => {
  await editorStore.openFile(projectId.value, path)
  mobileSidebarVisible.value = false
}

const handleCloseTab = (tabId: string) => {
  editorStore.closeTab(tabId)
}

const handleClosePane = (paneId: string) => {
  editorStore.closePane(paneId)
}

const handleActivateTab = (paneId: string, tabId: string) => {
  editorStore.setActiveTab(paneId, tabId)
}

const handleSplit = (direction: 'horizontal' | 'vertical') => {
  editorStore.splitPane(direction)
}

const handleSearch = async () => {
  if (!searchQuery.value.trim()) return
  
  try {
    searchResults.value = await analysisApi.searchSymbols(projectId.value, searchQuery.value)
    searchDrawerVisible.value = true
  } catch (error) {
    console.error('Search failed:', error)
  }
}

const handleSearchResultSelect = async (result: Symbol) => {
  await editorStore.openFile(projectId.value, result.location.file_path)
  searchDrawerVisible.value = false
}

const handleAddHighlight = async (highlight: Omit<Highlight, 'id' | 'created_at' | 'project_id'>) => {
  await editorStore.addHighlight({
    ...highlight,
    project_id: projectId.value,
  })
}

const handleRemoveHighlight = async (highlightId: string) => {
  await editorStore.removeHighlight(highlightId)
}

const handleAddAnnotation = async (annotation: Omit<Annotation, 'id' | 'created_at' | 'updated_at' | 'project_id'>) => {
  await editorStore.addAnnotation({
    ...annotation,
    project_id: projectId.value,
  })
}

const handleRemoveAnnotation = async (annotationId: string) => {
  await editorStore.removeAnnotation(annotationId)
}

const toggleMobileSidebar = () => {
  mobileSidebarVisible.value = !mobileSidebarVisible.value
}
</script>

<style lang="scss" scoped>
.project-view {
  height: 100vh;
  overflow: hidden;
}

.header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  background: var(--el-bg-color);
  border-bottom: 1px solid var(--el-border-color);
}

.header-left {
  display: flex;
  align-items: center;
  gap: 16px;
}

.project-name {
  font-weight: 600;
  font-size: 16px;
}

.main-container {
  height: calc(100vh - 60px);
}

.sidebar {
  background: var(--el-bg-color);
  border-right: 1px solid var(--el-border-color);
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.sidebar-header {
  padding: 12px;
  border-bottom: 1px solid var(--el-border-color);
  
  span {
    display: block;
    font-weight: 600;
    margin-bottom: 8px;
  }
}

.main-content {
  padding: 0 !important;
  background: var(--el-bg-color-page);
  position: relative;
  overflow: hidden;
  display: flex;
  flex-direction: column;
  height: 100%;
}

.panes-container {
  display: flex;
  flex-direction: row;
  width: 100%;
  flex: 1;
  min-height: 0;
}

.panes-container > :deep(.editor-pane) {
  flex: 1;
  min-width: 0;
  height: 100%;
}

.empty-state {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  text-align: center;
  color: var(--el-text-color-placeholder);
  pointer-events: none;
  
  p {
    margin-top: 16px;
    font-size: 16px;
  }
}

.mobile-only {
  display: none;
}

@media (max-width: 768px) {
  .sidebar {
    position: fixed;
    left: 0;
    top: 60px;
    height: calc(100vh - 60px);
    z-index: 100;
    width: 280px !important;
    
    &.mobile-hidden {
      transform: translateX(-100%);
    }
  }
  
  .mobile-only {
    display: inline-flex;
  }
  
  .header-center {
    display: none;
  }
}
</style>
