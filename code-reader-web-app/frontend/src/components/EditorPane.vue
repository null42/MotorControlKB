﻿<template>
  <div class="editor-pane" :class="{ active }">
    <div class="tabs-container">
      <div class="tabs">
        <div
          v-for="tab in pane.tabs"
          :key="tab.id"
          class="tab"
          :class="{ active: tab.id === pane.activeTabId }"
          @click="$emit('activate', pane.id, tab.id)"
        >
          <span class="tab-name">{{ tab.name }}</span>
          <el-icon
            class="tab-close"
            @click.stop="$emit('close-tab', tab.id)"
          >
            <Close />
          </el-icon>
        </div>
      </div>
      <div class="tabs-actions">
        <el-dropdown trigger="click">
          <el-button link>
            <el-icon><Plus /></el-icon>
          </el-button>
          <template #dropdown>
            <el-dropdown-menu>
              <el-dropdown-item @click="$emit('split', 'horizontal')">
                水平分屏
              </el-dropdown-item>
              <el-dropdown-item @click="$emit('split', 'vertical')">
                垂直分屏
              </el-dropdown-item>
              <el-dropdown-item 
                v-if="canClose" 
                divided 
                @click="$emit('close-pane', pane.id)"
              >
                关闭分屏
              </el-dropdown-item>
            </el-dropdown-menu>
          </template>
        </el-dropdown>
      </div>
    </div>
    
    <div class="editor-container" v-if="activeTab">
      <MonacoEditor
        :model-value="activeTab.content"
        :language="activeTab.language"
        :path="activeTab.path"
        :highlights="highlights"
        :annotations="annotations"
        @highlight="$emit('add-highlight', $event)"
        @remove-highlight="$emit('remove-highlight', $event)"
        @annotate="$emit('add-annotation', $event)"
        @remove-annotation="$emit('remove-annotation', $event)"
      />
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue'
import { Plus, Close } from '@element-plus/icons-vue'
import type { SplitPane, Highlight, Annotation } from '@/types'
import MonacoEditor from './MonacoEditor.vue'

const props = defineProps<{
  pane: SplitPane
  active: boolean
  canClose: boolean
  highlights: Highlight[]
  annotations: Annotation[]
}>()

defineEmits<{
  'split': [direction: 'horizontal' | 'vertical']
  'close-tab': [tabId: string]
  'close-pane': [paneId: string]
  'activate': [paneId: string, tabId: string]
  'add-highlight': [highlight: Omit<Highlight, 'id' | 'created_at' | 'project_id'>]
  'remove-highlight': [highlightId: string]
  'add-annotation': [annotation: Omit<Annotation, 'id' | 'created_at' | 'updated_at' | 'project_id'>]
  'remove-annotation': [annotationId: string]
}>()

const activeTab = computed(() => {
  return props.pane.tabs.find((t) => t.id === props.pane.activeTabId)
})
</script>

<style lang="scss" scoped>
.editor-pane {
  height: 100%;
  width: 100%;
  display: flex;
  flex-direction: column;
  background: var(--el-bg-color);
  border: 1px solid var(--el-border-color-lighter);
  overflow: hidden;
  
  &.active {
    border-color: var(--el-color-primary-light-5);
  }
}

.tabs-container {
  display: flex;
  align-items: center;
  background: var(--el-fill-color-light);
  border-bottom: 1px solid var(--el-border-color-lighter);
  flex-shrink: 0;
}

.tabs {
  display: flex;
  flex: 1;
  overflow-x: auto;
  
  &::-webkit-scrollbar {
    height: 4px;
  }
}

.tab {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 8px 12px;
  cursor: pointer;
  border-right: 1px solid var(--el-border-color-lighter);
  white-space: nowrap;
  transition: background-color 0.2s;
  
  &:hover {
    background: var(--el-fill-color);
  }
  
  &.active {
    background: var(--el-bg-color);
    border-bottom: 2px solid var(--el-color-primary);
  }
}

.tab-name {
  font-size: 13px;
  max-width: 150px;
  overflow: hidden;
  text-overflow: ellipsis;
}

.tab-close {
  font-size: 12px;
  opacity: 0;
  transition: opacity 0.2s;
  
  .tab:hover &,
  .tab.active & {
    opacity: 1;
  }
  
  &:hover {
    color: var(--el-color-danger);
  }
}

.tabs-actions {
  padding: 0 8px;
  flex-shrink: 0;
}

.editor-container {
  flex: 1;
  min-height: 0;
  overflow: hidden;
  position: relative;
}
</style>
