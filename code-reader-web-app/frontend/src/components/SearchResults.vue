﻿﻿﻿<template>
  <div class="search-results">
    <div v-if="results.length === 0" class="empty">
      无搜索结果
    </div>
    <div v-else class="results-list">
      <div
        v-for="(result, index) in results"
        :key="index"
        class="result-item"
        @click="$emit('select', result)"
      >
        <div class="result-header">
          <span class="result-name">{{ result.name }}</span>
          <span class="result-kind">{{ result.kind }}</span>
        </div>
        <div class="result-location">
          {{ result.location.file_path }}:{{ result.location.line }}
        </div>
        <div v-if="result.preview" class="result-preview">
          {{ result.preview }}
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import type { Symbol } from '@/types'

defineProps<{
  results: Symbol[]
}>()

defineEmits<{
  select: [result: Symbol]
}>()
</script>

<style lang="scss" scoped>
.search-results {
  height: 100%;
}

.empty {
  padding: 24px;
  text-align: center;
  color: var(--el-text-color-placeholder);
}

.results-list {
  overflow-y: auto;
}

.result-item {
  padding: 12px;
  border-bottom: 1px solid var(--el-border-color-lighter);
  cursor: pointer;
  transition: background-color 0.2s;
  
  &:hover {
    background: var(--el-fill-color-light);
  }
}

.result-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 4px;
}

.result-name {
  font-weight: 500;
  color: var(--el-text-color-primary);
}

.result-kind {
  font-size: 12px;
  color: var(--el-text-color-secondary);
  background: var(--el-fill-color);
  padding: 2px 6px;
  border-radius: 4px;
}

.result-location {
  font-size: 12px;
  color: var(--el-text-color-secondary);
  margin-bottom: 4px;
}

.result-preview {
  font-size: 12px;
  color: var(--el-text-color-regular);
  background: var(--el-fill-color-lighter);
  padding: 4px 8px;
  border-radius: 4px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
</style>
