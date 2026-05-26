﻿﻿﻿<template>
  <div class="file-tree">
    <el-scrollbar v-if="!loading">
      <div class="tree-content">
        <TreeNode
          v-for="node in nodes"
          :key="node.path"
          :node="node"
          :level="0"
          @select="$emit('select', $event)"
        />
      </div>
      <div v-if="!nodes.length" class="empty">
        暂无文件
      </div>
    </el-scrollbar>
    <div v-else class="loading">
      <el-icon class="is-loading"><Loading /></el-icon>
      <span>加载中...</span>
    </div>
  </div>
</template>

<script setup lang="ts">
import type { FileNode } from '@/types'
import TreeNode from './TreeNode.vue'

defineProps<{
  nodes: FileNode[]
  loading?: boolean
}>()

defineEmits<{
  select: [path: string]
}>()
</script>

<style lang="scss" scoped>
.file-tree {
  height: 100%;
  display: flex;
  flex-direction: column;
}

.tree-content {
  padding: 8px 0;
}

.empty {
  padding: 24px;
  text-align: center;
  color: var(--el-text-color-placeholder);
}

.loading {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 24px;
  color: var(--el-text-color-secondary);
  gap: 8px;
}
</style>
