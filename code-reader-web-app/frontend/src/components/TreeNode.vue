<template>
  <div
    class="tree-node"
    :style="{ paddingLeft: level * 16 + 8 + 'px' }"
    :class="{ active: isActive }"
  >
    <div class="node-content" @click="handleClick">
      <el-icon v-if="node.type === 'directory'" class="folder-icon">
        <Folder v-if="!expanded" />
        <FolderOpened v-else />
      </el-icon>
      <el-icon v-else class="file-icon">
        <Document />
      </el-icon>
      <span class="node-name">{{ node.name }}</span>
    </div>
    
    <div v-if="node.type === 'directory' && expanded && node.children" class="children">
      <TreeNode
        v-for="child in node.children"
        :key="child.path"
        :node="child"
        :level="level + 1"
        @select="$emit('select', $event)"
      />
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import type { FileNode } from '@/types'

const props = defineProps<{
  node: FileNode
  level: number
}>()

const emit = defineEmits<{
  select: [path: string]
}>()

const expanded = ref(false)

const isActive = computed(() => false)

const handleClick = () => {
  if (props.node.type === 'directory') {
    expanded.value = !expanded.value
  } else {
    emit('select', props.node.path)
  }
}
</script>

<style lang="scss" scoped>
.tree-node {
  user-select: none;
}

.node-content {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 4px 8px;
  cursor: pointer;
  border-radius: 4px;
  transition: background-color 0.2s;
  
  &:hover {
    background: var(--el-fill-color-light);
  }
}

.folder-icon {
  color: var(--el-color-warning);
}

.file-icon {
  color: var(--el-text-color-secondary);
}

.node-name {
  font-size: 13px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.children {
  border-left: 1px solid var(--el-border-color-lighter);
  margin-left: 12px;
}

.active > .node-content {
  background: var(--el-color-primary-light-9);
  color: var(--el-color-primary);
}
</style>
