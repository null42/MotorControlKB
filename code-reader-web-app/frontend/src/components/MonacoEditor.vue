﻿<template>
  <div ref="editorContainer" class="monaco-editor"></div>
  
  <el-dialog v-model="highlightDialogVisible" title="添加高亮" width="300px">
    <div class="highlight-dialog">
      <p>选择高亮颜色：</p>
      <div class="color-picker">
        <div
          v-for="color in highlightColors"
          :key="color.name"
          class="color-option"
          :class="{ selected: selectedColor === color.name }"
          :style="{ backgroundColor: color.bg }"
          @click="selectedColor = color.name"
        />
      </div>
    </div>
    <template #footer>
      <el-button @click="highlightDialogVisible = false">取消</el-button>
      <el-button type="primary" @click="confirmHighlight">确定</el-button>
    </template>
  </el-dialog>
  
  <el-dialog v-model="annotationDialogVisible" title="添加注释" width="400px">
    <el-input
      v-model="annotationContent"
      type="textarea"
      :rows="4"
      placeholder="输入注释内容..."
    />
    <template #footer>
      <el-button @click="annotationDialogVisible = false">取消</el-button>
      <el-button type="primary" @click="confirmAnnotation">确定</el-button>
    </template>
  </el-dialog>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch, nextTick } from 'vue'
import * as monaco from 'monaco-editor'
import type { Highlight, Annotation } from '@/types'

const props = defineProps<{
  modelValue: string
  language: string
  path: string
  highlights: Highlight[]
  annotations: Annotation[]
}>()

const emit = defineEmits<{
  highlight: [data: { 
    start_line: number
    start_column: number
    end_line: number
    end_column: number
    color: string
    file_path: string
  }]
  'remove-highlight': [highlightId: string]
  annotate: [data: { line_number: number; content: string; file_path: string }]
  'remove-annotation': [annotationId: string]
}>()

const editorContainer = ref<HTMLDivElement | null>(null)
let editor: monaco.editor.IStandaloneCodeEditor | null = null
let decorations: string[] = []

const highlightDialogVisible = ref(false)
const selectedColor = ref('yellow')
const pendingSelection = ref<{ startLine: number; startCol: number; endLine: number; endCol: number } | null>(null)

const annotationDialogVisible = ref(false)
const annotationContent = ref('')
const pendingAnnotationLine = ref(0)

const highlightColors = [
  { name: 'yellow', bg: 'rgba(255, 235, 59, 0.5)' },
  { name: 'green', bg: 'rgba(76, 175, 80, 0.5)' },
  { name: 'blue', bg: 'rgba(33, 150, 243, 0.5)' },
  { name: 'red', bg: 'rgba(244, 67, 54, 0.5)' },
  { name: 'purple', bg: 'rgba(156, 39, 176, 0.5)' },
  { name: 'orange', bg: 'rgba(255, 152, 0, 0.5)' },
]

onMounted(async () => {
  if (!editorContainer.value) return

  await nextTick()

  try {
    editor = monaco.editor.create(editorContainer.value, {
      value: props.modelValue,
      language: props.language,
      theme: 'vs',
      readOnly: true,
      minimap: { enabled: true },
      fontSize: 14,
      lineNumbers: 'on',
      renderLineHighlight: 'all',
      scrollBeyondLastLine: false,
      automaticLayout: true,
      wordWrap: 'on',
      folding: true,
      glyphMargin: true,
    })

    editor.addAction({
      id: 'add-highlight',
      label: '添加高亮',
      contextMenuGroupId: 'navigation',
      run: (ed) => {
        const selection = ed.getSelection()
        if (selection) {
          pendingSelection.value = {
            startLine: selection.startLineNumber,
            startCol: selection.startColumn,
            endLine: selection.endLineNumber,
            endCol: selection.endColumn,
          }
          selectedColor.value = 'yellow'
          highlightDialogVisible.value = true
        }
      },
    })

    editor.addAction({
      id: 'remove-highlight',
      label: '删除此高亮',
      contextMenuGroupId: 'navigation',
      run: (ed) => {
        const position = ed.getPosition()
        if (position) {
          const highlight = findHighlightAtPosition(position.lineNumber, position.column)
          if (highlight) {
            emit('remove-highlight', highlight.id)
          }
        }
      },
    })

    editor.addAction({
      id: 'add-annotation',
      label: '添加注释',
      contextMenuGroupId: 'navigation',
      run: (ed) => {
        const position = ed.getPosition()
        if (position) {
          pendingAnnotationLine.value = position.lineNumber
          annotationContent.value = ''
          annotationDialogVisible.value = true
        }
      },
    })

    editor.addAction({
      id: 'remove-annotation',
      label: '删除此行注释',
      contextMenuGroupId: 'navigation',
      run: (ed) => {
        const position = ed.getPosition()
        if (position) {
          const annotation = findAnnotationAtLine(position.lineNumber)
          if (annotation) {
            emit('remove-annotation', annotation.id)
          }
        }
      },
    })

    updateDecorations()
  } catch (error) {
    console.error('Failed to create Monaco editor:', error)
  }
})

onUnmounted(() => {
  editor?.dispose()
})

watch(
  () => props.modelValue,
  (newValue) => {
    if (editor) {
      editor.setValue(newValue)
    }
  }
)

watch(
  () => props.language,
  (newLang) => {
    if (editor) {
      const model = editor.getModel()
      if (model) {
        monaco.editor.setModelLanguage(model, newLang)
      }
    }
  }
)

watch(
  () => props.highlights,
  () => {
    updateDecorations()
  },
  { deep: true }
)

watch(
  () => props.annotations,
  () => {
    updateDecorations()
  },
  { deep: true }
)

const findHighlightAtPosition = (line: number, column: number): Highlight | undefined => {
  return props.highlights.find((h) => {
    if (h.file_path !== props.path) return false
    if (line < h.start_line || line > h.end_line) return false
    if (line === h.start_line && column < h.start_column) return false
    if (line === h.end_line && column > h.end_column) return false
    return true
  })
}

const findAnnotationAtLine = (line: number): Annotation | undefined => {
  return props.annotations.find((a) => a.file_path === props.path && a.line_number === line)
}

const updateDecorations = () => {
  if (!editor) return

  const model = editor.getModel()
  if (!model) return

  const newDecorations: monaco.editor.IModelDeltaDecoration[] = []

  props.highlights.forEach((h) => {
    if (h.file_path === props.path) {
      newDecorations.push({
        range: new monaco.Range(h.start_line, h.start_column, h.end_line, h.end_column),
        options: {
          className: `highlight-${h.color}`,
          hoverMessage: { value: '**高亮区域** - 右键可删除' },
        },
      })
    }
  })

  props.annotations.forEach((a) => {
    if (a.file_path === props.path) {
      newDecorations.push({
        range: new monaco.Range(a.line_number, 1, a.line_number, 1),
        options: {
          isWholeLine: false,
          glyphMarginClassName: 'annotation-glyph',
          glyphMarginHoverMessage: { value: `**注释:** ${a.content}` },
          linesDecorationsClassName: 'annotation-line-decoration',
        },
      })
    }
  })

  decorations = editor.deltaDecorations(decorations, newDecorations)
}

const confirmHighlight = () => {
  if (pendingSelection.value) {
    emit('highlight', {
      start_line: pendingSelection.value.startLine,
      start_column: pendingSelection.value.startCol,
      end_line: pendingSelection.value.endLine,
      end_column: pendingSelection.value.endCol,
      color: selectedColor.value,
      file_path: props.path,
    })
    highlightDialogVisible.value = false
    pendingSelection.value = null
  }
}

const confirmAnnotation = () => {
  if (annotationContent.value.trim()) {
    emit('annotate', {
      line_number: pendingAnnotationLine.value,
      content: annotationContent.value.trim(),
      file_path: props.path,
    })
    annotationDialogVisible.value = false
  }
}
</script>

<style lang="scss" scoped>
.monaco-editor {
  width: 100%;
  height: 100%;
}

.highlight-dialog {
  .color-picker {
    display: flex;
    gap: 12px;
    margin-top: 12px;
  }

  .color-option {
    width: 36px;
    height: 36px;
    border-radius: 6px;
    cursor: pointer;
    border: 2px solid transparent;
    transition: all 0.2s;

    &:hover {
      transform: scale(1.1);
    }

    &.selected {
      border-color: var(--el-color-primary);
      box-shadow: 0 0 0 2px var(--el-color-primary-light-5);
    }
  }
}

:deep(.highlight-yellow) {
  background-color: rgba(255, 235, 59, 0.4);
}

:deep(.highlight-green) {
  background-color: rgba(76, 175, 80, 0.4);
}

:deep(.highlight-blue) {
  background-color: rgba(33, 150, 243, 0.4);
}

:deep(.highlight-red) {
  background-color: rgba(244, 67, 54, 0.4);
}

:deep(.highlight-purple) {
  background-color: rgba(156, 39, 176, 0.4);
}

:deep(.highlight-orange) {
  background-color: rgba(255, 152, 0, 0.4);
}

:deep(.annotation-glyph) {
  background: var(--el-color-warning);
  border-radius: 50%;
  width: 10px;
  height: 10px;
  margin-left: 3px;
  margin-top: 4px;
  cursor: pointer;
  
  &:hover {
    background: var(--el-color-warning-dark-2);
  }
}

:deep(.annotation-line-decoration) {
  background: var(--el-color-warning-light-8);
  width: 4px !important;
  margin-left: 3px;
}
</style>
