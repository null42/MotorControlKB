<script setup lang="ts">
import { ref, onMounted, watch } from 'vue'

const props = defineProps<{
  content: string
}>()

const processedContent = ref('')

const processAnswerBlocks = (html: string): string => {
  const answerRegex = /(<h[1-6][^>]*>.*?(答案|解答|参考答案).*?<\/h[1-6]>)([\s\S]*?)(?=<h[1-6]|$)/gi
  
  return html.replace(answerRegex, (match, header, _, body) => {
    const id = `answer-${Math.random().toString(36).substr(2, 9)}`
    return `
      <div class="answer-block">
        <div class="answer-header" onclick="this.parentElement.classList.toggle('expanded')">
          <span class="answer-icon">▶</span>
          ${header}
        </div>
        <div class="answer-content">${body}</div>
      </div>
    `
  })
}

watch(() => props.content, (newContent) => {
  processedContent.value = processAnswerBlocks(newContent)
}, { immediate: true })
</script>

<template>
  <div class="answer-wrapper" v-html="processedContent"></div>
</template>

<style scoped>
.answer-wrapper {
  width: 100%;
}

.answer-wrapper :deep(.answer-block) {
  margin: 1rem 0;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  overflow: hidden;
}

.answer-wrapper :deep(.answer-header) {
  display: flex;
  align-items: center;
  padding: 0.75rem 1rem;
  background-color: var(--bg-color);
  cursor: pointer;
  user-select: none;
}

.answer-wrapper :deep(.answer-header:hover) {
  background-color: var(--code-bg);
}

.answer-wrapper :deep(.answer-icon) {
  margin-right: 0.5rem;
  transition: transform 0.3s ease;
  font-size: 0.75rem;
}

.answer-wrapper :deep(.answer-block.expanded .answer-icon) {
  transform: rotate(90deg);
}

.answer-wrapper :deep(.answer-content) {
  max-height: 0;
  overflow: hidden;
  transition: max-height 0.3s ease;
  padding: 0 1rem;
}

.answer-wrapper :deep(.answer-block.expanded .answer-content) {
  max-height: 2000px;
  padding: 1rem;
}
</style>
