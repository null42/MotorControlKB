<script setup lang="ts">
import { ref } from 'vue'
import axios from 'axios'

interface Match {
  line: number
  context: string
}

interface SearchResult {
  filename: string
  matches: Match[]
}

const props = defineProps<{
  collapsed: boolean
}>()

const emit = defineEmits<{
  select: [filePath: string]
}>()

const searchQuery = ref('')
const searchResults = ref<SearchResult[]>([])
const searching = ref(false)
const showResults = ref(false)

const handleSearch = async () => {
  if (!searchQuery.value.trim()) {
    searchResults.value = []
    showResults.value = false
    return
  }

  searching.value = true
  try {
    const response = await axios.get(`/api/docs/search/${encodeURIComponent(searchQuery.value)}`)
    searchResults.value = response.data.results
    showResults.value = true
  } catch (error) {
    console.error('Search failed:', error)
  } finally {
    searching.value = false
  }
}

const handleSelectResult = (filePath: string) => {
  emit('select', filePath)
  showResults.value = false
  searchQuery.value = ''
}

const getDisplayLabel = (filePath: string): string => {
  const parts = filePath.split('/')
  const filename = parts[parts.length - 1]
  if (parts.length >= 2) {
    const category = parts[0] === 'hardware' ? '🔧' : parts[0] === 'algorithm' ? '🧮' : '🔗'
    return `${category} ${filename.replace('.md', '')}`
  }
  return filename.replace('.md', '')
}

const clearSearch = () => {
  searchQuery.value = ''
  searchResults.value = []
  showResults.value = false
}
</script>

<template>
  <div class="search-container" :class="{ collapsed }">
    <div class="search-box">
      <input
        v-model="searchQuery"
        type="text"
        placeholder="搜索知识库..."
        @keyup.enter="handleSearch"
        class="search-input"
      />
      <button v-if="searchQuery" @click="clearSearch" class="clear-btn">×</button>
      <button @click="handleSearch" class="search-btn" :disabled="searching">
        {{ searching ? '...' : '🔍' }}
      </button>
    </div>

    <div v-if="showResults && searchResults.length > 0" class="search-results">
      <div class="results-header">
        找到 {{ searchResults.length }} 个结果
        <button @click="showResults = false" class="close-btn">×</button>
      </div>
      <div
        v-for="result in searchResults"
        :key="result.filename"
        class="result-item"
        @click="handleSelectResult(result.filename)"
      >
        <div class="result-filename">{{ getDisplayLabel(result.filename) }}</div>
        <div class="result-matches">
          {{ result.matches.length }} 处匹配
        </div>
      </div>
    </div>

    <div v-else-if="showResults && searchResults.length === 0" class="no-results">
      未找到相关内容
    </div>
  </div>
</template>

<style scoped>
.search-container {
  padding: 0.75rem;
  border-bottom: 1px solid var(--border-color);
}

.search-container.collapsed {
  display: none;
}

.search-box {
  display: flex;
  gap: 0.5rem;
  align-items: center;
}

.search-input {
  flex: 1;
  padding: 0.5rem 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  font-size: 0.875rem;
  outline: none;
}

.search-input:focus {
  border-color: var(--primary-color);
}

.clear-btn,
.search-btn {
  background: none;
  border: 1px solid var(--border-color);
  padding: 0.5rem;
  cursor: pointer;
  border-radius: 6px;
  font-size: 0.875rem;
}

.search-btn:hover {
  background-color: var(--primary-color);
  color: white;
  border-color: var(--primary-color);
}

.search-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.search-results {
  margin-top: 0.75rem;
  max-height: 300px;
  overflow-y: auto;
  border: 1px solid var(--border-color);
  border-radius: 6px;
}

.results-header {
  padding: 0.5rem 0.75rem;
  background-color: var(--bg-color);
  font-size: 0.75rem;
  color: var(--text-secondary);
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.close-btn {
  background: none;
  border: none;
  cursor: pointer;
  font-size: 1rem;
  color: var(--text-secondary);
}

.result-item {
  padding: 0.5rem 0.75rem;
  cursor: pointer;
  border-bottom: 1px solid var(--border-color);
}

.result-item:last-child {
  border-bottom: none;
}

.result-item:hover {
  background-color: var(--bg-color);
}

.result-filename {
  font-size: 0.85rem;
  font-weight: 500;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.result-matches {
  font-size: 0.75rem;
  color: var(--text-secondary);
}

.no-results {
  margin-top: 0.75rem;
  padding: 0.75rem;
  text-align: center;
  color: var(--text-secondary);
  font-size: 0.875rem;
}
</style>
