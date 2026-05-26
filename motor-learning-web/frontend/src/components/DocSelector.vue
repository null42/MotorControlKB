<script setup lang="ts">
import { ref, onMounted } from 'vue'
import axios from 'axios'

interface DocInfo {
  id: string
  title: string
  category: string
}

const docs = ref<DocInfo[]>([])
const selectedDoc = ref('')

const emit = defineEmits<{
  select: [docId: string]
}>()

const loadDocs = async () => {
  try {
    const response = await axios.get<DocInfo[]>('/api/docs')
    docs.value = response.data
  } catch (error) {
    console.error('Failed to load docs:', error)
  }
}

const handleChange = () => {
  emit('select', selectedDoc.value)
}

onMounted(() => {
  loadDocs()
})
</script>

<template>
  <div class="doc-selector">
    <select v-model="selectedDoc" @change="handleChange">
      <option value="" disabled>选择文档</option>
      <optgroup
        v-for="category in [...new Set(docs.map(d => d.category))]"
        :key="category"
        :label="category"
      >
        <option
          v-for="doc in docs.filter(d => d.category === category)"
          :key="doc.id"
          :value="doc.id"
        >
          {{ doc.title }}
        </option>
      </optgroup>
    </select>
  </div>
</template>

<style scoped>
.doc-selector {
  padding: 1rem;
}

.doc-selector select {
  width: 100%;
  padding: 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  font-size: 0.875rem;
  background-color: white;
  cursor: pointer;
}

.doc-selector select:focus {
  outline: none;
  border-color: var(--primary-color);
}
</style>
