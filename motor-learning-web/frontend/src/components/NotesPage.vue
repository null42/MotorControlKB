<script setup lang="ts">
import { ref, onMounted } from 'vue'
import axios from 'axios'

const emit = defineEmits<{ close: [] }>()

interface SectionAnnotation {
  docPath: string
  sectionId: string
  sectionTitle: string
  content: string
  updatedAt: string
}

interface FreeNote {
  id: string
  title: string
  content: string
  createdAt: string
  updatedAt: string
}

const activeTab = ref<'chapters' | 'free'>('chapters')
const chapterAnnotations = ref<SectionAnnotation[]>([])
const freeNotes = ref<FreeNote[]>([])
const loading = ref(true)

const editingNoteId = ref<string | null>(null)
const editingTitle = ref('')
const editingContent = ref('')
const creatingNew = ref(false)

const truncate = (text: string, maxLen: number) => {
  if (!text) return ''
  return text.length > maxLen ? text.slice(0, maxLen) + '...' : text
}

const formatDate = (dateStr: string) => {
  return new Date(dateStr).toLocaleDateString('zh-CN', {
    year: 'numeric', month: 'long', day: 'numeric', hour: '2-digit', minute: '2-digit'
  })
}

const loadData = async () => {
  loading.value = true
  try {
    const [annRes, notesRes] = await Promise.all([
      axios.get('/api/annotations/'),
      axios.get('/api/annotations/notes'),
    ])
    chapterAnnotations.value = annRes.data.annotations || []
    freeNotes.value = notesRes.data.notes || []
  } catch (e) {
    console.error(e)
  } finally {
    loading.value = false
  }
}

const groupedAnnotations = () => {
  const groups: Record<string, SectionAnnotation[]> = {}
  for (const ann of chapterAnnotations.value) {
    const path = ann.docPath || '未分类'
    if (!groups[path]) groups[path] = []
    groups[path].push(ann)
  }
  return groups
}

const startCreateNote = () => {
  creatingNew.value = true
  editingNoteId.value = null
  editingTitle.value = ''
  editingContent.value = ''
}

const cancelCreateNote = () => {
  creatingNew.value = false
  editingTitle.value = ''
  editingContent.value = ''
}

const saveNewNote = async () => {
  if (!editingTitle.value.trim()) return
  try {
    await axios.put('/api/annotations/notes', {
      title: editingTitle.value.trim(),
      content: editingContent.value,
    })
    creatingNew.value = false
    editingTitle.value = ''
    editingContent.value = ''
    await loadData()
  } catch (e) {
    console.error('Failed to save note:', e)
  }
}

const startEditNote = (note: FreeNote) => {
  editingNoteId.value = note.id
  editingTitle.value = note.title
  editingContent.value = note.content
  creatingNew.value = false
}

const cancelEditNote = () => {
  editingNoteId.value = null
  editingTitle.value = ''
  editingContent.value = ''
}

const saveEditNote = async () => {
  if (!editingNoteId.value || !editingTitle.value.trim()) return
  try {
    await axios.put(`/api/annotations/notes/${editingNoteId.value}`, {
      title: editingTitle.value.trim(),
      content: editingContent.value,
    })
    editingNoteId.value = null
    editingTitle.value = ''
    editingContent.value = ''
    await loadData()
  } catch (e) {
    console.error('Failed to update note:', e)
  }
}

const deleteNote = async (id: string) => {
  if (!window.confirm('确定删除这条笔记吗？')) return
  try {
    await axios.delete(`/api/annotations/notes/${id}`)
    await loadData()
  } catch (e) {
    console.error('Failed to delete note:', e)
  }
}

onMounted(loadData)
</script>

<template>
  <div class="notes-backdrop" @click="emit('close')"></div>
  <div class="notes-panel">
    <div class="notes-header">
      <div class="notes-title-row">
        <h2 class="notes-title">📝 我的笔记</h2>
        <button class="close-btn" @click="emit('close')">✕</button>
      </div>
      <div class="notes-tabs">
        <button
          class="tab-btn"
          :class="{ active: activeTab === 'chapters' }"
          @click="activeTab = 'chapters'"
        >
          章节注释
        </button>
        <button
          class="tab-btn"
          :class="{ active: activeTab === 'free' }"
          @click="activeTab = 'free'"
        >
          自由笔记
        </button>
      </div>
    </div>

    <div v-if="loading" class="notes-loading">加载中...</div>

    <div v-else class="notes-body">
      <template v-if="activeTab === 'chapters'">
        <div v-if="chapterAnnotations.length === 0" class="empty-state">
          <div class="empty-icon">📖</div>
          <div class="empty-text">暂无章节注释</div>
          <div class="empty-hint">阅读文档时点击标题旁的 📝 按钮即可添加</div>
        </div>
        <div v-else v-for="(annotations, docPath) in groupedAnnotations()" :key="docPath" class="annotation-group">
          <div class="group-header">
            <span class="group-icon">📂</span>
            <span class="group-path">{{ docPath }}</span>
            <span class="group-count">{{ annotations.length }}</span>
          </div>
          <div v-for="ann in annotations" :key="ann.sectionId" class="annotation-item">
            <div class="annotation-item-header">
              <span class="annotation-section-title">{{ ann.sectionTitle || ann.sectionId }}</span>
              <span class="annotation-date">{{ formatDate(ann.updatedAt) }}</span>
            </div>
            <div class="annotation-preview" v-html="truncate(ann.content, 80)"></div>
          </div>
        </div>
      </template>

      <template v-if="activeTab === 'free'">
        <div class="free-notes-toolbar">
          <button v-if="!creatingNew" class="create-btn" @click="startCreateNote">
            + 新建笔记
          </button>
          <div v-else class="create-form">
            <input
              v-model="editingTitle"
              class="create-title-input"
              placeholder="笔记标题..."
            />
            <textarea
              v-model="editingContent"
              class="create-content-input"
              placeholder="写下你的想法..."
              rows="4"
            ></textarea>
            <div class="create-actions">
              <button class="ann-btn ann-btn-save" @click="saveNewNote">保存</button>
              <button class="ann-btn" @click="cancelCreateNote">取消</button>
            </div>
          </div>
        </div>

        <div v-if="freeNotes.length === 0 && !creatingNew" class="empty-state">
          <div class="empty-icon">🗒️</div>
          <div class="empty-text">暂无自由笔记</div>
          <div class="empty-hint">点击上方按钮创建你的第一条笔记</div>
        </div>

        <div v-for="note in freeNotes" :key="note.id" class="note-card">
          <template v-if="editingNoteId === note.id">
            <input
              v-model="editingTitle"
              class="create-title-input"
              placeholder="笔记标题..."
            />
            <textarea
              v-model="editingContent"
              class="create-content-input"
              placeholder="笔记内容..."
              rows="4"
            ></textarea>
            <div class="create-actions">
              <button class="ann-btn ann-btn-save" @click="saveEditNote">保存</button>
              <button class="ann-btn" @click="cancelEditNote">取消</button>
            </div>
          </template>
          <template v-else>
            <div class="note-card-header">
              <h3 class="note-card-title">{{ note.title }}</h3>
              <span class="annotation-date">{{ formatDate(note.updatedAt) }}</span>
            </div>
            <div class="note-card-content" v-html="note.content"></div>
            <div class="note-card-actions">
              <button class="note-action-btn" @click="startEditNote(note)">编辑</button>
              <button class="note-action-btn note-action-delete" @click="deleteNote(note.id)">删除</button>
            </div>
          </template>
        </div>
      </template>
    </div>
  </div>
</template>

<style scoped>
.notes-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  background: rgba(0, 0, 0, 0.3);
  z-index: 199;
}

.notes-panel {
  position: fixed;
  top: 0;
  right: 0;
  width: 50%;
  min-width: 400px;
  max-width: 650px;
  height: 100vh;
  background: var(--content-bg);
  box-shadow: -4px 0 20px rgba(0, 0, 0, 0.15);
  z-index: 200;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.notes-header {
  padding: 1.25rem 1.5rem;
  border-bottom: 2px solid var(--primary-color);
  flex-shrink: 0;
}

.notes-title-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.notes-title {
  font-size: 1.2rem;
  color: var(--text-color);
  margin: 0;
}

.close-btn {
  background: none;
  border: 1px solid var(--border-color);
  width: 32px;
  height: 32px;
  border-radius: 6px;
  cursor: pointer;
  font-size: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.close-btn:hover {
  background-color: var(--code-bg);
}

.notes-tabs {
  display: flex;
  gap: 0.5rem;
  margin-top: 1rem;
}

.tab-btn {
  padding: 0.4rem 1rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  background: var(--bg-color);
  color: var(--text-secondary);
  cursor: pointer;
  font-size: 0.85rem;
  transition: all 0.2s;
}

.tab-btn.active {
  background: var(--primary-color);
  color: white;
  border-color: var(--primary-color);
}

.tab-btn:hover:not(.active) {
  border-color: var(--primary-color);
  color: var(--primary-color);
}

.notes-body {
  flex: 1;
  overflow-y: auto;
  padding: 1.5rem;
}

.notes-loading {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--text-secondary);
}

.empty-state {
  text-align: center;
  padding: 3rem 1rem;
  color: var(--text-secondary);
}

.empty-icon {
  font-size: 2.5rem;
  margin-bottom: 0.75rem;
}

.empty-text {
  font-size: 1rem;
  font-weight: 500;
  margin-bottom: 0.5rem;
}

.empty-hint {
  font-size: 0.8rem;
  color: var(--text-secondary);
}

.annotation-group {
  margin-bottom: 1.5rem;
}

.group-header {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem 0.75rem;
  background: var(--bg-color);
  border-radius: 6px;
  margin-bottom: 0.5rem;
  font-size: 0.85rem;
}

.group-icon {
  flex-shrink: 0;
}

.group-path {
  flex: 1;
  font-weight: 500;
  color: var(--text-color);
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.group-count {
  background: var(--primary-color);
  color: white;
  font-size: 0.7rem;
  padding: 0.1rem 0.45rem;
  border-radius: 10px;
  font-weight: 600;
}

.annotation-item {
  padding: 0.6rem 0.75rem 0.6rem 1.5rem;
  border-left: 2px solid var(--border-color);
  margin-left: 0.5rem;
  transition: border-color 0.2s;
}

.annotation-item:hover {
  border-left-color: var(--primary-color);
}

.annotation-item-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.25rem;
}

.annotation-section-title {
  font-size: 0.85rem;
  font-weight: 500;
  color: var(--text-color);
}

.annotation-date {
  font-size: 0.7rem;
  color: var(--text-secondary);
  flex-shrink: 0;
  margin-left: 0.5rem;
}

.annotation-preview {
  font-size: 0.8rem;
  color: var(--text-secondary);
  line-height: 1.5;
}

.free-notes-toolbar {
  margin-bottom: 1rem;
}

.create-btn {
  width: 100%;
  padding: 0.75rem;
  border: 2px dashed var(--border-color);
  border-radius: 8px;
  background: var(--bg-color);
  color: var(--primary-color);
  cursor: pointer;
  font-size: 0.9rem;
  font-weight: 500;
  transition: all 0.2s;
}

.create-btn:hover {
  border-color: var(--primary-color);
  background: var(--content-bg);
}

.create-form {
  padding: 1rem;
  border: 1px solid var(--border-color);
  border-radius: 8px;
  background: var(--bg-color);
}

.create-title-input {
  width: 100%;
  padding: 0.6rem 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  font-size: 0.9rem;
  background: var(--content-bg);
  color: var(--text-color);
  margin-bottom: 0.5rem;
  outline: none;
  box-sizing: border-box;
}

.create-title-input:focus {
  border-color: var(--primary-color);
}

.create-content-input {
  width: 100%;
  padding: 0.6rem 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  font-size: 0.85rem;
  background: var(--content-bg);
  color: var(--text-color);
  resize: vertical;
  font-family: inherit;
  outline: none;
  box-sizing: border-box;
}

.create-content-input:focus {
  border-color: var(--primary-color);
}

.create-actions {
  display: flex;
  gap: 0.5rem;
  margin-top: 0.5rem;
  justify-content: flex-end;
}

.ann-btn {
  padding: 0.35rem 0.85rem;
  border-radius: 4px;
  font-size: 0.8rem;
  cursor: pointer;
  border: 1px solid var(--border-color);
  background: var(--content-bg);
  color: var(--text-color);
}

.ann-btn-save {
  background: var(--primary-color);
  color: white;
  border-color: var(--primary-color);
}

.note-card {
  border: 1px solid var(--border-color);
  border-radius: 8px;
  padding: 1rem;
  margin-bottom: 0.75rem;
  background: var(--bg-color);
  transition: border-color 0.2s;
}

.note-card:hover {
  border-color: var(--primary-color);
}

.note-card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.5rem;
}

.note-card-title {
  font-size: 0.95rem;
  font-weight: 600;
  color: var(--text-color);
  margin: 0;
}

.note-card-content {
  font-size: 0.85rem;
  color: var(--text-secondary);
  line-height: 1.7;
}

.note-card-actions {
  display: flex;
  gap: 0.5rem;
  margin-top: 0.75rem;
  padding-top: 0.75rem;
  border-top: 1px solid var(--border-color);
}

.note-action-btn {
  padding: 0.25rem 0.65rem;
  border-radius: 4px;
  font-size: 0.75rem;
  cursor: pointer;
  border: 1px solid var(--border-color);
  background: var(--content-bg);
  color: var(--text-secondary);
}

.note-action-btn:hover {
  border-color: var(--primary-color);
  color: var(--primary-color);
}

.note-action-delete {
  color: #ef4444;
  border-color: #fca5a5;
}

.note-action-delete:hover {
  background: #fef2f2;
}

@media (max-width: 768px) {
  .notes-panel {
    width: 100%;
    min-width: unset;
  }
}
</style>