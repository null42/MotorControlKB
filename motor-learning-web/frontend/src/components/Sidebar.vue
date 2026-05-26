<script setup lang="ts">
import { ref, computed, onMounted } from 'vue'
import axios from 'axios'
import SearchBox from './SearchBox.vue'
import ThemeToggle from './ThemeToggle.vue'
import { useProgress } from '../stores/progress'

interface ModuleEntry {
  id: string
  title: string
  path: string
  assessment: string | null
}

interface SimulationEntry {
  id: string
  name: string
  simName: string
  docId: string
}

interface PathCategory {
  id: string
  title: string
  description: string
  modules: ModuleEntry[]
}

interface CrossRef {
  id: string
  title: string
  path: string
  assessment: string | null
}

interface ReferenceAppendix {
  id: string
  title: string
  description: string
  subSections: PathCategory[]
}

interface KnowledgeBase {
  simulations: SimulationEntry[]
  electronicsBasics: PathCategory
  controlTheory: PathCategory
  hardware: PathCategory
  algorithm: PathCategory
  mclib: PathCategory
  hpmc: PathCategory
  advHwBridge: PathCategory
  advAlgorithm: PathCategory
  sysMethodology: PathCategory
  powerPath: PathCategory
  communication: PathCategory
  mechanical: PathCategory
  crossReference: CrossRef
  referenceAppendix: ReferenceAppendix
  sectionsOrder: string[]
}

const props = defineProps<{
  collapsed: boolean
}>()

const emit = defineEmits<{
  toggle: []
  select: [filePath: string]
  startQuiz: [assessmentPath: string]
  selectSim: [simName: string]
  openNotes: []
}>()

const { isModuleVisited, isModuleCompleted, getOverallProgress } = useProgress()

const knowledgeBase = ref<KnowledgeBase | null>(null)
const activeDoc = ref('')
const expandedPaths = ref<Set<string>>(new Set())
const expandedModules = ref<Set<string>>(new Set())
const totalAnnotations = ref(0)

const sectionIconMap: Record<string, string> = {
  electronicsBasics: '🔌',
  controlTheory: '📐',
  hardware: '📄',
  algorithm: '📄',
  mclib: '💻',
  hpmc: '🏭',
  powerPath: '🔋',
  advHwBridge: '⚡',
  advAlgorithm: '🔬',
  sysMethodology: '🏗️',
  communication: '📡',
  mechanical: '⚙️',
}

const getSectionData = (key: string): PathCategory | null => {
  if (!knowledgeBase.value) return null
  const kb = knowledgeBase.value as any
  return kb[key] || null
}

const getDefaultIcon = (key: string): string => {
  return sectionIconMap[key] || '📄'
}

const isAdvancedSection = (key: string): boolean => {
  return key.startsWith('adv')
}

const overallProgress = computed(() => getOverallProgress())
const progressPercent = computed(() => {
  if (overallProgress.value.total === 0) return 0
  return Math.round((overallProgress.value.completed / overallProgress.value.total) * 100)
})

const loadDocs = async () => {
  try {
    const response = await axios.get<KnowledgeBase>('/api/docs/list')
    knowledgeBase.value = response.data
    if (knowledgeBase.value?.hardware.modules.length) {
      const firstMod = knowledgeBase.value.hardware.modules[0]
      activeDoc.value = firstMod.path
      emit('select', firstMod.path)
    }
  } catch (error) {
    console.error('Failed to load docs:', error)
  }
}

const loadAnnotationCount = async () => {
  try {
    const annRes = await axios.get('/api/annotations/')
    const list = annRes.data.annotations || []
    totalAnnotations.value = list.length
  } catch (e) {
    console.error('Failed to load annotation count:', e)
  }
}

const handleSelect = (filePath: string) => {
  activeDoc.value = filePath
  emit('select', filePath)
}

const togglePath = (pathId: string) => {
  if (expandedPaths.value.has(pathId)) {
    expandedPaths.value.delete(pathId)
  } else {
    expandedPaths.value.add(pathId)
  }
}

const toggleModule = (moduleId: string) => {
  if (expandedModules.value.has(moduleId)) {
    expandedModules.value.delete(moduleId)
  } else {
    expandedModules.value.add(moduleId)
  }
}

onMounted(() => {
  loadDocs()
  loadAnnotationCount()
})
</script>

<template>
  <aside class="sidebar" :class="{ collapsed }">
    <div class="sidebar-header">
      <h2 v-if="!collapsed">电控超级知识库</h2>
      <div class="header-actions">
        <ThemeToggle />
        <button class="toggle-btn" @click="emit('toggle')">
          {{ collapsed ? '→' : '←' }}
        </button>
      </div>
    </div>

    <SearchBox :collapsed="collapsed" @select="handleSelect" />

    <nav v-if="!collapsed && knowledgeBase" class="sidebar-nav">
      <template v-for="sectionKey in knowledgeBase.sectionsOrder" :key="sectionKey">
        <div v-if="sectionKey === 'divider-advanced'" class="section-divider">
          <span class="divider-text">高级篇</span>
        </div>
        <div v-else-if="sectionKey === 'divider-reference'" class="section-divider">
          <span class="divider-text">📚 参考附录</span>
        </div>
        <template v-else-if="sectionKey === 'referenceAppendix'">
          <div v-for="sub in knowledgeBase.referenceAppendix.subSections" :key="sub.id" class="path-section">
            <div
              class="path-header"
              :class="{ expanded: expandedPaths.has(sub.id) }"
              @click="togglePath(sub.id)"
            >
              <span class="expand-icon">{{ expandedPaths.has(sub.id) ? '▼' : '▶' }}</span>
              <span class="path-title">{{ sub.title }}</span>
            </div>
            <div v-show="expandedPaths.has(sub.id)" class="path-modules">
              <div v-for="mod in sub.modules" :key="mod.id" class="module-item">
                <div
                  class="module-header"
                  :class="{ active: activeDoc === mod.path }"
                  @click="handleSelect(mod.path)"
                >
                  <span class="module-icon">{{ isModuleCompleted(mod.id) ? '✅' : isModuleVisited(mod.id) ? '📖' : getDefaultIcon(sub.id) }}</span>
                  {{ mod.title }}
                </div>
                <div v-if="mod.assessment" class="assessment-link">
                  <div
                    class="nav-item sub-item"
                    :class="{ active: activeDoc === mod.assessment }"
                    @click="emit('startQuiz', mod.assessment)"
                  >
                    <span class="module-icon">📝</span>
                    检验题目
                  </div>
                </div>
              </div>
            </div>
          </div>
        </template>
        <div v-else class="path-section">
          <div
            class="path-header"
            :class="[isAdvancedSection(sectionKey) ? 'adv-path' : '', { expanded: expandedPaths.has(sectionKey) }]"
            @click="togglePath(sectionKey)"
          >
            <span class="expand-icon">{{ expandedPaths.has(sectionKey) ? '▼' : '▶' }}</span>
            <span class="path-title">{{ getSectionData(sectionKey)?.title }}</span>
          </div>
          <div v-show="expandedPaths.has(sectionKey)" class="path-modules">
            <div v-for="mod in getSectionData(sectionKey)?.modules" :key="mod.id" class="module-item">
              <div
                class="module-header"
                :class="{ active: activeDoc === mod.path }"
                @click="handleSelect(mod.path)"
              >
                <span class="module-icon">{{ isModuleCompleted(mod.id) ? '✅' : isModuleVisited(mod.id) ? '📖' : getDefaultIcon(sectionKey) }}</span>
                {{ mod.title }}
              </div>
              <div v-if="mod.assessment" class="assessment-link">
                <div
                  class="nav-item sub-item"
                  :class="{ active: activeDoc === mod.assessment }"
                  @click="emit('startQuiz', mod.assessment)"
                >
                  <span class="module-icon">📝</span>
                  检验题目
                </div>
              </div>
            </div>
          </div>
        </div>
      </template>
      <!-- 交叉引用 -->
      <div class="path-section">
        <div
          class="nav-item cross-ref-item"
          :class="{ active: activeDoc === knowledgeBase.crossReference.path }"
          @click="handleSelect(knowledgeBase.crossReference.path)"
        >
          {{ knowledgeBase.crossReference.title }}
        </div>
        <div v-if="knowledgeBase.crossReference.assessment" class="assessment-link">
          <div
            class="nav-item sub-item"
            :class="{ active: activeDoc === knowledgeBase.crossReference.assessment }"
            @click="emit('startQuiz', knowledgeBase.crossReference.assessment)"
          >
            <span class="module-icon">📝</span>
            检验题目
          </div>
        </div>
      </div>
      <!-- 仿真实验 -->
      <div v-if="knowledgeBase?.simulations?.length" class="path-section">
        <div
          class="path-header sim-path"
          :class="{ expanded: expandedPaths.has('simulations') }"
          @click="togglePath('simulations')"
        >
          <span class="expand-icon">{{ expandedPaths.has('simulations') ? '▼' : '▶' }}</span>
          <span class="path-title">🔬 仿真实验</span>
        </div>
        <div v-if="expandedPaths.has('simulations')" class="path-modules">
          <div
            v-for="sim in knowledgeBase.simulations"
            :key="sim.id"
            class="module-item sim-item"
            @click="emit('selectSim', sim.simName)"
          >
            <span class="module-name">🎮 {{ sim.name }}</span>
          </div>
        </div>
      </div>
      <!-- 我的笔记 -->
      <div class="notes-entry">
        <div class="nav-item notes-nav-item" @click="emit('openNotes')">
          <span class="module-icon">📝</span>
          我的笔记
          <span v-if="totalAnnotations > 0" class="badge">{{ totalAnnotations }}</span>
        </div>
      </div>
    </nav>

    <!-- 进度摘要 -->
    <div v-if="!collapsed" class="progress-summary">
      <div class="progress-bar-container">
        <div class="progress-bar" :style="{ width: progressPercent + '%' }"></div>
      </div>
      <div class="progress-text">
        学习进度：{{ overallProgress.completed }} / {{ overallProgress.total }}（{{ progressPercent }}%）
      </div>
    </div>
  </aside>
</template>

<style scoped>
.sidebar {
  width: 300px;
  background-color: var(--sidebar-bg);
  border-right: 1px solid var(--border-color);
  display: flex;
  flex-direction: column;
  transition: width 0.3s ease;
  box-shadow: var(--shadow);
  position: fixed;
  left: 0;
  top: 0;
  height: 100vh;
  z-index: 100;
}

.sidebar.collapsed {
  width: 50px;
}

.sidebar-header {
  padding: 1rem;
  border-bottom: 1px solid var(--border-color);
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.sidebar-header h2 {
  font-size: 1.1rem;
  font-weight: 700;
  color: var(--primary-color);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.header-actions {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.toggle-btn {
  background: none;
  border: 1px solid var(--border-color);
  padding: 0.25rem 0.5rem;
  cursor: pointer;
  border-radius: 4px;
  font-size: 0.875rem;
  flex-shrink: 0;
}

.toggle-btn:hover {
  background-color: var(--bg-color);
}

.sidebar-nav {
  flex: 1;
  overflow-y: auto;
  padding: 0.5rem 0;
}

.path-section {
  margin-bottom: 0.25rem;
}

.path-header {
  padding: 0.6rem 0.75rem;
  cursor: pointer;
  font-weight: 600;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  transition: background-color 0.2s;
  font-size: 0.9rem;
  color: var(--text-color);
  border-left: 3px solid var(--primary-color);
}

.path-header:hover {
  background-color: var(--bg-color);
}

.expand-icon {
  font-size: 0.7rem;
  color: var(--text-secondary);
  flex-shrink: 0;
}

.path-title {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.path-modules {
  padding-left: 0.5rem;
}

.module-item {
  margin: 0;
}

.module-header {
  padding: 0.45rem 0.75rem 0.45rem 1.5rem;
  cursor: pointer;
  font-size: 0.85rem;
  display: flex;
  align-items: center;
  gap: 0.35rem;
  transition: all 0.2s;
  border-radius: 0 6px 6px 0;
  color: var(--text-color);
}

.module-header:hover {
  background-color: var(--bg-color);
}

.module-header.active {
  background-color: var(--primary-color);
  color: white;
}

.module-icon {
  font-size: 0.8rem;
  flex-shrink: 0;
}

.nav-item {
  padding: 0.45rem 0.75rem;
  margin: 0;
  cursor: pointer;
  border-radius: 6px;
  transition: all 0.2s;
  font-size: 0.85rem;
}

.nav-item:hover {
  background-color: var(--bg-color);
}

.nav-item.active {
  background-color: var(--primary-color);
  color: white;
}

.sub-item {
  padding: 0.35rem 0.75rem 0.35rem 2.5rem;
  font-size: 0.8rem;
  color: var(--text-secondary);
}

.assessment-link {
  margin: 0;
}

.cross-ref-item {
  margin: 0.5rem 0.75rem;
  font-weight: 500;
  text-align: center;
  border: 1px dashed var(--border-color);
  border-radius: 6px;
  padding: 0.6rem;
}

.section-divider {
  margin: 0.75rem 0.75rem 0.25rem;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.section-divider::before,
.section-divider::after {
  content: '';
  flex: 1;
  height: 1px;
  background: linear-gradient(to right, transparent, var(--border-color), transparent);
}

.divider-text {
  font-size: 0.75rem;
  color: var(--text-secondary);
  white-space: nowrap;
  font-weight: 600;
  letter-spacing: 0.1em;
}

.adv-path {
  border-left-color: #e67e22;
}

.sim-path {
  border-left-color: #27ae60;
}

.sim-item {
  padding: 0.45rem 0.75rem 0.45rem 1.5rem;
  cursor: pointer;
  font-size: 0.85rem;
  display: flex;
  align-items: center;
  gap: 0.35rem;
  transition: all 0.2s;
  border-radius: 0 6px 6px 0;
  color: var(--text-color);
}

.sim-item:hover {
  background-color: var(--bg-color);
}

.module-name {
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.notes-entry {
  padding: 0.25rem 0.75rem;
}

.notes-nav-item {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.55rem 0.75rem;
  margin: 0.25rem 0;
  cursor: pointer;
  border-radius: 6px;
  transition: all 0.2s;
  font-size: 0.85rem;
  font-weight: 500;
  border: 1px dashed var(--border-color);
}

.notes-nav-item:hover {
  background-color: var(--bg-color);
  border-color: var(--primary-color);
}

.badge {
  margin-left: auto;
  background-color: var(--primary-color);
  color: white;
  font-size: 0.7rem;
  padding: 0.15rem 0.45rem;
  border-radius: 10px;
  font-weight: 600;
}

.progress-summary {
  padding: 0.75rem;
  border-top: 1px solid var(--border-color);
  margin-top: auto;
}

.progress-bar-container {
  height: 6px;
  background-color: var(--code-bg);
  border-radius: 3px;
  overflow: hidden;
  margin-bottom: 0.5rem;
}

.progress-bar {
  height: 100%;
  background-color: var(--primary-color);
  border-radius: 3px;
  transition: width 0.3s ease;
}

.progress-text {
  font-size: 0.75rem;
  color: var(--text-secondary);
  text-align: center;
}

@media (max-width: 768px) {
  .sidebar {
    transform: translateX(-100%);
  }

  .sidebar:not(.collapsed) {
    transform: translateX(0);
  }
}
</style>
