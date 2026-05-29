<script setup lang="ts">
import { ref, watch, onMounted, nextTick, createApp, computed } from 'vue'
import type { Component } from 'vue'
import axios from 'axios'
import { renderMarkdown, extractToc } from '../utils/markdown'
import TocNav from './TocNav.vue'
import ParkTransformSim from './simulations/ParkTransformSim.vue'
import BodePlotSim from './simulations/BodePlotSim.vue'
import PIDTunerSim from './simulations/PIDTunerSim.vue'
import MosfetSwitchingSim from './simulations/MosfetSwitchingSim.vue'
import SVPWMSim from './simulations/SVPWMSim.vue'
import RLCSim from './simulations/RLCSim.vue'
import EmachenySim from './simulations/EmachenySim.vue'
import ClarkeParkSim from './simulations/ClarkeParkSim.vue'
import SamplingTimingSim from './simulations/SamplingTimingSim.vue'
import OvermodulationSim from './simulations/OvermodulationSim.vue'
import CurrentLoopSim from './simulations/CurrentLoopSim.vue'
import ObserverSim from './simulations/ObserverSim.vue'
import MTPASim from './simulations/MTPASim.vue'
import THDSim from './simulations/THDSim.vue'
import DeadtimeCompSim from './simulations/DeadtimeCompSim.vue'
import InitialPositionSim from './simulations/InitialPositionSim.vue'
import SpeedLoopSim from './simulations/SpeedLoopSim.vue'
import { useProgress } from '../stores/progress'

const SimRegistry: Record<string, Component> = {
  'park-transform': ParkTransformSim,
  'bode-plot': BodePlotSim,
  'pid-tuner': PIDTunerSim,
  'mosfet-switching': MosfetSwitchingSim,
  'svpwm': SVPWMSim,
  'rlc': RLCSim,
  'emacheny': EmachenySim,
  'clarke-park': ClarkeParkSim,
  'sampling-timing': SamplingTimingSim,
  'overmodulation': OvermodulationSim,
  'current-loop': CurrentLoopSim,
  'observer': ObserverSim,
  'mtpa': MTPASim,
  'thd': THDSim,
  'deadtime-comp': DeadtimeCompSim,
  'initial-position': InitialPositionSim,
  'speed-loop': SpeedLoopSim,
}

const props = defineProps<{
  sidebarCollapsed: boolean
  currentDoc: string
  simulations?: Array<{ id: string; name: string; simName: string; docId: string }>
}>()

const emit = defineEmits<{
  navigate: [filePath: string]
  selectSim: [simName: string]
}>()

const { markVisited, updateReadingProgress } = useProgress()

const content = ref('')
const loading = ref(false)
const errorMsg = ref('')
const contentRef = ref<HTMLElement | null>(null)
const navigationStack = ref<string[]>([])
const tocHeadings = ref<Array<{ level: number; text: string; id: string }>>([])
const readingProgress = ref(0)

const relatedSims = computed(() => {
  if (!props.simulations || !props.currentDoc) return []
  return props.simulations.filter(sim => sim.docId === props.currentDoc)
})

const annotations = ref<Record<string, { sectionId: string; sectionTitle: string; content: string }>>({})
const activeAnnotationId = ref<string | null>(null)
const annotationDraft = ref('')
const annotationDirty = ref(false)
const annotationEditMode = ref(false)
const savedToast = ref(false)

const loadContent = async (docId: string) => {
  if (!docId) return

  loading.value = true
  readingProgress.value = 0
  errorMsg.value = ''

  if (docId.endsWith('.html')) {
    try {
      content.value = `<iframe src="/api/docs/file/${docId}" style="width:100%;height:calc(100vh - 80px);border:none;" sandbox="allow-scripts allow-same-origin"></iframe>`
      markVisited(docId)
      scrollToTop()
    } catch (err) {
      errorMsg.value = `加载HTML工具失败: ${err}`
    }
    loading.value = false
    return
  }

  try {
    const response = await axios.get(`/api/docs/file/${docId}`, {
      responseType: 'text',
    })
    const rawContent = typeof response.data === 'string' ? response.data : String(response.data)
    if (!rawContent || rawContent.length < 10) {
      throw new Error(`Empty or invalid response for ${docId}`)
    }
    try {
      tocHeadings.value = extractToc(rawContent)
    } catch (tocErr) {
      console.error('extractToc error:', tocErr)
    }
    try {
      content.value = renderMarkdown(rawContent)
    } catch (renderErr) {
      console.error('renderMarkdown error:', renderErr)
      content.value = `<pre>${rawContent.replace(/</g, '&lt;').replace(/>/g, '&gt;')}</pre>`
    }
    content.value = content.value.replace(/<p>:::sim\s+([a-z0-9-]+)<\/p>/gi, '<div data-sim="$1"></div>')
    content.value = content.value.replace(/<p>:::sim-html\s+(\S+\.html)<\/p>/gi, '<div data-sim-html="$1"></div>')
    markVisited(docId)
    await nextTick()
    try {
      processAnswerBlocks()
    } catch (domErr) {
      console.error('processAnswerBlocks error:', domErr)
    }
    try {
      processCrossReferenceLinks()
    } catch (linkErr) {
      console.error('processCrossReferenceLinks error:', linkErr)
    }
    try {
      processSimBlocks()
    } catch (simErr) {
      console.error('processSimBlocks error:', simErr)
    }
    try {
      processHtmlSimBlocks()
    } catch (htmlSimErr) {
      console.error('processHtmlSimBlocks error:', htmlSimErr)
    }
    scrollToTop()
    try {
      const annRes = await axios.get(`/api/annotations/?docPath=${encodeURIComponent(docId)}`)
      const list = annRes.data.annotations || []
      const map: Record<string, any> = {}
      list.forEach((a: any) => { map[a.sectionId] = a })
      annotations.value = map
    } catch (e) {
      console.error('Failed to load annotations:', e)
    }
    await nextTick()
    processAnnotationButtons()
  } catch (error: any) {
    console.error('Failed to load content:', error)
    const detail = error?.response?.status
      ? `HTTP ${error.response.status}: ${error.response.statusText}`
      : error?.message || String(error)
    errorMsg.value = `加载文档失败: ${detail}`
    content.value = ''
  } finally {
    loading.value = false
  }
}

const processAnswerBlocks = () => {
  if (!contentRef.value) return

  const headings = contentRef.value.querySelectorAll('h1, h2, h3, h4, h5, h6')
  headings.forEach(heading => {
    const text = heading.textContent || ''
    if (text.includes('答案') || text.includes('解答') || text.includes('参考答案')) {
      const wrapper = document.createElement('div')
      wrapper.className = 'answer-block'

      const header = document.createElement('div')
      header.className = 'answer-header'
      header.innerHTML = `<span class="answer-icon">▶</span>${heading.outerHTML}`

      const answerContent = document.createElement('div')
      answerContent.className = 'answer-content'

      let sibling = heading.nextElementSibling
      while (sibling && !sibling.matches('h1, h2, h3, h4, h5, h6')) {
        const nextSibling = sibling.nextElementSibling
        answerContent.appendChild(sibling)
        sibling = nextSibling
      }

      wrapper.appendChild(header)
      wrapper.appendChild(answerContent)
      heading.replaceWith(wrapper)

      header.addEventListener('click', () => {
        wrapper.classList.toggle('expanded')
      })
    }
  })
}

const processCrossReferenceLinks = () => {
  if (!contentRef.value) return

  const links = contentRef.value.querySelectorAll('a')
  links.forEach(link => {
    const href = link.getAttribute('href')
    if (!href) return

    if (href.startsWith('http://') || href.startsWith('https://') ||
        href.startsWith('mailto:') || href.startsWith('#') ||
        href.startsWith('javascript:')) return

    if (href.startsWith('../') || href.startsWith('./') ||
        (!href.startsWith('/') && href.endsWith('.md'))) {
      link.addEventListener('click', (e) => {
        e.preventDefault()
        const resolvedPath = resolveRelativePath(props.currentDoc, href)
        if (resolvedPath) {
          navigationStack.value.push(props.currentDoc)
          emit('navigate', resolvedPath)
        }
      })
      link.classList.add('cross-ref-link')
    }
  })
}

const processSimBlocks = () => {
  if (!contentRef.value) return
  const simPlaceholders = contentRef.value.querySelectorAll('[data-sim]')
  simPlaceholders.forEach(placeholder => {
    const simName = placeholder.getAttribute('data-sim')
    if (!simName || !SimRegistry[simName]) return
    const app = createApp(SimRegistry[simName])
    const mountEl = document.createElement('div')
    mountEl.className = 'sim-wrapper'

    app.config.errorHandler = (err, _instance, info) => {
      console.error(`[Sim Error] ${simName}:`, err, info)
      mountEl.innerHTML = `<div class="sim-error">
        <p><strong>仿真组件错误</strong></p>
        <p class="sim-error-detail">${simName} 渲染失败：${String(err).slice(0, 200)}</p>
        <p class="sim-error-hint">请尝试刷新页面或调整参数后重试</p>
      </div>`
    }

    placeholder.replaceWith(mountEl)
    app.mount(mountEl)
  })
}

const processHtmlSimBlocks = () => {
  if (!contentRef.value) return
  const htmlSimPlaceholders = contentRef.value.querySelectorAll('[data-sim-html]')
  htmlSimPlaceholders.forEach(placeholder => {
    const filename = placeholder.getAttribute('data-sim-html')
    if (!filename) return
    const container = document.createElement('div')
    container.className = 'html-sim-container'
    const header = document.createElement('div')
    header.className = 'html-sim-header'
    const title = document.createElement('span')
    title.className = 'html-sim-title'
    title.textContent = filename.replace('.html', '').replace(/_/g, ' ')
    const fullBtn = document.createElement('button')
    fullBtn.className = 'html-sim-fullscreen-btn'
    fullBtn.textContent = '⛶ 全屏打开'
    fullBtn.onclick = () => {
      const simName = filename === 'foc_sim.html' ? 'foc-sim' :
                      filename === 'comm_sim.html' ? 'comm-sim' :
                      filename === 'serial_assistant.html' ? 'serial-assistant' :
                      'oscillator-sim'
      emit('selectSim', simName)
    }
    header.appendChild(title)
    header.appendChild(fullBtn)
    const iframe = document.createElement('iframe')
    iframe.src = `/sims/${filename}`
    iframe.style.width = '100%'
    iframe.style.height = '600px'
    iframe.style.border = 'none'
    iframe.style.borderRadius = '8px'
    iframe.style.borderTop = '1px solid var(--border-color)'
    container.appendChild(header)
    container.appendChild(iframe)
    placeholder.replaceWith(container)
  })
}

const resolveRelativePath = (currentPath: string, relativePath: string): string | null => {
  try {
    const cleanRelative = relativePath.split('#')[0]
    if (!cleanRelative) return null

    const parts = currentPath.split('/')
    parts.pop()

    const relParts = cleanRelative.split('/')
    for (const part of relParts) {
      if (part === '..') {
        parts.pop()
      } else if (part !== '.' && part !== '') {
        parts.push(part)
      }
    }

    return parts.join('/')
  } catch {
    return null
  }
}

const processAnnotationButtons = () => {
  if (!contentRef.value) return
  const headings = contentRef.value.querySelectorAll('h2, h3')
  headings.forEach((heading) => {
    const existing = heading.querySelector('.annotation-btn')
    if (existing) existing.remove()
    const id = heading.getAttribute('id') || ''
    if (!id) return
    const btn = document.createElement('button')
    btn.className = 'annotation-btn'
    const hasAnnotation = annotations.value[id] && annotations.value[id].content
    btn.innerHTML = hasAnnotation ? '📝' : '📝 添加注释'
    btn.title = hasAnnotation ? '编辑注释' : '添加注释'
    btn.style.opacity = hasAnnotation ? '1' : '0.5'
    btn.style.padding = '2px 6px'
    btn.style.fontSize = '0.8rem'
    btn.style.borderRadius = '4px'
    btn.style.border = hasAnnotation ? '1px solid var(--primary-color)' : '1px dashed var(--border-color)'
    btn.style.backgroundColor = hasAnnotation ? 'var(--primary-color)15' : 'transparent'
    btn.onclick = (e: Event) => {
      e.stopPropagation()
      handleAnnotationClick(id)
    }
    heading.appendChild(btn)
  })
}

const handleAnnotationClick = (sectionId: string) => {
  if (annotationDirty.value && activeAnnotationId.value !== sectionId) {
    if (!window.confirm('你有未保存的注释，确定放弃修改吗？')) return
  }
  if (activeAnnotationId.value === sectionId) {
    closeAnnotationPanel()
    return
  }
  activeAnnotationId.value = sectionId
  const existing = annotations.value[sectionId]
  if (existing && existing.content) {
    annotationDraft.value = existing.content
    annotationDirty.value = false
    annotationEditMode.value = false
  } else {
    annotationDraft.value = ''
    annotationDirty.value = false
    annotationEditMode.value = true
  }
}

const closeAnnotationPanel = () => {
  activeAnnotationId.value = null
  annotationDraft.value = ''
  annotationDirty.value = false
  annotationEditMode.value = false
}

const handleSaveAnnotation = async () => {
  if (!activeAnnotationId.value) return
  const sectionId = activeAnnotationId.value
  const heading = contentRef.value?.querySelector(`#${CSS.escape(sectionId)}`)
  const sectionTitle = heading?.textContent?.replace(/📝/g, '').trim() || sectionId
  try {
    if (annotationDraft.value.trim() === '') {
      await axios.delete('/api/annotations/', { data: { docPath: props.currentDoc, sectionId } })
      delete annotations.value[sectionId]
    } else {
      await axios.put('/api/annotations/', { docPath: props.currentDoc, sectionId, sectionTitle, content: annotationDraft.value })
      annotations.value[sectionId] = { sectionId, sectionTitle, content: annotationDraft.value }
    }
    closeAnnotationPanel()
    savedToast.value = true
    setTimeout(() => { savedToast.value = false }, 2000)
    processAnnotationButtons()
  } catch (e) {
    console.error('Failed to save annotation:', e)
  }
}

const handleCancelAnnotation = () => {
  if (annotationDirty.value) {
    if (!window.confirm('你有未保存的注释，确定放弃修改吗？')) return
  }
  closeAnnotationPanel()
}

const handleDeleteAnnotation = async () => {
  if (!window.confirm('确定删除这条注释吗？')) return
  if (!activeAnnotationId.value) return
  try {
    await axios.delete('/api/annotations/', { data: { docPath: props.currentDoc, sectionId: activeAnnotationId.value } })
    delete annotations.value[activeAnnotationId.value]
    closeAnnotationPanel()
    processAnnotationButtons()
  } catch (e) {
    console.error('Failed to delete annotation:', e)
  }
}

const goBack = () => {
  if (navigationStack.value.length > 0) {
    const prevDoc = navigationStack.value.pop()!
    emit('navigate', prevDoc)
  }
}

const scrollToTop = () => {
  if (contentRef.value) {
    contentRef.value.scrollTop = 0
  }
}

const handleScroll = () => {
  if (!contentRef.value) return
  const el = contentRef.value
  const scrollTop = el.scrollTop
  const scrollHeight = el.scrollHeight - el.clientHeight
  if (scrollHeight > 0) {
    readingProgress.value = Math.round((scrollTop / scrollHeight) * 100)
    if (readingProgress.value > 0) {
      updateReadingProgress(props.currentDoc, readingProgress.value)
    }
  }
}

watch(() => props.currentDoc, loadContent)

onMounted(() => {
  if (props.currentDoc) {
    loadContent(props.currentDoc)
  }
})

watch(content, async () => {
  await nextTick()
  processAnnotationButtons()
})
</script>

<template>
  <main class="content" :class="{ expanded: sidebarCollapsed }">
    <div class="reading-progress-bar">
      <div class="reading-progress-fill" :style="{ width: readingProgress + '%' }"></div>
    </div>
    <div v-if="navigationStack.length > 0" class="back-nav">
      <button class="back-btn" @click="goBack">← 返回上一文档</button>
    </div>
    <div v-if="loading" class="loading">
      <div class="loading-spinner"></div>
      加载中...
    </div>
    <div v-else-if="errorMsg" class="error-container">
      <p class="error">{{ errorMsg }}</p>
      <button class="retry-btn" @click="loadContent(props.currentDoc)">重试</button>
    </div>
    <template v-else>
      <div v-if="relatedSims.length > 0" class="related-sims-banner">
        <span class="sims-label">🔬 相关仿真：</span>
        <button
          v-for="sim in relatedSims"
          :key="sim.simName"
          class="sim-jump-btn"
          @click="emit('selectSim', sim.simName)"
        >
          {{ sim.name }}
        </button>
      </div>
      <article ref="contentRef" class="markdown-body" v-html="content" @scroll="handleScroll"></article>
    </template>
    <div v-if="activeAnnotationId" class="annotation-panel">
      <div class="annotation-panel-header">
        <span class="annotation-panel-title">注释</span>
        <div class="annotation-panel-actions">
          <template v-if="annotationEditMode">
            <button class="ann-btn ann-btn-save" @click="handleSaveAnnotation">保存</button>
            <button class="ann-btn ann-btn-cancel" @click="handleCancelAnnotation">取消</button>
          </template>
          <template v-else>
            <button class="ann-btn ann-btn-edit" @click="annotationEditMode = true; annotationDirty = false">编辑</button>
            <button class="ann-btn ann-btn-delete" @click="handleDeleteAnnotation">删除</button>
          </template>
        </div>
      </div>
      <textarea
        v-if="annotationEditMode"
        v-model="annotationDraft"
        @input="annotationDirty = true"
        class="annotation-textarea"
        placeholder="输入你的理解和注释..."
      ></textarea>
      <div v-else class="annotation-content" v-html="annotationDraft"></div>
    </div>
    <div v-if="savedToast" class="annotation-toast">✅ 注释已保存</div>
    <TocNav v-if="tocHeadings.length > 2" :headings="tocHeadings" />
  </main>
</template>

<style scoped>
.content {
  flex: 1;
  margin-left: 300px;
  padding: 2rem;
  overflow-y: auto;
  height: 100vh;
  transition: margin-left 0.3s ease;
  position: relative;
}

.content.expanded {
  margin-left: 50px;
}

.reading-progress-bar {
  position: sticky;
  top: 0;
  height: 3px;
  background: var(--border-color);
  z-index: 10;
}

.reading-progress-fill {
  height: 100%;
  background: linear-gradient(90deg, var(--primary-color), #8b5cf6);
  transition: width 0.1s linear;
}

.back-nav {
  margin-bottom: 1rem;
}

.back-btn {
  background: none;
  border: 1px solid var(--border-color);
  padding: 0.4rem 0.8rem;
  cursor: pointer;
  border-radius: 6px;
  font-size: 0.85rem;
  color: var(--primary-color);
  transition: all 0.2s;
}

.back-btn:hover {
  background-color: var(--primary-color);
  color: white;
  border-color: var(--primary-color);
}

.loading {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 50vh;
  color: var(--text-secondary);
  gap: 1rem;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 3px solid var(--border-color);
  border-top-color: var(--primary-color);
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.markdown-body {
  max-width: 900px;
  margin: 0 auto;
  background-color: var(--content-bg);
  padding: 2rem;
  border-radius: 8px;
  box-shadow: var(--shadow);
}

.markdown-body :deep(h1) {
  font-size: 2rem;
  margin-bottom: 1rem;
  padding-bottom: 0.5rem;
  border-bottom: 2px solid var(--primary-color);
  color: var(--text-color);
}

.markdown-body :deep(h2) {
  font-size: 1.5rem;
  margin: 1.5rem 0 1rem;
  color: var(--text-color);
}

.markdown-body :deep(h3) {
  font-size: 1.25rem;
  margin: 1rem 0 0.75rem;
  color: var(--text-color);
}

.markdown-body :deep(p) {
  margin-bottom: 1rem;
  line-height: 1.8;
}

.markdown-body :deep(pre) {
  background-color: var(--code-bg);
  padding: 1rem;
  border-radius: 6px;
  overflow-x: auto;
  margin: 1rem 0;
}

.markdown-body :deep(code) {
  font-family: 'Consolas', 'Monaco', monospace;
  font-size: 0.875rem;
}

.markdown-body :deep(ul),
.markdown-body :deep(ol) {
  margin: 1rem 0;
  padding-left: 2rem;
}

.markdown-body :deep(li) {
  margin-bottom: 0.5rem;
}

.markdown-body :deep(blockquote) {
  border-left: 4px solid var(--primary-color);
  padding-left: 1rem;
  margin: 1rem 0;
  color: var(--text-secondary);
  background-color: var(--bg-color);
  padding: 1rem;
  border-radius: 0 6px 6px 0;
}

.markdown-body :deep(table) {
  width: 100%;
  border-collapse: collapse;
  margin: 1rem 0;
}

.markdown-body :deep(th),
.markdown-body :deep(td) {
  border: 1px solid var(--border-color);
  padding: 0.75rem;
  text-align: left;
}

.markdown-body :deep(th) {
  background-color: var(--bg-color);
  font-weight: 600;
}

.markdown-body :deep(.answer-block) {
  margin: 1rem 0;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  overflow: hidden;
}

.markdown-body :deep(.answer-header) {
  display: flex;
  align-items: center;
  padding: 0.75rem 1rem;
  background-color: var(--bg-color);
  cursor: pointer;
  user-select: none;
}

.markdown-body :deep(.answer-header:hover) {
  background-color: var(--code-bg);
}

.markdown-body :deep(.answer-icon) {
  margin-right: 0.5rem;
  transition: transform 0.3s ease;
  font-size: 0.75rem;
  color: var(--primary-color);
}

.markdown-body :deep(.answer-block.expanded .answer-icon) {
  transform: rotate(90deg);
}

.markdown-body :deep(.answer-content) {
  max-height: 0;
  overflow: hidden;
  transition: max-height 0.3s ease, padding 0.3s ease;
  padding: 0 1rem;
}

.markdown-body :deep(.answer-block.expanded .answer-content) {
  max-height: 5000px;
  padding: 1rem;
}

.markdown-body :deep(.cross-ref-link) {
  color: var(--primary-color);
  text-decoration: none;
  border-bottom: 1px dashed var(--primary-color);
  cursor: pointer;
}

.markdown-body :deep(.cross-ref-link:hover) {
  color: var(--primary-hover);
  border-bottom-style: solid;
}

.markdown-body :deep(.sim-wrapper) {
  margin: 1.5rem 0;
}

.markdown-body :deep(.sim-error) {
  border: 1px solid #fca5a5;
  border-radius: 8px;
  padding: 1rem;
  background: #fef2f2;
  text-align: center;
  color: #991b1b;
}

.markdown-body :deep(.sim-error strong) {
  font-size: 0.95rem;
}

.markdown-body :deep(.sim-error-detail) {
  font-family: 'Consolas', 'Monaco', monospace;
  font-size: 0.78rem;
  color: #dc2626;
  margin: 0.5rem 0;
  word-break: break-all;
}

.markdown-body :deep(.sim-error-hint) {
  font-size: 0.75rem;
  color: #6b7280;
  margin-top: 0.5rem;
}

.error {
  color: #ef4444;
  text-align: center;
  padding: 2rem;
}

.error-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  min-height: 200px;
  gap: 1rem;
}

.retry-btn {
  background-color: var(--primary-color);
  color: white;
  border: none;
  padding: 0.5rem 1.5rem;
  border-radius: 6px;
  cursor: pointer;
  font-size: 0.875rem;
  transition: opacity 0.2s;
}

.retry-btn:hover {
  opacity: 0.85;
}

@media (max-width: 768px) {
  .content {
    margin-left: 0;
    padding: 1rem;
  }

  .content.expanded {
    margin-left: 0;
  }

  .markdown-body {
    padding: 1rem;
  }
}

.annotation-btn {
  background: none;
  border: none;
  cursor: pointer;
  font-size: 0.8rem;
  padding: 2px 6px;
  border-radius: 4px;
  border: 1px dashed var(--border-color);
  opacity: 0.5;
  transition: all 0.2s;
}

.annotation-btn:hover {
  opacity: 1 !important;
  background: var(--primary-color);
  color: white;
  border-color: var(--primary-color);
}

.annotation-panel {
  max-width: 900px;
  margin: 0 auto 1rem auto;
  padding: 1rem;
  background: var(--bg-color);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  animation: annotationSlideIn 0.2s ease;
}

@keyframes annotationSlideIn {
  from { opacity: 0; transform: translateY(-8px); }
  to { opacity: 1; transform: translateY(0); }
}

.annotation-panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.75rem;
}

.annotation-panel-title {
  font-size: 0.85rem;
  font-weight: 600;
  color: var(--primary-color);
}

.annotation-panel-actions {
  display: flex;
  gap: 0.5rem;
}

.ann-btn {
  padding: 0.25rem 0.75rem;
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

.ann-btn-delete {
  color: #ef4444;
  border-color: #fca5a5;
}

.annotation-textarea {
  width: 100%;
  min-height: 100px;
  padding: 0.75rem;
  border: 1px solid var(--border-color);
  border-radius: 6px;
  background: var(--content-bg);
  color: var(--text-color);
  font-size: 0.9rem;
  resize: vertical;
  font-family: inherit;
}

.annotation-content {
  font-size: 0.9rem;
  line-height: 1.7;
  color: var(--text-secondary);
}

.annotation-toast {
  position: fixed;
  top: 1rem;
  right: 1rem;
  background: #10b981;
  color: white;
  padding: 0.5rem 1rem;
  border-radius: 6px;
  font-size: 0.85rem;
  z-index: 1000;
  animation: annotationSlideIn 0.2s ease;
}

.related-sims-banner {
  max-width: 900px;
  margin: 0 auto 1rem auto;
  padding: 0.6rem 0.8rem;
  background: linear-gradient(135deg, var(--primary-color)08, var(--content-bg));
  border: 1px solid var(--primary-color)30;
  border-radius: 8px;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  flex-wrap: wrap;
}

.sims-label {
  font-size: 0.85rem;
  font-weight: 600;
  color: var(--primary-color);
  white-space: nowrap;
}

.sim-jump-btn {
  padding: 0.25rem 0.6rem;
  background: var(--primary-color);
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 0.78rem;
  font-weight: 500;
  transition: all 0.15s;
}

.sim-jump-btn:hover {
  filter: brightness(1.1);
  transform: translateY(-1px);
}

.markdown-body :deep(.html-sim-container) {
  margin: 1.5rem 0;
  border: 1px solid var(--border-color);
  border-radius: 8px;
  overflow: hidden;
}

.markdown-body :deep(.html-sim-header) {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.5rem 0.8rem;
  background: var(--bg-color);
  border-bottom: 1px solid var(--border-color);
}

.markdown-body :deep(.html-sim-title) {
  font-size: 0.85rem;
  font-weight: 600;
  color: var(--primary-color);
}

.markdown-body :deep(.html-sim-fullscreen-btn) {
  padding: 0.2rem 0.6rem;
  background: var(--primary-color);
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 0.75rem;
  transition: all 0.15s;
}

.markdown-body :deep(.html-sim-fullscreen-btn:hover) {
  filter: brightness(1.1);
  transform: translateY(-1px);
}
</style>
