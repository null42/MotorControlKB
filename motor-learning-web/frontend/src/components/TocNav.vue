<script setup lang="ts">
import { ref, watch, onMounted, onUnmounted, nextTick } from 'vue'

interface TocItem {
  level: number
  text: string
  id: string
}

const props = defineProps<{
  headings: TocItem[]
}>()

const activeId = ref('')
const isCollapsed = ref(false)

const scrollToHeading = (id: string) => {
  const el = document.getElementById(id)
  if (el) {
    el.scrollIntoView({ behavior: 'smooth', block: 'start' })
  }
}

const updateActiveHeading = () => {
  const headings = props.headings
  for (let i = headings.length - 1; i >= 0; i--) {
    const el = document.getElementById(headings[i].id)
    if (el) {
      const rect = el.getBoundingClientRect()
      if (rect.top <= 100) {
        activeId.value = headings[i].id
        return
      }
    }
  }
  if (headings.length > 0) {
    activeId.value = headings[0].id
  }
}

let scrollListener: (() => void) | null = null

onMounted(() => {
  scrollListener = () => updateActiveHeading()
  window.addEventListener('scroll', updateActiveHeading, { passive: true })
})

onUnmounted(() => {
  if (scrollListener) {
    window.removeEventListener('scroll', scrollListener)
  }
})

watch(() => props.headings, () => {
  nextTick(() => updateActiveHeading())
})
</script>

<template>
  <div class="toc-nav" :class="{ collapsed: isCollapsed }">
    <div class="toc-header" @click="isCollapsed = !isCollapsed">
      <span class="toc-title">目录</span>
      <span class="toc-toggle">{{ isCollapsed ? '◀' : '▶' }}</span>
    </div>
    <nav v-if="!isCollapsed" class="toc-list">
      <a
        v-for="heading in headings"
        :key="heading.id"
        class="toc-item"
        :class="{ active: activeId === heading.id }"
        :style="{ paddingLeft: (heading.level - 1) * 12 + 8 + 'px' }"
        @click.prevent="scrollToHeading(heading.id)"
      >
        {{ heading.text }}
      </a>
    </nav>
  </div>
</template>

<style scoped>
.toc-nav {
  position: fixed;
  right: 20px;
  top: 80px;
  width: 220px;
  max-height: calc(100vh - 100px);
  background: var(--content-bg);
  border: 1px solid var(--border-color);
  border-radius: 8px;
  box-shadow: var(--shadow);
  overflow: hidden;
  z-index: 50;
  transition: width 0.2s;
}

.toc-nav.collapsed {
  width: 40px;
}

.toc-header {
  padding: 0.6rem 0.75rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
  cursor: pointer;
  border-bottom: 1px solid var(--border-color);
  font-size: 0.8rem;
  font-weight: 600;
  color: var(--text-secondary);
}

.toc-toggle {
  font-size: 0.65rem;
}

.toc-list {
  padding: 0.5rem 0;
  max-height: calc(100vh - 160px);
  overflow-y: auto;
}

.toc-item {
  display: block;
  padding: 0.3rem 0.75rem;
  font-size: 0.75rem;
  color: var(--text-secondary);
  text-decoration: none;
  cursor: pointer;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  transition: all 0.15s;
  border-left: 2px solid transparent;
}

.toc-item:hover {
  color: var(--primary-color);
  background: var(--bg-color);
}

.toc-item.active {
  color: var(--primary-color);
  font-weight: 600;
  border-left-color: var(--primary-color);
  background: var(--bg-color);
}

@media (max-width: 1200px) {
  .toc-nav {
    display: none;
  }
}
</style>
