<script setup lang="ts">
import { ref, onMounted } from 'vue'
import axios from 'axios'
import Sidebar from './components/Sidebar.vue'
import Content from './components/Content.vue'
import QuizPanel from './components/QuizPanel.vue'
import NotesPage from './components/NotesPage.vue'
import { useProgress } from './stores/progress'
import ParkTransformSim from './components/simulations/ParkTransformSim.vue'
import BodePlotSim from './components/simulations/BodePlotSim.vue'
import PIDTunerSim from './components/simulations/PIDTunerSim.vue'
import MosfetSwitchingSim from './components/simulations/MosfetSwitchingSim.vue'
import SVPWMSim from './components/simulations/SVPWMSim.vue'
import RLCSim from './components/simulations/RLCSim.vue'
import EmachenySim from './components/simulations/EmachenySim.vue'
import ClarkeParkSim from './components/simulations/ClarkeParkSim.vue'
import SamplingTimingSim from './components/simulations/SamplingTimingSim.vue'
import OvermodulationSim from './components/simulations/OvermodulationSim.vue'
import CurrentLoopSim from './components/simulations/CurrentLoopSim.vue'
import ObserverSim from './components/simulations/ObserverSim.vue'
import MTPASim from './components/simulations/MTPASim.vue'
import THDSim from './components/simulations/THDSim.vue'
import DeadtimeCompSim from './components/simulations/DeadtimeCompSim.vue'
import InitialPositionSim from './components/simulations/InitialPositionSim.vue'
import SpeedLoopSim from './components/simulations/SpeedLoopSim.vue'

const sidebarCollapsed = ref(false)
const currentDoc = ref('')
const currentSim = ref('')
const showQuiz = ref(false)
const showNotes = ref(false)
const currentAssessmentPath = ref('')
const previousDoc = ref('')
const knowledgeBase = ref<any>(null)

const simComponents: Record<string, any> = {
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

const htmlSims: Record<string, string> = {
  'foc-sim': '/sims/foc_sim.html',
  'oscillator-sim': '/sims/oscillator_sim.html',
  'serial-assistant': '/sims/serial_assistant.html',
  'comm-sim': '/sims/comm_sim.html',
}

const isHtmlSim = (simName: string) => !!htmlSims[simName]

const { markVisited } = useProgress()

const toggleSidebar = () => {
  sidebarCollapsed.value = !sidebarCollapsed.value
}

const selectDoc = (filePath: string) => {
  currentDoc.value = filePath
  markVisited(filePath)
  history.pushState(null, '', '/' + filePath)
}

const navigateToDoc = (filePath: string) => {
  currentDoc.value = filePath
  history.pushState(null, '', '/' + filePath)
}

const handleSelectSim = (simName: string) => {
  previousDoc.value = currentDoc.value
  currentSim.value = simName
  currentDoc.value = ''
  showQuiz.value = false
}

const handleStartQuiz = (assessmentPath: string) => {
  currentAssessmentPath.value = assessmentPath
  showQuiz.value = true
}

const handleCloseQuiz = () => {
  showQuiz.value = false
}

const handleOpenNotes = () => {
  showNotes.value = true
  currentDoc.value = ''
  showQuiz.value = false
}

const handleCloseNotes = () => {
  showNotes.value = false
}

const handleReturnFromSim = () => {
  const prev = previousDoc.value
  currentSim.value = ''
  if (prev) {
    currentDoc.value = prev
    previousDoc.value = ''
  }
}

onMounted(async () => {
  try {
    const res = await axios.get('/api/docs/list')
    knowledgeBase.value = res.data
  } catch (e) {}

  const path = window.location.pathname
  if (path && path !== '/' && path.endsWith('.md')) {
    const docPath = path.replace(/^\//, '')
    currentDoc.value = docPath
  }
})
</script>

<template>
  <div v-show="!!currentSim" class="sim-fullscreen">
    <button class="sim-close-btn" @click="handleReturnFromSim">← 返回知识库</button>
    <component :is="simComponents[currentSim]" v-if="simComponents[currentSim]" />
    <iframe v-if="isHtmlSim(currentSim)" :src="htmlSims[currentSim]" class="sim-fullscreen-iframe"></iframe>
  </div>
  <Sidebar
    v-show="!currentSim"
    :collapsed="sidebarCollapsed"
    @toggle="toggleSidebar"
    @select="selectDoc"
    @start-quiz="handleStartQuiz"
    @select-sim="handleSelectSim"
    @open-notes="handleOpenNotes"
  />
  <Content
    v-show="!currentSim"
    :sidebar-collapsed="sidebarCollapsed"
    :current-doc="currentDoc"
    :simulations="knowledgeBase?.simulations || []"
    @navigate="navigateToDoc"
    @select-sim="handleSelectSim"
  />
  <NotesPage v-if="showNotes && !currentSim" @close="handleCloseNotes" />
  <QuizPanel
    v-if="showQuiz && currentAssessmentPath && !currentSim"
    :assessment-path="currentAssessmentPath"
    @close="handleCloseQuiz"
  />
</template>

<style scoped>
.sim-fullscreen {
  flex: 1;
  margin-left: 300px;
  padding: 2rem;
  overflow-y: auto;
  height: 100vh;
}

.sim-close-btn {
  background: none;
  border: 1px solid var(--border-color);
  padding: 0.5rem 1rem;
  cursor: pointer;
  border-radius: 6px;
  font-size: 0.9rem;
  color: var(--primary-color);
  margin-bottom: 1rem;
  transition: all 0.2s;
}

.sim-close-btn:hover {
  background-color: var(--primary-color);
  color: white;
}

.sim-fullscreen-iframe {
  width: 100%;
  height: calc(100vh - 80px);
  border: none;
  border-radius: 8px;
}
</style>
